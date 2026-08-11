#include "render/parma_blind_renderer.hpp"

#include "dtsx/speaker_mask.hpp"
#include "render/parma_blind_config.hpp"
#include "render/parma_critical_bands.hpp"
#include "render/parma_layout.hpp"
#include "render/parma_pairwise.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#ifdef DTSX_PARMA_TESTING
#include <cstdio>
#include <cstdlib>
#endif
#include <limits>
#include <string_view>

namespace dtsx_decode {
namespace {

constexpr std::size_t kBlockSize = 64U;
constexpr float kInt24Scale = 8388608.0F;
constexpr float kDenormalOffset = 1.0e-18F;
// Exact IEEE-754 constants loaded by the native ARM implementation at
// ParmaDec_Mono2StereoConversion and ParmaDec_SoundfieldModificationsStereo.
constexpr float kParmaMinusHalfPi = -1.5707963705062866F;
constexpr float kParmaHalfPi = 1.5707963705062866F;
constexpr float kParmaSqrtHalf = 0.7079457640647888F;
constexpr float kStereoRear0 = 0.24119840562343597F;
constexpr float kStereoRear1 = 0.3324204683303833F;
constexpr float kStereoRear2 = 0.6675795316696167F;
constexpr float kStereoRear3 = 0.7588015794754028F;

std::int32_t parma_channel_for_speaker(
    std::uint32_t speaker_mask) noexcept {
    if (speaker_mask == (1U << 5U)) {
        return -2;
    }
    for (std::uint32_t activity = 0U; activity < 20U; ++activity) {
        const std::uint32_t activity_mask =
            dtsx::speaker_mask_to_activity_mask(speaker_mask);
        if ((activity_mask & (1U << activity)) == 0U) {
            continue;
        }
        const std::uint32_t parma_mask =
            parma_channel_mask_from_speaker_activity(
                1U << activity);
        const std::vector<std::uint32_t> speakers =
            dtsx::expand_speaker_activity_mask(1U << activity);
        const auto found =
            std::find(speakers.begin(), speakers.end(), speaker_mask);
        if (found == speakers.end()) {
            return -1;
        }
        const std::size_t ordinal = static_cast<std::size_t>(
            std::distance(speakers.begin(), found));
        std::size_t selected = 0U;
        for (std::uint32_t bit = 0U; bit < 28U; ++bit) {
            if ((parma_mask & (1U << bit)) == 0U) {
                continue;
            }
            if (selected++ == ordinal) {
                return static_cast<std::int32_t>(bit);
            }
        }
    }
    return -1;
}

std::int32_t parma_channel_for_name(
    std::string_view name) noexcept {
    std::uint32_t speaker_mask = 0U;
    if (!dtsx::standard_speaker_mask(name, speaker_mask)) {
        return -1;
    }
    return parma_channel_for_speaker(speaker_mask);
}

std::int32_t int24_from_float(float value) noexcept {
    const double scaled =
        static_cast<double>(value) * kInt24Scale;
    const double clipped =
        std::max(-8388608.0, std::min(8388607.0, scaled));
    return static_cast<std::int32_t>(
        clipped >= 0.0
            ? std::floor(clipped + 0.5)
            : std::ceil(clipped - 0.5));
}

float smoothing_coefficient(
    std::uint32_t sample_rate,
    float seconds) noexcept {
    return 1.0F
        - std::exp(
            -64.0F
            / (static_cast<float>(sample_rate) * seconds));
}

void parma_phase_shift(
    float* real, float* imaginary, float angle) noexcept {
    const float sine = std::sin(angle);
    const float cosine = std::cos(angle);
    for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
        const float old_real = real[bin];
        const float old_imaginary = imaginary[bin];
        real[bin] = old_real * cosine - old_imaginary * sine;
        imaginary[bin] = old_imaginary * cosine + old_real * sine;
    }
}

float parma_stereo_smooth(
    float previous, float value) noexcept {
    // ParmaDec_SpatialAnalysisStereo uses the three fixed native controls
    // at decoder words 19734..19736.
    float coefficient = 0.0080000162F;
    if (value > 0.95F) {
        coefficient = previous < value ? 0.100000024F : 0.025F;
    } else if (value > 0.7F && previous < value) {
        coefficient = 0.025F;
    }
    return ((1.0F - coefficient) * previous
            + coefficient * value + kDenormalOffset)
        - kDenormalOffset;
}

float parma_stereo_radial_value(
    float first_axis, float second_axis) noexcept {
    float magnitude = std::sqrt(
        first_axis * first_axis + second_axis * second_axis);
    const float angle = std::atan2(first_axis, second_axis);
    if (angle < 0.0F) {
        float weight = std::fabs(angle * -2.0F * 0.31831F - 1.0F);
        weight = std::max(0.0F, weight * 2.0F - 1.0F);
        magnitude *= weight;
    }
    return magnitude;
}

[[nodiscard]] bool row_17_pair_rule(
    const ParmaBlindLayerConfig& layer,
    std::uint8_t target,
    std::uint8_t first_source,
    std::uint8_t second_source,
    float& decoding_angle) noexcept {
    const ParmaBlindChannelRule& rule = layer.channels[target];
    if (rule.mode != 1U
        || rule.sources[0U] != first_source
        || rule.sources[1U] != second_source
        || rule.sources[2U] != kParmaUnusedChannel
        || rule.sources[3U] != kParmaUnusedChannel) {
        return false;
    }
    decoding_angle = rule.decoding_angle;
    return decoding_angle >= 0.0F && decoding_angle <= 1.0F;
}

} // namespace

bool ParmaBlindRenderer::initialize(
    std::uint32_t sample_rate) noexcept {
    if (!left_analysis_.initialize(sample_rate)
        || !right_analysis_.initialize(sample_rate)
        || !front_analysis_.initialize(sample_rate)
        || !rear_analysis_.initialize(sample_rate)
        || !auxiliary_left_analysis_.initialize(sample_rate)
        || !auxiliary_right_analysis_.initialize(sample_rate)) {
        return false;
    }
    for (ParmaAnalysisFilterBank& filter : analysis_filters_) {
        filter.reset();
    }
    for (ParmaSynthesisFilterBank& filter : synthesis_filters_) {
        filter.reset();
    }
    for (auto& gain : intermediate_gain_) {
        gain.fill(1.0F);
    }
    stereo_primary_state_.fill(1.0F);
    stereo_secondary_state_.fill(1.0F);
    stereo_repan_state_.fill(1.0F);
    stereo_output_state_.fill(1.0F);
    output_gain_.fill(1.0F);
    for (auto& delay : analysis_input_delay_) {
        delay.fill(0);
    }
    for (auto& delay : direct_delay_) {
        delay.fill(0.0F);
    }
    direct_delay_position_ = 0U;
    lfe_delay_.fill(0.0F);
    lfe_position_ = 0U;
    mono_delay_.fill(0.0F);
    mono_delay_position_ = 0U;
    energy_attack_ =
        smoothing_coefficient(sample_rate, 0.0063275F);
    energy_release_ =
        smoothing_coefficient(sample_rate, 0.012655F);
    sample_rate_ = sample_rate;
    initialized_ = true;
    return true;
}

void ParmaBlindRenderer::flush() noexcept {
    // DTS_ParmaDec_Flush -> DTS_ParmaDec_CleanInternalBuffers. The native
    // routine does not drain an audio tail; it merely makes the next block a
    // fresh analysis/synthesis sequence with the existing controls.
    if (!initialized_) {
        return;
    }
    for (ParmaAnalysisFilterBank& filter : analysis_filters_) {
        filter.reset();
    }
    for (ParmaSynthesisFilterBank& filter : synthesis_filters_) {
        filter.reset();
    }
    left_analysis_.reset();
    right_analysis_.reset();
    front_analysis_.reset();
    rear_analysis_.reset();
    auxiliary_left_analysis_.reset();
    auxiliary_right_analysis_.reset();
    for (auto& gain : intermediate_gain_) {
        gain.fill(1.0F);
    }
    stereo_primary_state_.fill(1.0F);
    stereo_secondary_state_.fill(1.0F);
    stereo_repan_state_.fill(1.0F);
    stereo_output_state_.fill(1.0F);
    output_gain_.fill(1.0F);
    for (auto& delay : analysis_input_delay_) {
        delay.fill(0);
    }
    for (auto& delay : direct_delay_) {
        delay.fill(0.0F);
    }
    direct_delay_position_ = 0U;
    lfe_delay_.fill(0.0F);
    lfe_position_ = 0U;
    mono_delay_.fill(0.0F);
    mono_delay_position_ = 0U;
    sample_rate_ = 0U;
    initialized_ = false;
}

bool ParmaBlindRenderer::render(
    const std::vector<std::vector<std::int32_t>>& input,
    std::uint32_t input_speaker_activity_mask,
    const ChannelLayout& output_layout,
    std::uint32_t sample_rate,
    std::vector<std::vector<std::int32_t>>& output) noexcept {
    // Native SetBlind horizontal elevation paths.  Every supported route
    // ends in the same 0x360DF height layer; rows 14/15 first reconstruct
    // their 7.1 intermediate layer.
    ParmaLayoutControls controls;
    std::uint32_t output_physical_mask = 0U;
    for (const std::string& channel : output_layout.channels) {
        std::uint32_t speaker = 0U;
        if (!dtsx::standard_speaker_mask(channel, speaker)) {
            return false;
        }
        output_physical_mask |= speaker;
    }
    const std::uint32_t output_activity_mask =
        dtsx::speaker_mask_to_activity_mask(output_physical_mask);
    if (!derive_parma_layout_controls(
            input_speaker_activity_mask,
            output_activity_mask,
            controls)
        || controls.blind_table_row < 0
        || (controls.input_main_channel_mask != 0x01U
            && controls.input_main_channel_mask != 0x06U
            && controls.input_main_channel_mask != 0x07U
            && controls.input_main_channel_mask != 0x26U
            && controls.input_main_channel_mask != 0x27U
            && controls.input_main_channel_mask != 0x1EU
            && controls.input_main_channel_mask != 0x1FU
            && controls.input_main_channel_mask != 0x3FU
            && controls.input_main_channel_mask != 0xDFU)
        || (controls.output_main_channel_mask != 0xDFU
            && controls.output_main_channel_mask != 0x360DFU)
        || parma_blind_layer_count(controls)
            != (controls.output_is_horizontal ? 1U : 2U)) {
        return false;
    }
    const bool mono_input = controls.input_main_channel_mask == 0x01U;
    const std::uint32_t processing_input_mask = mono_input
        ? 0x06U : controls.input_main_channel_mask;
    const std::int32_t effective_row = mono_input
        ? 10
        : controls.blind_table_row
            + (controls.output_is_horizontal ? 9 : 0);
    const bool row_10 = effective_row == 10;
    const bool row_11 = effective_row == 11;
    const bool row_14 = effective_row == 14;
    const bool row_12 = effective_row == 12;
    const bool row_13 = effective_row == 13;
    const bool row_15 = effective_row == 15;
    const bool row_16 = effective_row == 16;
    const bool has_first_layer = effective_row != 17;
    const bool has_height_output = !controls.output_is_horizontal;
    const bool has_side_rear_layer = row_14 || row_15;
    if (!initialized_) {
        if (!initialize(sample_rate)) {
            return false;
        }
    } else if (sample_rate_ != sample_rate) {
        return false;
    }

    const std::vector<std::uint32_t> input_speakers =
        dtsx::expand_speaker_activity_mask(
            input_speaker_activity_mask);
    if (input_speakers.size() != input.size()
        || input.empty()) {
        return false;
    }
    const std::size_t frame_count = input.front().size();
    if (frame_count == 0U || frame_count % kBlockSize != 0U) {
        return false;
    }
    std::array<std::int32_t, kParmaChannelCount>
        input_index_by_channel{};
    input_index_by_channel.fill(-1);
    std::int32_t lfe_input_index = -1;
    for (std::size_t input_index = 0U;
         input_index < input.size();
         ++input_index) {
        if (input[input_index].size() != frame_count) {
            return false;
        }
        const std::int32_t parma_channel =
            parma_channel_for_speaker(input_speakers[input_index]);
        if (parma_channel == -2) {
            lfe_input_index =
                static_cast<std::int32_t>(input_index);
        } else if (parma_channel >= 0
                   && parma_channel
                       < static_cast<std::int32_t>(
                             kParmaChannelCount)) {
            input_index_by_channel[
                static_cast<std::size_t>(parma_channel)] =
                static_cast<std::int32_t>(input_index);
        }
    }
    std::array<std::uint8_t, 11U> input_main{};
    std::size_t input_main_count = 0U;
    for (std::uint32_t channel = 0U;
         channel < kParmaChannelCount;
         ++channel) {
        if ((processing_input_mask & (1U << channel)) != 0U) {
            input_main[input_main_count++] =
                static_cast<std::uint8_t>(channel);
        }
    }
    for (std::size_t input_ordinal = 0U;
         input_ordinal < input_main_count;
         ++input_ordinal) {
        const std::uint8_t channel =
            input_main[input_ordinal];
        if (!mono_input && input_index_by_channel[channel] < 0) {
            return false;
        }
    }
    if (mono_input && input_index_by_channel[0U] < 0) {
        return false;
    }
    std::array<std::int32_t, kParmaChannelCount>
        output_index_by_channel{};
    output_index_by_channel.fill(-1);
    std::int32_t lfe_output_index = -1;
    for (std::size_t output_index = 0U;
         output_index < output_layout.channels.size();
         ++output_index) {
        const std::int32_t parma_channel =
            parma_channel_for_name(
                output_layout.channels[output_index]);
        if (parma_channel == -2) {
            lfe_output_index =
                static_cast<std::int32_t>(output_index);
        } else if (parma_channel >= 0
                   && parma_channel
                       < static_cast<std::int32_t>(
                             kParmaChannelCount)) {
            output_index_by_channel[
                static_cast<std::size_t>(parma_channel)] =
                static_cast<std::int32_t>(output_index);
        } else {
            return false;
        }
    }
    constexpr std::array<std::uint8_t, 11U> kOutputMain = {
        0U, 1U, 2U, 3U, 4U, 6U, 7U, 13U, 14U, 16U, 17U};
    const std::size_t output_main_count = has_height_output ? 11U : 7U;
    for (std::size_t ordinal = 0U;
         ordinal < output_main_count; ++ordinal) {
        const std::uint8_t channel = kOutputMain[ordinal];
        if (output_index_by_channel[channel] < 0) {
            return false;
        }
    }
    if (lfe_output_index < 0) {
        return false;
    }

    // Keep SetBlind source pairs and decoding angles in the native table.
    const ParmaBlindTopology* const native_topology =
        parma_blind_topology(
            processing_input_mask,
            controls.output_is_horizontal);
    if (native_topology == nullptr
        || native_topology->layer_count
            != (has_height_output ? 2U : 1U)) {
        return false;
    }
    float left_front_angle = 0.0F;
    float left_rear_angle = 0.0F;
    float right_front_angle = 0.0F;
    float right_rear_angle = 0.0F;
    float rear_left_angle = 0.0F;
    float rear_right_angle = 0.0F;
    float front_center_angle = 0.0F;
    if (has_height_output) {
        const ParmaBlindLayerConfig& row_17_pairwise =
            native_topology->layers[1U];
        if (!row_17_pair_rule(
                row_17_pairwise, 13U, 1U, 6U, left_front_angle)
            || !row_17_pair_rule(
                row_17_pairwise, 16U, 1U, 6U, left_rear_angle)
            || !row_17_pair_rule(
                row_17_pairwise, 14U, 2U, 7U, right_front_angle)
            || !row_17_pair_rule(
                row_17_pairwise, 17U, 2U, 7U, right_rear_angle)
            || left_front_angle > left_rear_angle
            || right_front_angle > right_rear_angle) {
            return false;
        }
    }
    if (row_14
        && !row_17_pair_rule(
            native_topology->layers[0U], 0U, 1U, 2U,
            front_center_angle)) {
        return false;
    }
    if (has_side_rear_layer
        && (!row_17_pair_rule(
                native_topology->layers[0U], 6U, 3U, 4U,
                rear_left_angle)
            || !row_17_pair_rule(
                native_topology->layers[0U], 7U, 3U, 4U,
                rear_right_angle)
            || rear_left_angle > rear_right_angle)) {
        return false;
    }

    std::array<bool, kParmaChannelCount> synthesized_channels{};
    for (std::uint32_t layer_index = 0U;
         layer_index < native_topology->layer_count;
         ++layer_index) {
        for (std::uint32_t target = 0U;
             target < kParmaChannelCount;
             ++target) {
            const ParmaBlindChannelRule& rule =
                native_topology->layers[layer_index].channels[target];
            if (rule.mode != 1U) {
                continue;
            }
            synthesized_channels[target] = true;
            synthesized_channels[rule.sources[0U]] = true;
            synthesized_channels[rule.sources[1U]] = true;
        }
    }
    if (row_10 || row_11) {
        for (std::size_t ordinal = 0U;
             ordinal < output_main_count; ++ordinal) {
            synthesized_channels[kOutputMain[ordinal]] = true;
        }
        if (row_11) {
            synthesized_channels[0U] = false;
        }
    }

    output.assign(
        output_layout.channels.size(),
        std::vector<std::int32_t>(frame_count, 0));
    std::array<std::array<float, kBlockSize>,
               kParmaChannelCount>
        input_real{};
    std::array<std::array<float, kBlockSize>,
               kParmaChannelCount>
        input_imaginary{};
    std::array<std::array<float, kBlockSize>,
               kParmaChannelCount>
        output_real{};
    std::array<std::array<float, kBlockSize>,
               kParmaChannelCount>
        output_imaginary{};
    std::array<float, kBlockSize> position{};
    std::array<float, kBlockSize> balance{};
    std::array<float, kBlockSize> diffuseness{};
    std::array<float, kBlockSize> stereo_output_scale{};
    stereo_output_scale.fill(1.0F);
    std::array<float, kBlockSize> synthesized_first{};
    std::array<float, kBlockSize> synthesized_second{};
    std::array<std::uint32_t, 16U> widths{};
    std::vector<std::uint32_t> width_vector;
    std::vector<float> bark_scale;
    if (!parma_initialize_critical_band_partitions(
            sample_rate, kBlockSize, 16U,
            width_vector, bark_scale)
        || width_vector.size() != widths.size()) {
        return false;
    }
    std::copy(
        width_vector.begin(), width_vector.end(), widths.begin());

    for (std::size_t frame_offset = 0U;
         frame_offset < frame_count;
         frame_offset += kBlockSize) {
        if (mono_input) {
            const std::size_t mono_index = static_cast<std::size_t>(
                input_index_by_channel[0U]);
            const std::size_t delay_length = static_cast<std::size_t>(
                static_cast<float>(sample_rate) * 0.0028F + 0.5F);
            if (delay_length == 0U
                || delay_length > 134U
                || delay_length > mono_delay_.size()) {
                return false;
            }
            constexpr std::array<std::uint8_t, 2U> kMonoChannels = {
                1U, 2U};
            for (const std::uint8_t channel : kMonoChannels) {
                analysis_filters_[channel].process(
                    analysis_input_delay_[channel].data(),
                    input_real[channel].data(),
                    input_imaginary[channel].data());
            }
            for (std::size_t sample = 0U;
                 sample < kBlockSize; ++sample) {
                const float current = static_cast<float>(
                    input[mono_index][frame_offset + sample]) / kInt24Scale;
                const float delayed = mono_delay_[mono_delay_position_];
                mono_delay_[mono_delay_position_] = current;
                mono_delay_position_ =
                    (mono_delay_position_ + 1U) % delay_length;
                analysis_input_delay_[1U][sample] =
                    (current + delayed) * kParmaSqrtHalf;
                analysis_input_delay_[2U][sample] =
                    (current - delayed) * kParmaSqrtHalf;
            }
        } else for (std::size_t input_ordinal = 0U;
             input_ordinal < input_main_count;
             ++input_ordinal) {
            const std::uint8_t channel =
                input_main[input_ordinal];
            const std::size_t input_index =
                static_cast<std::size_t>(
                    input_index_by_channel[channel]);
            // DTS_ParmaDec_Process invokes the native analysis filter on
            // its saved 64-sample cache, then updates that cache from the
            // caller's PCM pointers.  Feeding the current hop here advances
            // the pairwise controller by one hop and breaks its native
            // staging/latency relationship.
            analysis_filters_[channel].process(
                analysis_input_delay_[channel].data(),
                input_real[channel].data(),
                input_imaginary[channel].data());
            for (std::size_t sample = 0U;
                 sample < kBlockSize;
                 ++sample) {
                analysis_input_delay_[channel][sample] =
                    static_cast<float>(
                        input[input_index][frame_offset + sample])
                    / kInt24Scale;
            }
        }
        for (auto& channel : output_real) {
            channel.fill(0.0F);
        }
        for (auto& channel : output_imaginary) {
            channel.fill(0.0F);
        }
        for (std::size_t input_ordinal = 0U;
             input_ordinal < input_main_count;
             ++input_ordinal) {
            const std::uint8_t channel =
                input_main[input_ordinal];
            output_real[channel] = input_real[channel];
            output_imaginary[channel] = input_imaginary[channel];
        }
        if (row_10 || row_11) {
            front_analysis_.process(
                input_real[1U].data(), input_imaginary[1U].data(),
                input_real[2U].data(), input_imaginary[2U].data(),
                position.data(), balance.data(), diffuseness.data());

            std::array<float, kBlockSize> stereo_primary{};
            std::array<float, kBlockSize> stereo_secondary{};
            std::array<float, kBlockSize> stereo_repan{};
            std::array<float, kBlockSize> stereo_energy_ratio{};
            if (mono_input) {
                position.fill(0.5F);
                balance.fill(0.85F);
                diffuseness.fill(0.0F);
                stereo_primary.fill(1.0F);
                stereo_secondary.fill(0.4466836F);
                stereo_repan.fill(0.7079458F);
                stereo_output_scale.fill(1.0F);
            } else {
            parma_ungroup_critical_bands(
                front_analysis_.energy_ratio_state().data(),
                stereo_energy_ratio.data(), widths.data(), widths.size());
            // DTS_ParmaDec_SetupDecoder stores
            // 1/sqrt(output_soundfield_channel_count/2) at decoder+17656.
            const float stereo_setup_gain = std::sqrt(
                2.0F / static_cast<float>(output_main_count));
            std::size_t first_bin = 0U;
            for (std::size_t band = 0U; band < widths.size(); ++band) {
                const float band_balance = balance[first_bin];
                const float band_energy_ratio = stereo_energy_ratio[first_bin];
                float primary = stereo_setup_gain
                    + (1.0F - stereo_setup_gain)
                        * parma_stereo_radial_value(
                            2.0F * band_balance - 1.0F,
                            2.0F * band_energy_ratio - 1.0F);
                float secondary = stereo_setup_gain
                    + (1.0F - stereo_setup_gain)
                        * parma_stereo_radial_value(
                            1.0F - 2.0F * band_balance,
                            2.0F * band_energy_ratio - 1.0F);
                const float balance_axis =
                    2.0F * (1.2F * band_energy_ratio - 0.1F) - 1.0F;
                const float balance_metric =
                    balance_axis >= -1.0F && balance_axis <= 1.0F
                    ? balance_axis * balance_axis : 1.0F;
                float repan = stereo_setup_gain
                    + (1.0F - stereo_setup_gain)
                        * std::sqrt(
                            balance_metric
                            + (2.0F * band_balance - 1.0F)
                                * (2.0F * band_balance - 1.0F));
                const float radial = std::sqrt(
                    balance_metric
                    + (2.0F * band_balance - 1.0F)
                        * (2.0F * band_balance - 1.0F));
                float output_scale = std::max(
                    1.0F, 1.99526227F + radial * (1.0F - 1.99526227F));
                primary = std::min(1.0F, primary);
                secondary = std::min(1.0F, secondary);
                repan = std::min(1.0F, repan);
                stereo_primary_state_[band] = parma_stereo_smooth(
                    stereo_primary_state_[band], primary);
                stereo_secondary_state_[band] = parma_stereo_smooth(
                    stereo_secondary_state_[band], secondary);
                stereo_repan_state_[band] = parma_stereo_smooth(
                    stereo_repan_state_[band], repan);
                const float output_coefficient = output_scale >= 1.05F
                    ? 0.0080000162F
                    : (stereo_output_state_[band] <= output_scale
                        ? 0.025F : 0.100000024F);
                stereo_output_state_[band] =
                    ((1.0F - output_coefficient)
                         * stereo_output_state_[band]
                     + output_coefficient * output_scale
                     + kDenormalOffset)
                    - kDenormalOffset;
                for (std::size_t bin = first_bin;
                     bin < first_bin + widths[band]; ++bin) {
                    stereo_primary[bin] = stereo_primary_state_[band];
                    stereo_secondary[bin] = stereo_secondary_state_[band];
                    stereo_repan[bin] = stereo_repan_state_[band];
                    stereo_output_scale[bin] = stereo_output_state_[band];
                }
                first_bin += widths[band];
            }
            }

            const auto extract_stereo =
                [&](std::uint8_t target, std::uint8_t mode,
                    float bound0, float bound1,
                    float bound2, float bound3) {
                    const float first_inverse = 1.0F / (bound1 - bound0);
                    const float second_inverse = 1.0F / (bound3 - bound2);
                    const float center = 1.0F - (bound1 + bound2) * 0.5F;
                    for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                        const float direct = 1.0F - diffuseness[bin];
                        const float spatial =
                            center + direct * (position[bin] - center);
                        const float blend =
                            direct * (balance[bin] - 0.5F) + 0.5F;
                        float mirror = -spatial;
                        if (bound0 < spatial) {
                            if (spatial <= bound1) {
                                mirror = (spatial - bound0)
                                    * first_inverse - spatial;
                            } else if (spatial <= bound2) {
                                mirror = 1.0F - spatial;
                            } else if (spatial <= bound3) {
                                mirror = (spatial - bound2)
                                    * second_inverse - spatial + 1.0F;
                            } else {
                                mirror = 2.0F - spatial;
                            }
                        }
                        const float normalized_angle = mode == 6U
                            ? (1.0F - blend) * spatial + blend * mirror
                            : (1.0F - blend) * mirror + blend * spatial;
                        const float angle = normalized_angle * kParmaHalfPi;
                        const float modifier = mode == 6U
                            ? stereo_primary[bin] : stereo_secondary[bin];
                        const float first_coefficient =
                            std::sin(angle) * modifier;
                        float second_coefficient =
                            std::cos(angle) * modifier;
                        if (mode == 7U) {
                            second_coefficient = -second_coefficient;
                        }
                        output_real[target][bin] =
                            input_real[2U][bin] * second_coefficient
                            + input_real[1U][bin] * first_coefficient;
                        output_imaginary[target][bin] =
                            input_imaginary[2U][bin] * second_coefficient
                            + input_imaginary[1U][bin] * first_coefficient;
                    }
                    if (mode == 7U) {
                        const bool left = target == 3U || target == 6U;
                        // ParmaDec_ExtractMatrixedChansStereo negates a
                        // right target when the native pair is ordered
                        // left,right before applying its -pi/2 phase shift.
                        // The mode-7 coefficient already negates the second
                        // source; this additional target inversion is a
                        // separate operation in the native routine.
                        if (!left) {
                            for (std::size_t bin = 0U;
                                 bin < kBlockSize; ++bin) {
                                output_real[target][bin] =
                                    -output_real[target][bin];
                                output_imaginary[target][bin] =
                                    -output_imaginary[target][bin];
                            }
                        }
                        parma_phase_shift(
                            output_real[target].data(),
                            output_imaginary[target].data(),
                            left ? 1.57079637F : -1.57079637F);
                    }
                };
            if (row_10) {
                extract_stereo(0U, 6U, 0.0F, 0.5F, 0.5F, 1.0F);
            }
            extract_stereo(
                3U, 7U, 0.0F, kStereoRear0,
                kStereoRear0, kStereoRear1);
            extract_stereo(
                6U, 7U, kStereoRear0, kStereoRear1,
                kStereoRear1, kStereoRear2);
            extract_stereo(
                7U, 7U, kStereoRear1, kStereoRear2,
                kStereoRear2, kStereoRear3);
            extract_stereo(
                4U, 7U, kStereoRear2, kStereoRear3,
                kStereoRear3, 1.0F);

            const float direct_bound = row_10 ? 0.5F : 1.0F;
            for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                const float direct = 1.0F - diffuseness[bin];
                const float source_position = direct * position[bin];
                const float second_source_position =
                    direct * (1.0F - position[bin]);
                // ParmaDec_ComputeRepanCoeffsStereo selects a fixed 0.5
                // left/right blend for the mono-to-stereo entry path.
                const float repan_blend = mono_input
                    ? 0.5F
                    : 1.0F + direct * (balance[bin] - 1.0F);
                const float inverse_blend = 1.0F - repan_blend;
                const auto direct_position = [](float value, float bound) {
                    return value <= bound
                        ? 1.0F - value / bound + value
                        : value;
                };
                const auto matrixed_position = [](float value) {
                    constexpr float kBound = 0.241198406F;
                    return value <= kBound
                        ? value / kBound - value + 1.0F
                        : 2.0F - value;
                };
                const float first_angle =
                    (inverse_blend * matrixed_position(source_position)
                     + repan_blend
                         * direct_position(source_position, direct_bound))
                    * kParmaHalfPi;
                const float second_angle =
                    (inverse_blend * matrixed_position(second_source_position)
                     + repan_blend
                         * direct_position(
                             second_source_position, direct_bound))
                    * kParmaHalfPi;
                const float first_sine =
                    std::sin(first_angle) * stereo_repan[bin];
                const float first_cosine =
                    std::cos(first_angle) * stereo_repan[bin];
                const float second_sine =
                    std::sin(second_angle) * stereo_repan[bin];
                const float second_cosine =
                    std::cos(second_angle) * stereo_repan[bin];
                output_real[1U][bin] =
                    first_sine * output_real[1U][bin]
                    - input_real[2U][bin] * first_cosine;
                output_imaginary[1U][bin] =
                    first_sine * output_imaginary[1U][bin]
                    - input_imaginary[2U][bin] * first_cosine;
                output_real[2U][bin] =
                    second_sine * output_real[2U][bin]
                    - input_real[1U][bin] * second_cosine;
                output_imaginary[2U][bin] =
                    second_sine * output_imaginary[2U][bin]
                    - input_imaginary[1U][bin] * second_cosine;
            }
        } else if (row_12 || row_13 || row_16) {
            const ParmaBlindLayerConfig& first_layer =
                native_topology->layers[0U];
            const auto apply_first_layer_pair =
                [&](ParmaPairwiseAnalysis& analysis,
                    std::uint8_t first_source,
                    std::uint8_t second_source,
                    const std::array<std::uint8_t, 2U>& targets,
                    std::size_t target_count) {
                    std::array<float, kBlockSize> first_real =
                        input_real[first_source];
                    std::array<float, kBlockSize> first_imaginary =
                        input_imaginary[first_source];
                    std::array<float, kBlockSize> second_real =
                        input_real[second_source];
                    std::array<float, kBlockSize> second_imaginary =
                        input_imaginary[second_source];
                    analysis.process(
                        first_real.data(), first_imaginary.data(),
                        second_real.data(), second_imaginary.data(),
                        position.data(), balance.data(), diffuseness.data());
                    float previous_angle = 0.0F;
                    for (std::size_t target_index = 0U;
                         target_index < target_count;
                         ++target_index) {
                        const std::uint8_t target = targets[target_index];
                        const ParmaBlindChannelRule& rule =
                            first_layer.channels[target];
                        if (rule.mode != 1U
                            || rule.sources[0U] != first_source
                            || rule.sources[1U] != second_source) {
                            return false;
                        }
                        const float next_angle = target_index + 1U < target_count
                            ? first_layer.channels[targets[target_index + 1U]]
                                  .decoding_angle
                            : 1.0F;
                        parma_extract_matrixed_pairwise_channel(
                            position.data(), balance.data(), diffuseness.data(),
                            first_real.data(), first_imaginary.data(),
                            second_real.data(), second_imaginary.data(),
                            output_real[target].data(),
                            output_imaginary[target].data(),
                            previous_angle, rule.decoding_angle, next_angle,
                            kBlockSize);
                        previous_angle = rule.decoding_angle;
                    }
                    const float residual_angle =
                        first_layer.channels[targets[0U]].decoding_angle;
                    const float residual_inverse_angle =
                        1.0F
                        - first_layer.channels[
                              targets[target_count - 1U]].decoding_angle;
                    parma_repan_pairwise_channel(
                        position.data(), balance.data(), diffuseness.data(),
                        first_real.data(), first_imaginary.data(),
                        second_real.data(), second_imaginary.data(),
                        output_real[first_source].data(),
                        output_imaginary[first_source].data(),
                        output_real[second_source].data(),
                        output_imaginary[second_source].data(),
                        residual_angle, residual_inverse_angle, kBlockSize);
                    return true;
                };
            if (row_12
                && !apply_first_layer_pair(
                    front_analysis_, 1U, 2U, {{0U, 0U}}, 1U)) {
                return false;
            }
            if ((row_12 || row_13)
                && (!apply_first_layer_pair(
                        auxiliary_left_analysis_, 1U, 5U,
                        {{3U, 6U}}, 2U)
                    || !apply_first_layer_pair(
                        auxiliary_right_analysis_, 2U, 5U,
                        {{4U, 7U}}, 2U))) {
                return false;
            }
            if (row_16
                && (!apply_first_layer_pair(
                        auxiliary_left_analysis_, 3U, 5U,
                        {{6U, 6U}}, 1U)
                    || !apply_first_layer_pair(
                        auxiliary_right_analysis_, 4U, 5U,
                        {{7U, 7U}}, 1U))) {
                return false;
            }
        } else if (row_14) {
            // Row 14, layer 0: reconstruct the missing FC from the front
            // side pair before that layer is used by the height pairs.
            front_analysis_.process(
                input_real[1U].data(), input_imaginary[1U].data(),
                input_real[2U].data(), input_imaginary[2U].data(),
                position.data(), balance.data(), diffuseness.data());
            parma_extract_matrixed_pairwise_channel(
                position.data(), balance.data(), diffuseness.data(),
                input_real[1U].data(), input_imaginary[1U].data(),
                input_real[2U].data(), input_imaginary[2U].data(),
                output_real[0U].data(), output_imaginary[0U].data(),
                0.0F, front_center_angle, 1.0F, kBlockSize);
            parma_repan_pairwise_channel(
                position.data(), balance.data(), diffuseness.data(),
                input_real[1U].data(), input_imaginary[1U].data(),
                input_real[2U].data(), input_imaginary[2U].data(),
                output_real[1U].data(), output_imaginary[1U].data(),
                output_real[2U].data(), output_imaginary[2U].data(),
                front_center_angle, front_center_angle, kBlockSize);
        }
        if (has_side_rear_layer) {
            // Row 15, layer 0: 5.1 slots 3/4 are analyzed as the native
            // rear pair, expanded to 7.1 slots 6/7, then residually repanned
            // into 3/4 before the common elevation layer.
            rear_analysis_.process(
                input_real[3U].data(), input_imaginary[3U].data(),
                input_real[4U].data(), input_imaginary[4U].data(),
                position.data(), balance.data(), diffuseness.data());
            parma_extract_matrixed_pairwise_channel(
                position.data(), balance.data(), diffuseness.data(),
                input_real[3U].data(), input_imaginary[3U].data(),
                input_real[4U].data(), input_imaginary[4U].data(),
                output_real[6U].data(), output_imaginary[6U].data(),
                0.0F, rear_left_angle, rear_right_angle, kBlockSize);
            parma_extract_matrixed_pairwise_channel(
                position.data(), balance.data(), diffuseness.data(),
                input_real[3U].data(), input_imaginary[3U].data(),
                input_real[4U].data(), input_imaginary[4U].data(),
                output_real[7U].data(), output_imaginary[7U].data(),
                rear_left_angle, rear_right_angle, 1.0F, kBlockSize);
            parma_repan_pairwise_channel(
                position.data(), balance.data(), diffuseness.data(),
                input_real[3U].data(), input_imaginary[3U].data(),
                input_real[4U].data(), input_imaginary[4U].data(),
                output_real[3U].data(), output_imaginary[3U].data(),
                output_real[4U].data(), output_imaginary[4U].data(),
                rear_left_angle, rear_left_angle, kBlockSize);
        }
        if (row_12 || row_13 || row_16) {
            // DTS_ParmaDec_MixIntermediateChannels, first blind layer for
            // masks 0x26/0x27/0x3f.  The native SetBlind graph fans the
            // single rear-centre residual into BR and BL with -pi/2 phase
            // rotation and 1/sqrt(2) gain, then restores the graph's
            // uncorrelated target energy per critical band.
            constexpr float kIntermediateCoefficient = 0.70710677F;
            if (row_16) {
                // ParmaDec_HandleSpecialCasesIntermediate applies +pi/2
                // to slot 5 for the 0x3f -> 0xdf first layer.  The mixer
                // below applies -pi/2 to its copied source.
                parma_phase_shift(
                    output_real[5U].data(),
                    output_imaginary[5U].data(),
                    1.57079637F);
            }
            const auto mix_intermediate =
                [&](std::size_t state_index, std::uint8_t target) {
                    std::array<float, kBlockSize> reference_energy{};
                    std::array<float, kBlockSize> mixed_energy{};
                    for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                        const float target_real = output_real[target][bin];
                        const float target_imaginary =
                            output_imaginary[target][bin];
                        const float source_real = output_real[5U][bin];
                        const float source_imaginary =
                            output_imaginary[5U][bin];
                        reference_energy[bin] =
                            target_real * target_real
                            + target_imaginary * target_imaginary
                            + kIntermediateCoefficient
                                * kIntermediateCoefficient
                                * (source_real * source_real
                                   + source_imaginary * source_imaginary);
                        // ParmaDecIntermediate_FilterbankPhaseShift uses
                        // the stored float angle, not an exact component
                        // swap. Preserve the tiny cosf(-pi/2) term.
                        constexpr float kNegativeHalfPi = -1.57079637F;
                        const float shifted_real =
                            source_real * std::cos(kNegativeHalfPi)
                            - source_imaginary * std::sin(kNegativeHalfPi);
                        const float shifted_imaginary =
                            source_imaginary * std::cos(kNegativeHalfPi)
                            + source_real * std::sin(kNegativeHalfPi);
                        output_real[target][bin] =
                            target_real
                            + kIntermediateCoefficient * shifted_real;
                        output_imaginary[target][bin] =
                            target_imaginary
                            + kIntermediateCoefficient * shifted_imaginary;
                        mixed_energy[bin] =
                            output_real[target][bin]
                                * output_real[target][bin]
                            + output_imaginary[target][bin]
                                * output_imaginary[target][bin];
                    }
                    std::array<float, 16U> grouped_reference{};
                    std::array<float, 16U> grouped_mixed{};
                    parma_group_critical_bands(
                        reference_energy.data(), grouped_reference.data(),
                        widths.data(), widths.size());
                    parma_group_critical_bands(
                        mixed_energy.data(), grouped_mixed.data(),
                        widths.data(), widths.size());
                    std::array<float, kBlockSize> bin_gain{};
                    for (std::size_t band = 0U;
                         band < widths.size(); ++band) {
                        float target_gain = std::sqrt(
                            grouped_reference[band]
                            / (grouped_mixed[band] + 1.0e-12F));
                        target_gain = std::max(
                            0.31623F, std::min(1.0F, target_gain));
                        const float coefficient =
                            intermediate_gain_[state_index][band]
                                    >= target_gain
                                ? energy_release_
                                : energy_attack_;
                        intermediate_gain_[state_index][band] =
                            ((1.0F - coefficient)
                                 * intermediate_gain_[state_index][band]
                             + coefficient * target_gain
                             + kDenormalOffset)
                            - kDenormalOffset;
                    }
                    parma_ungroup_critical_bands(
                        intermediate_gain_[state_index].data(),
                        bin_gain.data(), widths.data(), widths.size());
                    for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                        output_real[target][bin] *= bin_gain[bin];
                        output_imaginary[target][bin] *= bin_gain[bin];
                    }
                };
            // Native graph order: target 7, then target 6.
            mix_intermediate(0U, 7U);
            mix_intermediate(1U, 6U);
        }
#ifdef DTSX_PARMA_TESTING
        if (frame_offset == 1024U
            && std::getenv("PARMA_TRACE_LAYER0") != nullptr) {
            for (std::uint8_t channel = 0U;
                 channel < kParmaChannelCount; ++channel) {
                float energy = 0.0F;
                for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                    energy += output_real[channel][bin]
                            * output_real[channel][bin]
                        + output_imaginary[channel][bin]
                            * output_imaginary[channel][bin];
                }
                if (energy <= 0.0F) continue;
                std::printf("candidate_layer0[%u]", channel);
                for (std::size_t bin = 0U; bin < 8U; ++bin) {
                    std::printf(" %.9g:%.9g",
                        output_real[channel][bin],
                        output_imaginary[channel][bin]);
                }
                std::printf(" energy=%.9g\n", energy);
            }
        }
#endif
        if (has_height_output) {
        // PairwiseCalc retains each layer's source subbands while it writes
        // the next layer.  For row 15, layer-one source slots 1/6 are also
        // residual destinations; passing aliased pointers would make the
        // second residual read the first residual already written in this
        // hop.  The native source and destination workspaces are distinct.
        std::array<float, kBlockSize> left_front_layer_real = has_first_layer
            ? output_real[1U] : input_real[1U];
        std::array<float, kBlockSize> left_front_layer_imaginary = has_first_layer
            ? output_imaginary[1U] : input_imaginary[1U];
        std::array<float, kBlockSize> left_rear_layer_real = has_first_layer
            ? output_real[6U] : input_real[6U];
        std::array<float, kBlockSize> left_rear_layer_imaginary = has_first_layer
            ? output_imaginary[6U] : input_imaginary[6U];
        const float* const left_front_real = left_front_layer_real.data();
        const float* const left_front_imaginary =
            left_front_layer_imaginary.data();
        const float* const left_rear_real = left_rear_layer_real.data();
        const float* const left_rear_imaginary =
            left_rear_layer_imaginary.data();
        left_analysis_.process(
            left_front_real, left_front_imaginary,
            left_rear_real, left_rear_imaginary,
            position.data(), balance.data(), diffuseness.data());
        parma_extract_matrixed_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            left_front_real, left_front_imaginary,
            left_rear_real, left_rear_imaginary,
            output_real[13U].data(),
            output_imaginary[13U].data(),
            0.0F, left_front_angle, left_rear_angle, kBlockSize);
        parma_extract_matrixed_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            left_front_real, left_front_imaginary,
            left_rear_real, left_rear_imaginary,
            output_real[16U].data(),
            output_imaginary[16U].data(),
            left_front_angle, left_rear_angle, 1.0F, kBlockSize);
        parma_repan_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            left_front_real, left_front_imaginary,
            left_rear_real, left_rear_imaginary,
            output_real[1U].data(),
            output_imaginary[1U].data(),
            output_real[6U].data(),
            output_imaginary[6U].data(),
            left_front_angle, left_front_angle, kBlockSize);

        std::array<float, kBlockSize> right_front_layer_real = has_first_layer
            ? output_real[2U] : input_real[2U];
        std::array<float, kBlockSize> right_front_layer_imaginary = has_first_layer
            ? output_imaginary[2U] : input_imaginary[2U];
        std::array<float, kBlockSize> right_rear_layer_real = has_first_layer
            ? output_real[7U] : input_real[7U];
        std::array<float, kBlockSize> right_rear_layer_imaginary = has_first_layer
            ? output_imaginary[7U] : input_imaginary[7U];
        const float* const right_front_real = right_front_layer_real.data();
        const float* const right_front_imaginary =
            right_front_layer_imaginary.data();
        const float* const right_rear_real = right_rear_layer_real.data();
        const float* const right_rear_imaginary =
            right_rear_layer_imaginary.data();
        right_analysis_.process(
            right_front_real, right_front_imaginary,
            right_rear_real, right_rear_imaginary,
            position.data(), balance.data(), diffuseness.data());
        parma_extract_matrixed_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            right_front_real, right_front_imaginary,
            right_rear_real, right_rear_imaginary,
            output_real[14U].data(),
            output_imaginary[14U].data(),
            0.0F, right_front_angle, right_rear_angle, kBlockSize);
        parma_extract_matrixed_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            right_front_real, right_front_imaginary,
            right_rear_real, right_rear_imaginary,
            output_real[17U].data(),
            output_imaginary[17U].data(),
            right_front_angle, right_rear_angle, 1.0F, kBlockSize);
        parma_repan_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            right_front_real, right_front_imaginary,
            right_rear_real, right_rear_imaginary,
            output_real[2U].data(),
            output_imaginary[2U].data(),
            output_real[7U].data(),
            output_imaginary[7U].data(),
            right_front_angle, right_front_angle, kBlockSize);
        }

#ifdef DTSX_PARMA_TESTING
        if (frame_offset == 1024U
            && std::getenv("PARMA_TRACE_LAYER1") != nullptr) {
            constexpr std::array<std::uint8_t, 8U> kTraceChannels = {
                1U, 2U, 6U, 7U, 13U, 14U, 16U, 17U};
            for (const std::uint8_t channel : kTraceChannels) {
                float energy = 0.0F;
                std::printf("candidate_layer1_subband[%u]", channel);
                for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                    energy += output_real[channel][bin]
                            * output_real[channel][bin]
                        + output_imaginary[channel][bin]
                            * output_imaginary[channel][bin];
                    if (bin < 8U) {
                        std::printf(" %.9g:%.9g",
                            output_real[channel][bin],
                            output_imaginary[channel][bin]);
                    }
                }
                std::printf(" energy=%.9g\n", energy);
            }
        }
#endif

        if (row_10 || row_11) {
            // ParmaDec_SoundfieldModificationsStereo with the native
            // default stereo phase control (zero).
#ifdef DTSX_PARMA_TESTING
            const auto trace_stereo_soundfield = [&](const char* stage) {
                if (frame_offset != 64U
                    || std::getenv("PARMA_TRACE_SOUNDFIELD_STEREO") == nullptr) {
                    return;
                }
                for (const std::uint8_t channel : {3U, 4U, 6U, 7U, 16U, 17U}) {
                    std::printf("candidate_soundfield_%s[%u]", stage, channel);
                    for (std::size_t bin = 0U; bin < 8U; ++bin)
                        std::printf(" %.9g:%.9g", output_real[channel][bin],
                            output_imaginary[channel][bin]);
                    std::putchar('\n');
                }
            };
            trace_stereo_soundfield("before");
#endif
            constexpr std::array<std::uint8_t, 3U> kRearLeft = {
                3U, 6U, 16U};
            constexpr std::array<std::uint8_t, 3U> kRearRight = {
                4U, 7U, 17U};
            for (const std::uint8_t channel : kRearLeft) {
                parma_phase_shift(
                    output_real[channel].data(),
                    output_imaginary[channel].data(),
                    kParmaMinusHalfPi);
            }
            for (const std::uint8_t channel : kRearRight) {
                parma_phase_shift(
                    output_real[channel].data(),
                    output_imaginary[channel].data(),
                    kParmaHalfPi);
            }
            for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                output_real[1U][bin] *= stereo_output_scale[bin];
                output_imaginary[1U][bin] *= stereo_output_scale[bin];
                output_real[2U][bin] *= stereo_output_scale[bin];
                output_imaginary[2U][bin] *= stereo_output_scale[bin];
            }
            constexpr std::array<std::uint8_t, 4U> kHeight = {
                13U, 14U, 16U, 17U};
            for (const std::uint8_t channel : kHeight) {
                for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                    output_real[channel][bin] *= kParmaSqrtHalf;
                    output_imaginary[channel][bin] *= kParmaSqrtHalf;
                }
            }
#ifdef DTSX_PARMA_TESTING
            trace_stereo_soundfield("after");
#endif
        }

        // DTS_ParmaDec_RepanSingleChansPairwise uses pair_layer[34..35]
        // as the two floor residual destinations.  The extracted channels
        // 13/16 (and 14/17 on the right) are inputs to the residual step,
        // not its destinations.  DTS_ParmaDec_UpdateMixChannel has no row-17
        // graph entries, so no additional height-to-floor mix is applied.

        std::array<float, kBlockSize> input_energy{};
        std::array<float, kBlockSize> output_energy{};
        for (std::size_t input_ordinal = 0U;
             input_ordinal < input_main_count;
             ++input_ordinal) {
            const std::uint8_t channel =
                input_main[input_ordinal];
            for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                input_energy[bin] +=
                    input_real[channel][bin]
                        * input_real[channel][bin]
                    + input_imaginary[channel][bin]
                        * input_imaginary[channel][bin];
            }
        }
        // DTS_ParmaDec_IdentifyPassthroughChans marks every source and
        // destination participating in a pairwise layer for OSFB synthesis.
        // All other enabled channels use the native 1024-sample direct path.
        for (std::size_t ordinal = 0U;
             ordinal < output_main_count; ++ordinal) {
            const std::uint8_t channel = kOutputMain[ordinal];
            if (!synthesized_channels[channel]) {
                continue;
            }
            for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                output_energy[bin] +=
                    output_real[channel][bin]
                        * output_real[channel][bin]
                    + output_imaginary[channel][bin]
                        * output_imaginary[channel][bin];
            }
        }
        std::array<float, 16U> grouped_input{};
        std::array<float, 16U> grouped_output{};
        parma_group_critical_bands(
            input_energy.data(), grouped_input.data(),
            widths.data(), widths.size());
        parma_group_critical_bands(
            output_energy.data(), grouped_output.data(),
            widths.data(), widths.size());
        for (std::size_t band = 0U; band < widths.size(); ++band) {
            float target = std::sqrt(
                grouped_input[band]
                / (grouped_output[band] + 1.0e-12F));
            target = std::max(0.31622776F, std::min(3.1622777F, target));
            const float coefficient =
                output_gain_[band] >= target
                ? energy_release_
                : energy_attack_;
            output_gain_[band] =
                ((1.0F - coefficient) * output_gain_[band]
                 + coefficient * target
                 + kDenormalOffset)
                - kDenormalOffset;
        }
#ifdef DTSX_PARMA_TESTING
        if (std::getenv("PARMA_TRACE_ENERGY") != nullptr) {
            std::printf("candidate_energy[%zu] %.9g %.9g %.9g\n",
                frame_offset / kBlockSize, grouped_input[0U],
                grouped_output[0U], output_gain_[0U]);
        }
#endif
        std::array<float, kBlockSize> bin_gain{};
        parma_ungroup_critical_bands(
            output_gain_.data(), bin_gain.data(),
            widths.data(), widths.size());
#ifdef DTSX_PARMA_TESTING
        if ((frame_offset == 64U || frame_offset == 1024U)
            && std::getenv("PARMA_TRACE_SUBBAND") != nullptr) {
            for (std::size_t ordinal = 0U;
                 ordinal < output_main_count; ++ordinal) {
                const std::uint8_t channel = kOutputMain[ordinal];
                float channel_energy = 0.0F;
                std::printf("candidate_subband[%u]", channel);
                for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                    const float traced_real =
                        output_real[channel][bin] * bin_gain[bin];
                    const float traced_imaginary =
                        output_imaginary[channel][bin] * bin_gain[bin];
                    channel_energy += traced_real * traced_real
                        + traced_imaginary * traced_imaginary;
                    std::printf(" %.9g:%.9g",
                        traced_real, traced_imaginary);
                }
                std::printf(" energy=%.9g\n", channel_energy);
            }
        }
#endif
        std::array<std::uint8_t, kOutputMain.size()>
            synthesized_order{};
        std::size_t synthesized_count = 0U;
        for (std::size_t ordinal = 0U;
             ordinal < output_main_count; ++ordinal) {
            const std::uint8_t channel = kOutputMain[ordinal];
            if (!synthesized_channels[channel]) {
                continue;
            }
            for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                output_real[channel][bin] *= bin_gain[bin];
                output_imaginary[channel][bin] *= bin_gain[bin];
            }
            synthesized_order[synthesized_count++] = channel;
        }
        std::size_t synthesized_ordinal = 0U;
        for (; synthesized_ordinal + 1U < synthesized_count;
             synthesized_ordinal += 2U) {
            const std::uint8_t first_channel =
                synthesized_order[synthesized_ordinal];
            const std::uint8_t second_channel =
                synthesized_order[synthesized_ordinal + 1U];
            synthesis_filters_[first_channel].process_x2(
                output_real[first_channel].data(),
                output_imaginary[first_channel].data(),
                synthesis_filters_[second_channel],
                output_real[second_channel].data(),
                output_imaginary[second_channel].data(),
                synthesized_first.data(),
                synthesized_second.data());
            const std::size_t first_output_index =
                static_cast<std::size_t>(
                    output_index_by_channel[first_channel]);
            const std::size_t second_output_index =
                static_cast<std::size_t>(
                    output_index_by_channel[second_channel]);
            for (std::size_t sample = 0U;
                 sample < kBlockSize; ++sample) {
                output[first_output_index][frame_offset + sample] =
                    int24_from_float(synthesized_first[sample]);
                output[second_output_index][frame_offset + sample] =
                    int24_from_float(synthesized_second[sample]);
            }
        }
        if (synthesized_ordinal < synthesized_count) {
            const std::uint8_t channel =
                synthesized_order[synthesized_ordinal];
            synthesis_filters_[channel].process(
                output_real[channel].data(),
                output_imaginary[channel].data(),
                synthesized_first.data());
            const std::size_t output_index = static_cast<std::size_t>(
                output_index_by_channel[channel]);
            for (std::size_t sample = 0U;
                 sample < kBlockSize; ++sample) {
                output[output_index][frame_offset + sample] =
                    int24_from_float(synthesized_first[sample]);
            }
        }
        for (std::size_t ordinal = 0U;
             ordinal < output_main_count; ++ordinal) {
            const std::uint8_t channel = kOutputMain[ordinal];
            if (synthesized_channels[channel]
                || input_index_by_channel[channel] < 0) {
                continue;
            }
            const std::size_t input_index = static_cast<std::size_t>(
                input_index_by_channel[channel]);
            const std::size_t output_index = static_cast<std::size_t>(
                output_index_by_channel[channel]);
            for (std::size_t sample = 0U;
                 sample < kBlockSize;
                 ++sample) {
                const std::size_t delay_position =
                    (direct_delay_position_ + sample)
                    % direct_delay_[channel].size();
                const float delayed = direct_delay_[channel][delay_position];
                direct_delay_[channel][delay_position] =
                    static_cast<float>(
                        input[input_index][frame_offset + sample])
                    / kInt24Scale;
                output[output_index][frame_offset + sample] =
                    int24_from_float(delayed);
            }
        }
        direct_delay_position_ =
            (direct_delay_position_ + kBlockSize)
            % direct_delay_[0U].size();

        if (lfe_input_index >= 0) {
            const std::size_t input_lfe =
                static_cast<std::size_t>(lfe_input_index);
            const std::size_t output_lfe =
                static_cast<std::size_t>(lfe_output_index);
            for (std::size_t sample = 0U; sample < kBlockSize; ++sample) {
                const float delayed = lfe_delay_[lfe_position_];
                lfe_delay_[lfe_position_] =
                    static_cast<float>(
                        input[input_lfe][frame_offset + sample])
                    / kInt24Scale;
                lfe_position_ =
                    (lfe_position_ + 1U) % lfe_delay_.size();
                output[output_lfe][frame_offset + sample] =
                    int24_from_float(delayed);
            }
        }
    }
    return true;
}

} // namespace dtsx_decode
