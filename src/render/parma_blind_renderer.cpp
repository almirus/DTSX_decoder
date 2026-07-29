#include "render/parma_blind_renderer.hpp"

#include "dtsx/speaker_mask.hpp"
#include "render/parma_critical_bands.hpp"
#include "render/parma_layout.hpp"
#include "render/parma_pairwise.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string_view>

namespace dtsx_decode {
namespace {

constexpr std::size_t kBlockSize = 64U;
constexpr std::size_t kParmaChannelCount = 28U;
constexpr float kInt24Scale = 8388608.0F;
constexpr float kDenormalOffset = 1.0e-18F;

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

} // namespace

bool ParmaBlindRenderer::initialize(
    std::uint32_t sample_rate) noexcept {
    if (!left_analysis_.initialize(sample_rate)
        || !right_analysis_.initialize(sample_rate)) {
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
    output_gain_.fill(1.0F);
    for (auto& delay : bed_delay_) {
        delay.fill(0);
    }
    bed_delay_position_ = 0U;
    lfe_delay_.fill(0.0F);
    lfe_position_ = 0U;
    energy_attack_ =
        smoothing_coefficient(sample_rate, 0.0063275F);
    energy_release_ =
        smoothing_coefficient(sample_rate, 0.012655F);
    sample_rate_ = sample_rate;
    initialized_ = true;
    return true;
}

bool ParmaBlindRenderer::render(
    const std::vector<std::vector<std::int32_t>>& input,
    std::uint32_t input_speaker_activity_mask,
    const ChannelLayout& output_layout,
    std::uint32_t sample_rate,
    std::vector<std::vector<std::int32_t>>& output) noexcept {
    // Constrained native blind path selected by SetBlind row 17:
    // 7.1 (0xDF main mask) -> 7.1.4 (0x360DF main mask), two layers.
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
        || controls.blind_table_row != 17
        || controls.input_main_channel_mask != 0xDFU
        || controls.output_main_channel_mask != 0x360DFU
        || parma_blind_layer_count(controls) != 2U) {
        return false;
    }
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
    constexpr std::array<std::uint8_t, 7U> kInputMain = {
        0U, 1U, 2U, 3U, 4U, 6U, 7U};
    for (std::uint8_t channel : kInputMain) {
        if (input_index_by_channel[channel] < 0) {
            return false;
        }
    }
    if (lfe_input_index < 0) {
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
    for (std::uint8_t channel : kOutputMain) {
        if (output_index_by_channel[channel] < 0) {
            return false;
        }
    }
    if (lfe_output_index < 0) {
        return false;
    }

    output.assign(
        output_layout.channels.size(),
        std::vector<std::int32_t>(frame_count, 0));
    std::array<std::array<float, kBlockSize>,
               kParmaChannelCount>
        input_blocks{};
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
    std::array<float, kBlockSize> synthesized{};
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
        for (std::uint8_t channel : kInputMain) {
            const std::size_t input_index =
                static_cast<std::size_t>(
                    input_index_by_channel[channel]);
            for (std::size_t sample = 0U;
                 sample < kBlockSize;
                 ++sample) {
                input_blocks[channel][sample] =
                    static_cast<float>(
                        input[input_index][frame_offset + sample])
                    / kInt24Scale;
            }
            analysis_filters_[channel].process(
                input_blocks[channel].data(),
                input_real[channel].data(),
                input_imaginary[channel].data());
        }
        for (auto& channel : output_real) {
            channel.fill(0.0F);
        }
        for (auto& channel : output_imaginary) {
            channel.fill(0.0F);
        }
        for (std::uint8_t channel : kInputMain) {
            output_real[channel] = input_real[channel];
            output_imaginary[channel] = input_imaginary[channel];
        }
        left_analysis_.process(
            input_real[1U].data(),
            input_imaginary[1U].data(),
            input_real[6U].data(),
            input_imaginary[6U].data(),
            position.data(), balance.data(), diffuseness.data());
        parma_extract_matrixed_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            input_real[1U].data(), input_imaginary[1U].data(),
            input_real[6U].data(), input_imaginary[6U].data(),
            output_real[13U].data(),
            output_imaginary[13U].data(),
            0.0F, 0.25F, 0.75F, kBlockSize);
        parma_extract_matrixed_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            input_real[1U].data(), input_imaginary[1U].data(),
            input_real[6U].data(), input_imaginary[6U].data(),
            output_real[16U].data(),
            output_imaginary[16U].data(),
            0.25F, 0.75F, 1.0F, kBlockSize);
        parma_repan_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            input_real[1U].data(), input_imaginary[1U].data(),
            input_real[6U].data(), input_imaginary[6U].data(),
            output_real[13U].data(),
            output_imaginary[13U].data(),
            output_real[16U].data(),
            output_imaginary[16U].data(),
            0.25F, 0.25F, kBlockSize);

        right_analysis_.process(
            input_real[2U].data(),
            input_imaginary[2U].data(),
            input_real[7U].data(),
            input_imaginary[7U].data(),
            position.data(), balance.data(), diffuseness.data());
        parma_extract_matrixed_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            input_real[2U].data(), input_imaginary[2U].data(),
            input_real[7U].data(), input_imaginary[7U].data(),
            output_real[14U].data(),
            output_imaginary[14U].data(),
            0.0F, 0.25F, 0.75F, kBlockSize);
        parma_extract_matrixed_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            input_real[2U].data(), input_imaginary[2U].data(),
            input_real[7U].data(), input_imaginary[7U].data(),
            output_real[17U].data(),
            output_imaginary[17U].data(),
            0.25F, 0.75F, 1.0F, kBlockSize);
        parma_repan_pairwise_channel(
            position.data(), balance.data(), diffuseness.data(),
            input_real[2U].data(), input_imaginary[2U].data(),
            input_real[7U].data(), input_imaginary[7U].data(),
            output_real[14U].data(),
            output_imaginary[14U].data(),
            output_real[17U].data(),
            output_imaginary[17U].data(),
            0.25F, 0.25F, kBlockSize);

        // DTS_ParmaDec_Process invokes PairwiseCalc before
        // MixIntermediateChannels. UpdateMixChannel groups each new
        // elevated intermediate with the same-azimuth floor output:
        // 1<-13, 2<-14, 6<-16 and 7<-17, all at unity gain.
        // MixIntermediateChannels rotates the intermediate by -pi/2,
        // accumulates it into the floor output, then normalizes that
        // floor channel to the pre-mix combined energy.
        constexpr float kNegativeHalfPi = -1.5707963705062866F;
        const auto mix_intermediate =
            [&](std::size_t state_index,
                std::uint8_t floor,
                std::uint8_t intermediate) {
                std::array<float, kBlockSize> desired_energy{};
                for (std::size_t bin = 0U;
                     bin < kBlockSize;
                     ++bin) {
                    desired_energy[bin] =
                        output_real[floor][bin]
                            * output_real[floor][bin]
                        + output_imaginary[floor][bin]
                            * output_imaginary[floor][bin]
                        + output_real[intermediate][bin]
                            * output_real[intermediate][bin]
                        + output_imaginary[intermediate][bin]
                            * output_imaginary[intermediate][bin];
                }
                std::array<float, kBlockSize> mixed_real =
                    output_real[intermediate];
                std::array<float, kBlockSize> mixed_imaginary =
                    output_imaginary[intermediate];
                parma_filterbank_phase_shift(
                    mixed_real.data(),
                    mixed_imaginary.data(),
                    kNegativeHalfPi,
                    kBlockSize);
                for (std::size_t bin = 0U;
                     bin < kBlockSize;
                     ++bin) {
                    output_real[floor][bin] += mixed_real[bin];
                    output_imaginary[floor][bin] +=
                        mixed_imaginary[bin];
                }
                std::array<float, kBlockSize> actual_energy{};
                for (std::size_t bin = 0U;
                     bin < kBlockSize;
                     ++bin) {
                    actual_energy[bin] =
                        output_real[floor][bin]
                            * output_real[floor][bin]
                        + output_imaginary[floor][bin]
                            * output_imaginary[floor][bin];
                }
                std::array<float, 16U> grouped_desired{};
                std::array<float, 16U> grouped_actual{};
                parma_group_critical_bands(
                    desired_energy.data(),
                    grouped_desired.data(),
                    widths.data(),
                    widths.size());
                parma_group_critical_bands(
                    actual_energy.data(),
                    grouped_actual.data(),
                    widths.data(),
                    widths.size());
                auto& smoothed_gain =
                    intermediate_gain_[state_index];
                for (std::size_t band = 0U;
                     band < widths.size();
                     ++band) {
                    float target_gain = std::sqrt(
                        grouped_desired[band]
                        / (grouped_actual[band] + 1.0e-12F));
                    target_gain = std::max(
                        0.31623F,
                        std::min(1.0F, target_gain));
                    const float coefficient =
                        smoothed_gain[band] >= target_gain
                        ? energy_release_
                        : energy_attack_;
                    smoothed_gain[band] =
                        ((1.0F - coefficient)
                             * smoothed_gain[band]
                         + coefficient * target_gain
                         + kDenormalOffset)
                        - kDenormalOffset;
                }
                std::array<float, kBlockSize> bin_gain{};
                parma_ungroup_critical_bands(
                    smoothed_gain.data(),
                    bin_gain.data(),
                    widths.data(),
                    widths.size());
                for (std::size_t bin = 0U;
                     bin < kBlockSize;
                     ++bin) {
                    output_real[floor][bin] *= bin_gain[bin];
                    output_imaginary[floor][bin] *=
                        bin_gain[bin];
                }
            };
        mix_intermediate(0U, 1U, 13U);
        mix_intermediate(1U, 2U, 14U);
        mix_intermediate(2U, 6U, 16U);
        mix_intermediate(3U, 7U, 17U);

        std::array<float, kBlockSize> input_energy{};
        std::array<float, kBlockSize> output_energy{};
        for (std::uint8_t channel : kInputMain) {
            for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                input_energy[bin] +=
                    input_real[channel][bin]
                        * input_real[channel][bin]
                    + input_imaginary[channel][bin]
                        * input_imaginary[channel][bin];
            }
        }
        for (std::uint8_t channel : kOutputMain) {
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
            target = std::max(0.31623F, std::min(3.1623F, target));
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
        std::array<float, kBlockSize> bin_gain{};
        parma_ungroup_critical_bands(
            output_gain_.data(), bin_gain.data(),
            widths.data(), widths.size());
        constexpr std::array<std::uint8_t, 4U> kHeightOutput = {
            13U, 14U, 16U, 17U};
        for (std::uint8_t channel : kHeightOutput) {
            for (std::size_t bin = 0U; bin < kBlockSize; ++bin) {
                output_real[channel][bin] *= bin_gain[bin];
                output_imaginary[channel][bin] *= bin_gain[bin];
            }
            synthesis_filters_[channel].process(
                output_real[channel].data(),
                output_imaginary[channel].data(),
                synthesized.data());
            const std::size_t output_index =
                static_cast<std::size_t>(
                    output_index_by_channel[channel]);
            for (std::size_t sample = 0U;
                 sample < kBlockSize;
                 ++sample) {
                output[output_index][frame_offset + sample] =
                    int24_from_float(synthesized[sample]);
            }
        }

        for (std::size_t sample = 0U;
             sample < kBlockSize;
             ++sample) {
            for (std::size_t channel_index = 0U;
                 channel_index < kInputMain.size();
                 ++channel_index) {
                const std::uint8_t channel =
                    kInputMain[channel_index];
                const std::size_t input_index =
                    static_cast<std::size_t>(
                        input_index_by_channel[channel]);
                const std::size_t output_index =
                    static_cast<std::size_t>(
                        output_index_by_channel[channel]);
                output[output_index][frame_offset + sample] =
                    bed_delay_[channel_index][
                        bed_delay_position_];
                bed_delay_[channel_index][bed_delay_position_] =
                    input[input_index][frame_offset + sample];
            }
            bed_delay_position_ =
                (bed_delay_position_ + 1U)
                % bed_delay_[0U].size();
        }

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
    return true;
}

} // namespace dtsx_decode
