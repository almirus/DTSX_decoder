#include "render/object_audio_renderer.hpp"

#include "dtsx/object_waveform_map.hpp"
#include "dtsx/speaker_mask.hpp"
#include "render/object_gain.hpp"
#include "render/object_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

namespace dtsx_decode {
namespace {

std::int32_t wrapping_gain_add(
    std::int32_t left,
    std::int32_t right) noexcept {
    return static_cast<std::int32_t>(
        static_cast<std::uint32_t>(left)
        + static_cast<std::uint32_t>(right));
}

std::int32_t multiply_q15(
    std::int32_t left,
    std::int32_t right) noexcept {
    return static_cast<std::int32_t>(
        (static_cast<std::int64_t>(left) * right + 0x4000LL)
        >> 15U);
}

std::int32_t q23_to_native_renderer_q15(
    std::int32_t gain) noexcept {
    // libdtsx.so: dtsPlayerObjectRenderer_GetSizeof configures the
    // object renderer with 15 gain fractional bits (0x5e32c), while
    // dts_base_math_round_32f rounds halves away from zero.
    if (gain < 0) {
        return static_cast<std::int32_t>(
            -((-static_cast<std::int64_t>(gain) + 0x80LL)
              >> 8U));
    }
    return static_cast<std::int32_t>(
        (static_cast<std::int64_t>(gain) + 0x80LL)
        >> 8U);
}

std::uint32_t layout_channel_speaker_mask(
    const std::string& channel) noexcept {
    std::uint32_t speaker_mask = 0U;
    (void)dtsx::standard_speaker_mask(channel, speaker_mask);
    return speaker_mask;
}

bool equivalent_speaker_masks(
    std::uint32_t output,
    std::uint32_t metadata) noexcept {
    if (output == metadata) {
        return true;
    }
    return (output == (1U << 9U) && metadata == (1U << 3U))
        || (output == (1U << 10U) && metadata == (1U << 4U))
        || (output == (1U << 3U) && metadata == (1U << 9U))
        || (output == (1U << 4U) && metadata == (1U << 10U));
}

std::size_t metadata_speaker_index(
    std::uint32_t output_speaker,
    const std::vector<std::uint32_t>& metadata_speakers) noexcept {
    for (std::size_t index = 0U;
         index < metadata_speakers.size();
         ++index) {
        if (equivalent_speaker_masks(
                output_speaker, metadata_speakers[index])) {
            return index;
        }
    }
    return metadata_speakers.size();
}

std::size_t alternative_speaker_index(
    std::uint32_t output_speaker,
    const std::vector<std::uint32_t>& metadata_speakers) noexcept {
    // libdtsx.so: sub_60444, 0x6068c..0x608d4.
    // Mode 5 first searches for the exact physical speaker. Only output
    // LSS/RSS may fall back to coded SL/SR destinations; the reverse
    // substitution is not performed on this path.
    for (std::size_t index = 0U;
         index < metadata_speakers.size();
         ++index) {
        if (output_speaker == metadata_speakers[index]
            || (output_speaker == (1U << 9U)
                && metadata_speakers[index] == (1U << 3U))
            || (output_speaker == (1U << 10U)
                && metadata_speakers[index] == (1U << 4U))) {
            return index;
        }
    }
    return metadata_speakers.size();
}

std::uint8_t sparse_gain_code(
    const std::vector<dtsx::SixBitUpdate>& values,
    std::size_t index) noexcept {
    for (const dtsx::SixBitUpdate& value : values) {
        if (value.index == index) {
            return value.value;
        }
    }
    return 0U;
}

bool calculate_spatial_destination_gains(
    const dtsx::ObjectMetadataBlock& object,
    const LayoutPanner& panner,
    std::int32_t presentation_gain_q23,
    std::vector<std::vector<std::int32_t>>& waveform_gains) {
    if (waveform_gains.empty()) {
        return true;
    }
    const std::size_t destination_count =
        waveform_gains.front().size();
    for (const dtsx::PointSourceMetadata& point : object.points) {
        if (!dtsx::point_source_is_renderable(
                object.preamble.metadata_mode, point)) {
            continue;
        }
        if (point.waveform_index >= waveform_gains.size()) {
            return false;
        }
        const bool use_noncoherent_rendering =
            point.extended
            ? !object.spatial_header.flag_at_664
            : object.spatial_header.flag_at_660;
        std::vector<std::int32_t> panner_gains;
        if (!panner.gains_q15(
                point.coordinates,
                point.extended
                    ? static_cast<float>(point.width_degrees)
                    : 0.0F,
                point.extended
                    ? static_cast<float>(point.height_degrees)
                    : 0.0F,
                point.extended
                    ? static_cast<float>(point.rotation_degrees)
                    : 0.0F,
                point.snap_to_nearest_speaker,
                static_cast<float>(
                    object.spatial_header.snap_tolerance_degrees),
                object.spatial_header.flag_at_632,
                use_noncoherent_rendering,
                panner_gains)
            || panner_gains.size() != destination_count) {
            return false;
        }
        const std::uint8_t object_gain_code =
            object.spatial_header.gain_present
            ? object.spatial_header.gain_code
            : static_cast<std::uint8_t>(61U);
        const std::uint8_t object_gain_exponent =
            object.spatial_header.gain_present
            ? object.spatial_header.gain_exponent
            : static_cast<std::uint8_t>(0U);
        const std::int32_t source_gain =
            q23_to_native_renderer_q15(
                decode_object_source_gain_q23(
                    object_gain_code,
                    object_gain_exponent,
                    point.gain_code,
                    presentation_gain_q23));
        for (std::size_t destination = 0U;
             destination < destination_count;
             ++destination) {
            const std::int32_t contribution =
                multiply_q15(
                    panner_gains[destination],
                    source_gain);
            waveform_gains[point.waveform_index]
                          [destination] =
                wrapping_gain_add(
                    waveform_gains[point.waveform_index]
                                  [destination],
                    contribution);
        }
    }
    return true;
}

void calculate_explicit_destination_gains(
    const dtsx::ObjectMetadataBlock& object,
    const ChannelLayout& layout,
    const std::vector<std::uint32_t>& metadata_speakers,
    std::int32_t presentation_gain_q23,
    std::vector<std::vector<std::int32_t>>& waveform_gains) {
    const std::uint8_t mode = object.preamble.metadata_mode;
    for (std::size_t waveform = 0U;
         waveform < waveform_gains.size();
         ++waveform) {
        for (std::size_t destination = 0U;
             destination < layout.channels.size();
             ++destination) {
            const std::size_t metadata_index =
                metadata_speaker_index(
                    layout_channel_speaker_mask(
                        layout.channels[destination]),
                    metadata_speakers);
            if (metadata_index == metadata_speakers.size()) {
                continue;
            }
            if (mode == 1U || mode == 2U) {
                if (waveform >= object.updates.size()) {
                    continue;
                }
                const dtsx::WaveformMetadataUpdate& update =
                    object.updates[waveform];
                const std::int32_t coherent =
                    q23_to_native_renderer_q15(
                        decode_object_destination_gain_q23(
                            sparse_gain_code(
                                update.first_values,
                                metadata_index),
                            presentation_gain_q23));
                const std::int32_t noncoherent =
                    q23_to_native_renderer_q15(
                        decode_object_destination_gain_q23(
                            sparse_gain_code(
                                update.second_values,
                                metadata_index),
                            presentation_gain_q23));
                waveform_gains[waveform][destination] =
                    wrapping_gain_add(coherent, noncoherent);
            } else if (mode == 3U
                       && waveform
                           < object.mode_three.waveforms.size()) {
                const dtsx::ModeThreeWaveformValue& value =
                    object.mode_three.waveforms[waveform];
                if (metadata_index < value.values.size()) {
                    waveform_gains[waveform][destination] =
                        q23_to_native_renderer_q15(
                            decode_object_destination_gain_q23(
                                value.values[metadata_index],
                                presentation_gain_q23));
                }
            } else if (mode == 3U
                       && object.mode_three.fallback_gain_present
                       && object.mode_three.per_waveform_gain_present) {
                // The native mode-3 parser emits a fallback coefficient
                // when the destination update set is empty.  It applies to
                // every output destination that survived layout selection.
                waveform_gains[waveform][destination] =
                    q23_to_native_renderer_q15(
                        decode_object_destination_gain_q23(
                            object.mode_three.fallback_gain_code,
                            presentation_gain_q23));
            }
        }
    }
}

const dtsx::AlternativeRenderSet* find_alternative_render_set(
    const dtsx::ObjectMetadataBlock& object,
    std::uint32_t output_speaker_mask,
    std::uint32_t output_activity_mask) noexcept {
    // libdtsx.so: sub_5DFE4, 0x5dfe4.
    constexpr std::uint32_t kIgnoredActivityChannels = 0x1008U;
    constexpr std::uint32_t kFiveChannelActivityMask = 7U;
    constexpr std::uint32_t kFiveChannelFrontBackMask = 0x1FU;
    constexpr std::uint32_t kFiveChannelFrontSideMask = 0x607U;

    const std::uint32_t normalized_output =
        output_activity_mask & ~kIgnoredActivityChannels;

    for (const dtsx::AlternativeRenderSet& render_set :
         object.alternative_render_sets) {
        if ((render_set.speaker_activity_mask
             & ~kIgnoredActivityChannels)
            == normalized_output) {
            return &render_set;
        }
    }

    const bool has_front_back_five =
        (output_speaker_mask & kFiveChannelFrontBackMask)
        == kFiveChannelFrontBackMask;
    const bool has_front_side_five =
        (output_speaker_mask & kFiveChannelFrontSideMask)
        == kFiveChannelFrontSideMask;
    if (has_front_back_five || has_front_side_five) {
        const std::uint32_t without_lfe_and_top_back_center =
            output_speaker_mask & 0xFFFEFFDFU;
        const bool is_plain_five_channel_layout =
            without_lfe_and_top_back_center
            == (has_front_side_five
                    ? kFiveChannelFrontSideMask
                    : kFiveChannelFrontBackMask);
        for (const dtsx::AlternativeRenderSet& render_set :
             object.alternative_render_sets) {
            if ((render_set.speaker_activity_mask
                 & ~kIgnoredActivityChannels)
                    == kFiveChannelActivityMask
                && (is_plain_five_channel_layout
                    || render_set.subset_allowed)) {
                return &render_set;
            }
        }
    }

    for (const dtsx::AlternativeRenderSet& render_set :
         object.alternative_render_sets) {
        const std::uint32_t set_mask =
            render_set.speaker_activity_mask;
        const std::uint32_t normalized_set =
            set_mask & ~kIgnoredActivityChannels;
        if (normalized_set == normalized_output
            || (render_set.subset_allowed
                && set_mask != 0U
                && ((set_mask | output_activity_mask)
                    & ~kIgnoredActivityChannels)
                    == normalized_output)) {
            return &render_set;
        }
    }
    return nullptr;
}

const dtsx::AlternativeRenderSet* find_alternative_render_set(
    const dtsx::ObjectMetadataBlock& object,
    const ChannelLayout& layout) noexcept {
    std::uint32_t output_speaker_mask = 0U;
    for (const std::string& channel : layout.channels) {
        output_speaker_mask |= layout_channel_speaker_mask(channel);
    }
    return find_alternative_render_set(
        object,
        output_speaker_mask,
        dtsx::speaker_mask_to_activity_mask(output_speaker_mask));
}

bool calculate_alternative_destination_gains(
    const dtsx::AlternativeRenderSet& render_set,
    const std::vector<std::uint32_t>& destination_speakers,
    std::int32_t presentation_gain_q23,
    std::vector<std::vector<std::int32_t>>& waveform_gains) {
    const std::vector<std::uint32_t> set_speakers =
        dtsx::expand_speaker_activity_mask(
            render_set.speaker_activity_mask);
    if (render_set.waveform_gain_codes.size()
        != waveform_gains.size()) {
        return false;
    }
    for (std::size_t waveform = 0U;
         waveform < waveform_gains.size();
         ++waveform) {
        if (render_set.waveform_gain_codes[waveform].size()
            != set_speakers.size()) {
            return false;
        }
        for (std::size_t destination = 0U;
             destination < destination_speakers.size();
             ++destination) {
            const std::size_t set_index =
                alternative_speaker_index(
                    destination_speakers[destination],
                    set_speakers);
            if (set_index == set_speakers.size()) {
                continue;
            }
            waveform_gains[waveform][destination] =
                q23_to_native_renderer_q15(
                    decode_object_destination_gain_q23(
                        render_set.waveform_gain_codes[
                            waveform][set_index],
                        presentation_gain_q23));
        }
    }
    return true;
}

bool calculate_alternative_destination_gains(
    const dtsx::AlternativeRenderSet& render_set,
    const ChannelLayout& layout,
    std::int32_t presentation_gain_q23,
    std::vector<std::vector<std::int32_t>>& waveform_gains) {
    std::vector<std::uint32_t> destination_speakers;
    destination_speakers.reserve(layout.channels.size());
    for (const std::string& channel : layout.channels) {
        destination_speakers.push_back(
            layout_channel_speaker_mask(channel));
    }
    return calculate_alternative_destination_gains(
        render_set,
        destination_speakers,
        presentation_gain_q23,
        waveform_gains);
}

} // namespace

ObjectAudioRenderer::ObjectAudioRenderer(
    const ChannelLayout& layout,
    bool verbose)
    : layout_(layout), panner_(layout), verbose_(verbose) {}

bool ObjectAudioRenderer::render(
    const DecodedObjectAudioFrame& frame,
    std::vector<std::vector<std::int32_t>>& output) {
    output.assign(
        layout_.channels.size(),
        std::vector<std::int32_t>(
            frame.samples_per_channel, 0));
    // Player renderers are distinct native instances: mode 0 handles
    // spatial metadata, mode 1 handles channel/1-to-1 metadata, and mode 5
    // handles layout-specific alternative render sets.
    std::array<std::vector<WaveformRenderBlock>, 3> render_groups;
    std::array<std::vector<GainKey>, 3> render_key_groups;
    std::vector<GainKey> render_keys;
    const std::vector<std::uint32_t> metadata_speakers =
        dtsx::expand_speaker_activity_mask(
            frame.metadata_speaker_activity_mask);
    const std::int32_t presentation_gain_q23 =
        decode_object_presentation_gain_q23(
            frame.presentation_gain_code);
    const std::int32_t alternative_presentation_gain_q23 =
        frame.alternative_presentation_gain_present
        ? decode_object_alternative_presentation_gain_q23(
              frame.alternative_presentation_gain_code)
        : presentation_gain_q23;
    for (std::size_t object_index = 0;
         object_index < frame.objects.size();
         ++object_index) {
        const dtsx::ObjectMetadataBlock& object =
            frame.objects[object_index];
        const std::uint32_t object_id =
            object.object_id_available
            ? object.object_id
            : static_cast<std::uint32_t>(object_index);
        std::vector<std::uint32_t> waveform_channels;
        if (!dtsx::object_waveform_channel_indices(
                object,
                waveform_channels,
                &frame.waveform_base_by_id)) {
            // libdtsx.so only consumes/renders an object's metadata when
            // object+0x800 contains a mapped waveform decoder
            // (dtsParseExSSChunks 0xa1f48 and player update paths).
            continue;
        }
        std::vector<std::vector<std::int32_t>> waveform_gains(
            waveform_channels.size(),
            std::vector<std::int32_t>(layout_.channels.size(), 0));
        const std::uint8_t metadata_mode =
            object.preamble.metadata_mode;
        std::int32_t maximum_destination_gain = 0;
        std::uint32_t renderer_mode = 0U;
        std::size_t renderer_group = 0U;
        // visio-libdtsx.so.c:
        // dtsPlayerObjectRenderer_RenderObjects dispatches metadata modes
        // 0/1 through player renderer mode 0, metadata modes 2/3 through
        // player renderer mode 1, and a matching alternative render set
        // through player renderer mode 5. The three renderer instances own
        // independent gain state and block counters.
        if (metadata_mode > 1U) {
            renderer_mode = 1U;
            renderer_group = 1U;
            calculate_explicit_destination_gains(
                object,
                layout_,
                metadata_speakers,
                presentation_gain_q23,
                waveform_gains);
        } else {
          const dtsx::AlternativeRenderSet* alternative =
              find_alternative_render_set(object, layout_);
          if (alternative != nullptr) {
              // libdtsx.so: renderer mode 5 partitions spatial objects
              // with a matching coded destination layout away from mode 0.
              renderer_mode = 5U;
              renderer_group = 2U;
              if (!calculate_alternative_destination_gains(
                      *alternative,
                      layout_,
                      alternative_presentation_gain_q23,
                      waveform_gains)) {
                  return false;
              }
          } else {
            if (!calculate_spatial_destination_gains(
                    object,
                    panner_,
                    presentation_gain_q23,
                    waveform_gains)) {
                return false;
            }
          }
        }
        for (const auto& gains : waveform_gains) {
            for (const std::int32_t gain : gains) {
                maximum_destination_gain = (std::max)(
                    maximum_destination_gain,
                    static_cast<std::int32_t>(
                        gain < 0 ? -static_cast<std::int64_t>(gain)
                                 : gain));
            }
        }
        if (verbose_ && !reported_metadata_) {
            std::cerr
                << "Object renderer: id=" << object_id
                << " mode=" << static_cast<unsigned>(metadata_mode)
                << " waveforms=" << waveform_channels.size()
                << " waveformDecoderId="
                << static_cast<unsigned>(object.waveform_id)
                << " waveformBaseOffset="
                << object.waveform_channel_base_offset
                << " decodedWaveformChannels=";
            for (const std::uint32_t channel :
                 waveform_channels) {
                std::cerr << channel << ',';
            }
            std::cerr
                << " presentationGain="
                << static_cast<unsigned>(frame.presentation_gain_code)
                << " alternativePresentationGain="
                << (frame.alternative_presentation_gain_present
                        ? std::to_string(
                              frame.alternative_presentation_gain_code)
                        : std::string("normal"))
                << " objectGainPresent="
                << (object.spatial_header.gain_present ? 1 : 0)
                << " objectGainExponent="
                << static_cast<unsigned>(
                       object.spatial_header.gain_exponent)
                << " objectGainCode="
                << static_cast<unsigned>(
                       object.spatial_header.gain_code)
                << " preserveSpatialSeparation="
                << (object.spatial_header.flag_at_632 ? 1 : 0)
                << " useNoncoherentRendering="
                << (object.spatial_header.flag_at_660 ? 1 : 0)
                << " maxGainQ15=" << maximum_destination_gain
                << '\n';
        }
        if (verbose_
            && maximum_destination_gain > reported_maximum_gain_) {
            reported_maximum_gain_ = maximum_destination_gain;
            std::cerr
                << "Object renderer gain: id=" << object_id
                << " mode=" << static_cast<unsigned>(metadata_mode)
                << " maxGainQ15=" << maximum_destination_gain
                << " presentationGain="
                << static_cast<unsigned>(frame.presentation_gain_code)
                << " objectGainExponent="
                << static_cast<unsigned>(
                       object.spatial_header.gain_exponent)
                << " objectGainCode="
                << static_cast<unsigned>(
                       object.spatial_header.gain_code)
                << '\n';
        }
        bool waveform_channels_available = true;
        for (const std::uint32_t channel_index : waveform_channels) {
            if (channel_index >= frame.waveform_channels.size()
                || frame.waveform_channels[channel_index].size()
                    != frame.samples_per_channel) {
                waveform_channels_available = false;
                break;
            }
        }
        if (!waveform_channels_available) {
            continue;
        }

        for (std::size_t waveform = 0;
             waveform < waveform_channels.size();
             ++waveform) {
            const std::uint32_t channel_index =
                waveform_channels[waveform];
            WaveformRenderBlock block;
            block.samples =
                frame.waveform_channels[channel_index].data();
            block.sample_count = frame.samples_per_channel;
            block.destination_gains.resize(layout_.channels.size());
            for (std::size_t destination = 0;
                 destination < layout_.channels.size();
                 ++destination) {
                const GainKey key{
                    renderer_mode,
                    object_id,
                    static_cast<std::uint32_t>(waveform),
                    static_cast<std::uint32_t>(destination),
                };
                NativeGainRamp& state = gain_state_[key];
                const std::int32_t expected_base_gain =
                    renderer_mode == 0U ? 36 : 492;
                if (state.fractional_bits == 0U ||
                    state.base_gain != expected_base_gain) {
                    // libdtsx.so:
                    // dts_3d_complex_channel_renderer_t_initialize,
                    // 0xe6588. dtsPlayerObjectRenderer_RenderObjects
                    // uses Q15 smoothing factor 36 for renderer mode 0
                    // and 492 for renderer modes 1 and 5.
                    state.fractional_bits = 15U;
                    state.base_gain = expected_base_gain;
                    state.accumulator = 0;
                }
                block.destination_gains[destination].ramp = state;
                block.destination_gains[destination].destination_gain =
                    waveform_gains[waveform][destination];
                render_key_groups[renderer_group].push_back(key);
                render_keys.push_back(key);
            }
            render_groups[renderer_group].push_back(
                std::move(block));
        }
    }
    if (render_groups[0].empty()
        && render_groups[1].empty()
        && render_groups[2].empty()) {
        gain_state_.clear();
        return true;
    }
    reported_metadata_ = true;
    // DTSXDecFramePlayer renders the player instances in mode order 1, 5, 0.
    constexpr std::array<std::size_t, 3> kNativeForwardOrder{{1U, 2U, 0U}};
    for (const std::size_t group : kNativeForwardOrder) {
        if (render_groups[group].empty()) {
            continue;
        }
        // libdtsx.so: every native object-renderer instance owns
        // its own alternating snap-to-0-dB block counter.
        const bool snap_gain_to_zero_db =
            (rendered_block_counts_[group] & 1U) != 0U;
        ++rendered_block_counts_[group];
        if (!render_object_waveforms(
                render_groups[group],
                output,
                15U,
                GainApplyMode::Add,
                false,
                snap_gain_to_zero_db)) {
            return false;
        }
    }
    for (std::size_t group = 0U;
         group < render_groups.size();
         ++group) {
        std::size_t key_index = 0U;
        for (const WaveformRenderBlock& block :
             render_groups[group]) {
            for (const DestinationGainState& gain :
                 block.destination_gains) {
                gain_state_[render_key_groups[group][key_index++]] =
                    gain.ramp;
            }
        }
    }
    for (auto state = gain_state_.begin();
         state != gain_state_.end();) {
        if (std::find(
                render_keys.begin(),
                render_keys.end(),
                state->first)
            == render_keys.end()) {
            state = gain_state_.erase(state);
        } else {
            ++state;
        }
    }
    return true;
}

bool ObjectAudioRenderer::remove_embedded_object_fold_down(
    DecodedObjectAudioFrame& frame) {
    // libdtsx.so: DTSHD_UHDAssetDecoder_DecodeSubframe invokes renderer
    // modes 4, 3 and 2 before the player renderer.  Mode 4 selects spatial
    // objects whose alternative render set matches the decoded asset layout;
    // dtsPlayerObjectRenderer_ReverseRenderObjects/sub_60444 then renders the
    // coded alternative gains back into the same PCM buffers with subtraction.
    const std::vector<std::uint32_t> bed_speakers =
        dtsx::expand_speaker_activity_mask(
            frame.bed_speaker_activity_mask);
    if (frame.bed_channels.empty()
        || bed_speakers.size() != frame.bed_channels.size()) {
        return frame.objects.empty();
    }
    for (const auto& channel : frame.bed_channels) {
        if (channel.size() != frame.samples_per_channel) {
            return false;
        }
    }
    ChannelLayout bed_layout;
    bed_layout.name = "dtsx-native-bed";
    bed_layout.channels.reserve(bed_speakers.size());
    for (const std::uint32_t speaker : bed_speakers) {
        if (speaker == (1U << 5U)) {
            bed_layout.channels.emplace_back("LFE");
            continue;
        }
        float azimuth = 0.0F;
        float elevation = 0.0F;
        std::string_view name;
        if (!dtsx::standard_speaker_coordinates(
                speaker, azimuth, elevation, name)) {
            return false;
        }
        bed_layout.channels.emplace_back(name);
    }
    const LayoutPanner bed_panner(bed_layout);

    std::array<std::vector<WaveformRenderBlock>, 3> render_groups;
    std::array<std::vector<GainKey>, 3> render_key_groups;
    const std::int32_t presentation_gain_q23 =
        decode_object_presentation_gain_q23(
            frame.presentation_gain_code);
    const std::int32_t alternative_presentation_gain_q23 =
        frame.alternative_presentation_gain_present
        ? decode_object_alternative_presentation_gain_q23(
              frame.alternative_presentation_gain_code)
        : presentation_gain_q23;

    for (std::size_t object_index = 0U;
         object_index < frame.objects.size();
         ++object_index) {
        const dtsx::ObjectMetadataBlock& object =
            frame.objects[object_index];
        const std::uint8_t metadata_mode =
            object.preamble.metadata_mode;
        if (metadata_mode > 3U
            || !object.body_parsed) {
            if (verbose_ && !reported_reverse_metadata_) {
                std::cerr
                    << "Object reverse renderer: id="
                    << (object.object_id_available
                            ? object.object_id
                            : static_cast<std::uint32_t>(object_index))
                    << " mode="
                    << static_cast<unsigned>(
                           object.preamble.metadata_mode)
                    << " body=" << (object.body_parsed ? 1 : 0)
                    << " eligible=0\n";
            }
            continue;
        }
        std::vector<std::uint32_t> waveform_channels;
        if (!dtsx::object_waveform_channel_indices(
                object,
                waveform_channels,
                &frame.waveform_base_by_id)
            || waveform_channels.empty()) {
            continue;
        }
        std::uint32_t source_activity_mask = 0U;
        const std::uint32_t first_waveform =
            waveform_channels.front();
        if (first_waveform
            < frame.waveform_source_activity_masks.size()) {
            source_activity_mask =
                frame.waveform_source_activity_masks[first_waveform];
        }
        if (source_activity_mask == 0U) {
            source_activity_mask =
                frame.bed_speaker_activity_mask;
        }
        if (source_activity_mask == 0U) {
            source_activity_mask =
                frame.metadata_speaker_activity_mask;
        }
        const dtsx::AlternativeRenderSet* alternative =
            metadata_mode <= 1U
            ? find_alternative_render_set(
                  object,
                  0U,
                  source_activity_mask)
            : nullptr;
        if (verbose_ && !reported_reverse_metadata_) {
            std::cerr
                << "Object reverse renderer: id="
                << (object.object_id_available
                        ? object.object_id
                        : static_cast<std::uint32_t>(object_index))
                << " mode="
                << static_cast<unsigned>(
                       object.preamble.metadata_mode)
                << " body=" << (object.body_parsed ? 1 : 0)
                << " sourceActivityMask=0x"
                << std::hex << source_activity_mask
                << " bedActivityMask=0x"
                << frame.bed_speaker_activity_mask
                << " metadataActivityMask=0x"
                << frame.metadata_speaker_activity_mask
                << std::dec
                << " alternatives="
                << object.alternative_render_sets.size()
                << " matched=" << (alternative != nullptr ? 1 : 0)
                << " alternativeMasks=";
            for (const dtsx::AlternativeRenderSet& render_set :
                 object.alternative_render_sets) {
                std::cerr
                    << "0x" << std::hex
                    << render_set.speaker_activity_mask
                    << (render_set.subset_allowed ? '+' : '-')
                    << std::dec << ',';
            }
            std::cerr << '\n';
        }
        std::vector<std::vector<std::int32_t>> waveform_gains(
            waveform_channels.size(),
            std::vector<std::int32_t>(bed_speakers.size(), 0));
        std::uint32_t reverse_mode = 4U;
        std::int32_t smoothing_factor = 492;
        if (alternative != nullptr) {
            if (!calculate_alternative_destination_gains(
                    *alternative,
                    bed_speakers,
                    alternative_presentation_gain_q23,
                    waveform_gains)) {
                return false;
            }
        } else {
            // visio-libdtsx.so.c: ReverseRenderObjects routes metadata modes
            // 0/1 through renderer mode 2 (smoothing 36) and modes 2/3
            // through renderer mode 3 (smoothing 492). Renderer mode 4 above
            // handles layout-specific alternative rendering for modes 0/1.
            reverse_mode = metadata_mode <= 1U ? 2U : 3U;
            smoothing_factor = reverse_mode == 2U ? 36 : 492;
            if (metadata_mode >= 1U) {
                const std::uint32_t metadata_activity_mask =
                    frame.metadata_speaker_activity_mask != 0U
                    ? frame.metadata_speaker_activity_mask
                    : source_activity_mask;
                const std::vector<std::uint32_t> metadata_speakers =
                    dtsx::expand_speaker_activity_mask(
                        metadata_activity_mask);
                calculate_explicit_destination_gains(
                    object,
                    bed_layout,
                    metadata_speakers,
                    presentation_gain_q23,
                    waveform_gains);
            } else if (metadata_mode == 0U) {
                if (!calculate_spatial_destination_gains(
                        object,
                        bed_panner,
                        presentation_gain_q23,
                        waveform_gains)) {
                    return false;
                }
            }
        }

        const std::size_t reverse_group =
            static_cast<std::size_t>(reverse_mode - 2U);

        const std::uint32_t object_id =
            object.object_id_available
            ? object.object_id
            : static_cast<std::uint32_t>(object_index);
        for (std::size_t waveform = 0U;
             waveform < waveform_channels.size();
             ++waveform) {
            const std::uint32_t channel_index =
                waveform_channels[waveform];
            if (channel_index >= frame.waveform_channels.size()
                || frame.waveform_channels[channel_index].size()
                    != frame.samples_per_channel) {
                return false;
            }
            WaveformRenderBlock block;
            block.samples =
                frame.waveform_channels[channel_index].data();
            block.sample_count = frame.samples_per_channel;
            block.destination_gains.resize(bed_speakers.size());
            for (std::size_t destination = 0U;
                 destination < bed_speakers.size();
                 ++destination) {
                const GainKey key{
                    reverse_mode,
                    object_id,
                    static_cast<std::uint32_t>(waveform),
                    static_cast<std::uint32_t>(destination),
                };
                NativeGainRamp& state = reverse_gain_state_[key];
                if (state.fractional_bits == 0U
                    || state.base_gain != smoothing_factor) {
                    state.fractional_bits = 15U;
                    state.base_gain = smoothing_factor;
                    state.accumulator = 0;
                }
                block.destination_gains[destination].ramp =
                    state;
                block.destination_gains[destination]
                    .destination_gain =
                    waveform_gains[waveform][destination];
                render_key_groups[reverse_group].push_back(key);
            }
            render_groups[reverse_group].push_back(std::move(block));
        }
    }

    if (render_groups[0].empty()
        && render_groups[1].empty()
        && render_groups[2].empty()) {
        reported_reverse_metadata_ = true;
        reverse_gain_state_.clear();
        return true;
    }
    reported_reverse_metadata_ = true;
    // Native control invokes reverse modes 4, 3, then 2. Each renderer owns
    // an independent alternating snap-to-0-dB block counter.
    constexpr std::array<std::size_t, 3> kNativeOrder{{2U, 1U, 0U}};
    for (const std::size_t group : kNativeOrder) {
        if (render_groups[group].empty()) {
            continue;
        }
        const bool snap_gain_to_zero_db =
            (reverse_block_counts_[group] & 1U) != 0U;
        ++reverse_block_counts_[group];
        if (!render_object_waveforms(
                render_groups[group],
                frame.bed_channels,
                15U,
                GainApplyMode::Subtract,
                false,
                snap_gain_to_zero_db)) {
            return false;
        }
        std::size_t key_index = 0U;
        for (const WaveformRenderBlock& block : render_groups[group]) {
            for (const DestinationGainState& gain :
                 block.destination_gains) {
                reverse_gain_state_[
                    render_key_groups[group][key_index++]] = gain.ramp;
            }
        }
    }
    for (auto state = reverse_gain_state_.begin();
         state != reverse_gain_state_.end();) {
        const std::size_t mode =
            static_cast<std::size_t>(std::get<0>(state->first));
        if (mode < 2U
            || mode > 4U
            || std::find(
                   render_key_groups[mode - 2U].begin(),
                   render_key_groups[mode - 2U].end(),
                   state->first)
               == render_key_groups[mode - 2U].end()) {
            state = reverse_gain_state_.erase(state);
        } else {
            ++state;
        }
    }
    return true;
}

} // namespace dtsx_decode
