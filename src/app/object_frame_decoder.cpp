#include "app/object_frame_decoder.hpp"

#include "bitstream/dtsx_word_buffer.hpp"
#include "dtsx/crc16.hpp"
#include "dtsx/exss_asset.hpp"
#include "dtsx/exss_header.hpp"
#include "dtsx/frame_sync.hpp"
#include "dtsx/metadata_chunk.hpp"
#include "dtsx/object_waveform_map.hpp"
#include "dtsx/preliminary_metadata.hpp"
#include "dtsx/speaker_mask.hpp"
#include "dtsx/xll_channel_set.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <utility>

namespace dtsx_decode {
namespace {

struct AssociatedWaveformRange final {
    std::uint8_t association_index = 0U;
    std::uint32_t base_channel = 0U;
    std::uint32_t channel_count = 0U;
    bool renderer_auxiliary_metadata_present = false;
};

std::vector<std::uint8_t> resolve_metadata_element_sizes(
    dtsx::bitstream::Cursor metadata_source,
    const std::vector<std::uint8_t>& descriptor_sizes,
    const std::vector<std::uint8_t>& associated_chunk_types) {
    if (descriptor_sizes.empty()
        || associated_chunk_types.empty()) {
        return descriptor_sizes;
    }
    const std::uint32_t available_bytes =
        metadata_source.remaining_bits() / 8U;
    const std::uint32_t overhead =
        2U * static_cast<std::uint32_t>(descriptor_sizes.size()) + 2U;
    std::uint32_t prefix_payload_size = 0U;
    for (std::size_t index = 0U;
         index + 1U < descriptor_sizes.size();
         ++index) {
        prefix_payload_size += descriptor_sizes[index];
    }

    std::vector<std::uint8_t> resolved = descriptor_sizes;
    std::size_t best_associated_chunks = 0U;
    for (std::uint32_t last_size = 0U;
         last_size <= 0xFFU;
         ++last_size) {
        const std::uint32_t region_size =
            prefix_payload_size + last_size + overhead;
        if (region_size > available_bytes) {
            continue;
        }
        dtsx::bitstream::Cursor crc_source = metadata_source;
        if (!dtsx::valid_crc16(crc_source, 8U * region_size)) {
            continue;
        }

        dtsx::bitstream::Cursor associated_source = metadata_source;
        associated_source.fast_forward(
            static_cast<std::int32_t>(8U * region_size));
        std::size_t parsed_chunks = 0U;
        for (const std::uint8_t chunk_type : associated_chunk_types) {
            if (chunk_type != 65U && chunk_type != 68U) {
                break;
            }
            if (associated_source.remaining_bits() < 40U) {
                break;
            }
            associated_source.fast_forward(8);
            const std::uint32_t sync =
                associated_source.lookahead_unsigned(32U);
            if (sync != dtsx::kXllSync
                && sync != dtsx::kXllSyncLegacy) {
                break;
            }
            dtsx::bitstream::Cursor common_source = associated_source;
            dtsx::XllCommonHeader common;
            if (!dtsx::unpack_xll_common_header(
                    common_source, common)
                || common.frame_size == 0U
                || 8U * common.frame_size
                       > associated_source.remaining_bits()) {
                break;
            }
            associated_source.fast_forward(
                static_cast<std::int32_t>(
                    8U * common.frame_size));
            ++parsed_chunks;
        }
        if (parsed_chunks > best_associated_chunks) {
            best_associated_chunks = parsed_chunks;
            resolved.back() = static_cast<std::uint8_t>(last_size);
        }
    }
    return best_associated_chunks == 0U
        ? descriptor_sizes
        : resolved;
}

void merge_sparse_gain_set(
    bool update_present,
    std::uint32_t update_mask,
    const std::vector<dtsx::SixBitUpdate>& update_values,
    bool& state_present,
    std::uint32_t& state_mask,
    std::vector<dtsx::SixBitUpdate>& state_values) {
    if (!update_present) {
        return;
    }
    state_present = true;
    state_mask = update_mask;
    for (const dtsx::SixBitUpdate& replacement : update_values) {
        bool replaced = false;
        for (dtsx::SixBitUpdate& value : state_values) {
            if (value.index == replacement.index) {
                value.value = replacement.value;
                replaced = true;
                break;
            }
        }
        if (!replaced) {
            state_values.push_back(replacement);
        }
    }
}

void merge_waveform_gain_updates(
    const std::vector<dtsx::WaveformMetadataUpdate>& updates,
    std::vector<dtsx::WaveformMetadataUpdate>& state) {
    if (state.size() < updates.size()) {
        state.resize(updates.size());
    }
    for (std::size_t waveform = 0U;
         waveform < updates.size();
         ++waveform) {
        const dtsx::WaveformMetadataUpdate& update = updates[waveform];
        dtsx::WaveformMetadataUpdate& persistent = state[waveform];
        merge_sparse_gain_set(
            update.first_set_present,
            update.first_mask,
            update.first_values,
            persistent.first_set_present,
            persistent.first_mask,
            persistent.first_values);
        merge_sparse_gain_set(
            update.second_set_present,
            update.second_mask,
            update.second_values,
            persistent.second_set_present,
            persistent.second_mask,
            persistent.second_values);
    }
}

void merge_object_update(
    dtsx::ObjectMetadataBlock update,
    dtsx::ObjectMetadataBlock& state) {
    const std::uint8_t mode =
        update.preamble.metadata_mode;
    if (mode == 0U || !state.metadata_present) {
        state = std::move(update);
        return;
    }
    if (mode == 1U) {
        std::vector<dtsx::WaveformMetadataUpdate>
            sparse_updates = std::move(update.updates);
        std::vector<dtsx::WaveformMetadataUpdate>
            persistent_updates;
        if (state.preamble.metadata_mode == 1U
            || state.preamble.metadata_mode == 2U) {
            persistent_updates = std::move(state.updates);
        }
        state = std::move(update);
        state.updates = std::move(persistent_updates);
        merge_waveform_gain_updates(
            sparse_updates, state.updates);
        return;
    }

    // Mode 2/3 metadata blocks are sparse.  The native object state keeps
    // identifiers, waveform mapping and grouping fields when their presence
    // bits are clear; replacing those fields with zero here breaks dynamic
    // objects on the first gain-only update.
    if (update.object_id_available) {
        state.object_id_available = true;
        state.object_id = update.object_id;
    }
    if (update.waveform_id_available) {
        state.waveform_channel_base_offset =
            update.waveform_channel_base_offset;
        state.waveform_id_available = true;
        state.waveform_id = update.waveform_id;
        state.waveform_channel_offsets =
            std::move(update.waveform_channel_offsets);
    }
    if (update.group_assignment_present) {
        state.group_assignment_present = true;
        state.group_assignment = update.group_assignment;
        state.group_assignment_coherent =
            update.group_assignment_coherent;
    }
    if (update.spatial_group_present) {
        state.spatial_group_present = true;
        state.spatial_group = update.spatial_group;
    }
    state.inter_object_metadata_present =
        update.inter_object_metadata_present;
    state.flag_at_580 = update.flag_at_580;
    const std::uint8_t previous_mode =
        state.preamble.metadata_mode;
    state.preamble.metadata_mode = mode;
    state.preamble.waveform_count =
        update.preamble.waveform_count;
    state.preamble.waveform_types =
        std::move(update.preamble.waveform_types);
    if (mode == 1U || mode == 2U) {
        if (previous_mode != 1U && previous_mode != 2U) {
            state.updates.clear();
        }
        merge_waveform_gain_updates(update.updates, state.updates);
    } else {
        state.updates = std::move(update.updates);
    }
    state.mode_three = std::move(update.mode_three);
    if (update.alternative_rendering_present) {
        state.alternative_rendering_present = true;
        state.alternative_speaker_mask_bits =
            update.alternative_speaker_mask_bits;
        state.alternative_render_sets =
            std::move(update.alternative_render_sets);
    }
    state.body_parsed = update.body_parsed;
}

bool is_core_stream_packing(dtsx::StreamPacking packing) noexcept {
    return packing == dtsx::StreamPacking::Core16BitBigEndian
        || packing == dtsx::StreamPacking::Core16BitLittleEndian
        || packing == dtsx::StreamPacking::Core14BitBigEndian
        || packing == dtsx::StreamPacking::Core14BitLittleEndian;
}

bool core_swap_byte_pairs(dtsx::StreamPacking packing) noexcept {
    return packing == dtsx::StreamPacking::Core16BitLittleEndian
        || packing == dtsx::StreamPacking::Core14BitLittleEndian;
}

} // namespace

void ObjectFrameDecoder::remember_core_metadata(
    const dtsx::ElementaryFrame& frame) {
    pending_core_bytes_.clear();
    pending_core_valid_ = false;
    if (!is_core_stream_packing(frame.packing) || frame.bytes.empty()) {
        return;
    }
    pending_core_bytes_ = frame.bytes;
    pending_core_packing_ = frame.packing;
    pending_core_valid_ = true;
}

ObjectFrameDecodeResult ObjectFrameDecoder::decode(
    const dtsx::ElementaryFrame& elementary,
    DecodedObjectAudioFrame& decoded,
    const std::vector<dtsx::XllLossyBaseChannel>&
        lossy_base_channels,
    const std::vector<DcaDecodedObjectAsset>&
        lossy_object_assets) {
    decoded = {};
    last_error_.clear();
    if (elementary.packing != dtsx::StreamPacking::ExtensionBigEndian
        && elementary.packing
            != dtsx::StreamPacking::ExtensionLittleEndian) {
        return ObjectFrameDecodeResult::Ignored;
    }

    const bool swap =
        elementary.packing
        == dtsx::StreamPacking::ExtensionBigEndian;
    dtsx::bitstream::WordBuffer words(elementary.bytes, swap);
    dtsx::bitstream::Cursor exss_source = words.cursor();
    dtsx::ExssHeader exss;
    if (!dtsx::validate_exss_frame(words.cursor())
        || !dtsx::unpack_exss_header(exss_source, exss)) {
        return ObjectFrameDecodeResult::Malformed;
    }
    // Native decoder state is keyed by the three-bit asset index carried in
    // each asset header, not by the ordinal in the current ExSS asset list.
    // Dynamic presentations may reorder or omit asset ordinals while keeping
    // the waveform decoder identity stable across frames.
    if (xll_decoders_.size() < 256U) {
        xll_decoders_.resize(256U);
        uhd_xll_decoders_.resize(256U);
        uhd_type65_xll_decoders_.resize(256U);
        xll_pbr_buffers_.resize(256U);
        combined_mix_metadata_state_.resize(256U);
        combined_mix_metadata_valid_.resize(256U, false);
    }
    std::vector<std::pair<std::uint8_t, std::uint32_t>>
        decoded_asset_bases;
    std::vector<std::pair<std::uint8_t, std::uint32_t>>
        decoded_uhd_waveform_bases;
    std::vector<AssociatedWaveformRange>
        decoded_uhd_waveform_ranges;
    std::vector<dtsx::MetadataChunkLocation> chunks;

    // DTS_ObjectDecoder has two independent compressed-audio backends:
    // 0x400 (XLL) and 0x800 (lossy XXCH). DTSHD_UHDAssetDecoder iterates the
    // same persistent object-decoder array for both. Register the separately
    // decoded XXCH assets before parsing type-241 metadata so waveform IDs
    // resolve identically to XLL object assets. Legacy LBR is not an
    // ObjectDecoder waveform backend. Channel-based HRA beds
    // (object_audio_type == 0) stay in DcaBedDecoder and must not be copied
    // into waveform_channels: that would reverse-render the 8ch bed and
    // break the dtshdHRA-DTSX-001 PCM hash.
    for (const DcaDecodedObjectAsset& asset : lossy_object_assets) {
        if (asset.channels.empty()) {
            continue;
        }
        if (decoded.sample_rate != 0U
            && decoded.sample_rate != asset.sample_rate) {
            last_error_ = "lossy object asset sample rate";
            return ObjectFrameDecodeResult::Malformed;
        }
        if (decoded.samples_per_channel != 0U
            && decoded.samples_per_channel
                != asset.samples_per_channel) {
            last_error_ = "lossy object asset frame duration";
            return ObjectFrameDecodeResult::Malformed;
        }
        decoded.sample_rate = asset.sample_rate;
        decoded.samples_per_channel = asset.samples_per_channel;
        decoded_asset_bases.emplace_back(
            asset.asset_index,
            static_cast<std::uint32_t>(
                decoded.waveform_channels.size()));
        for (const auto& channel : asset.channels) {
            decoded.waveform_channels.push_back(channel);
            decoded.waveform_speaker_masks.push_back(0U);
            decoded.waveform_source_activity_masks.push_back(0U);
            decoded.waveform_is_supplemental.push_back(false);
        }
    }

    for (std::uint32_t asset_index = 0;
         asset_index < exss.asset_count
         && asset_index < exss.asset_header_bit_offsets.size();
         ++asset_index) {
        dtsx::bitstream::Cursor asset_header_source = words.cursor();
        asset_header_source.fast_forward(static_cast<std::int32_t>(
            exss.asset_header_bit_offsets[asset_index]));
        dtsx::ExssAssetSummary asset;
        if (!dtsx::unpack_exss_asset_summary(
                asset_header_source,
                exss,
                asset,
                asset_index)) {
            return ObjectFrameDecodeResult::Malformed;
        }
        decoded.imax_enhanced =
            decoded.imax_enhanced
            || asset.type1_certified_content;
        if ((asset.coding_components & (1U << 9U)) == 0U) {
            continue;
        }

        std::uint32_t asset_payload_offset = exss.header_size;
        for (std::uint32_t previous = 0;
             previous < asset_index;
             ++previous) {
            asset_payload_offset += exss.asset_sizes[previous];
        }
        dtsx::bitstream::Cursor xll_source = words.cursor();
        xll_source.fast_forward(static_cast<std::int32_t>(
            8U * (asset_payload_offset
                + asset.component_byte_offsets[9U])));
        xll_source = xll_source.limited(
            8U * asset.component_size_bytes[9U]);
        std::vector<std::uint8_t> xll_packet(
            asset.component_size_bytes[9U]);
        dtsx::bitstream::Cursor xll_packet_source = xll_source;
        for (std::uint8_t& byte : xll_packet) {
            byte = static_cast<std::uint8_t>(
                xll_packet_source.extract_unsigned(8U));
        }
        if (!xll_packet_source.valid()) {
            last_error_ = "XLL component bounds";
            return ObjectFrameDecodeResult::Malformed;
        }
        std::vector<std::uint8_t>& xll_pbr =
            xll_pbr_buffers_[asset.asset_index];
        const std::size_t previous_pbr_size = xll_pbr.size();
        const std::size_t maximum_pbr_size =
            asset.xll_smoothing_buffer_kbytes != 0U
            ? static_cast<std::size_t>(
                  asset.xll_smoothing_buffer_kbytes)
                  << 10U
            : 256U << 10U;
        if (xll_packet.size()
            > maximum_pbr_size - (std::min)(
                  maximum_pbr_size, xll_pbr.size())) {
            xll_pbr.clear();
            last_error_ = "XLL PBR smoothing buffer overflow";
            return ObjectFrameDecodeResult::Malformed;
        }
        xll_pbr.insert(
            xll_pbr.end(),
            xll_packet.begin(),
            xll_packet.end());
        std::shared_ptr<dtsx::bitstream::WordBuffer>
            assembled_xll_words;
        std::uint32_t metadata_component_offset =
            asset.xll_metadata_offset;
        bool metadata_in_assembled_xll = false;
        std::uint32_t associated_audio_offset = 0U;
        bool associated_audio_offset_valid = false;
        if (previous_pbr_size != 0U
            || asset.xll_sync_offset != 0U) {
            assembled_xll_words =
                std::make_shared<dtsx::bitstream::WordBuffer>(
                    xll_pbr, false);
            dtsx::bitstream::Cursor common_source =
                assembled_xll_words->cursor();
            dtsx::XllCommonHeader assembled_common;
            if (dtsx::unpack_xll_common_header(
                    common_source, assembled_common)
                && assembled_common.frame_size <= xll_pbr.size()
                && asset.xll_metadata_offset
                    < assembled_common.frame_size) {
                metadata_component_offset =
                    asset.xll_metadata_offset;
                metadata_in_assembled_xll = true;
            }
        }
        std::vector<dtsx::XllSupplementalChannelSet>
            supplemental_channel_sets;
        dtsx::CombinedMixMetadata combined_mix;
        bool combined_mix_available = false;
        if (asset.xll_metadata_present
            && asset.xll_object_metadata_present
            && !asset.xll_metadata_chunk_sizes.empty()
            && !asset.xll_associated_chunk_types.empty()) {
            std::uint32_t raw_metadata_bytes = 2U;
            for (const std::uint8_t payload_size :
                 asset.xll_metadata_chunk_sizes) {
                raw_metadata_bytes +=
                    static_cast<std::uint32_t>(payload_size) + 2U;
            }
            std::uint32_t associated_offset =
                metadata_component_offset + raw_metadata_bytes;
            dtsx::bitstream::Cursor combined_source =
                metadata_in_assembled_xll
                ? assembled_xll_words->cursor()
                : words.cursor();
            if (metadata_in_assembled_xll) {
                combined_source.fast_forward(
                    static_cast<std::int32_t>(
                        8U * metadata_component_offset));
                combined_source = combined_source.limited(
                    static_cast<std::uint32_t>(
                        8U * (xll_pbr.size()
                              - metadata_component_offset)));
            } else {
                combined_source.fast_forward(
                    static_cast<std::int32_t>(
                        8U
                        * (asset_payload_offset
                           + asset.component_byte_offsets[9U]
                           + asset.xll_metadata_offset)));
                combined_source = combined_source.limited(
                    8U
                    * (asset.component_size_bytes[9U]
                       - asset.xll_metadata_offset));
            }
            combined_mix_available =
                dtsx::unpack_combined_mix_metadata(
                    combined_source,
                    static_cast<std::uint8_t>(
                        exss.asset_count),
                    asset.speaker_activity_mask,
                    combined_mix);
            if (combined_mix_available) {
                combined_mix_metadata_state_[
                    asset.asset_index] = combined_mix;
                combined_mix_metadata_valid_[
                    asset.asset_index] = true;
            } else if (combined_mix_metadata_valid_[
                           asset.asset_index]) {
                combined_mix =
                    combined_mix_metadata_state_[
                        asset.asset_index];
                combined_mix_available = true;
            }
            if (combined_mix_available) {
                // This is the parsed native combined-mix matrix, not an
                // inferred bed fold-down. Preserve it for the guided PARMA
                // control path even when this frame also carries type-69
                // supplemental XLL audio.
                decoded.parma_guided_metadata = combined_mix;
                decoded.supplemental_speaker_activity_mask |=
                    combined_mix.added_speaker_activity_mask;
            }
            for (std::size_t associated_index = 0U;
                 associated_index
                     < asset.xll_associated_chunk_types.size();
                 ++associated_index) {
                if (asset.xll_associated_chunk_types[
                        associated_index]
                    == 69U) {
                    const std::size_t metadata_buffer_size =
                        metadata_in_assembled_xll
                        ? xll_pbr.size()
                        : asset.component_size_bytes[9U];
                    const std::uint32_t associated_chunk_length =
                        associated_index
                                < asset.xll_associated_chunk_extents.size()
                            ? asset.xll_associated_chunk_extents[
                                  associated_index]
                            : 0U;
                    std::uint32_t channel_set_offset = 0U;
                    bool channel_set_valid = false;
                    if (associated_chunk_length != 0U
                        && associated_offset < metadata_buffer_size) {
                        const std::uint32_t chunk_end =
                            static_cast<std::uint32_t>((std::min)(
                                metadata_buffer_size,
                                static_cast<std::size_t>(associated_offset)
                                    + associated_chunk_length));
                        for (std::uint32_t candidate = associated_offset;
                             candidate < chunk_end;
                             ++candidate) {
                            dtsx::bitstream::Cursor probe_source =
                                metadata_in_assembled_xll
                                ? assembled_xll_words->cursor()
                                : words.cursor();
                            const std::uint32_t absolute_candidate =
                                metadata_in_assembled_xll
                                ? candidate
                                : asset_payload_offset
                                      + asset.component_byte_offsets[9U]
                                      + candidate;
                            probe_source.fast_forward(
                                static_cast<std::int32_t>(
                                    8U * absolute_candidate));
                            probe_source = probe_source.limited(
                                8U * (chunk_end - candidate));
                            dtsx::XllChannelSetProbe probe;
                            if (dtsx::probe_xll_channel_set_header(
                                    probe_source, false, probe)
                                && probe.channel_count
                                       == combined_mix
                                              .added_speaker_masks
                                              .size()
                                && probe.sample_rate == asset.sample_rate
                                && probe.bit_depth != 0U
                                && probe.bit_depth
                                       <= probe.storage_bit_depth
                                && probe.storage_bit_depth == 24U
                                && probe.frequency_ratio == 1U) {
                                channel_set_offset = candidate;
                                channel_set_valid = true;
                                break;
                            }
                        }
                    }
                    if (channel_set_valid) {
                        dtsx::bitstream::Cursor
                            supplemental_source =
                                metadata_in_assembled_xll
                                ? assembled_xll_words->cursor()
                                : words.cursor();
                        const std::uint32_t absolute_offset =
                            metadata_in_assembled_xll
                            ? channel_set_offset
                            : asset_payload_offset
                                  + asset.component_byte_offsets[9U]
                                  + channel_set_offset;
                        supplemental_source.fast_forward(
                            static_cast<std::int32_t>(
                                8U * absolute_offset));
                        supplemental_source =
                            supplemental_source.limited(
                                static_cast<std::uint32_t>(
                                    8U
                                    * (metadata_buffer_size
                                       - channel_set_offset)));
                        if (!combined_mix_available) {
                            last_error_ =
                                "combined type-2 XLL metadata";
                            return ObjectFrameDecodeResult::Malformed;
                        }
                        dtsx::XllSupplementalChannelSet
                            supplemental{supplemental_source};
                        supplemental.speaker_activity_mask =
                            combined_mix
                                .added_speaker_activity_mask;
                        supplemental.reference_speaker_masks =
                            combined_mix.reference_speaker_masks;
                        supplemental
                            .lossy_reference_speaker_masks =
                                combined_mix
                                    .lossy_reference_speaker_masks;
                        supplemental.downmix_coefficients.reserve(
                            combined_mix
                                .downmix_coefficient_codes.size());
                        for (const std::uint8_t code :
                             combined_mix
                                 .downmix_coefficient_codes) {
                            supplemental.downmix_coefficients
                                .push_back(
                                    dtsx::
                                        xll_metadata_downmix_coefficient(
                                            code));
                        }
                        supplemental_channel_sets.push_back(
                            std::move(supplemental));
                    }
                }
                if (associated_index
                    < asset.xll_associated_chunk_extents.size()) {
                    associated_offset +=
                        asset.xll_associated_chunk_extents[
                            associated_index];
                }
            }
        }
        dtsx::XllDecodedFrame xll;
        auto decode_pbr =
            [&]() {
                dtsx::bitstream::WordBuffer pbr_words(
                    xll_pbr, false);
                return xll_decoders_[asset.asset_index]
                    .decode_msb_frame(
                        pbr_words.cursor(),
                        xll,
                        asset.one_to_one_mapping,
                        supplemental_channel_sets,
                        lossy_base_channels);
            };
        bool xll_decoded =
            asset.asset_index < xll_decoders_.size()
            && decode_pbr();
        if (!xll_decoded
            && previous_pbr_size == 0U
            && asset.xll_sync_present
            && asset.xll_sync_offset < xll_packet.size()) {
            xll_pbr.erase(
                xll_pbr.begin(),
                xll_pbr.begin()
                    + static_cast<std::ptrdiff_t>(
                        asset.xll_sync_offset));
            xll_decoded = decode_pbr();
        }
        if (!xll_decoded) {
            last_error_ = asset.asset_index
                    < xll_decoders_.size()
                ? xll_decoders_[asset.asset_index].last_error()
                : "XLL decoder index";
            return ObjectFrameDecodeResult::Malformed;
        }
        if (xll.common.frame_size == 0U
            || xll.common.frame_size > xll_pbr.size()) {
            xll_pbr.clear();
            last_error_ = "XLL PBR decoded frame size";
            return ObjectFrameDecodeResult::Malformed;
        }
        if (xll.extension.present) {
            decoded.dtsx_extension_sync_word =
                xll.extension.sync_word;
        }
        xll_pbr.erase(
            xll_pbr.begin(),
            xll_pbr.begin()
                + static_cast<std::ptrdiff_t>(
                    xll.common.frame_size));
        bool renderer_auxiliary_metadata_present = false;
        const bool use_assembled_metadata =
            metadata_in_assembled_xll
            && assembled_xll_words != nullptr
            && metadata_component_offset < xll.common.frame_size;
        if (asset.xll_metadata_present
            && !asset.xll_metadata_chunk_sizes.empty()
            && (use_assembled_metadata
                || asset.xll_metadata_offset
                    < asset.component_size_bytes[9U])) {
            const std::uint32_t metadata_byte_offset =
                use_assembled_metadata
                ? metadata_component_offset
                : asset_payload_offset
                      + asset.component_byte_offsets[9U]
                      + asset.xll_metadata_offset;
            dtsx::bitstream::Cursor metadata_source =
                use_assembled_metadata
                ? assembled_xll_words->cursor()
                : words.cursor();
            metadata_source.fast_forward(static_cast<std::int32_t>(
                8U * metadata_byte_offset));
            metadata_source = metadata_source.limited(
                8U
                * (use_assembled_metadata
                       ? xll.common.frame_size
                             - metadata_component_offset
                       : asset.component_size_bytes[9U]
                             - asset.xll_metadata_offset));
            std::uint64_t declared_metadata_bytes = 2U;
            for (const std::uint8_t element_size :
                 asset.xll_metadata_chunk_sizes) {
                declared_metadata_bytes += 2U + element_size;
            }
            const std::uint64_t available_metadata_bytes =
                metadata_source.remaining_bits() / 8U;
            dtsx::MetadataChunkEnvelope envelope;
            const std::vector<std::uint8_t> metadata_element_sizes =
                resolve_metadata_element_sizes(
                    metadata_source,
                    asset.xll_metadata_chunk_sizes,
                    asset.xll_associated_chunk_types);
            if (dtsx::unpack_metadata_chunk_payload(
                    metadata_source,
                    metadata_element_sizes,
                    envelope,
                    static_cast<std::uint8_t>(
                        exss.asset_count))) {
                associated_audio_offset =
                    metadata_component_offset
                    + envelope.crc_region_size;
                associated_audio_offset_valid = true;
                for (const dtsx::MetadataElementHeader& element :
                     envelope.elements) {
                    if (element.chunk_id == 247U) {
                        renderer_auxiliary_metadata_present = true;
                    }
                }
                ++decoded.raw_metadata_envelopes;
                decoded.raw_metadata_elements +=
                    static_cast<std::uint32_t>(
                        envelope.elements.size());
                if (!envelope.crc_valid) {
                    ++decoded.raw_metadata_crc_failures;
                    decoded.raw_metadata_crc_failure_bytes +=
                        envelope.crc_region_size;
                }
                chunks.push_back(dtsx::MetadataChunkLocation{
                    metadata_byte_offset,
                    0U,
                    std::move(envelope),
                    use_assembled_metadata
                        ? assembled_xll_words
                        : nullptr});
            } else {
                ++decoded.raw_metadata_envelope_parse_failures;
                decoded.raw_metadata_envelope_failure_bytes +=
                    (std::min)(declared_metadata_bytes,
                               available_metadata_bytes);
            }
        }
        decoded.sample_rate =
            decoded.sample_rate == 0U
            ? xll.sample_rate
            : decoded.sample_rate;
        if (decoded.sample_rate != xll.sample_rate
            || (decoded.samples_per_channel != 0U
                && decoded.samples_per_channel
                    != xll.samples_per_channel)) {
            return ObjectFrameDecodeResult::Malformed;
        }
        decoded.samples_per_channel = xll.samples_per_channel;
        const bool object_audio = asset.object_audio_type == 1U;
        const std::size_t main_channel_count =
            (std::min<std::size_t>)(
                xll.main_planar_channel_count,
                xll.planar_channels.size());
        const bool supplemental_object_audio =
            main_channel_count < xll.planar_channels.size();
        if (object_audio) {
            decoded_asset_bases.emplace_back(
                asset.asset_index,
                static_cast<std::uint32_t>(
                    decoded.waveform_channels.size()));
        }
        if (!object_audio) {
            decoded.bed_downmix_outputs =
                std::move(xll.hierarchical_downmix_outputs);
            // Native XLL bed assets carry the physical speaker activity mask
            // in the asset header.  Some streams omit it there and carry the
            // equivalent physical mask in the XLL channel-set header.
            std::uint32_t bed_mask = asset.speaker_mask_present
                ? asset.speaker_activity_mask
                : 0U;
            if (bed_mask == 0U) {
                for (const dtsx::XllChannelSetHeader& channel_set :
                     xll.channel_sets) {
                    if (channel_set.channel_mask_enabled) {
                        bed_mask |= dtsx::speaker_mask_to_activity_mask(
                            channel_set.speaker_channel_mask);
                    }
                }
            }
            if (bed_mask != 0U) {
                decoded.bed_speaker_activity_mask = bed_mask;
                std::vector<std::uint32_t> decoded_speakers;
                decoded_speakers.reserve(main_channel_count);
                std::size_t mapped_channel_sets = 0U;
                for (const dtsx::XllChannelSetHeader& channel_set :
                     xll.channel_sets) {
                    if (mapped_channel_sets
                            + channel_set.probe.channel_count
                        > main_channel_count) {
                        break;
                    }
                    mapped_channel_sets +=
                        channel_set.probe.channel_count;
                    if (!channel_set.channel_mask_enabled) {
                        decoded_speakers.clear();
                        break;
                    }
                    for (std::uint32_t bit = 0U;
                         bit < 32U;
                         ++bit) {
                        const std::uint32_t speaker = 1U << bit;
                        if ((channel_set.speaker_channel_mask
                             & speaker)
                            != 0U) {
                            decoded_speakers.push_back(speaker);
                        }
                    }
                }
                const std::vector<std::uint32_t> canonical_speakers =
                    dtsx::expand_speaker_activity_mask(bed_mask);
                if (decoded_speakers.size()
                        == main_channel_count
                    && canonical_speakers.size()
                        == main_channel_count) {
                    for (std::uint32_t speaker :
                         canonical_speakers) {
                        const auto source = std::find(
                            decoded_speakers.begin(),
                            decoded_speakers.end(),
                            speaker);
                        if (source == decoded_speakers.end()) {
                            decoded.bed_channels.clear();
                            break;
                        }
                        decoded.bed_channels.push_back(
                            xll.planar_channels[
                                static_cast<std::size_t>(
                                    std::distance(
                                        decoded_speakers.begin(),
                                        source))]);
                    }
                }
                if (decoded.bed_channels.empty()) {
                    for (std::size_t channel = 0U;
                         channel < main_channel_count;
                         ++channel) {
                        decoded.bed_channels.push_back(
                            xll.planar_channels[channel]);
                    }
                }
            }
        } else {
            for (auto& channel : xll.planar_channels) {
                decoded.waveform_channels.push_back(
                    std::move(channel));
                decoded.waveform_speaker_masks.push_back(0U);
                decoded.waveform_source_activity_masks.push_back(
                    asset.speaker_mask_present
                        ? asset.speaker_activity_mask
                        : decoded.bed_speaker_activity_mask);
                decoded.waveform_is_supplemental.push_back(false);
            }
        }
        if (!object_audio && supplemental_object_audio) {
            decoded_asset_bases.emplace_back(
                asset.asset_index,
                static_cast<std::uint32_t>(
                    decoded.waveform_channels.size()));
            for (std::size_t channel = main_channel_count;
                 channel < xll.planar_channels.size();
                 ++channel) {
                decoded.waveform_channels.push_back(
                    std::move(xll.planar_channels[channel]));
                const std::size_t supplemental_channel =
                    channel - main_channel_count;
                decoded.waveform_speaker_masks.push_back(
                    combined_mix_available
                        && supplemental_channel
                               < combined_mix
                                     .added_speaker_masks.size()
                    ? combined_mix.added_speaker_masks[
                          supplemental_channel]
                    : supplemental_channel
                              < xll.supplemental_speaker_masks.size()
                    ? xll.supplemental_speaker_masks[
                          supplemental_channel]
                    : 0U);
                decoded.waveform_source_activity_masks.push_back(
                    decoded.bed_speaker_activity_mask);
                decoded.waveform_is_supplemental.push_back(true);
            }
            decoded.supplemental_downmix_outputs =
                std::move(xll.embedded_downmix_outputs);
            for (const std::uint32_t speaker_mask :
                 xll.supplemental_speaker_masks) {
                decoded.supplemental_speaker_activity_mask |=
                    dtsx::speaker_mask_to_activity_mask(
                        speaker_mask);
            }
        }

        if (asset.xll_object_metadata_present
            && !asset.xll_metadata_chunk_sizes.empty()
            && !asset.xll_associated_chunk_types.empty()
            && associated_audio_offset_valid) {
            std::uint32_t associated_offset = associated_audio_offset;
            const std::size_t object_buffer_size =
                metadata_in_assembled_xll
                ? static_cast<std::size_t>(xll.common.frame_size)
                : xll_packet.size();
            dtsx::bitstream::WordBuffer current_xll_words(
                xll_packet, false);
            for (std::size_t associated_index = 0U;
                 associated_index
                     < asset.xll_associated_chunk_types.size();
                 ++associated_index) {
                const std::uint8_t associated_chunk_type =
                    asset.xll_associated_chunk_types[
                        associated_index];
                if (associated_offset + 5U > object_buffer_size) {
                    break;
                }
                static constexpr std::array<std::uint8_t, 4U>
                    kAssociationTypeBits = {0U, 1U, 2U, 2U};
                const std::uint8_t association_type_bits =
                    exss.asset_count >= 1U
                            && exss.asset_count <= 4U
                        ? kAssociationTypeBits[
                              exss.asset_count - 1U]
                        : 2U;
                const std::uint8_t association_index_bits =
                    static_cast<std::uint8_t>(
                        8U - association_type_bits);
                dtsx::bitstream::Cursor association_source =
                    metadata_in_assembled_xll
                    ? assembled_xll_words->cursor()
                    : current_xll_words.cursor();
                association_source.fast_forward(
                    static_cast<std::int32_t>(
                        8U * associated_offset));
                const std::uint8_t association_index =
                    static_cast<std::uint8_t>(
                        association_source.extract_unsigned(8U)
                        & ((1U << association_index_bits) - 1U));
                const std::uint32_t object_frame_offset =
                    associated_offset + 1U;
                dtsx::bitstream::Cursor object_source =
                    association_source;
                const std::uint32_t sync =
                    object_source.lookahead_unsigned(32U);
                if (sync != dtsx::kXllSync
                    && sync != dtsx::kXllSyncLegacy) {
                    break;
                }
                dtsx::XllCommonHeader object_common;
                dtsx::bitstream::Cursor common_source = object_source;
                if (!dtsx::unpack_xll_common_header(
                        common_source, object_common)
                    || object_common.frame_size == 0U
                    || object_common.frame_size
                           > object_buffer_size - object_frame_offset) {
                    break;
                }
                const std::uint32_t associated_length =
                    object_common.frame_size + 1U;
                associated_offset += associated_length;
                if (associated_chunk_type != 65U
                    && associated_chunk_type != 68U) {
                    continue;
                }
                auto& object_decoders = associated_chunk_type == 65U
                    ? uhd_type65_xll_decoders_[asset.asset_index]
                    : uhd_xll_decoders_[asset.asset_index];
                if (object_decoders.size()
                    <= association_index) {
                    object_decoders.resize(
                        static_cast<std::size_t>(
                            association_index) + 1U);
                }
                dtsx::XllDecodedFrame object_xll;
                object_source = object_source.limited(
                    8U * object_common.frame_size);
                if (!object_decoders[association_index]
                         .decode_msb_frame(
                             object_source,
                             object_xll,
                             false)
                    || object_xll.common.frame_size
                           != object_common.frame_size
                    || object_xll.samples_per_channel
                           != decoded.samples_per_channel
                    || object_xll.sample_rate
                           != decoded.sample_rate) {
                    continue;
                }
                const std::uint32_t waveform_base =
                    static_cast<std::uint32_t>(
                        decoded.waveform_channels.size());
                decoded_uhd_waveform_bases.emplace_back(
                    association_index,
                    waveform_base);
                decoded_uhd_waveform_ranges.push_back(
                    AssociatedWaveformRange{
                        association_index,
                        waveform_base,
                        static_cast<std::uint32_t>(
                            object_xll.planar_channels.size()),
                        renderer_auxiliary_metadata_present});
                std::uint32_t object_source_activity_mask = 0U;
                for (const dtsx::XllChannelSetHeader& channel_set :
                     object_xll.channel_sets) {
                    if (channel_set.channel_mask_enabled) {
                        object_source_activity_mask |=
                            dtsx::speaker_mask_to_activity_mask(
                                channel_set.speaker_channel_mask);
                    }
                }
                if (object_source_activity_mask == 0U) {
                    object_source_activity_mask =
                        asset.speaker_mask_present
                        ? asset.speaker_activity_mask
                        : decoded.bed_speaker_activity_mask;
                }
                for (auto& channel :
                     object_xll.planar_channels) {
                    decoded.waveform_channels.push_back(
                        std::move(channel));
                    decoded.waveform_speaker_masks.push_back(0U);
                    decoded.waveform_source_activity_masks.push_back(
                        object_source_activity_mask);
                    decoded.waveform_is_supplemental.push_back(false);
                }
            }
        }

    }
    if (!decoded_uhd_waveform_ranges.empty()) {
        bool unique_associations = true;
        std::array<bool, 256U> current_associations{};
        for (const AssociatedWaveformRange& range :
             decoded_uhd_waveform_ranges) {
            if (current_associations[range.association_index]) {
                unique_associations = false;
                break;
            }
            current_associations[range.association_index] = true;
        }

        bool registered_subset = unique_associations
            && !associated_waveform_layout_state_.empty();
        if (registered_subset) {
            for (const AssociatedWaveformRange& current :
                 decoded_uhd_waveform_ranges) {
                const auto registered = std::find_if(
                    associated_waveform_layout_state_.begin(),
                    associated_waveform_layout_state_.end(),
                    [&current](const AssociatedWaveformLayout& candidate) {
                        return candidate.association_index
                                   == current.association_index
                            && candidate.channel_count
                                   == current.channel_count;
                    });
                if (registered
                    == associated_waveform_layout_state_.end()) {
                    registered_subset = false;
                    break;
                }
            }
        }

        if (registered_subset
            && waveform_channel_count_state_ != 0U) {
            const std::size_t current_channel_count =
                decoded.waveform_channels.size();
            std::vector<bool> claimed_current(
                current_channel_count, false);
            std::vector<bool> reserved_registered(
                waveform_channel_count_state_, false);
            bool valid_layout = true;
            for (const AssociatedWaveformLayout& registered :
                 associated_waveform_layout_state_) {
                if (registered.base_channel
                        + registered.channel_count
                    > waveform_channel_count_state_) {
                    valid_layout = false;
                    break;
                }
                for (std::uint32_t channel = 0U;
                     channel < registered.channel_count;
                     ++channel) {
                    reserved_registered[
                        registered.base_channel + channel] = true;
                }
            }
            for (const AssociatedWaveformRange& current :
                 decoded_uhd_waveform_ranges) {
                if (current.base_channel + current.channel_count
                    > current_channel_count) {
                    valid_layout = false;
                    break;
                }
                for (std::uint32_t channel = 0U;
                     channel < current.channel_count;
                     ++channel) {
                    claimed_current[
                        current.base_channel + channel] = true;
                }
            }
            for (std::size_t channel = 0U;
                 valid_layout && channel < current_channel_count;
                 ++channel) {
                if (!claimed_current[channel]
                    && channel < reserved_registered.size()
                    && reserved_registered[channel]) {
                    valid_layout = false;
                }
            }

            if (valid_layout) {
                std::vector<std::vector<std::int32_t>> channels(
                    waveform_channel_count_state_,
                    std::vector<std::int32_t>(
                        decoded.samples_per_channel, 0));
                std::vector<std::uint32_t> speaker_masks(
                    waveform_channel_count_state_, 0U);
                std::vector<std::uint32_t> source_masks(
                    waveform_channel_count_state_, 0U);
                std::vector<bool> supplemental(
                    waveform_channel_count_state_, false);
                const auto move_channel =
                    [&](std::size_t source, std::size_t destination) {
                        channels[destination] = std::move(
                            decoded.waveform_channels[source]);
                        if (source
                            < decoded.waveform_speaker_masks.size()) {
                            speaker_masks[destination] =
                                decoded.waveform_speaker_masks[source];
                        }
                        if (source
                            < decoded.waveform_source_activity_masks.size()) {
                            source_masks[destination] =
                                decoded.waveform_source_activity_masks[source];
                        }
                        if (source
                            < decoded.waveform_is_supplemental.size()) {
                            supplemental[destination] =
                                decoded.waveform_is_supplemental[source];
                        }
                    };
                for (std::size_t channel = 0U;
                     channel < current_channel_count;
                     ++channel) {
                    if (!claimed_current[channel]
                        && channel < channels.size()) {
                        move_channel(channel, channel);
                    }
                }
                for (const AssociatedWaveformRange& current :
                     decoded_uhd_waveform_ranges) {
                    const auto registered = std::find_if(
                        associated_waveform_layout_state_.begin(),
                        associated_waveform_layout_state_.end(),
                        [&current](
                            const AssociatedWaveformLayout& candidate) {
                            return candidate.association_index
                                == current.association_index;
                        });
                    for (std::uint32_t channel = 0U;
                         channel < current.channel_count;
                         ++channel) {
                        move_channel(
                            current.base_channel + channel,
                            registered->base_channel + channel);
                    }
                }
                decoded.waveform_channels = std::move(channels);
                decoded.waveform_speaker_masks =
                    std::move(speaker_masks);
                decoded.waveform_source_activity_masks =
                    std::move(source_masks);
                decoded.waveform_is_supplemental =
                    std::move(supplemental);
                decoded_uhd_waveform_bases.clear();
                decoded_uhd_waveform_ranges.clear();
                for (const AssociatedWaveformLayout& registered :
                     associated_waveform_layout_state_) {
                    decoded_uhd_waveform_bases.emplace_back(
                        registered.association_index,
                        registered.base_channel);
                    decoded_uhd_waveform_ranges.push_back(
                        AssociatedWaveformRange{
                            registered.association_index,
                            registered.base_channel,
                            registered.channel_count,
                            registered
                                .renderer_auxiliary_metadata_present});
                }
            } else {
                registered_subset = false;
            }
        }

        if (!registered_subset) {
            associated_waveform_layout_state_.clear();
            associated_waveform_layout_state_.reserve(
                decoded_uhd_waveform_ranges.size());
            for (const AssociatedWaveformRange& range :
                 decoded_uhd_waveform_ranges) {
                associated_waveform_layout_state_.push_back(
                    AssociatedWaveformLayout{
                        range.association_index,
                        range.base_channel,
                        range.channel_count,
                        range.renderer_auxiliary_metadata_present});
            }
            waveform_channel_count_state_ =
                static_cast<std::uint32_t>(
                    decoded.waveform_channels.size());
        }
    }
    if (!decoded_asset_bases.empty()
        || !decoded_uhd_waveform_bases.empty()) {
        decoded.waveform_base_by_id.assign(
            256U, std::numeric_limits<std::uint32_t>::max());
    }
    std::vector<std::pair<std::uint8_t, std::uint32_t>>
        decoded_waveform_bases = decoded_asset_bases;
    decoded_waveform_bases.insert(
        decoded_waveform_bases.end(),
        decoded_uhd_waveform_bases.begin(),
        decoded_uhd_waveform_bases.end());
    if (!decoded_waveform_bases.empty()) {
        std::stable_sort(
            decoded_waveform_bases.begin(),
            decoded_waveform_bases.end(),
            [](const auto& left, const auto& right) {
                return left.first < right.first;
            });
        std::size_t decoder_index = 0U;
        for (std::uint32_t waveform_id =
                 decoded_waveform_bases.front().first;
             waveform_id < decoded.waveform_base_by_id.size();
             ++waveform_id) {
            while (decoder_index + 1U
                       < decoded_waveform_bases.size()
                   && decoded_waveform_bases[
                           decoder_index + 1U]
                               .first
                       <= waveform_id) {
                ++decoder_index;
            }
            decoded.waveform_base_by_id[waveform_id] =
                decoded_waveform_bases[decoder_index].second;
        }
    }
    if (!decoded_asset_bases.empty()
        || !decoded_uhd_waveform_bases.empty()) {
        waveform_base_by_id_state_ =
            decoded.waveform_base_by_id;
    } else {
        // Native ExSS object state keeps decoder pointers across frames where
        // static fields or the XLL asset payload are omitted.
        decoded.waveform_base_by_id = waveform_base_by_id_state_;
    }

    bool have_current_object_presentation = false;
    std::size_t current_object_count = 0U;
    bool current_object_ids_complete = false;
    std::vector<std::uint32_t> current_object_ids;
    std::shared_ptr<const dtsx::bitstream::WordBuffer> core_words;
    if (pending_core_valid_) {
        core_words = std::make_shared<dtsx::bitstream::WordBuffer>(
            pending_core_bytes_,
            core_swap_byte_pairs(pending_core_packing_));
        std::vector<dtsx::MetadataChunkLocation> core_chunks =
            dtsx::scan_metadata_chunks(
                core_words->cursor(),
                static_cast<std::uint32_t>(pending_core_bytes_.size()),
                static_cast<std::uint8_t>(exss.asset_count),
                1U);
        for (dtsx::MetadataChunkLocation& chunk : core_chunks) {
            chunk.source_words = core_words;
            ++decoded.raw_metadata_envelopes;
            ++decoded.core_metadata_envelopes;
            decoded.raw_metadata_elements +=
                static_cast<std::uint32_t>(chunk.envelope.elements.size());
            decoded.core_metadata_elements +=
                static_cast<std::uint32_t>(chunk.envelope.elements.size());
            for (const dtsx::MetadataElementHeader& element :
                 chunk.envelope.elements) {
                decoded.core_metadata_chunk_ids.push_back(
                    element.chunk_id);
            }
            if (!chunk.envelope.crc_valid) {
                ++decoded.raw_metadata_crc_failures;
                ++decoded.core_metadata_crc_failures;
                decoded.raw_metadata_crc_failure_bytes +=
                    chunk.envelope.crc_region_size;
            }
        }
        chunks.insert(
            chunks.end(),
            std::make_move_iterator(core_chunks.begin()),
            std::make_move_iterator(core_chunks.end()));
        pending_core_valid_ = false;
        pending_core_bytes_.clear();
    }
    std::vector<dtsx::MetadataChunkLocation> scanned_chunks =
        dtsx::scan_metadata_chunks(
            words.cursor(),
            static_cast<std::uint32_t>(elementary.bytes.size()),
            static_cast<std::uint8_t>(exss.asset_count));
    chunks.insert(
        chunks.end(),
        std::make_move_iterator(scanned_chunks.begin()),
        std::make_move_iterator(scanned_chunks.end()));
    for (const dtsx::MetadataChunkLocation& chunk : chunks) {
        // dtsCheckValidMetaDataChunk keeps CRC validity as state, while the
        // native metadata parser rejects the chunk before applying objects
        // when that CRC is bad.
        if (!chunk.envelope.crc_valid) {
            continue;
        }
        for (const dtsx::MetadataElementHeader& element :
             chunk.envelope.elements) {
            if (element.chunk_id != 241U || !element.primary) {
                continue;
            }
            const std::uint32_t element_byte_offset =
                chunk.byte_offset
                + chunk.element_prefix_bytes
                + element.byte_offset + 2U;
            dtsx::bitstream::Cursor metadata_source =
                chunk.source_words != nullptr
                ? chunk.source_words->cursor()
                : words.cursor();
            metadata_source.fast_forward(static_cast<std::int32_t>(
                8U * element_byte_offset));

            dtsx::PreliminaryMetadataHeader preliminary;
            preliminary.chunk_id = element.chunk_id;
            preliminary.raw_flags = element.flags;
            preliminary.primary = element.primary;
            preliminary.short_form = element.short_form;
            preliminary.alternate_association =
                element.alternate_association;
            preliminary.association_type =
                element.association_type;
            preliminary.association_index =
                element.association_index;
            dtsx::AudioPresentationMetadata presentation;
            if (!dtsx::unpack_audio_presentation_metadata(
                    metadata_source,
                    preliminary,
                    preliminary.short_form,
                    presentation)) {
                continue;
            }
            ++decoded.metadata_presentation_headers;
            std::vector<bool> waveform_decoder_available(
                presentation.objects.size(), false);
            const auto decoder_available_for_waveform =
                [&decoded](bool available, std::uint8_t waveform_id) {
                    return available
                        && waveform_id < decoded.waveform_base_by_id.size()
                        && decoded.waveform_base_by_id[waveform_id]
                            != std::numeric_limits<
                                std::uint32_t>::max();
                };
            for (std::size_t object_index = 0U;
                 object_index < presentation.objects.size();
                 ++object_index) {
                const dtsx::ObjectMetadataBlock& object =
                    presentation.objects[object_index];
                waveform_decoder_available[object_index] =
                    decoder_available_for_waveform(
                        object.waveform_id_available,
                        object.waveform_id);
                if (!waveform_decoder_available[object_index]
                    && !object.metadata_present) {
                    std::size_t previous_index = object_state_.size();
                    if (object.object_id_available) {
                        for (std::size_t candidate = 0U;
                             candidate < object_state_.size();
                             ++candidate) {
                            if (object_state_[candidate].object_id_available
                                && object_state_[candidate].object_id
                                    == object.object_id) {
                                previous_index = candidate;
                                break;
                            }
                        }
                    } else if (object_index < object_state_.size()) {
                        previous_index = object_index;
                    }
                    if (previous_index < object_state_.size()) {
                        const dtsx::ObjectMetadataBlock& previous =
                            object_state_[previous_index];
                        waveform_decoder_available[object_index] =
                            decoder_available_for_waveform(
                                previous.waveform_id_available,
                                previous.waveform_id);
                    }
                }
            }
            if (!dtsx::unpack_audio_presentation_object_bodies(
                metadata_source,
                presentation,
                &waveform_decoder_available)) {
                ++decoded.metadata_body_parse_failures;
                decoded.metadata_body_failure_declared_bytes +=
                    element.payload_size;
                continue;
            }
            if (presentation.speaker_count != 0U) {
                metadata_speaker_activity_mask_ =
                    presentation.speaker_activity_mask;
            }
            presentation_gain_code_ =
                presentation.render_gain_code;
            alternative_presentation_gain_present_ =
                presentation.alternative_render_gain_present;
            alternative_presentation_gain_code_ =
                presentation.alternative_render_gain_code;
            if (presentation.metadata_present) {
                have_current_object_presentation = true;
                current_object_count = presentation.objects.size();
                current_object_ids.clear();
                current_object_ids_complete = true;
                for (const dtsx::ObjectMetadataBlock& object :
                     presentation.objects) {
                    if (!object.object_id_available) {
                        current_object_ids_complete = false;
                        break;
                    }
                    current_object_ids.push_back(object.object_id);
                }
            }
            for (std::size_t object_index = 0U;
                 object_index < presentation.objects.size();
                 ++object_index) {
                dtsx::ObjectMetadataBlock& update =
                    presentation.objects[object_index];
                if (!update.metadata_present) {
                    continue;
                }
                std::size_t state_index = object_state_.size();
                if (update.object_id_available) {
                    for (std::size_t candidate = 0U;
                         candidate < object_state_.size();
                         ++candidate) {
                        if (object_state_[candidate]
                                .object_id_available
                            && object_state_[candidate].object_id
                                == update.object_id) {
                            state_index = candidate;
                            break;
                        }
                    }
                } else if (object_index < object_state_.size()) {
                    state_index = object_index;
                }
                if (state_index == object_state_.size()) {
                    object_state_.emplace_back();
                }
                merge_object_update(
                    std::move(update),
                    object_state_[state_index]);
            }
        }
    }
    if (have_current_object_presentation
        && current_object_count < object_state_.size()) {
        if (current_object_ids_complete) {
            object_state_.erase(
                std::remove_if(
                    object_state_.begin(),
                    object_state_.end(),
                    [&current_object_ids](
                        const dtsx::ObjectMetadataBlock& object) {
                        if (!object.object_id_available) {
                            return true;
                        }
                        return std::find(
                                   current_object_ids.begin(),
                                   current_object_ids.end(),
                                   object.object_id)
                            == current_object_ids.end();
                    }),
                object_state_.end());
        } else {
            object_state_.resize(current_object_count);
        }
    }
    decoded.metadata_speaker_activity_mask =
        metadata_speaker_activity_mask_;
    decoded.alternative_presentation_gain_present =
        alternative_presentation_gain_present_;
    decoded.alternative_presentation_gain_code =
        alternative_presentation_gain_code_;
    decoded.presentation_gain_code =
        presentation_gain_code_;
    decoded.objects = object_state_;
    decoded.objects.erase(
        std::remove_if(
            decoded.objects.begin(),
            decoded.objects.end(),
            [&decoded](
                const dtsx::ObjectMetadataBlock& object) {
                std::vector<std::uint32_t> channels;
                if (!dtsx::object_waveform_channel_indices(
                        object,
                        channels,
                        &decoded.waveform_base_by_id)
                    || channels.empty()) {
                    ++decoded.ignored_unmapped_objects;
                    ++decoded.unmapped_objects_missing_waveform;
                    return true;
                }
                for (const std::uint32_t channel : channels) {
                    if (channel
                        >= decoded.waveform_channels.size()) {
                        if (decoded.waveform_channels.empty()) {
                            ++decoded
                                  .inactive_objects_without_waveform_audio;
                        } else {
                            ++decoded.ignored_unmapped_objects;
                            ++decoded
                                  .unmapped_objects_channel_out_of_range;
                        }
                        if (!decoded.unmapped_object_detail_available) {
                            decoded.unmapped_object_detail_available = true;
                            decoded.unmapped_object_waveform_id =
                                object.waveform_id;
                            decoded.unmapped_object_requested_channel =
                                channel;
                            decoded.unmapped_object_available_channels =
                                static_cast<std::uint32_t>(
                                    decoded.waveform_channels.size());
                        }
                        return true;
                    }
                }
                return false;
            }),
        decoded.objects.end());
    std::array<bool, 256U> referenced_waveform_decoders{};
    for (const dtsx::ObjectMetadataBlock& object :
         decoded.objects) {
        if (object.waveform_id_available) {
            referenced_waveform_decoders[object.waveform_id] = true;
        }
    }
    for (const AssociatedWaveformRange& range :
         decoded_uhd_waveform_ranges) {
        if (!range.renderer_auxiliary_metadata_present
            || referenced_waveform_decoders[
                   range.association_index]
            || range.channel_count != 4U
            || range.base_channel
                   + range.channel_count
               > decoded.waveform_channels.size()) {
            continue;
        }
        static constexpr std::array<const char*, 4U>
            kUpperLayerOrder = {
                "TFL", "TFR", "TBL", "TBR"};
        for (std::uint32_t channel = 0U;
             channel < range.channel_count;
             ++channel) {
            std::uint32_t speaker_mask =
                channel < dtsx::kStandardHeightSpeakerMasks.size()
                ? dtsx::kStandardHeightSpeakerMasks[channel]
                : 0U;
            if (speaker_mask == 0U
                && !dtsx::standard_speaker_mask(
                    kUpperLayerOrder[channel],
                    speaker_mask)) {
                break;
            }
            const std::size_t waveform =
                range.base_channel + channel;
            decoded.waveform_speaker_masks[waveform] =
                speaker_mask;
            decoded.waveform_is_supplemental[waveform] = true;
            decoded.supplemental_speaker_activity_mask |=
                dtsx::speaker_mask_to_activity_mask(
                    speaker_mask);
        }
    }
    return decoded.waveform_channels.empty()
            && decoded.bed_channels.empty()
            && decoded.objects.empty()
        ? ObjectFrameDecodeResult::Ignored
        : ObjectFrameDecodeResult::Decoded;
}

} // namespace dtsx_decode
