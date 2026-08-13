#include "dtsx/exss_asset.hpp"

#include "dtsx/speaker_mask.hpp"

#include <array>

namespace dtsx {

bool unpack_exss_asset_summary(bitstream::Cursor& source,
                               const ExssHeader& exss,
                               ExssAssetSummary& asset,
                               std::uint32_t asset_ordinal) noexcept {
    // libdtsx.so: dtsParseAsset, 0x2f738..0x2fa68 and 0x302e0..0x3039c.
    const bitstream::Cursor asset_start = source;
    asset = {};
    asset.header_size = source.extract_unsigned(9U) + 1U;
    if (asset.header_size > asset_start.remaining_bits() / 8U
        || asset_ordinal >= exss.asset_count
        || asset_ordinal >= exss.asset_sizes.size()) {
        return false;
    }
    asset.asset_index = static_cast<std::uint8_t>(
        source.extract_unsigned(3U));
    bool mix_metadata_present = false;
    if (exss.static_fields_present) {
        if (source.extract_unsigned(1U) != 0U) {
            // ARCAM AVRx0 firmware, DTS Decoder SDK 3.90.50.1:
            // dtsxSubstreamParseAsset stores this four-bit content-type
            // value and raises its dedicated Type1-certified flag exactly
            // when the value is 13.  This is the native IMAX-content
            // classifier consumed by the AVR controller.
            asset.content_type_present = true;
            asset.content_type = static_cast<std::uint8_t>(
                source.extract_unsigned(4U));
            asset.type1_certified_content =
                asset.content_type == 13U;
        }
        if (source.extract_unsigned(1U) != 0U) {
            source.fast_forward(24);
        }
        if (source.extract_unsigned(1U) != 0U) {
            const std::uint32_t text_bytes = source.extract_unsigned(10U) + 1U;
            source.fast_forward(static_cast<std::int32_t>(8U * text_bytes));
        }
        source.fast_forward(5);
        static constexpr std::array<std::uint32_t, 16> kSampleRates = {
            8000U, 16000U, 32000U, 64000U, 128000U, 22050U,
            44100U, 88200U, 176400U, 352800U, 12000U, 24000U,
            48000U, 96000U, 192000U, 384000U,
        };
        asset.sample_rate = kSampleRates[source.extract_unsigned(4U)];
        asset.channel_count = source.extract_unsigned(8U) + 1U;
        asset.one_to_one_mapping = source.extract_unsigned(1U) != 0U;
        if (asset.one_to_one_mapping) {
            if (asset.channel_count > 2U) {
                asset.embedded_stereo = source.extract_unsigned(1U) != 0U;
            }
            if (asset.channel_count > 6U) {
                asset.embedded_six_channel =
                    source.extract_unsigned(1U) != 0U;
            }
            asset.speaker_mask_present =
                source.extract_unsigned(1U) != 0U;
            std::uint32_t mask_bits = 0U;
            if (asset.speaker_mask_present) {
                mask_bits = 4U * (source.extract_unsigned(2U) + 1U);
                asset.speaker_activity_mask =
                    source.extract_unsigned(mask_bits);
                asset.speaker_count = speaker_count_from_activity_mask(
                    asset.speaker_activity_mask);
            }
            const std::uint32_t remap_sets = source.extract_unsigned(3U);
            for (std::uint32_t set = 0; set < remap_sets; ++set) {
                const std::uint32_t remap_mask = source.extract_unsigned(mask_bits);
                const std::uint32_t destinations =
                    speaker_count_from_activity_mask(remap_mask);
                const std::uint32_t coefficient_bits =
                    source.extract_unsigned(5U) + 1U;
                for (std::uint32_t destination = 0;
                     destination < destinations;
                     ++destination) {
                    const std::uint32_t coefficient_mask =
                        source.extract_unsigned(coefficient_bits);
                    std::uint32_t coefficient_count = 0U;
                    for (std::uint32_t bit = 0;
                         bit < coefficient_bits;
                         ++bit) {
                        coefficient_count +=
                            (coefficient_mask >> bit) & 1U;
                    }
                    source.fast_forward(static_cast<std::int32_t>(
                        5U * coefficient_count));
                }
            }
        } else {
            asset.representation_type = static_cast<std::uint8_t>(
                source.extract_unsigned(3U));
            // dtsParseAsset turns the three-bit representation selector into
            // the object-audio flag at asset+0x40; only selector value 1 is
            // treated as object audio.  Keep the raw selector separately.
            asset.object_audio_type =
                asset.representation_type == 1U ? 1U : 0U;
        }
    }
    const bool language_present = source.extract_unsigned(1U) != 0U;
    if (language_present) {
        source.fast_forward(8);
    }
    const bool info_text_present = source.extract_unsigned(1U) != 0U;
    if (info_text_present) {
        source.fast_forward(5);
    }
    if (language_present && asset.embedded_stereo) {
        source.fast_forward(8);
    }
    if (exss.speaker_metadata_present != 0U) {
        mix_metadata_present =
            source.extract_unsigned(1U) != 0U;
        if (mix_metadata_present) {
            source.fast_forward(7);
            const std::uint32_t adjustment_mode =
                source.extract_unsigned(2U);
            source.fast_forward(
                static_cast<std::int32_t>(
                    adjustment_mode <= 2U ? 3U : 8U));
            const bool per_speaker_adjustment =
                source.extract_unsigned(1U) != 0U;
            for (std::uint32_t config = 0;
                 config < exss.speaker_mask_count;
                 ++config) {
                const std::uint32_t coefficient_count =
                    per_speaker_adjustment
                    && config < exss.speaker_counts.size()
                    ? exss.speaker_counts[config]
                    : 1U;
                source.fast_forward(static_cast<std::int32_t>(
                    6U * coefficient_count));
            }

            std::array<std::uint32_t, 3> channel_sets{};
            std::uint32_t channel_set_count = 1U;
            channel_sets[0] = asset.channel_count;
            if (asset.embedded_six_channel) {
                channel_sets[channel_set_count++] = 6U;
            }
            if (asset.embedded_stereo) {
                channel_sets[channel_set_count++] = 2U;
            }
            for (std::uint32_t config = 0;
                 config < exss.speaker_mask_count;
                 ++config) {
                const std::uint32_t speaker_count =
                    config < exss.speaker_counts.size()
                    ? exss.speaker_counts[config]
                    : 0U;
                for (std::uint32_t set = 0;
                     set < channel_set_count;
                     ++set) {
                    for (std::uint32_t channel = 0;
                         channel < channel_sets[set];
                         ++channel) {
                        const std::uint32_t mask =
                            source.extract_unsigned(speaker_count);
                        std::uint32_t active = 0U;
                        for (std::uint32_t bit = 0;
                             bit < speaker_count;
                             ++bit) {
                            active += (mask >> bit) & 1U;
                        }
                        source.fast_forward(static_cast<std::int32_t>(
                            6U * active));
                    }
                }
            }
        }
    }
    asset.coding_mode = static_cast<std::uint8_t>(
        source.extract_unsigned(2U));
    asset.coding_mode_available = true;
    if (asset.coding_mode == 0U) {
        asset.coding_components = static_cast<std::uint16_t>(
            source.extract_unsigned(12U));
        std::uint32_t component_offset = 0U;
        for (std::uint32_t component = 4U;
             component <= 8U;
             ++component) {
            if ((asset.coding_components & (1U << component)) == 0U) {
                continue;
            }
            const std::uint32_t size_bits = component == 7U ? 12U : 14U;
            const std::uint32_t size =
                (source.extract_unsigned(size_bits) + 3U) & ~3U;
            asset.component_size_bytes[component] = size;
            asset.component_byte_offsets[component] = component_offset;
            component_offset += size;
            if (component == 4U
                && source.extract_unsigned(1U) != 0U) {
                source.fast_forward(2);
            }
            if (component == 8U
                && source.extract_unsigned(1U) != 0U) {
                (void)source.extract_unsigned(2U);
            }
        }
        if ((asset.coding_components & (1U << 9U)) != 0U) {
            asset.component_byte_offsets[9U] = component_offset;
            asset.component_size_bytes[9U] =
                source.extract_unsigned(exss.size_field_bits) + 1U;
            asset.xll_sync_present =
                source.extract_unsigned(1U) != 0U;
            if (asset.xll_sync_present) {
                asset.xll_smoothing_buffer_kbytes =
                    static_cast<std::uint8_t>(
                        16U * source.extract_unsigned(4U));
                asset.xll_initial_delay_bits =
                    static_cast<std::uint8_t>(
                        source.extract_unsigned(5U) + 1U);
                asset.xll_initial_delay_frames =
                    source.extract_unsigned(
                        asset.xll_initial_delay_bits);
                asset.xll_sync_offset =
                    source.extract_unsigned(exss.size_field_bits);
            }
        }
        if ((asset.coding_components & (1U << 10U)) != 0U) {
            source.fast_forward(16);
        }
        if ((asset.coding_components & (1U << 11U)) != 0U) {
            source.fast_forward(16);
        }
    } else if (asset.coding_mode == 1U) {
        source.fast_forward(static_cast<std::int32_t>(exss.size_field_bits));
    } else {
        source.fast_forward(14);
    }

    const auto bits_consumed = [&]() noexcept {
        return asset_start.remaining_bits() - source.remaining_bits();
    };
    const auto bits_available = [&](std::uint32_t count) noexcept {
        return bits_consumed() + count <= 8U * asset.header_size;
    };
    const bool has_xll =
        asset.coding_mode == 1U
        || (asset.coding_mode == 0U
            && (asset.coding_components & (1U << 9U)) != 0U);
    if (has_xll && bits_available(3U)) {
        asset.dts_hd_stream_id = static_cast<std::uint8_t>(
            source.extract_unsigned(3U));
    }
    if (asset.one_to_one_mapping
        && exss.static_fields_present
        && exss.speaker_metadata_present != 0U
        && !mix_metadata_present
        && bits_available(1U)) {
        const bool one_to_one_mixing =
            source.extract_unsigned(1U) != 0U;
        if (one_to_one_mixing && bits_available(1U)) {
            const bool per_speaker =
                source.extract_unsigned(1U) != 0U;
            const std::uint32_t coefficient_count =
                per_speaker ? asset.channel_count : 1U;
            if (bits_available(6U * coefficient_count)) {
                source.fast_forward(static_cast<std::int32_t>(
                    6U * coefficient_count));
            }
        }
    }
    if (bits_available(1U)) {
        asset.decode_in_secondary_decoder =
            source.extract_unsigned(1U) != 0U;
    }
    if (bits_available(1U)) {
        const bool reserved_data_present =
            source.extract_unsigned(1U) != 0U;
        if (reserved_data_present && bits_available(4U)) {
            (void)source.extract_unsigned(4U);
            const std::uint32_t reserved_bytes =
                exss.frame_duration >> 8U;
            if (bits_available(8U * reserved_bytes)) {
                source.fast_forward(static_cast<std::int32_t>(
                    8U * reserved_bytes));
            }
        }
    }
    if (has_xll && bits_available(1U)) {
        (void)source.extract_unsigned(1U);
    }
    if (asset.speaker_mask_present && bits_available(2U)) {
        asset.object_audio_type = static_cast<std::uint8_t>(
            source.extract_unsigned(2U));
    }
    if (has_xll && bits_available(2U)) {
        asset.xll_navigation_bit_offset = bits_consumed();
        asset.xll_metadata_present =
            source.extract_unsigned(1U) != 0U;
        asset.xll_object_metadata_present =
            source.extract_unsigned(1U) != 0U;
        if ((asset.xll_metadata_present
             || asset.xll_object_metadata_present)
            && bits_available(exss.size_field_bits)) {
            asset.xll_metadata_offset =
                source.extract_unsigned(exss.size_field_bits);
            if (asset.xll_metadata_present && bits_available(4U)) {
                const std::uint32_t count =
                    source.extract_unsigned(4U) + 1U;
                if (bits_available(8U * count)) {
                    asset.xll_metadata_chunk_sizes.resize(count);
                    for (std::uint8_t& size :
                         asset.xll_metadata_chunk_sizes) {
                        size = static_cast<std::uint8_t>(
                            source.extract_unsigned(8U));
                    }
                }
            }
            if (asset.xll_object_metadata_present
                && bits_available(4U)) {
                const std::uint32_t count =
                    source.extract_unsigned(4U) + 1U;
                asset.xll_associated_chunk_types.resize(count);
                for (std::uint8_t& size :
                     asset.xll_associated_chunk_types) {
                    if (bits_available(8U)) {
                        size = static_cast<std::uint8_t>(
                            source.extract_unsigned(8U));
                    }
                }
                asset.xll_associated_chunk_extents.resize(count);
                asset.xll_object_sizes_bit_offset = bits_consumed();
                for (std::uint16_t& samples :
                     asset.xll_associated_chunk_extents) {
                    if (source.remaining_bits() >= 15U) {
                        samples = static_cast<std::uint16_t>(
                            source.extract_unsigned(15U) + 1U);
                    }
                }
            }
        }
    }
    const std::uint32_t parsed_bits =
        asset_start.remaining_bits() - source.remaining_bits();
    asset.descriptor_bits_used = parsed_bits;
    // libdtsx.so dtsxSubstreamParseAsset intentionally continues reading
    // private XLL navigation arrays beyond the nominal asset-descriptor
    // length, then restores the saved position and advances by header_size.
    // Multi-element DTS:X/Pro descriptors can exceed the public header by
    // considerably more than one 32-bit lookahead word.
    if (!source.valid()) {
        source = asset_start;
        source.fast_forward(
            static_cast<std::int32_t>(8U * asset.header_size));
        return false;
    }
    source = asset_start;
    source.fast_forward(static_cast<std::int32_t>(asset.header_size << 3U));
    return true;
}

} // namespace dtsx
