#include "dtsx/exss_header.hpp"

#include "dtsx/crc16.hpp"
#include "dtsx/speaker_mask.hpp"

namespace dtsx {

bool unpack_exss_header(bitstream::Cursor& source,
                        ExssHeader& header,
                        std::uint32_t selected_waveform_index) noexcept {
    const bitstream::Cursor frame_start = source;
    const std::uint32_t frame_available_bits = source.remaining_bits();
    if (source.remaining_bits() < 32U
        || source.extract_unsigned(32U) != 0x64582025U) {
        return false;
    }

    header = {};
    bool valid = true;
    const auto extract = [&source, &valid](std::uint32_t bit_count) noexcept {
        if (source.remaining_bits() < bit_count) {
            valid = false;
            return 0U;
        }
        return source.extract_unsigned(bit_count);
    };
    const auto skip = [&source, &valid](std::uint32_t bit_count) noexcept {
        if (source.remaining_bits() < bit_count) {
            valid = false;
            return;
        }
        source.fast_forward(static_cast<std::int32_t>(bit_count));
    };

    skip(8U);
    header.stream_index = static_cast<std::uint8_t>(extract(2U));
    header.long_size_fields = extract(1U) != 0U;
    header.size_field_bits = header.long_size_fields ? 20U : 16U;
    const std::uint32_t size_bits = header.long_size_fields ? 12U : 8U;
    header.header_size = extract(size_bits) + 1U;
    header.frame_size = extract(header.size_field_bits) + 1U;
    if (!valid || header.header_size > header.frame_size
        || header.header_size > frame_available_bits / 8U
        || header.frame_size > frame_available_bits / 8U) {
        return false;
    }

    const std::uint32_t consumed_bits =
        frame_available_bits - source.remaining_bits();
    const std::uint32_t header_bits = header.header_size << 3U;
    if (consumed_bits > header_bits) {
        return false;
    }
    source = frame_start.limited(header_bits);
    source.fast_forward(static_cast<std::int32_t>(consumed_bits));

    const bool presentation = extract(1U) != 0U;
    header.static_fields_present = presentation;
    if (!presentation) {
        header.waveform_count = 1U;
        header.asset_count = 1U;
    } else {
        header.reference_clock_code = static_cast<std::uint8_t>(
            extract(2U));
        header.frame_duration =
            (extract(3U) + 1U) << 9U;
        if (extract(1U) != 0U) {
            skip(36U);
        }
        header.waveform_count = extract(3U) + 1U;
        header.asset_count = extract(3U) + 1U;
        for (std::uint32_t index = 0; index < header.waveform_count; ++index) {
            const std::uint8_t mask = static_cast<std::uint8_t>(
                extract(header.stream_index + 1U));
            if (index < header.presentation_asset_masks.size()) {
                header.presentation_asset_masks[index] = mask;
            }
        }
        for (std::uint32_t presentation_index = 0;
             presentation_index < header.waveform_count;
             ++presentation_index) {
            const std::uint8_t mask = presentation_index
                    < header.presentation_asset_masks.size()
                ? header.presentation_asset_masks[presentation_index]
                : 0U;
            for (std::uint32_t stream_index = 0;
                 stream_index <= header.stream_index;
                 ++stream_index) {
                if (((mask >> stream_index) & 1U) != 0U) {
                    (void)extract(8U);
                }
            }
        }
        header.speaker_metadata_present =
            static_cast<std::uint8_t>(extract(1U));
        if (header.speaker_metadata_present != 0U) {
            skip(2U);
            header.speaker_mask_bits = extract(2U);
            header.speaker_mask_count = extract(2U) + 1U;
            for (std::uint32_t index = 0;
                 index < header.speaker_mask_count;
                 ++index) {
                const std::uint32_t mask = extract(
                    4U * (header.speaker_mask_bits + 1U));
                if (index < header.speaker_counts.size()) {
                    header.speaker_counts[index] = static_cast<std::uint8_t>(
                        speaker_count_from_activity_mask(mask));
                }
            }
        }
    }

    for (std::uint32_t index = 0; index < header.asset_count; ++index) {
        const std::uint32_t asset_size =
            extract(header.size_field_bits) + 1U;
        if (index < header.asset_sizes.size()) {
            header.asset_sizes[index] = asset_size;
        }
        header.total_asset_bytes += asset_size;
    }
    if (!valid
        || header.total_asset_bytes > header.frame_size - header.header_size) {
        return false;
    }

    for (std::uint32_t index = 0; index < header.asset_count; ++index) {
        bitstream::Cursor asset_source = source;
        if (index < header.asset_header_bit_offsets.size()) {
            header.asset_header_bit_offsets[index] =
                header_bits - source.remaining_bits();
        }
        if (asset_source.remaining_bits() < 9U) {
            return false;
        }
        const std::uint32_t asset_header_size =
            asset_source.extract_unsigned(9U) + 1U;
        const std::uint32_t asset_header_bits = asset_header_size << 3U;
        if (asset_header_bits > source.remaining_bits()) {
            return false;
        }
        asset_source = source.limited(asset_header_bits);
        if (asset_source.remaining_bits() < 12U) {
            return false;
        }
        (void)asset_source.extract_unsigned(9U);
        const std::uint8_t asset_type = static_cast<std::uint8_t>(
            asset_source.extract_unsigned(3U));
        std::uint8_t object_audio = 0U;
        bool asset_valid = true;
        const auto asset_extract =
            [&asset_source, &asset_valid](std::uint32_t bit_count) noexcept {
                if (asset_source.remaining_bits() < bit_count) {
                    asset_valid = false;
                    return 0U;
                }
                return asset_source.extract_unsigned(bit_count);
            };
        const auto asset_skip =
            [&asset_source, &asset_valid](std::uint32_t bit_count) noexcept {
                if (asset_source.remaining_bits() < bit_count) {
                    asset_valid = false;
                    return;
                }
                asset_source.fast_forward(
                    static_cast<std::int32_t>(bit_count));
            };
        if (presentation) {
            if (asset_extract(1U) != 0U) {
                asset_skip(4U);
            }
            if (asset_extract(1U) != 0U) {
                asset_skip(24U);
            }
            if (asset_extract(1U) != 0U) {
                const std::uint32_t bytes =
                    asset_extract(10U) + 1U;
                asset_skip(bytes << 3U);
            }
            asset_skip(17U);
            if (asset_extract(1U) == 0U
                && asset_extract(3U) == 1U) {
                object_audio = 1U;
            }
        }
        if (!asset_valid) {
            return false;
        }
        if (index < header.asset_header_sizes.size()) {
            header.asset_header_sizes[index] = asset_header_size;
            header.asset_types[index] = asset_type;
            header.asset_object_audio[index] = object_audio;
        }
        skip(asset_header_bits);
    }

    if (header.waveform_count != 0U) {
        for (std::uint32_t index = 0;
             index < header.waveform_count;
             ++index) {
            const std::uint8_t present = static_cast<std::uint8_t>(
                extract(1U));
            if (index < header.waveform_present.size()) {
                header.waveform_present[index] = present;
            }
        }
        for (std::uint32_t index = 0;
             index < header.waveform_count;
             ++index) {
            const bool present = index < header.waveform_present.size()
                && header.waveform_present[index] != 0U;
            if (!present) {
                continue;
            }
            if (index == selected_waveform_index) {
                header.selected_waveform = 1U;
                header.waveform_metadata_mode =
                    static_cast<std::uint8_t>(extract(2U));
                header.waveform_metadata_type =
                    static_cast<std::uint8_t>(extract(3U));
            } else {
                skip(5U);
            }
        }
    }
    if (!valid) {
        return false;
    }

    source = frame_start;
    source.fast_forward(
        static_cast<std::int32_t>(header.frame_size << 3U));
    return true;
}

bool validate_exss_frame(bitstream::Cursor source) noexcept {
    if (source.remaining_bits() < 32U
        || source.extract_unsigned(32U) != 0x64582025U) {
        return false;
    }
    if (source.remaining_bits() < 8U + 2U + 1U) {
        return false;
    }
    source.fast_forward(8);
    const bitstream::Cursor crc_start = source;
    (void)source.extract_unsigned(2U);
    const bool long_size = source.extract_unsigned(1U) != 0U;
    const std::uint32_t size_bits = long_size ? 12U : 8U;
    if (source.remaining_bits() < size_bits + (long_size ? 20U : 16U)) {
        return false;
    }
    const std::uint32_t raw_size = source.extract_unsigned(size_bits);
    const std::uint32_t raw_frame_size =
        source.extract_unsigned(long_size ? 20U : 16U);
    if (raw_size < 4U || raw_size + 1U > raw_frame_size + 1U
        || raw_size + 1U > crc_start.remaining_bits() / 8U) {
        return false;
    }
    bitstream::Cursor crc_source = crc_start;
    return valid_crc16(crc_source, (raw_size - 4U) * 8U);
}

} // namespace dtsx
