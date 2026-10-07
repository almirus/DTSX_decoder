#include "dtsx/object_metadata_block.hpp"

namespace dtsx {

bool unpack_object_metadata_bodies(
    bitstream::Cursor& source,
    bool object_groups_present,
    std::uint8_t update_value_count,
    bool sequential_waveform_offsets,
    std::uint8_t waveform_offset_bits,
    std::vector<ObjectMetadataBlock>& blocks) {
    for (ObjectMetadataBlock& block : blocks) {
        if (!block.metadata_present) {
            continue;
        }

        (void)object_groups_present;
        block.spatial_group_present =
            source.extract_unsigned(1U) != 0U;
        block.spatial_group = block.spatial_group_present
            ? static_cast<std::uint8_t>(source.extract_unsigned(4U))
            : static_cast<std::uint8_t>(15U);

        block.inter_object_metadata_present =
            source.extract_unsigned(1U) != 0U;
        block.flag_at_580 =
            source.extract_unsigned(1U) != 0U;

        block.waveform_channel_offsets.assign(
            block.preamble.waveform_count, 0U);
        if (sequential_waveform_offsets) {
            for (std::uint32_t waveform = 0;
                 waveform < block.preamble.waveform_count;
                 ++waveform) {
                block.waveform_channel_offsets[waveform] =
                    static_cast<std::uint8_t>(waveform);
            }
        } else {
            for (std::uint32_t waveform = 1;
                 waveform < block.preamble.waveform_count;
                 ++waveform) {
                block.waveform_channel_offsets[waveform] =
                    static_cast<std::uint8_t>(
                        source.extract_unsigned(
                            waveform_offset_bits)
                        + 1U);
            }
        }

        const std::uint8_t mode = block.preamble.metadata_mode;
        if (mode <= 1U) {
            block.spatial_header =
                unpack_object_spatial_header(source, block.preamble);
            if (!unpack_spatial_metadata(
                    source, block.spatial_header.parser_config, block.points)) {
                return false;
            }
        }

        if (mode == 1U || mode == 2U) {
            block.updates = unpack_waveform_metadata_updates(
                source, block.preamble.waveform_count, update_value_count);
        } else if (mode == 3U) {
            block.mode_three = unpack_mode_three_metadata(
                source, block.preamble.waveform_count, update_value_count);
        }
        block.body_parsed = true;
    }
    return source.valid();
}

} // namespace dtsx
