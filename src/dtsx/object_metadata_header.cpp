#include "dtsx/object_metadata_header.hpp"

namespace dtsx {

ObjectMetadataPreamble unpack_object_metadata_preamble(
    bitstream::Cursor& source, std::uint8_t waveform_count_bits) {
    ObjectMetadataPreamble result;
    result.metadata_mode =
        static_cast<std::uint8_t>(source.extract_unsigned(2U));
    result.waveform_count = static_cast<std::uint8_t>(
        source.extract_unsigned(waveform_count_bits) + 1U);

    result.waveform_types.reserve(result.waveform_count);
    for (std::uint32_t index = 0; index < result.waveform_count; ++index) {
        result.waveform_types.push_back(
            static_cast<std::uint8_t>(source.extract_unsigned(2U)));
    }

    if (result.metadata_mode <= 1U) {
        result.group_index_present = source.extract_unsigned(1U) != 0U;
        if (result.group_index_present) {
            result.group_index =
                static_cast<std::uint8_t>(source.extract_unsigned(4U));
        }

        result.extent_mode =
            static_cast<std::uint8_t>(source.extract_unsigned(3U));
        if (result.extent_mode > 3U) {
            result.extent_value_bits =
                static_cast<std::uint8_t>(source.extract_unsigned(6U));
        }

        result.point_source_count_by_waveform.reserve(result.waveform_count);
        for (std::uint32_t index = 0; index < result.waveform_count; ++index) {
            result.point_source_count_by_waveform.push_back(
                static_cast<std::uint8_t>(source.extract_unsigned(3U) + 1U));
        }
    }
    return result;
}

ObjectSpatialHeader unpack_object_spatial_header(
    bitstream::Cursor& source, const ObjectMetadataPreamble& preamble) {
    ObjectSpatialHeader result;
    result.parser_config.extent_mode = preamble.extent_mode;
    result.parser_config.extent_value_bits = preamble.extent_value_bits;
    result.parser_config.point_source_count_by_waveform =
        preamble.point_source_count_by_waveform;

    result.parser_config.fixed_distance = source.extract_unsigned(1U) != 0U;
    result.parser_config.elevation_present = source.extract_unsigned(1U) != 0U;
    result.flag_at_620 = source.extract_unsigned(1U) != 0U;
    result.parser_config.snap_flag_present =
        source.extract_unsigned(1U) != 0U;
    if (result.parser_config.snap_flag_present) {
        const std::uint32_t angle = 12U * source.extract_unsigned(5U);
        result.snap_tolerance_degrees =
            angle >= 360U ? 360U : angle;
        result.snap_tolerance_present = true;
    }

    result.flag_at_632 = source.extract_unsigned(1U) != 0U;
    result.flag_at_660 = source.extract_unsigned(1U) != 0U;
    if (preamble.extent_mode != 0U) {
        result.flag_at_664 = source.extract_unsigned(1U) != 0U;
        result.parser_config.copy_extent_from_first_source =
            source.extract_unsigned(1U) != 0U;
    }

    result.gain_present = source.extract_unsigned(1U) != 0U;
    if (result.gain_present) {
        result.gain_exponent =
            static_cast<std::uint8_t>(source.extract_unsigned(2U));
        result.gain_code =
            static_cast<std::uint8_t>(source.extract_unsigned(6U));
    }
    return result;
}

} // namespace dtsx
