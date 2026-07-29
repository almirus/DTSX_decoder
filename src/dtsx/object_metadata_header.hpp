#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/object_spatial_metadata.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct ObjectMetadataPreamble final {
    std::uint8_t metadata_mode = 0;
    std::uint8_t waveform_count = 0;
    std::vector<std::uint8_t> waveform_types;
    bool group_index_present = false;
    std::uint8_t group_index = 0;
    std::uint8_t extent_mode = 0;
    std::uint8_t extent_value_bits = 0;
    std::vector<std::uint8_t> point_source_count_by_waveform;
};

struct ObjectSpatialHeader final {
    SpatialMetadataConfig parser_config;
    bool flag_at_620 = false;
    bool snap_tolerance_present = false;
    std::uint32_t snap_tolerance_degrees = 0;
    bool flag_at_632 = false;
    bool flag_at_660 = false;
    bool flag_at_664 = false;
    bool gain_present = false;
    std::uint8_t gain_exponent = 0;
    std::uint8_t gain_code = 61;
};

[[nodiscard]] ObjectMetadataPreamble unpack_object_metadata_preamble(
    bitstream::Cursor& source,
    std::uint8_t waveform_count_bits);

[[nodiscard]] ObjectSpatialHeader unpack_object_spatial_header(
    bitstream::Cursor& source,
    const ObjectMetadataPreamble& preamble);

} // namespace dtsx
