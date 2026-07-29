#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/object_coordinates.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct SpatialMetadataConfig final {
    bool fixed_distance = false;
    bool elevation_present = false;
    bool snap_flag_present = false;
    bool copy_extent_from_first_source = false;
    std::uint8_t extent_mode = 0;
    std::uint8_t extent_value_bits = 0;
    std::vector<std::uint8_t> point_source_count_by_waveform;
};

struct PointSourceMetadata final {
    std::uint32_t waveform_index = 0;
    std::uint32_t point_source_index = 0;
    bool coherent_rendering = false;
    std::uint8_t source_type = 0;
    std::uint8_t gain_code = 61;
    std::uint32_t distance_code = 0;
    std::int32_t azimuth_code = 0;
    std::int32_t elevation_code = 0;
    bool snap_to_nearest_speaker = false;
    bool extended = false;
    RendererCoordinates coordinates;
    std::uint32_t width_degrees = 0;
    std::uint32_t height_degrees = 0;
    std::int32_t rotation_degrees = 0;
};

[[nodiscard]] bool point_source_is_renderable(
    std::uint8_t metadata_mode,
    const PointSourceMetadata& point) noexcept;

[[nodiscard]] bool unpack_spatial_metadata(bitstream::Cursor& source,
                                           const SpatialMetadataConfig& config,
                                           std::vector<PointSourceMetadata>& points);

} // namespace dtsx
