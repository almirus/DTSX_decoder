#pragma once

#include <cstdint>

namespace dtsx {

struct RendererCoordinates final {
    std::uint32_t coordinate_system = 10;
    float azimuth_degrees = 0.0F;
    float elevation_degrees = 0.0F;
    float distance = 0.0F;
};

struct ExtendedSourceGeometry final {
    RendererCoordinates coordinates;
    float width_degrees = 0.0F;
    float height_degrees = 0.0F;
    float rotation_degrees = 0.0F;
};

[[nodiscard]] RendererCoordinates decode_renderer_coordinates(
    std::int32_t azimuth_code,
    std::int32_t elevation_code,
    std::uint32_t distance_code) noexcept;

[[nodiscard]] ExtendedSourceGeometry decode_extended_source_geometry(
    std::int32_t azimuth_code,
    std::int32_t elevation_code,
    std::uint32_t distance_code,
    std::uint32_t width_code,
    std::uint32_t height_code,
    std::int32_t rotation_code) noexcept;

} // namespace dtsx
