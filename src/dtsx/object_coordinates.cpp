#include "dtsx/object_coordinates.hpp"

namespace dtsx {

RendererCoordinates decode_renderer_coordinates(std::int32_t azimuth_code,
                                                 std::int32_t elevation_code,
                                                 std::uint32_t distance_code) noexcept {
    RendererCoordinates result;
    result.coordinate_system = 10U;
    result.azimuth_degrees = static_cast<float>(azimuth_code) * 0.5F;
    result.elevation_degrees = static_cast<float>(elevation_code) * 0.5F;
    result.distance = distance_code == 1U
        ? 0.0F
        : static_cast<float>(distance_code) * 0.015625F;
    return result;
}

ExtendedSourceGeometry decode_extended_source_geometry(
    std::int32_t azimuth_code,
    std::int32_t elevation_code,
    std::uint32_t distance_code,
    std::uint32_t width_code,
    std::uint32_t height_code,
    std::int32_t rotation_code) noexcept {
    ExtendedSourceGeometry result;
    result.coordinates =
        decode_renderer_coordinates(azimuth_code, elevation_code, distance_code);
    result.width_degrees = static_cast<float>(width_code);
    result.height_degrees = static_cast<float>(height_code);
    result.rotation_degrees = static_cast<float>(rotation_code);
    return result;
}

} // namespace dtsx
