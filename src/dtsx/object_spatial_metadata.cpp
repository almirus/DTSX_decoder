#include "dtsx/object_spatial_metadata.hpp"

#include <cstddef>
#include <limits>

namespace dtsx {
namespace {

std::int32_t unpack_azimuth(bitstream::Cursor& source) noexcept {
    const std::int32_t tripled =
        3 * static_cast<std::int32_t>(source.extract_unsigned(8U));
    if (tripled == 579) {
        return 220;
    }
    if (tripled == 141) {
        return -220;
    }

    const std::int32_t centered = tripled - 360;
    return centered >= 357 ? 357 : centered;
}

std::int32_t unpack_elevation(bitstream::Cursor& source) noexcept {
    const std::int32_t centered =
        3 * static_cast<std::int32_t>(source.extract_unsigned(7U)) - 180;
    return centered >= 180 ? 180 : centered;
}

std::uint32_t unpack_extent_angle(bitstream::Cursor& source) noexcept {
    const std::uint32_t code = source.extract_unsigned(6U);
    const std::uint32_t angle = code > 6U ? 6U * code - 18U : 3U * code;
    return angle >= 360U ? 360U : angle;
}

std::int32_t unpack_rotation(bitstream::Cursor& source) noexcept {
    const std::int32_t rotation =
        3 * static_cast<std::int32_t>(source.extract_unsigned(6U)) - 90;
    return rotation >= 90 ? 90 : rotation;
}

} // namespace

bool point_source_is_renderable(
    std::uint8_t metadata_mode,
    const PointSourceMetadata& point) noexcept {
    // libdtsx(v2).so.c: registerObject_part_0. Standard renderer mode 0
    // excludes a mode-0/1 source only when it is neither marked coherent
    // nor fixed at the renderer reference distance, or when its source type
    // is 2/3. Renderer modes that consume channel metadata accept all
    // registered sources.
    if (metadata_mode > 1U) {
        return true;
    }
    return (point.coherent_rendering || point.distance_code == 64U)
        && point.source_type <= 1U;
}

bool unpack_spatial_metadata(bitstream::Cursor& source,
                             const SpatialMetadataConfig& config,
                             std::vector<PointSourceMetadata>& points) {
    // libdtsx.so: dtsParseExSSChunks, 0xa15bc..0xa199c.
    std::size_t total_points = 0;
    for (const std::uint8_t count : config.point_source_count_by_waveform) {
        total_points += count;
    }
    if (total_points > 32U
        || total_points > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
        return false;
    }

    points.clear();
    points.reserve(total_points);
    bool have_extent_template = false;
    std::size_t extent_template_index = 0;

    for (std::size_t waveform = 0;
         waveform < config.point_source_count_by_waveform.size();
         ++waveform) {
        const bool per_source_gain = source.extract_unsigned(1U) != 0U;
        const std::uint8_t source_count =
            config.point_source_count_by_waveform[waveform];
        for (std::uint32_t local_index = 0; local_index < source_count; ++local_index) {
            PointSourceMetadata point;
            point.waveform_index = static_cast<std::uint32_t>(waveform);
            point.point_source_index = static_cast<std::uint32_t>(points.size());
            point.coherent_rendering = source.extract_unsigned(1U) != 0U;
            point.source_type = static_cast<std::uint8_t>(source.extract_unsigned(2U));
            point.gain_code = per_source_gain
                ? static_cast<std::uint8_t>(source.extract_unsigned(6U))
                : static_cast<std::uint8_t>(61U);

            if (config.fixed_distance) {
                point.distance_code = 64U;
            } else {
                const std::uint32_t distance = source.extract_unsigned(6U);
                point.distance_code = distance == 0U ? 0U : distance + 1U;
            }
            if (config.snap_flag_present) {
                point.snap_to_nearest_speaker =
                    source.extract_unsigned(1U) != 0U;
            }

            point.azimuth_code = unpack_azimuth(source);
            point.elevation_code =
                config.elevation_present ? unpack_elevation(source) : 0;
            point.extended = source.extract_unsigned(1U) != 0U;

            if (point.extended) {
                const bool used_extent_template = have_extent_template;
                if (have_extent_template) {
                    const PointSourceMetadata& extent_template =
                        points[extent_template_index];
                    point.width_degrees = extent_template.width_degrees;
                    point.height_degrees = extent_template.height_degrees;
                    point.rotation_degrees = extent_template.rotation_degrees;
                } else if (config.extent_mode < 1U || config.extent_mode > 3U) {
                    (void)source.extract_unsigned(config.extent_value_bits);
                } else {
                    point.width_degrees = unpack_extent_angle(source);
                    if (config.extent_mode > 1U) {
                        point.height_degrees = unpack_extent_angle(source);
                        if (config.extent_mode > 2U
                            && (point.width_degrees != 0U
                                || point.height_degrees != 0U)) {
                            point.rotation_degrees = unpack_rotation(source);
                        }
                    }
                }

                if (!used_extent_template && config.copy_extent_from_first_source) {
                    if (waveform == 0U) {
                        extent_template_index = points.size();
                    }
                    have_extent_template = waveform == 0U;
                }
            }

            point.coordinates = decode_renderer_coordinates(
                point.azimuth_code, point.elevation_code, point.distance_code);
            points.push_back(point);
        }
    }
    return true;
}

} // namespace dtsx
