#include "render/vector_base_panner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace dtsx_decode {
namespace {

constexpr float kPi = 3.14159265358979323846F;

float determinant(const std::array<PannerVector, 3>& columns) noexcept {
    return columns[0].x
            * (columns[1].y * columns[2].z
               - columns[2].y * columns[1].z)
        - columns[1].x
            * (columns[0].y * columns[2].z
               - columns[2].y * columns[0].z)
        + columns[2].x
            * (columns[0].y * columns[1].z
               - columns[1].y * columns[0].z);
}

PannerVector cross(
    const PannerVector& left,
    const PannerVector& right) noexcept {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

float dot(
    const PannerVector& left,
    const PannerVector& right) noexcept {
    return left.x * right.x
        + left.y * right.y
        + left.z * right.z;
}

float length(const PannerVector& value) noexcept {
    return std::sqrt(dot(value, value));
}

PannerVector normalize(PannerVector value) noexcept {
    const float magnitude = length(value);
    if (magnitude > 0.0F) {
        value.x /= magnitude;
        value.y /= magnitude;
        value.z /= magnitude;
    }
    return value;
}

PannerVector rotate_extended_corner(
    const PannerVector& corner,
    float azimuth_degrees,
    float elevation_degrees,
    float rotation_degrees) noexcept {
    const float azimuth = azimuth_degrees * (kPi / 180.0F);
    const float elevation = elevation_degrees * (kPi / 180.0F);
    const float rotation = rotation_degrees * (kPi / 180.0F);
    const float cy = std::cos(azimuth);
    const float sy = std::sin(azimuth);
    const float cx = std::cos(elevation);
    const float sx = std::sin(elevation);
    const float cz = std::cos(rotation);
    const float sz = std::sin(rotation);
    const PannerVector about_z = {
        cz * corner.x - sz * corner.y,
        sz * corner.x + cz * corner.y,
        corner.z,
    };
    const PannerVector about_x = {
        about_z.x,
        cx * about_z.y - sx * about_z.z,
        sx * about_z.y + cx * about_z.z,
    };
    return {
        cy * about_x.x + sy * about_x.z,
        about_x.y,
        -sy * about_x.x + cy * about_x.z,
    };
}

bool pan_point_source_raw(
    const PannerVector& source,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    bool average_overlapping_triplets,
    std::vector<float>& gains,
    float& accumulated_weight) noexcept {
    gains.assign(destination_channel_count, 0.0F);
    accumulated_weight = 0.0F;
    for (const PannerTriplet& triplet : triplets) {
        std::array<float, 3> triplet_gains{};
        bool inside = true;
        std::uint32_t positive_count = 0U;
        for (std::size_t row = 0U; row < 3U; ++row) {
            triplet_gains[row] =
                triplet.inverse[row][0] * source.x
                + triplet.inverse[row][1] * source.y
                + triplet.inverse[row][2] * source.z;
            inside =
                inside && triplet_gains[row] >= -hull_epsilon;
            positive_count +=
                triplet_gains[row] > hull_epsilon ? 1U : 0U;
            if (triplet.destination_channels[row]
                >= destination_channel_count) {
                gains.clear();
                return false;
            }
        }
        if (!inside) {
            continue;
        }
        const float weight =
            positive_count == 2U ? 0.5F : 1.0F;
        accumulated_weight += weight;
        for (std::size_t row = 0U; row < 3U; ++row) {
            gains[triplet.destination_channels[row]] +=
                triplet_gains[row] * weight;
        }
    }
    if (accumulated_weight <= 0.0F) {
        gains.clear();
        return false;
    }
    if (average_overlapping_triplets
        && accumulated_weight > 1.0F) {
        for (float& gain : gains) {
            gain /= accumulated_weight;
        }
    }
    return true;
}

bool normalize_panner_gains(
    PannerNormalization normalization,
    std::vector<float>& gains) noexcept {
    if (normalization == PannerNormalization::Unnormalized) {
        return true;
    }
    float normalizer = 0.0F;
    if (normalization == PannerNormalization::ConstantPower) {
        for (const float gain : gains) {
            normalizer += gain * gain;
        }
        normalizer = std::sqrt(normalizer);
    } else {
        for (const float gain : gains) {
            normalizer += std::fabs(gain);
        }
    }
    if (normalizer <= 0.0F) {
        gains.clear();
        return false;
    }
    for (float& gain : gains) {
        gain = std::max(
            0.0F, std::min(1.0F, gain / normalizer));
    }
    return true;
}

bool convert_power_gains_to_amplitude(
    std::vector<float>& gains) noexcept {
    for (float& gain : gains) {
        if (gain < -1.0e-6F || !std::isfinite(gain)) {
            gains.clear();
            return false;
        }
        gain = std::sqrt(std::max(0.0F, gain));
    }
    return true;
}

bool append_polygon_vertex(
    std::vector<PannerVector>& polygon,
    const PannerVector& vertex) {
    // libdtsx.so uses a fixed 64-vertex work buffer.
    if (polygon.size() >= 64U) {
        return false;
    }
    polygon.push_back(vertex);
    return true;
}

bool clip_spherical_polygon(
    const PannerVector& plane_normal,
    float threshold,
    std::vector<PannerVector>& polygon) {
    if (polygon.empty()) {
        return true;
    }
    std::vector<PannerVector> clipped;
    clipped.reserve(std::min<std::size_t>(
        64U, polygon.size() + 2U));
    for (std::size_t index = 0U;
         index < polygon.size();
         ++index) {
        const PannerVector& previous = polygon[index];
        const PannerVector& current =
            polygon[(index + 1U) % polygon.size()];
        const float previous_distance =
            dot(plane_normal, previous);
        const float current_distance =
            dot(plane_normal, current);
        const bool previous_inside =
            previous_distance >= threshold;
        const bool current_inside =
            current_distance >= threshold;
        const PannerVector edge_normal =
            cross(previous, current);

        if (previous_inside && !current_inside) {
            if (!append_polygon_vertex(clipped, previous)
                || !append_polygon_vertex(
                    clipped,
                    cross(edge_normal, plane_normal))) {
                return false;
            }
        } else if (!previous_inside && current_inside) {
            if (!append_polygon_vertex(
                    clipped,
                    cross(plane_normal, edge_normal))) {
                return false;
            }
        } else if (previous_inside) {
            if (!append_polygon_vertex(clipped, previous)) {
                return false;
            }
        }
    }
    polygon = std::move(clipped);
    return true;
}

bool spherical_polygon_vector(
    std::vector<PannerVector> polygon,
    PannerVector& vector) noexcept {
    if (polygon.size() < 3U) {
        return false;
    }
    for (PannerVector& vertex : polygon) {
        vertex = normalize(vertex);
        if (length(vertex) == 0.0F) {
            return false;
        }
    }
    vector = {};
    for (std::size_t index = 0U;
         index < polygon.size();
         ++index) {
        const PannerVector& first = polygon[index];
        const PannerVector& second =
            polygon[(index + 1U) % polygon.size()];
        const float angle = std::acos(std::max(
            -1.0F, std::min(1.0F, dot(first, second))));
        const float sinc =
            angle == 0.0F ? 1.0F : std::sin(angle) / angle;
        if (std::fabs(sinc)
            <= std::numeric_limits<float>::epsilon()) {
            return false;
        }
        const PannerVector edge = cross(first, second);
        vector.x += edge.x / sinc;
        vector.y += edge.y / sinc;
        vector.z += edge.z / sinc;
    }
    vector.x *= 0.5F;
    vector.y *= 0.5F;
    vector.z *= 0.5F;
    return true;
}

bool pan_spherical_polygon_raw(
    const std::vector<PannerVector>& source_polygon,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    std::vector<float>& gains) noexcept {
    gains.assign(destination_channel_count, 0.0F);
    bool contributed = false;
    for (const PannerTriplet& clipping_triplet : triplets) {
        std::vector<PannerVector> clipped = source_polygon;
        for (std::size_t edge = 0U; edge < 3U; ++edge) {
            PannerVector plane_normal = cross(
                clipping_triplet.speakers[edge],
                clipping_triplet.speakers[
                    (edge + 1U) % 3U]);
            if (dot(
                    plane_normal,
                    clipping_triplet.speakers[
                        (edge + 2U) % 3U])
                < 0.0F) {
                plane_normal.x = -plane_normal.x;
                plane_normal.y = -plane_normal.y;
                plane_normal.z = -plane_normal.z;
            }
            if (!clip_spherical_polygon(
                    plane_normal, -hull_epsilon, clipped)) {
                gains.clear();
                return false;
            }
            if (clipped.empty()) {
                break;
            }
        }
        if (clipped.empty()) {
            continue;
        }
        PannerVector area_vector;
        if (!spherical_polygon_vector(
                std::move(clipped), area_vector)) {
            gains.clear();
            return false;
        }
        if (length(area_vector) > 1.0F + hull_epsilon) {
            continue;
        }
        std::vector<float> polygon_gains;
        float point_weight = 0.0F;
        if (!pan_point_source_raw(
                area_vector,
                destination_channel_count,
                triplets,
                hull_epsilon,
                false,
                polygon_gains,
                point_weight)) {
            gains.clear();
            return false;
        }
        for (std::size_t channel = 0U;
             channel < gains.size();
             ++channel) {
            gains[channel] += polygon_gains[channel];
        }
        contributed = true;
    }
    if (!contributed) {
        gains.clear();
    }
    return contributed;
}

PannerVector spherical_line_intersection(
    const PannerVector& first,
    const PannerVector& second,
    const PannerVector& plane_normal,
    bool first_is_inside) noexcept {
    const PannerVector edge_normal = cross(first, second);
    return first_is_inside
        ? cross(edge_normal, plane_normal)
        : cross(plane_normal, edge_normal);
}

bool pan_spherical_line_raw(
    std::array<PannerVector, 2> source_line,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    std::vector<float>& gains) noexcept {
    gains.assign(destination_channel_count, 0.0F);
    bool contributed = false;
    for (const PannerTriplet& clipping_triplet : triplets) {
        std::vector<PannerVector> clipped{
            source_line[0], source_line[1]};
        for (std::size_t edge = 0U;
             edge < 3U && clipped.size() == 2U;
             ++edge) {
            PannerVector plane_normal = cross(
                clipping_triplet.speakers[edge],
                clipping_triplet.speakers[
                    (edge + 1U) % 3U]);
            if (dot(
                    plane_normal,
                    clipping_triplet.speakers[
                        (edge + 2U) % 3U])
                < 0.0F) {
                plane_normal.x = -plane_normal.x;
                plane_normal.y = -plane_normal.y;
                plane_normal.z = -plane_normal.z;
            }
            const float first_distance =
                dot(plane_normal, clipped[0]);
            const float second_distance =
                dot(plane_normal, clipped[1]);
            const bool first_inside =
                first_distance >= -hull_epsilon;
            const bool second_inside =
                second_distance >= -hull_epsilon;
            if (!first_inside && !second_inside) {
                clipped.clear();
            } else if (first_inside != second_inside) {
                const PannerVector intersection =
                    spherical_line_intersection(
                        clipped[0],
                        clipped[1],
                        plane_normal,
                        first_inside);
                if (first_inside) {
                    clipped[1] = intersection;
                } else {
                    clipped[0] = intersection;
                }
            }
        }
        if (clipped.size() != 2U) {
            continue;
        }
        const PannerVector first = normalize(clipped[0]);
        const PannerVector second = normalize(clipped[1]);
        const float endpoint_dot = std::max(
            -1.0F, std::min(1.0F, dot(first, second)));
        const float denominator = endpoint_dot + 1.0F;
        if (denominator <= 0.0F) {
            gains.clear();
            return false;
        }
        const float scale = std::sqrt(
            std::max(0.0F, (1.0F - endpoint_dot) / denominator));
        const PannerVector line_vector = {
            (first.x + second.x) * scale,
            (first.y + second.y) * scale,
            (first.z + second.z) * scale,
        };
        std::vector<float> line_gains;
        float point_weight = 0.0F;
        if (!pan_point_source_raw(
                line_vector,
                destination_channel_count,
                triplets,
                hull_epsilon,
                false,
                line_gains,
                point_weight)) {
            gains.clear();
            return false;
        }
        for (std::size_t channel = 0U;
             channel < gains.size();
             ++channel) {
            gains[channel] += line_gains[channel];
        }
        contributed = true;
    }
    if (!contributed) {
        gains.clear();
    }
    return contributed;
}

std::array<PannerVector, 4> make_extended_polygon(
    float azimuth_degrees,
    float elevation_degrees,
    float width_degrees,
    float height_degrees,
    float rotation_degrees) noexcept {
    const float half_width =
        width_degrees * (kPi / 360.0F);
    const float half_height =
        height_degrees * (kPi / 360.0F);
    const PannerVector horizontal = {
        -std::cos(half_width),
        0.0F,
        -std::sin(half_width),
    };
    const PannerVector vertical = {
        0.0F,
        -std::cos(half_height),
        -std::sin(half_height),
    };
    const PannerVector normal =
        normalize(cross(vertical, horizontal));
    std::array<PannerVector, 4> polygon = {{
        {normal.x, normal.y, normal.z},
        {normal.x, -normal.y, normal.z},
        {-normal.x, -normal.y, normal.z},
        {-normal.x, normal.y, normal.z},
    }};
    for (PannerVector& corner : polygon) {
        corner = rotate_extended_corner(
            corner,
            azimuth_degrees,
            elevation_degrees,
            rotation_degrees);
    }
    return polygon;
}

bool pan_extended_polygon_raw(
    float azimuth_degrees,
    float elevation_degrees,
    float width_degrees,
    float height_degrees,
    float rotation_degrees,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    std::vector<float>& gains) noexcept {
    const std::array<PannerVector, 4> polygon =
        make_extended_polygon(
            azimuth_degrees,
            elevation_degrees,
            width_degrees,
            height_degrees,
            rotation_degrees);
    return pan_spherical_polygon_raw(
        {polygon.begin(), polygon.end()},
        destination_channel_count,
        triplets,
        hull_epsilon,
        gains);
}

bool pan_extended_line_raw(
    float azimuth_degrees,
    float elevation_degrees,
    float width_degrees,
    float height_degrees,
    float rotation_degrees,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    std::vector<float>& gains) noexcept {
    const std::array<PannerVector, 4> polygon =
        make_extended_polygon(
            azimuth_degrees,
            elevation_degrees,
            width_degrees,
            height_degrees,
            rotation_degrees);
    const std::array<PannerVector, 2> line =
        height_degrees == 0.0F
        ? std::array<PannerVector, 2>{
              polygon[0], polygon[2]}
        : std::array<PannerVector, 2>{
              polygon[0], polygon[1]};
    return pan_spherical_line_raw(
        line,
        destination_channel_count,
        triplets,
        hull_epsilon,
        gains);
}

} // namespace

PannerVector panner_vector_from_degrees(
    float azimuth_degrees,
    float elevation_degrees) noexcept {
    // libdtsx.so: sub_E7E40 and dts_base_vector3_f32_t_on_sphere_xz,
    // 0xe7e40 and 0xd8b80.
    const float azimuth = azimuth_degrees * (kPi / 180.0F);
    const float elevation = elevation_degrees * (kPi / 180.0F);
    const float cos_elevation = std::cos(elevation);
    return {
        std::sin(azimuth) * cos_elevation,
        std::sin(elevation),
        -std::cos(azimuth) * cos_elevation,
    };
}

bool make_panner_triplet(
    const std::array<PannerVector, 3>& speakers,
    const std::array<std::uint32_t, 3>& destination_channels,
    PannerTriplet& triplet) noexcept {
    const float value = determinant(speakers);
    if (std::fabs(value) <= std::numeric_limits<float>::epsilon()) {
        return false;
    }

    triplet.destination_channels = destination_channels;
    triplet.speakers = speakers;
    const float reciprocal = 1.0F / value;
    triplet.inverse = {{
        {{
            (speakers[1].y * speakers[2].z
             - speakers[2].y * speakers[1].z)
                * reciprocal,
            (speakers[2].x * speakers[1].z
             - speakers[1].x * speakers[2].z)
                * reciprocal,
            (speakers[1].x * speakers[2].y
             - speakers[2].x * speakers[1].y)
                * reciprocal,
        }},
        {{
            (speakers[2].y * speakers[0].z
             - speakers[0].y * speakers[2].z)
                * reciprocal,
            (speakers[0].x * speakers[2].z
             - speakers[2].x * speakers[0].z)
                * reciprocal,
            (speakers[2].x * speakers[0].y
             - speakers[0].x * speakers[2].y)
                * reciprocal,
        }},
        {{
            (speakers[0].y * speakers[1].z
             - speakers[1].y * speakers[0].z)
                * reciprocal,
            (speakers[1].x * speakers[0].z
             - speakers[0].x * speakers[1].z)
                * reciprocal,
            (speakers[0].x * speakers[1].y
             - speakers[1].x * speakers[0].y)
                * reciprocal,
        }},
    }};
    return true;
}

bool pan_point_source(
    const PannerVector& source,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    PannerNormalization normalization,
    std::vector<float>& gains) noexcept {
    // libdtsx.so: dts_3d_hull_f32_t_pan point-source path and sub_E7EBC,
    // 0xe9890 and 0xe7ebc.
    if (destination_channel_count == 0U || triplets.empty()
        || hull_epsilon < 1.0e-12F || hull_epsilon > 0.001F) {
        return false;
    }
    float accumulated_weight = 0.0F;
    if (!pan_point_source_raw(
            source,
            destination_channel_count,
            triplets,
            hull_epsilon,
            true,
            gains,
            accumulated_weight)) {
        return false;
    }
    // libdtsx.so: virtual custom/auto vector-base panners receive
    // normalization_mode == 1 from sub_5F12C and apply
    // dts_base_math_safe_sqrt_32f_vv to the hull gains before mapping
    // and final normalization (0x8f66c/0x87e14).
    if (!convert_power_gains_to_amplitude(gains)) {
        return false;
    }
    return normalize_panner_gains(normalization, gains);
}

bool pan_point_source_power(
    const PannerVector& source,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    std::vector<float>& gains) noexcept {
    if (destination_channel_count == 0U || triplets.empty()
        || hull_epsilon < 1.0e-12F || hull_epsilon > 0.001F) {
        return false;
    }
    float accumulated_weight = 0.0F;
    return pan_point_source_raw(
        source,
        destination_channel_count,
        triplets,
        hull_epsilon,
        true,
        gains,
        accumulated_weight);
}

bool pan_extended_source(
    float azimuth_degrees,
    float elevation_degrees,
    float width_degrees,
    float height_degrees,
    float rotation_degrees,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    PannerNormalization normalization,
    std::vector<float>& gains) noexcept {
    // libdtsx.so: sub_E8320 and sub_E8E68, 0xe8320/0xe8e68.
    if (width_degrees < 0.0F || width_degrees > 360.0F
        || height_degrees < 0.0F || height_degrees > 360.0F
        || elevation_degrees < -90.0F || elevation_degrees > 90.0F) {
        return false;
    }
    float native_width = width_degrees;
    float native_height = height_degrees;
    if (native_width >= 179.0F && native_width <= 180.0F) {
        native_width = 179.0F;
    } else if (native_width > 180.0F
               && native_width <= 181.0F) {
        native_width = 181.0F;
    }
    if (native_height >= 179.0F && native_height <= 180.0F) {
        native_height = 179.0F;
    } else if (native_height > 180.0F
               && native_height <= 181.0F) {
        native_height = 181.0F;
    }
    if (native_width > 180.0F && native_height <= 180.0F) {
        native_width = 179.0F;
    } else if (native_width <= 180.0F
               && native_height > 180.0F) {
        native_height = 179.0F;
    }

    if (native_width == 0.0F || native_height == 0.0F) {
        const float extent =
            native_width == 0.0F
            ? native_height
            : native_width;
        if (extent < 180.0F) {
            if (!pan_extended_line_raw(
                    azimuth_degrees,
                    elevation_degrees,
                    native_width,
                    native_height,
                    rotation_degrees,
                    destination_channel_count,
                    triplets,
                    hull_epsilon,
                    gains)) {
                return false;
            }
        } else {
            const bool horizontal = native_height == 0.0F;
            std::vector<float> first_half;
            std::vector<float> second_half;
            if (!pan_extended_line_raw(
                    azimuth_degrees,
                    elevation_degrees,
                    horizontal ? 179.0F : 0.0F,
                    horizontal ? 0.0F : 179.0F,
                    rotation_degrees,
                    destination_channel_count,
                    triplets,
                    hull_epsilon,
                    first_half)
                || !pan_extended_line_raw(
                    azimuth_degrees + 180.0F,
                    -elevation_degrees,
                    horizontal ? 179.0F : 0.0F,
                    horizontal ? 0.0F : 179.0F,
                    rotation_degrees,
                    destination_channel_count,
                    triplets,
                    hull_epsilon,
                    second_half)) {
                return false;
            }
            gains.resize(destination_channel_count);
            for (std::size_t channel = 0U;
                 channel < gains.size();
                 ++channel) {
                gains[channel] =
                    first_half[channel] + second_half[channel];
            }
            if (extent < 360.0F) {
                std::vector<float> complement;
                if (!pan_extended_line_raw(
                        azimuth_degrees + 180.0F,
                        -elevation_degrees,
                        horizontal ? 360.0F - extent : 0.0F,
                        horizontal ? 0.0F : 360.0F - extent,
                        rotation_degrees,
                        destination_channel_count,
                        triplets,
                        hull_epsilon,
                        complement)) {
                    return false;
                }
                for (std::size_t channel = 0U;
                     channel < gains.size();
                     ++channel) {
                    gains[channel] -= complement[channel];
                }
            }
        }
    } else if (native_width > 180.0F
               && native_height > 180.0F) {
        std::vector<float> first_hemisphere;
        std::vector<float> second_hemisphere;
        std::vector<float> complement;
        if (!pan_extended_polygon_raw(
                0.0F,
                0.0F,
                179.0F,
                179.0F,
                0.0F,
                destination_channel_count,
                triplets,
                hull_epsilon,
                first_hemisphere)
            || !pan_extended_polygon_raw(
                180.0F,
                0.0F,
                179.0F,
                179.0F,
                0.0F,
                destination_channel_count,
                triplets,
                hull_epsilon,
                second_hemisphere)
            || !pan_extended_polygon_raw(
                azimuth_degrees,
                elevation_degrees,
                native_width,
                native_height,
                rotation_degrees,
                destination_channel_count,
                triplets,
                hull_epsilon,
                complement)) {
            return false;
        }
        gains.resize(destination_channel_count);
        for (std::size_t channel = 0U;
             channel < gains.size();
             ++channel) {
            gains[channel] =
                first_hemisphere[channel]
                + second_hemisphere[channel]
                - complement[channel];
        }
    } else if (!pan_extended_polygon_raw(
                   azimuth_degrees,
                   elevation_degrees,
                   native_width,
                   native_height,
                   rotation_degrees,
                   destination_channel_count,
                   triplets,
                   hull_epsilon,
                   gains)) {
        return false;
    }
    if (!convert_power_gains_to_amplitude(gains)) {
        return false;
    }
    return normalize_panner_gains(normalization, gains);
}

bool quantize_panner_gains(
    const std::vector<float>& gains,
    std::uint8_t fractional_bits,
    std::vector<std::int32_t>& quantized) noexcept {
    // libdtsx.so: dts_base_math_convert_32f_to_32sq, 0xd5a18.
    if (fractional_bits > 30U) {
        return false;
    }
    const float scale =
        static_cast<float>(std::uint32_t{1} << fractional_bits);
    quantized.clear();
    quantized.reserve(gains.size());
    for (const float gain : gains) {
        const double value = std::round(
            static_cast<double>(gain) * scale);
        if (value < std::numeric_limits<std::int32_t>::min()
            || value > std::numeric_limits<std::int32_t>::max()) {
            quantized.clear();
            return false;
        }
        quantized.push_back(static_cast<std::int32_t>(value));
    }
    return true;
}

} // namespace dtsx_decode
