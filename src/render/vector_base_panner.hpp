#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace dtsx_decode {

struct PannerVector final {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct PannerTriplet final {
    std::array<std::uint32_t, 3> destination_channels{};
    std::array<PannerVector, 3> speakers{};
    std::array<std::array<float, 3>, 3> inverse{};
};

enum class PannerNormalization {
    Unnormalized,
    ConstantAmplitude,
    ConstantPower,
};

[[nodiscard]] PannerVector panner_vector_from_degrees(
    float azimuth_degrees,
    float elevation_degrees) noexcept;

[[nodiscard]] bool make_panner_triplet(
    const std::array<PannerVector, 3>& speakers,
    const std::array<std::uint32_t, 3>& destination_channels,
    PannerTriplet& triplet) noexcept;

[[nodiscard]] bool pan_point_source(
    const PannerVector& source,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    PannerNormalization normalization,
    std::vector<float>& gains) noexcept;

[[nodiscard]] bool pan_point_source_power(
    const PannerVector& source,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    std::vector<float>& gains) noexcept;

[[nodiscard]] bool pan_extended_source(
    float azimuth_degrees,
    float elevation_degrees,
    float width_degrees,
    float height_degrees,
    float rotation_degrees,
    std::uint32_t destination_channel_count,
    const std::vector<PannerTriplet>& triplets,
    float hull_epsilon,
    PannerNormalization normalization,
    std::vector<float>& gains) noexcept;

[[nodiscard]] bool quantize_panner_gains(
    const std::vector<float>& gains,
    std::uint8_t fractional_bits,
    std::vector<std::int32_t>& quantized) noexcept;

} // namespace dtsx_decode
