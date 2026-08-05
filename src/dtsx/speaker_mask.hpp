#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

namespace dtsx {

[[nodiscard]] std::uint32_t speaker_count_from_activity_mask(
    std::uint32_t mask) noexcept;

[[nodiscard]] bool has_height_channels(std::uint32_t mask) noexcept;

[[nodiscard]] std::vector<std::uint32_t>
expand_speaker_activity_mask(std::uint32_t mask);

[[nodiscard]] std::uint32_t speaker_mask_to_activity_mask(
    std::uint32_t mask) noexcept;

[[nodiscard]] bool standard_speaker_coordinates(
    std::uint32_t speaker_mask,
    float& azimuth_degrees,
    float& elevation_degrees,
    std::string_view& name) noexcept;

[[nodiscard]] bool standard_speaker_name(
    std::uint32_t speaker_mask,
    std::string_view& name) noexcept;

[[nodiscard]] bool standard_speaker_mask(
    std::string_view name,
    std::uint32_t& speaker_mask) noexcept;

} // namespace dtsx
