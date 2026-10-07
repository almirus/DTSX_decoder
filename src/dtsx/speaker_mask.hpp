#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

namespace dtsx {

constexpr std::array<std::uint32_t, 4U> kStandardHeightSpeakerMasks = {{
    1U << 13U,
    1U << 15U,
    1U << 23U,
    1U << 24U,
}};

[[nodiscard]] std::uint32_t speaker_count_from_activity_mask(
    std::uint32_t mask) noexcept;

[[nodiscard]] bool has_height_channels(std::uint32_t mask) noexcept;

[[nodiscard]] std::vector<std::uint32_t>
expand_speaker_activity_mask(std::uint32_t mask);

// Physical LFE is activity 3 / speaker bit 5.  ACE encodes it as a
// dedicated LFE stream, so bed mono/stereo waveform buses are packed in
// expansion order with that speaker omitted.  ACEW object descriptors that
// include LFE in their own mask still use expand_speaker_activity_mask.
constexpr std::uint32_t kPhysicalLfeSpeakerMask = 1U << 5U;

[[nodiscard]] std::vector<std::uint32_t>
expand_speaker_activity_mask_without_lfe(std::uint32_t mask);

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

[[nodiscard]] std::vector<std::uint32_t>
alternate_extension_speaker_masks(
    std::size_t first_set_channel_count,
    std::size_t total_channel_count) noexcept;

} // namespace dtsx
