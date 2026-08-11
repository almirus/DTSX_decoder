#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace dtsx_decode {

constexpr std::uint8_t kParmaUnusedChannel = 31U;
constexpr std::size_t kParmaChannelCount = 28U;

struct ParmaBlindChannelRule final {
    std::uint8_t mode = 8U;
    std::array<std::uint8_t, 4> sources = {
        kParmaUnusedChannel,
        kParmaUnusedChannel,
        kParmaUnusedChannel,
        kParmaUnusedChannel,
    };
    float decoding_angle = -1.0F;
    // Raw post-DTS_ParmaDec_UpdateDecodingAngles controls.  They are kept
    // separately from decoding_angle because modes 5/6/7 use all three
    // values in the native special pairwise route.
    std::array<float, 3U> native_angle_controls = {
        -1.0F, -1.0F, -1.0F};
};

struct ParmaBlindLayerConfig final {
    std::array<ParmaBlindChannelRule, kParmaChannelCount> channels{};
};

// Fully decoded output of DTS_ParmaDec_SetBlind for one native input row.
// `layer_count` is one for horizontal output and two for the 7.1.4
// elevation path. Modes 5/6/7 deliberately remain represented even though
// the current renderer has not enabled their intermediate DSP path yet.
struct ParmaBlindTopology final {
    std::uint32_t input_main_channel_mask = 0U;
    std::uint32_t output_main_channel_mask = 0U;
    std::uint8_t layer_count = 0U;
    std::array<ParmaBlindLayerConfig, 3U> layers{};
};

[[nodiscard]] const ParmaBlindTopology*
parma_blind_topology(
    std::uint32_t input_main_channel_mask,
    bool output_is_horizontal) noexcept;

[[nodiscard]] const std::array<ParmaBlindLayerConfig, 3>&
parma_blind_row_17_layers() noexcept;

} // namespace dtsx_decode
