#pragma once

#include <array>
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
};

struct ParmaBlindLayerConfig final {
    std::array<ParmaBlindChannelRule, kParmaChannelCount> channels{};
};

[[nodiscard]] const std::array<ParmaBlindLayerConfig, 3>&
parma_blind_row_17_layers() noexcept;

} // namespace dtsx_decode
