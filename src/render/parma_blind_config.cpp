#include "render/parma_blind_config.hpp"

namespace dtsx_decode {
namespace {

constexpr ParmaBlindChannelRule passthrough(
    std::uint8_t channel) noexcept {
    return {
        0U,
        {channel, kParmaUnusedChannel,
         kParmaUnusedChannel, kParmaUnusedChannel},
        -1.0F,
    };
}

constexpr ParmaBlindChannelRule pair(
    std::uint8_t first,
    std::uint8_t second,
    float decoding_angle) noexcept {
    return {
        1U,
        {first, second,
         kParmaUnusedChannel, kParmaUnusedChannel},
        decoding_angle,
    };
}

constexpr ParmaBlindChannelRule unused() noexcept {
    return {};
}

// libdtsx.so: DTS_ParmaDec_SetBlind, row 17.  The three native table
// planes start at 0x14060c/0x141dac/0x144b9c and advance by
// 2016/2016/8064 bytes for each subsequent decoder layer.
constexpr std::array<ParmaBlindLayerConfig, 3> kRow17Layers = {{
    {{
        passthrough(0U),
        passthrough(1U),
        passthrough(2U),
        passthrough(3U),
        passthrough(4U),
        unused(),
        passthrough(6U),
        passthrough(7U),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
        unused(),
    }},
    {{
        passthrough(0U),
        passthrough(1U),
        passthrough(2U),
        passthrough(3U),
        passthrough(4U),
        unused(),
        passthrough(6U),
        passthrough(7U),
        pair(1U, 3U, 0.5F),
        pair(2U, 4U, 0.5F),
        pair(0U, 1U, 0.5F),
        pair(0U, 2U, 0.5F),
        unused(),
        pair(1U, 6U, 0.25F),
        pair(2U, 7U, 0.25F),
        unused(),
        pair(1U, 6U, 0.75F),
        pair(2U, 7U, 0.75F),
        unused(),
        pair(1U, 6U, 0.5F),
        pair(2U, 7U, 0.5F),
        unused(),
        unused(),
        unused(),
        pair(1U, 6U, 0.375F),
        pair(2U, 7U, 0.375F),
        pair(1U, 6U, 0.625F),
        pair(2U, 7U, 0.625F),
    }},
    {{
        passthrough(0U),
        passthrough(1U),
        passthrough(2U),
        passthrough(3U),
        passthrough(4U),
        pair(6U, 7U, 0.5F),
        passthrough(6U),
        passthrough(7U),
        passthrough(8U),
        passthrough(9U),
        passthrough(10U),
        passthrough(11U),
        pair(13U, 14U, 0.5F),
        passthrough(13U),
        passthrough(14U),
        pair(16U, 17U, 0.5F),
        passthrough(16U),
        passthrough(17U),
        pair(19U, 20U, 0.5F),
        passthrough(19U),
        passthrough(20U),
        unused(),
        unused(),
        unused(),
        passthrough(24U),
        passthrough(25U),
        passthrough(26U),
        passthrough(27U),
    }},
}};

} // namespace

const std::array<ParmaBlindLayerConfig, 3>&
parma_blind_row_17_layers() noexcept {
    return kRow17Layers;
}

} // namespace dtsx_decode
