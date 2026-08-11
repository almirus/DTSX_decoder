#include "render/parma_blind_config.hpp"

#include <initializer_list>

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

// visio-libdtsx.so: DTS_ParmaDec_SetBlind, row 17. The three native
// planes are unk_171644 (four source indices per channel), unk_17C5D4
// (operation mode) and unk_16FEA4 (decoding angle). Per decoder layer
// they advance by 8064, 2016 and 2016 bytes respectively.
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

namespace {

struct BlindNode final {
    std::uint8_t target;
    ParmaBlindChannelRule rule;
};

constexpr ParmaBlindChannelRule direct_rule(
    std::uint8_t target) noexcept {
    return {0U, {target, kParmaUnusedChannel, kParmaUnusedChannel,
                 kParmaUnusedChannel}, -1.0F};
}

constexpr ParmaBlindChannelRule special_rule(
    std::uint8_t mode,
    std::uint8_t first,
    std::uint8_t second,
    float decoding_angle,
    float second_control = -1.0F,
    float third_control = -1.0F) noexcept {
    return {mode, {first, second, kParmaUnusedChannel,
                   kParmaUnusedChannel}, decoding_angle,
            {decoding_angle, second_control, third_control}};
}

ParmaBlindLayerConfig make_layer(
    std::initializer_list<BlindNode> nodes) noexcept {
    ParmaBlindLayerConfig result{};
    for (const BlindNode& node : nodes) {
        result.channels[node.target] = node.rule;
    }
    return result;
}

ParmaBlindLayerConfig height_layer() noexcept {
    // DTS_ParmaDec_SetBlind rows 9..17, layer 1 for 0x360DF.
    return make_layer({
        {0U, direct_rule(0U)}, {1U, direct_rule(1U)},
        {2U, direct_rule(2U)}, {3U, direct_rule(3U)},
        {4U, direct_rule(4U)}, {6U, direct_rule(6U)},
        {7U, direct_rule(7U)},
        {13U, special_rule(1U, 1U, 6U, 0.25F, 0.25F, 0.75F)},
        {14U, special_rule(1U, 2U, 7U, 0.25F, 0.25F, 0.75F)},
        {16U, special_rule(1U, 1U, 6U, 0.75F, 0.75F, 1.0F)},
        {17U, special_rule(1U, 2U, 7U, 0.75F, 0.75F, 1.0F)},
    });
}

ParmaBlindTopology make_height_topology(
    std::uint32_t input_mask,
    ParmaBlindLayerConfig base_layer) noexcept {
    ParmaBlindTopology result{};
    result.input_main_channel_mask = input_mask;
    result.output_main_channel_mask = 0x360DFU;
    result.layer_count = 2U;
    result.layers[0U] = base_layer;
    result.layers[1U] = height_layer();
    return result;
}

const std::array<ParmaBlindTopology, 8U>&
height_topologies() noexcept {
    // These records are the post-UpdateChannelInfo native topology dumped
    // from visio-libdtsx.so after DTS_ParmaDec_SetBlind.  Every blind row in
    // this decoder uses pairwise analysis; rows 10/11 additionally use the
    // documented special intermediate modes 5/6/7.
    static const std::array<ParmaBlindTopology, 8U> rows = {{
        make_height_topology(0x6U, make_layer({
            {0U, special_rule(6U, 1U, 2U, 0.5F, 0.5F, 1.0F)},
            {1U, special_rule(5U, 1U, kParmaUnusedChannel, 0.0F, 0.0F, 0.0F)},
            {2U, special_rule(5U, 2U, kParmaUnusedChannel, 0.0F, 0.0F, 0.0F)},
            {3U, special_rule(7U, 1U, 2U, 0.2411984F, 0.2411984F, 0.3324205F)},
            {4U, special_rule(7U, 1U, 2U, 0.7588016F, 0.7588016F, 1.0F)},
            {6U, special_rule(7U, 1U, 2U, 0.3324205F, 0.3324205F, 0.6675795F)},
            {7U, special_rule(7U, 1U, 2U, 0.6675795F, 0.6675795F, 0.7588016F)},
        })),
        make_height_topology(0x7U, make_layer({
            {0U, special_rule(5U, 0U, kParmaUnusedChannel, 0.0F, 0.0F, 0.0F)},
            {1U, special_rule(5U, 1U, kParmaUnusedChannel, 0.0F, 0.0F, 0.0F)},
            {2U, special_rule(5U, 2U, kParmaUnusedChannel, 0.0F, 0.0F, 0.0F)},
            {3U, special_rule(7U, 1U, 2U, 0.2411984F, 0.2411984F, 0.3324205F)},
            {4U, special_rule(7U, 1U, 2U, 0.7588016F, 0.7588016F, 1.0F)},
            {6U, special_rule(7U, 1U, 2U, 0.3324205F, 0.3324205F, 0.6675795F)},
            {7U, special_rule(7U, 1U, 2U, 0.6675795F, 0.6675795F, 0.7588016F)},
        })),
        make_height_topology(0x26U, make_layer({
            {0U, special_rule(1U, 1U, 2U, 0.5F, 0.5F, 1.0F)},
            {1U, direct_rule(1U)}, {2U, direct_rule(2U)},
            {3U, special_rule(1U, 1U, 5U, 0.5F, 0.5F, 0.75F)},
            {4U, special_rule(1U, 2U, 5U, 0.5F, 0.5F, 0.75F)},
            {6U, special_rule(1U, 1U, 5U, 0.75F, 0.75F, 1.0F)},
            {7U, special_rule(1U, 2U, 5U, 0.75F, 0.75F, 1.0F)},
        })),
        make_height_topology(0x27U, make_layer({
            {0U, direct_rule(0U)}, {1U, direct_rule(1U)},
            {2U, direct_rule(2U)},
            {3U, special_rule(1U, 1U, 5U, 0.5F, 0.5F, 0.75F)},
            {4U, special_rule(1U, 2U, 5U, 0.5F, 0.5F, 0.75F)},
            {6U, special_rule(1U, 1U, 5U, 0.75F, 0.75F, 1.0F)},
            {7U, special_rule(1U, 2U, 5U, 0.75F, 0.75F, 1.0F)},
        })),
        make_height_topology(0x1EU, make_layer({
            {0U, special_rule(1U, 1U, 2U, 0.5F, 0.5F, 1.0F)},
            {1U, direct_rule(1U)}, {2U, direct_rule(2U)},
            {3U, direct_rule(3U)}, {4U, direct_rule(4U)},
            {6U, special_rule(1U, 3U, 4U, 0.2857143F, 0.2857143F, 0.7142857F)},
            {7U, special_rule(1U, 3U, 4U, 0.7142857F, 0.7142857F, 1.0F)},
        })),
        make_height_topology(0x1FU, make_layer({
            {0U, direct_rule(0U)}, {1U, direct_rule(1U)},
            {2U, direct_rule(2U)}, {3U, direct_rule(3U)},
            {4U, direct_rule(4U)},
            {6U, special_rule(1U, 3U, 4U, 0.2857143F, 0.2857143F, 0.7142857F)},
            {7U, special_rule(1U, 3U, 4U, 0.7142857F, 0.7142857F, 1.0F)},
        })),
        make_height_topology(0x3FU, make_layer({
            {0U, direct_rule(0U)}, {1U, direct_rule(1U)},
            {2U, direct_rule(2U)}, {3U, direct_rule(3U)},
            {4U, direct_rule(4U)},
            {6U, special_rule(1U, 3U, 5U, 0.5F, 0.5F, 1.0F)},
            {7U, special_rule(1U, 4U, 5U, 0.5F, 0.5F, 1.0F)},
        })),
        make_height_topology(0xDFU, make_layer({
            {0U, direct_rule(0U)}, {1U, direct_rule(1U)},
            {2U, direct_rule(2U)}, {3U, direct_rule(3U)},
            {4U, direct_rule(4U)}, {6U, direct_rule(6U)},
            {7U, direct_rule(7U)},
        })),
    }};
    return rows;
}

} // namespace

const ParmaBlindTopology* parma_blind_topology(
    std::uint32_t input_main_channel_mask,
    bool output_is_horizontal) noexcept {
    // DTS_ParmaDec_Process performs Mono2StereoConversion before blind
    // setup. Consequently the native mono row reaches SetBlind as 0x06.
    if (input_main_channel_mask == 0x1U) {
        input_main_channel_mask = 0x6U;
    }
    for (const ParmaBlindTopology& topology : height_topologies()) {
        if (topology.input_main_channel_mask != input_main_channel_mask) {
            continue;
        }
        if (!output_is_horizontal) {
            return &topology;
        }
        // Horizontal SetBlind consists solely of the first native layer.
        static thread_local ParmaBlindTopology horizontal{};
        horizontal = topology;
        horizontal.output_main_channel_mask = 0xDFU;
        horizontal.layer_count = 1U;
        return &horizontal;
    }
    return nullptr;
}

} // namespace dtsx_decode
