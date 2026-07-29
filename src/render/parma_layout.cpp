#include "render/parma_layout.hpp"

#include <array>

namespace dtsx_decode {
namespace {

// libdtsx.so: DecSpkrActMaskToParmaChannelMaskTable at 0x13eee0,
// consumed by sub_6B0B0 at 0x6b0b0.
constexpr std::array<std::uint32_t, 20>
    kSpeakerActivityToParmaChannelMask = {{
        0x00000001U,
        0x00000006U,
        0x00000018U,
        0x10000000U,
        0x00000020U,
        0x00006000U,
        0x000000C0U,
        0x00001000U,
        0x00040000U,
        0x00000C00U,
        0x00000300U,
        0x00000018U,
        0x20000000U,
        0x00180000U,
        0x00008000U,
        0x00030000U,
        0x00200000U,
        0x00C00000U,
        0x03000000U,
        0x0C000000U,
    }};

constexpr std::array<std::uint32_t, 9>
    kBlindInputMainChannelMasks = {{
        0x00000001U,
        0x00000006U,
        0x00000007U,
        0x00000026U,
        0x00000027U,
        0x0000001EU,
        0x0000001FU,
        0x0000003FU,
        0x000000DFU,
    }};

} // namespace

std::uint32_t parma_channel_mask_from_speaker_activity(
    std::uint32_t speaker_activity_mask) noexcept {
    std::uint32_t result = 0U;
    for (std::uint32_t activity = 0U;
         activity < kSpeakerActivityToParmaChannelMask.size();
         ++activity) {
        if ((speaker_activity_mask & (1U << activity)) != 0U) {
            result |=
                kSpeakerActivityToParmaChannelMask[activity];
        }
    }
    return result;
}

std::uint32_t parma_disable_lfe_channels(
    std::uint32_t channel_mask) noexcept {
    // libdtsx.so: ParmaDec_DisableLFEChans, 0x724e4.
    return channel_mask & 0xCFFFFFFFU;
}

std::uint32_t parma_main_channel_count(
    std::uint32_t channel_mask) noexcept {
    // libdtsx.so: DTS_ParmaDec_GetNumEnabledMainChans, 0x72648.
    std::uint32_t count = 0U;
    const std::uint32_t main_mask =
        parma_disable_lfe_channels(channel_mask);
    for (std::uint32_t bit = 0U; bit < 32U; ++bit) {
        count += (main_mask >> bit) & 1U;
    }
    return count;
}

bool parma_is_horizontal_layout(
    std::uint32_t channel_mask) noexcept {
    // libdtsx.so: DTS_ParmaDec_IsHorizontalLayout, 0x72690.
    return (channel_mask & 0x0FFFF000U) == 0U;
}

std::int32_t parma_blind_table_row(
    bool output_is_horizontal,
    std::uint32_t input_main_channel_mask) noexcept {
    // libdtsx.so: ParmaDec_GetTableRow_Blind, 0x725d8.  The native
    // row-range table selects 0..8 for horizontal output and 9..17
    // for output containing height channels.
    for (std::size_t index = 0U;
         index < kBlindInputMainChannelMasks.size();
         ++index) {
        if (kBlindInputMainChannelMasks[index]
                == input_main_channel_mask) {
            return static_cast<std::int32_t>(
                index + (output_is_horizontal ? 0U : 9U));
        }
    }
    return -1;
}

bool derive_parma_layout_controls(
    std::uint32_t input_speaker_activity_mask,
    std::uint32_t output_speaker_activity_mask,
    ParmaLayoutControls& controls) noexcept {
    controls = {};
    controls.input_channel_mask =
        parma_channel_mask_from_speaker_activity(
            input_speaker_activity_mask);
    controls.output_channel_mask =
        parma_channel_mask_from_speaker_activity(
            output_speaker_activity_mask);
    controls.input_main_channel_mask =
        parma_disable_lfe_channels(
            controls.input_channel_mask);
    controls.output_main_channel_mask =
        parma_disable_lfe_channels(
            controls.output_channel_mask);
    controls.input_main_channel_count =
        parma_main_channel_count(
            controls.input_channel_mask);
    controls.output_main_channel_count =
        parma_main_channel_count(
            controls.output_channel_mask);
    controls.output_is_horizontal =
        parma_is_horizontal_layout(
            controls.output_channel_mask);
    controls.blind_table_row =
        parma_blind_table_row(
            controls.output_is_horizontal,
            controls.input_main_channel_mask);

    return controls.input_main_channel_mask != 0U
        && controls.output_main_channel_mask != 0U
        && controls.blind_table_row >= 0
        && controls.input_main_channel_count
            <= controls.output_main_channel_count
        && controls.output_main_channel_count <= 11U;
}

std::uint32_t parma_blind_layer_count(
    const ParmaLayoutControls& controls) noexcept {
    // libdtsx.so: DTS_ParmaDec_SetBlind, 0x74948..0x74ecc.
    if (controls.blind_table_row < 0) {
        return 0U;
    }
    if (controls.output_is_horizontal) {
        return 1U;
    }

    std::uint32_t second_intermediate =
        controls.output_main_channel_mask;
    if ((controls.output_main_channel_mask & 0x1000U) != 0U) {
        second_intermediate |= 0x6000U;
        second_intermediate &= ~0x1000U;
    }
    if ((controls.output_main_channel_mask & 0x8000U) != 0U) {
        second_intermediate |= 0x30000U;
        second_intermediate &= ~0x8000U;
    }
    if ((controls.output_main_channel_mask & 0x40000U) != 0U) {
        second_intermediate |= 0x180000U;
        second_intermediate &= ~0x40000U;
    }
    return second_intermediate
               == controls.output_main_channel_mask
        ? 2U
        : 3U;
}

} // namespace dtsx_decode
