#pragma once

#include <cstdint>

namespace dtsx_decode {

struct ParmaLayoutControls final {
    std::uint32_t input_channel_mask = 0;
    std::uint32_t input_main_channel_mask = 0;
    std::uint32_t output_channel_mask = 0;
    std::uint32_t output_main_channel_mask = 0;
    std::uint32_t input_main_channel_count = 0;
    std::uint32_t output_main_channel_count = 0;
    std::int32_t blind_table_row = -1;
    bool output_is_horizontal = true;
};

[[nodiscard]] std::uint32_t parma_channel_mask_from_speaker_activity(
    std::uint32_t speaker_activity_mask) noexcept;

[[nodiscard]] std::uint32_t parma_disable_lfe_channels(
    std::uint32_t channel_mask) noexcept;

[[nodiscard]] std::uint32_t parma_main_channel_count(
    std::uint32_t channel_mask) noexcept;

// DTS_ParmaDec_GetChanLocation uses the enabled-bit order of the PARMA
// channel mask.  These helpers keep metadata matrices in that order instead
// of the container/layout order used by the PCM writer.
[[nodiscard]] std::int32_t parma_channel_slot_from_speaker_mask(
    std::uint32_t speaker_mask) noexcept;

[[nodiscard]] std::int32_t parma_main_channel_ordinal(
    std::uint32_t channel_mask,
    std::uint32_t speaker_mask) noexcept;

[[nodiscard]] bool parma_is_horizontal_layout(
    std::uint32_t channel_mask) noexcept;

[[nodiscard]] std::int32_t parma_blind_table_row(
    bool output_is_horizontal,
    std::uint32_t input_main_channel_mask) noexcept;

[[nodiscard]] bool derive_parma_layout_controls(
    std::uint32_t input_speaker_activity_mask,
    std::uint32_t output_speaker_activity_mask,
    ParmaLayoutControls& controls) noexcept;

[[nodiscard]] std::uint32_t parma_blind_layer_count(
    const ParmaLayoutControls& controls) noexcept;

} // namespace dtsx_decode
