#pragma once

#include "dtsx/preliminary_metadata.hpp"

#include <array>
#include <cstdint>

namespace dtsx_decode {

// Exact logical input consumed by DTS_ParmaDec_SetGuided.  Rows are packed by
// added-output ordinal; columns are packed by enabled output-channel ordinal.
struct ParmaGuidedControls final {
    std::uint32_t downmix_speaker_activity_mask = 0U;
    std::uint32_t upmix_speaker_activity_mask = 0U;
    std::uint32_t downmix_channel_mask = 0U;
    std::uint32_t upmix_channel_mask = 0U;
    std::uint32_t downmix_main_channel_count = 0U;
    std::uint32_t upmix_main_channel_count = 0U;
    std::uint32_t added_output_count = 0U;
    // Indexed by native enabled-PARMA main-channel ordinal: added output
    // row, then input/reference column.  This intentionally excludes LFE.
    // Values are raw six-bit combined-mix codes scaled by 1 / 32768 exactly
    // as dtsPlayerParmaControl_UpdateParmaCustomCoeffs feeds SetGuided.
    std::array<std::array<float, 11U>, 11U>
        custom_encoder_coefficients{};
};

[[nodiscard]] bool derive_parma_guided_controls(
    const dtsx::CombinedMixMetadata& metadata,
    std::uint32_t decoded_bed_speaker_activity_mask,
    std::uint32_t requested_output_speaker_activity_mask,
    ParmaGuidedControls& controls) noexcept;

} // namespace dtsx_decode
