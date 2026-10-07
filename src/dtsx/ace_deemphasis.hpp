#pragma once

#include <cstdint>
#include <vector>

namespace dtsx {

// ETSI TS 103 491 §9.10.8.3.1. State is carried between ACE frames.
[[nodiscard]] float ace_deemphasis_channel(
    std::vector<float>& samples,
    float previous) noexcept;

// Native DTSAceDeEmphasis_FilterProcess fixed-point recurrence. `coefficient`
// is Q31 and the input/output vectors contain signed Q28 samples.
[[nodiscard]] std::int32_t ace_deemphasis_channel_q31(
    std::vector<std::int32_t>& samples,
    std::int32_t previous,
    std::int32_t coefficient) noexcept;

// Final native ACE PCM shift/saturation after LTS and optional de-emphasis.
// The TCL decoder uses shift=2 when de-emphasis ran, otherwise shift=3.
void ace_finalize_pcm_q28(
    std::vector<std::int32_t>& samples,
    bool deemphasis_applied) noexcept;

} // namespace dtsx
