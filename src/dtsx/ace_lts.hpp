#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx {

struct AceLtsHistory final {
    std::size_t lag = 0U;
    std::int32_t center_q31 = 0;
    std::int32_t adjacent_q31 = 0;
    std::int32_t outer_q31 = 0;
    std::vector<std::int32_t> samples;
    // Sony float ApplyLongTermSynthesis: previous 1024-sample output.
    std::vector<float> pcm;
    std::uint32_t filter_index = 0U;
};

enum class AceLtsWindowKind : std::uint8_t {
    Base,
    Low,
    High,
};

[[nodiscard]] bool ace_lts_window_coefficients(
    AceLtsWindowKind kind,
    std::vector<std::int32_t>& coefficients) noexcept;

[[nodiscard]] bool ace_lts_filter_coefficients(
    std::uint32_t filter_index,
    std::int32_t& center,
    std::int32_t& adjacent,
    std::int32_t& outer) noexcept;

// Scalar steady-state form of DTSAce_ApplyTimeDomainFilterSteadyState.
// Coefficients are signed Q31. Samples are signed Q28. The native caller
// supplies history around the lag; this standalone helper uses zero outside
// the supplied vector and is therefore suitable for deterministic staging
// tests until the native history buffers are wired into the stream decoder.
[[nodiscard]] bool ace_lts_filter_steady_state_q31(
    const std::vector<std::int32_t>& input,
    std::vector<std::int32_t>& output,
    std::size_t lag,
    std::int32_t center,
    std::int32_t adjacent,
    std::int32_t outer) noexcept;

[[nodiscard]] bool ace_lts_process_steady_state_q31(
    AceLtsHistory& state,
    const std::vector<std::int32_t>& input,
    std::vector<std::int32_t>& output,
    std::size_t lag,
    std::int32_t center,
    std::int32_t adjacent,
    std::int32_t outer) noexcept;

// Sony x64 DTSAceLTSynthesis_ApplyLongTermSynthesis /
// DTSAce_ApplyTimeDomainFilter. Runs on unbounded float PCM (never Q31).
// Filter index 0 is a native no-op but still updates the 1024-sample history.
[[nodiscard]] bool ace_lts_process_f32(
    AceLtsHistory& state,
    std::vector<float>& samples,
    bool enabled,
    std::uint32_t lag,
    std::uint32_t filter_index) noexcept;

} // namespace dtsx
