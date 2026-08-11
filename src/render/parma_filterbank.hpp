#pragma once

#include <array>
#include <cstddef>

namespace dtsx_decode {

class ParmaAnalysisFilterBank final {
public:
    ParmaAnalysisFilterBank() noexcept;
    void reset() noexcept;
    void process(
        const float* input,
        float* real,
        float* imaginary) noexcept;

private:
    std::array<float, 1024U> delay_{};
    std::size_t position_ = 0U;
};

class ParmaSynthesisFilterBank final {
public:
    ParmaSynthesisFilterBank() noexcept;
    void reset() noexcept;
    void process(
        const float* real,
        const float* imaginary,
        float* output) noexcept;
    void process_x2(
        const float* first_real,
        const float* first_imaginary,
        ParmaSynthesisFilterBank& second_filter,
        const float* second_real,
        const float* second_imaginary,
        float* first_output,
        float* second_output) noexcept;

private:
    std::array<float, 1024U> first_delay_{};
    std::array<float, 1024U> second_delay_{};
    std::array<float, 64U> history_{};
    std::size_t position_ = 0U;
};

[[nodiscard]] constexpr std::size_t
parma_filterbank_latency_samples() noexcept {
    // DTS_ParmaDec_GetLatency_Samples: OSFilter_GetLatency(1024, 64)
    // returns 960 and the native decoder adds one 64-sample hop.
    // This is also confirmed by the native ARM implementation through
    // DTS_ParmaDec_GetLatency_Samples().
    return 1024U;
}

} // namespace dtsx_decode
