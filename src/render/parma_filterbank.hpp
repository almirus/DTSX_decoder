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

private:
    std::array<float, 1024U> first_delay_{};
    std::array<float, 1024U> second_delay_{};
    std::array<float, 64U> history_{};
    std::size_t position_ = 0U;
};

[[nodiscard]] constexpr std::size_t
parma_filterbank_latency_samples() noexcept {
    // DTS_ParmaDec_GetLatency -> OSFilter_GetLatency(1024, 64) + 64.
    return 1024U;
}

} // namespace dtsx_decode
