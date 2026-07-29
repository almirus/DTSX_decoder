#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace dtsx_decode {

class ParmaPairwiseAnalysis final {
public:
    [[nodiscard]] bool initialize(std::uint32_t sample_rate) noexcept;
    void reset() noexcept;

    void process(
        const float* first_real,
        const float* first_imaginary,
        const float* second_real,
        const float* second_imaginary,
        float* position,
        float* balance,
        float* diffuseness) noexcept;

private:
    static constexpr std::size_t kBandCount = 16U;

    std::array<std::uint32_t, kBandCount> widths_{};
    std::array<float, kBandCount> first_energy_{};
    std::array<float, kBandCount> second_energy_{};
    std::array<float, kBandCount> sum_energy_{};
    std::array<float, kBandCount> difference_energy_{};
    std::array<float, kBandCount> energy_ratio_state_{};
    std::array<float, kBandCount> position_state_{};
    std::array<float, kBandCount> balance_state_{};
    std::array<float, kBandCount> energy_metric_state_{};
    std::array<float, kBandCount> diffuseness_state_{};
    float energy_attack_ = 0.0F;
    float energy_release_ = 0.0F;
    float metric_attack_ = 0.0F;
    float metric_fast_release_ = 0.0F;
    float metric_release_ = 0.0F;
    float metric_slow_release_ = 0.0F;
    float diffuseness_attack_ = 0.0F;
    float diffuseness_release_ = 0.0F;
    float diffuseness_slow_release_ = 0.0F;
    bool initialized_ = false;
};

} // namespace dtsx_decode
