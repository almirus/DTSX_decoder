#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace dtsx_decode {

class ParmaTripletAnalysis final {
public:
    [[nodiscard]] bool initialize(std::uint32_t sample_rate) noexcept;
    void reset() noexcept;

    void process(
        const std::array<const float*, 3U>& source_real,
        const std::array<const float*, 3U>& source_imaginary,
        std::uint32_t* order,
        float* position,
        float* diffuseness,
        float* coherence) noexcept;

private:
    static constexpr std::size_t kBandCount = 16U;
    std::array<std::uint32_t, kBandCount> widths_{};
    std::array<std::array<float, kBandCount>, 3U> energy_{};
    std::array<std::array<float, kBandCount>, 3U> position_energy_{};
    std::array<std::array<float, kBandCount>, 3U> sum_energy_{};
    std::array<std::array<float, kBandCount>, 3U> difference_energy_{};
    std::array<float, kBandCount> smoothed_coherence_{};
    float energy_attack_ = 0.0F;
    float energy_release_ = 0.0F;
    std::array<float, 4U> control_coefficients_{};
    bool initialized_ = false;
};

void parma_extract_matrixed_triplet_channel(
    const std::uint32_t* order,
    const float* position,
    const float* diffuseness,
    const std::array<const float*, 3U>& source_real,
    const std::array<const float*, 3U>& source_imaginary,
    float* target_real,
    float* target_imaginary,
    std::size_t count) noexcept;

void parma_repan_triplet_channels(
    const std::uint32_t* order,
    const float* position,
    const float* diffuseness,
    const float* coherence,
    const std::array<const float*, 3U>& source_real,
    const std::array<const float*, 3U>& source_imaginary,
    const std::array<float*, 3U>& target_real,
    const std::array<float*, 3U>& target_imaginary,
    std::size_t count) noexcept;

} // namespace dtsx_decode
