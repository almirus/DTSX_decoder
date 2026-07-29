#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace dtsx {

struct XllAdaptivePredictionState final {
    std::array<std::int32_t, 16> coefficients{};
    std::array<std::int32_t, 16> history{};
};

struct XllChannelPrediction final {
    std::vector<std::int32_t> adaptive_reflection_coefficients;
    std::uint8_t fixed_order = 0;
};

struct XllJointDecorrelationPair final {
    std::uint8_t source_channel = 0;
    std::uint8_t destination_channel = 0;
    std::int32_t coefficient = 0;
};

[[nodiscard]] bool inverse_xll_fixed_prediction(
    std::vector<std::int32_t>& samples,
    std::uint8_t order,
    bool first_segment,
    std::array<std::int32_t, 8>& state) noexcept;

[[nodiscard]] bool inverse_xll_adaptive_prediction(
    std::vector<std::int32_t>& samples,
    const std::vector<std::int32_t>& reflection_coefficients,
    bool first_segment,
    XllAdaptivePredictionState& state) noexcept;

[[nodiscard]] bool inverse_xll_joint_channel_decorrelation(
    const std::vector<std::int32_t>& source,
    std::vector<std::int32_t>& destination,
    std::int32_t coefficient) noexcept;

} // namespace dtsx
