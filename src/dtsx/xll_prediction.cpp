#include "dtsx/xll_prediction.hpp"

#include <algorithm>
#include <limits>

namespace dtsx {
namespace {

constexpr std::array<std::int32_t, 128> kHyperbolicTable = {
    0x0000, 0x0BFE, 0x13F6, 0x1BE4, 0x23C4, 0x2B92, 0x334C, 0x3AED,
    0x4272, 0x49D8, 0x511C, 0x583C, 0x5F35, 0x6605, 0x6CAA, 0x7322,
    0x796D, 0x7F88, 0x8574, 0x8B2F, 0x90B9, 0x9612, 0x9B3A, 0xA030,
    0xA4F6, 0xA98C, 0xADF2, 0xB229, 0xB632, 0xBA0E, 0xBDBF, 0xC145,
    0xC4A1, 0xC7D5, 0xCAE1, 0xCDC9, 0xD08B, 0xD32B, 0xD5A9, 0xD806,
    0xDA44, 0xDC65, 0xDE69, 0xE052, 0xE220, 0xE3D6, 0xE575, 0xE6FC,
    0xE86E, 0xE9CC, 0xEB16, 0xEC4E, 0xED75, 0xEE8A, 0xEF90, 0xF088,
    0xF171, 0xF24D, 0xF31C, 0xF3DF, 0xF497, 0xF544, 0xF5E7, 0xF681,
    0xF712, 0xF79A, 0xF81A, 0xF893, 0xF905, 0xF96F, 0xF9D4, 0xFA33,
    0xFA8C, 0xFAE0, 0xFB2E, 0xFB78, 0xFBBE, 0xFC00, 0xFC3D, 0xFC77,
    0xFCAE, 0xFCE1, 0xFD11, 0xFD3E, 0xFD69, 0xFD91, 0xFDB6, 0xFDDA,
    0xFDFB, 0xFE1A, 0xFE37, 0xFE53, 0xFE6D, 0xFE85, 0xFE9C, 0xFEB2,
    0xFEC6, 0xFED9, 0xFEEB, 0xFEFB, 0xFF0B, 0xFF1A, 0xFF28, 0xFF35,
    0xFF41, 0xFF4D, 0xFF58, 0xFF62, 0xFF6B, 0xFF74, 0xFF7D, 0xFF85,
    0xFF8C, 0xFF93, 0xFF9A, 0xFFA0, 0xFFA6, 0xFFAB, 0xFFB0, 0xFFB5,
    0xFFBA, 0xFFBE, 0xFFC2, 0xFFC6, 0xFFC9, 0xFFCD, 0xFFD0, 0xFFD3,
};

std::int32_t wrapping_add(
    std::int32_t left, std::int32_t right) noexcept {
    return static_cast<std::int32_t>(
        static_cast<std::uint32_t>(left)
        + static_cast<std::uint32_t>(right));
}

std::int32_t rounded_saturating_shift_three(
    std::int64_t value) noexcept {
    const std::int64_t shifted = (value + 4) >> 3U;
    return static_cast<std::int32_t>(std::max(
        static_cast<std::int64_t>(
            std::numeric_limits<std::int32_t>::min()),
        std::min(
            static_cast<std::int64_t>(
                std::numeric_limits<std::int32_t>::max()),
            shifted)));
}

std::int32_t rounded_q16(std::int64_t value) noexcept {
    return static_cast<std::int32_t>((value + 0x8000LL) >> 16U);
}

std::int32_t saturate_24(std::int64_t value) noexcept {
    return static_cast<std::int32_t>(
        std::max<std::int64_t>(
            -0x800000LL, std::min<std::int64_t>(0x7FFFFFLL, value)));
}

} // namespace

bool inverse_xll_fixed_prediction(
    std::vector<std::int32_t>& samples,
    std::uint8_t order,
    bool first_segment,
    std::array<std::int32_t, 8>& state) noexcept {
    // libdtsx.so: inverseFixedPrediction, 0x112998.
    if (order > 3U) {
        return false;
    }
    if (first_segment) {
        state.fill(0);
    }
    for (std::int32_t& sample : samples) {
        std::int32_t reconstructed = sample;
        state[0] = reconstructed;
        for (std::uint32_t stage = 0; stage < order; ++stage) {
            reconstructed = wrapping_add(
                reconstructed, state[2U * stage + 1U]);
            state[2U * stage + 1U] = reconstructed;
            state[2U * stage + 2U] = reconstructed;
        }
        sample = reconstructed;
    }
    return true;
}

bool inverse_xll_adaptive_prediction(
    std::vector<std::int32_t>& samples,
    const std::vector<std::int32_t>& reflection_coefficients,
    bool first_segment,
    XllAdaptivePredictionState& state) noexcept {
    // libdtsx.so: inverseAdaptivePrediction, 0x112aa8, and
    // inverseAdaptivePredictionCore, 0x115a3c.
    const std::size_t order = reflection_coefficients.size();
    if (order == 0U || order > state.coefficients.size()
        || samples.size() < order) {
        return false;
    }

    if (first_segment) {
        state.coefficients.fill(0);
        std::array<std::int32_t, 16> direct{};
        for (std::size_t stage = 0; stage < order; ++stage) {
            const std::int32_t encoded =
                reflection_coefficients[stage];
            const std::uint32_t magnitude = encoded < 0
                ? static_cast<std::uint32_t>(-static_cast<std::int64_t>(
                      encoded))
                : static_cast<std::uint32_t>(encoded);
            if (magnitude >= kHyperbolicTable.size()) {
                return false;
            }
            const std::int32_t reflection = encoded < 0
                ? -kHyperbolicTable[magnitude]
                : kHyperbolicTable[magnitude];

            const auto previous = direct;
            direct[stage] = reflection;
            for (std::size_t index = 0; index < stage; ++index) {
                direct[index] = wrapping_add(
                    previous[index],
                    rounded_q16(
                        static_cast<std::int64_t>(
                            previous[stage - index - 1U])
                        * reflection));
            }
        }
        for (std::size_t index = 0; index < order; ++index) {
            state.coefficients[index] =
                direct[order - index - 1U];
        }
    }

    std::size_t first_reconstructed = order;
    if (!first_segment) {
        for (std::size_t index = 0; index < order; ++index) {
            std::int64_t prediction = 0x8000LL;
            for (std::size_t tap = 0; tap < order; ++tap) {
                const std::size_t history_index =
                    tap + index;
                const std::int32_t prior =
                    history_index < order
                    ? state.history[history_index]
                    : samples[history_index - order];
                prediction += static_cast<std::int64_t>(prior)
                    * state.coefficients[tap];
            }
            samples[index] = static_cast<std::int32_t>(
                static_cast<std::uint32_t>(samples[index])
                - static_cast<std::uint32_t>(
                    saturate_24(prediction >> 16U)));
        }
        first_reconstructed = order;
    }

    for (std::size_t index = first_reconstructed;
         index < samples.size();
         ++index) {
        std::int64_t prediction = 0x8000LL;
        for (std::size_t tap = 0; tap < order; ++tap) {
            prediction +=
                static_cast<std::int64_t>(samples[index - order + tap])
                * state.coefficients[tap];
        }
        samples[index] = static_cast<std::int32_t>(
            static_cast<std::uint32_t>(samples[index])
            - static_cast<std::uint32_t>(
                saturate_24(prediction >> 16U)));
    }

    std::copy_n(
        samples.end() - static_cast<std::ptrdiff_t>(order),
        order,
        state.history.begin());
    return true;
}

bool inverse_xll_joint_channel_decorrelation(
    const std::vector<std::int32_t>& source,
    std::vector<std::int32_t>& destination,
    std::int32_t coefficient) noexcept {
    // libdtsx.so: inverseJChDecorrelationCore, 0x1159f8.
    if (source.size() != destination.size()) {
        return false;
    }
    for (std::size_t index = 0; index < source.size(); ++index) {
        const std::int32_t correction =
            rounded_saturating_shift_three(
                static_cast<std::int64_t>(source[index])
                * coefficient);
        destination[index] =
            wrapping_add(destination[index], correction);
    }
    return true;
}

} // namespace dtsx
