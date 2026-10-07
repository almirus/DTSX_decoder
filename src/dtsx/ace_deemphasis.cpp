#include "dtsx/ace_deemphasis.hpp"

#include <cmath>

namespace dtsx {

float ace_deemphasis_channel(
    std::vector<float>& samples,
    float previous) noexcept {
    constexpr float kCoefficient = 0.85F;
    float state = previous;
    for (float& sample : samples) {
        sample += state;
        state = kCoefficient * sample;
    }
    return std::isfinite(state) ? state : 0.0F;
}

std::int32_t ace_deemphasis_channel_q31(
    std::vector<std::int32_t>& samples,
    const std::int32_t previous,
    const std::int32_t coefficient) noexcept {
    std::int32_t state = previous;
    for (std::int32_t& sample : samples) {
        const std::int32_t filtered = static_cast<std::int32_t>(
            static_cast<std::int64_t>(state)
            + 2LL * static_cast<std::int64_t>(sample));
        sample = filtered;
        state = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(filtered)
             * static_cast<std::int64_t>(coefficient) + 0x40000000LL)
            >> 31);
    }
    return state;
}

void ace_finalize_pcm_q28(
    std::vector<std::int32_t>& samples,
    const bool deemphasis_applied) noexcept {
    const unsigned shift = deemphasis_applied ? 2U : 3U;
    for (std::int32_t& sample : samples) {
        const std::int64_t value = sample;
        const std::int64_t shifted = value << shift;
        if (shifted > static_cast<std::int64_t>(INT32_MAX)) {
            sample = INT32_MAX;
        } else if (shifted < static_cast<std::int64_t>(INT32_MIN)) {
            sample = INT32_MIN;
        } else {
            sample = static_cast<std::int32_t>(shifted);
        }
    }
}

} // namespace dtsx
