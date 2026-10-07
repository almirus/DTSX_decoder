#include "dtsx/ace_lts.hpp"

#include "dtsx/ace_lts_window_table.generated.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace dtsx {
namespace {

constexpr std::int32_t kLtsCoefficients[29][3] = {
    {0, 0, 0}, {139575696, 87391848, 44145820},
    {170967616, 102797896, 13033078}, {248558352, 63498944, 10550587},
    {330175616, 24159192, 12079596}, {209364624, 131088848, 66219804},
    {256450352, 154195776, 19550692}, {372837536, 95247344, 15824807},
    {495263424, 36238788, 18118320}, {279151392, 174783696, 88293792},
    {341933088, 205593648, 26068304}, {497114560, 126997888, 21099026},
    {660351232, 48318384, 24159192}, {348938176, 218478544, 110365624},
    {427415808, 256993664, 32583770}, {621393728, 158746288, 26373246},
    {825439040, 60397976, 30197916}, {418727104, 262175536, 132439608},
    {512900704, 308391552, 39101384}, {745672896, 190494688, 31647466},
    {990526848, 72477576, 36238788}, {488513888, 305870400, 154513600},
    {598383424, 359791552, 45618996}, {869952064, 222245232, 36923832},
    {1155614592, 84557168, 42277512}, {558302784, 349567392, 176585440},
    {683866176, 411189440, 52136608}, {994231232, 253993632, 42198052},
    {1320702464, 96636768, 48318384},
};

std::int32_t q31(std::int64_t value) noexcept {
    value += 0x40000000LL;
    value >>= 31;
    if (value > std::numeric_limits<std::int32_t>::max()) {
        return std::numeric_limits<std::int32_t>::max();
    }
    if (value < std::numeric_limits<std::int32_t>::min()) {
        return std::numeric_limits<std::int32_t>::min();
    }
    return static_cast<std::int32_t>(value);
}

std::int32_t at_or_zero(
    const std::vector<std::int32_t>& input,
    const std::ptrdiff_t index) noexcept {
    return index < 0 || static_cast<std::size_t>(index) >= input.size()
        ? 0
        : input[static_cast<std::size_t>(index)];
}

} // namespace

bool ace_lts_window_coefficients(
    const AceLtsWindowKind kind,
    std::vector<std::int32_t>& coefficients) noexcept {
    switch (kind) {
    case AceLtsWindowKind::Base:
        coefficients.assign(native_lts_windows::kLtsWindow102.begin(),
                            native_lts_windows::kLtsWindow102.end());
        return true;
    case AceLtsWindowKind::Low:
        coefficients.assign(native_lts_windows::kLtsLoWindow102.begin(),
                            native_lts_windows::kLtsLoWindow102.end());
        return true;
    case AceLtsWindowKind::High:
        coefficients.assign(native_lts_windows::kLtsHiWindow102.begin(),
                            native_lts_windows::kLtsHiWindow102.end());
        return true;
    default:
        coefficients.clear();
        return false;
    }
}

bool ace_lts_filter_coefficients(
    const std::uint32_t filter_index,
    std::int32_t& center,
    std::int32_t& adjacent,
    std::int32_t& outer) noexcept {
    if (filter_index >= 29U) {
        return false;
    }
    center = kLtsCoefficients[filter_index][0];
    adjacent = kLtsCoefficients[filter_index][1];
    outer = kLtsCoefficients[filter_index][2];
    return true;
}

bool ace_lts_filter_steady_state_q31(
    const std::vector<std::int32_t>& input,
    std::vector<std::int32_t>& output,
    const std::size_t lag,
    const std::int32_t center,
    const std::int32_t adjacent,
    const std::int32_t outer) noexcept {
    if (input.empty() || lag > input.size()) {
        output.clear();
        return false;
    }
    output.resize(input.size());
    for (std::size_t index = 0U; index < input.size(); ++index) {
        const auto center_index = static_cast<std::ptrdiff_t>(index)
            - static_cast<std::ptrdiff_t>(lag);
        const std::int64_t adjacent_sum =
            static_cast<std::int64_t>(at_or_zero(input, center_index - 1))
            + at_or_zero(input, center_index + 1);
        const std::int64_t outer_sum =
            static_cast<std::int64_t>(at_or_zero(input, center_index - 2))
            + at_or_zero(input, center_index + 2);
        const std::int64_t value = input[index]
            + q31(static_cast<std::int64_t>(at_or_zero(input, center_index))
                  * center)
            + q31(adjacent_sum * adjacent)
            + q31(outer_sum * outer);
        if (value > std::numeric_limits<std::int32_t>::max()) {
            output[index] = std::numeric_limits<std::int32_t>::max();
        } else if (value < std::numeric_limits<std::int32_t>::min()) {
            output[index] = std::numeric_limits<std::int32_t>::min();
        } else {
            output[index] = static_cast<std::int32_t>(value);
        }
    }
    return true;
}

bool ace_lts_process_steady_state_q31(
    AceLtsHistory& state,
    const std::vector<std::int32_t>& input,
    std::vector<std::int32_t>& output,
    const std::size_t lag,
    const std::int32_t center,
    const std::int32_t adjacent,
    const std::int32_t outer) noexcept {
    if (input.empty() || lag == 0U) {
        output.clear();
        return false;
    }
    std::vector<std::int32_t> extended;
    extended.reserve(state.samples.size() + input.size());
    extended.insert(extended.end(), state.samples.begin(), state.samples.end());
    extended.insert(extended.end(), input.begin(), input.end());
    std::vector<std::int32_t> filtered;
    if (!ace_lts_filter_steady_state_q31(
            extended, filtered, lag, center, adjacent, outer)
        || filtered.size() < input.size()) {
        output.clear();
        return false;
    }
    output.assign(filtered.end() - static_cast<std::ptrdiff_t>(input.size()),
                  filtered.end());
    state.lag = lag;
    state.center_q31 = center;
    state.adjacent_q31 = adjacent;
    state.outer_q31 = outer;
    const std::size_t keep = lag + 2U;
    if (extended.size() > keep) {
        state.samples.assign(extended.end() - static_cast<std::ptrdiff_t>(keep),
                             extended.end());
    } else {
        state.samples = std::move(extended);
    }
    return true;
}

namespace {

constexpr float kQ31ToFloat = 1.0F / 2147483648.0F;
constexpr float kQ30ToFloat = 1.0F / 1073741824.0F;
constexpr std::size_t kLtsFrame = 1024U;
constexpr std::size_t kLtsWindow = 102U;

float lts_at(const std::vector<float>& buffer, const std::ptrdiff_t index) noexcept {
    return index < 0 || static_cast<std::size_t>(index) >= buffer.size()
        ? 0.0F
        : buffer[static_cast<std::size_t>(index)];
}

float lts_fir(
    const std::vector<float>& buffer,
    const std::ptrdiff_t index,
    const std::uint32_t lag,
    const float center,
    const float adjacent,
    const float outer) noexcept {
    const std::ptrdiff_t mid = index - static_cast<std::ptrdiff_t>(lag);
    return lts_at(buffer, index)
        + center * lts_at(buffer, mid)
        + adjacent * (lts_at(buffer, mid - 1) + lts_at(buffer, mid + 1))
        + outer * (lts_at(buffer, mid - 2) + lts_at(buffer, mid + 2));
}

void lts_coeffs(
    const std::uint32_t index, float& center, float& adjacent, float& outer) noexcept {
    if (index >= 29U) {
        center = 0.0F;
        adjacent = 0.0F;
        outer = 0.0F;
        return;
    }
    center = static_cast<float>(kLtsCoefficients[index][0]) * kQ31ToFloat;
    adjacent = static_cast<float>(kLtsCoefficients[index][1]) * kQ31ToFloat;
    outer = static_cast<float>(kLtsCoefficients[index][2]) * kQ31ToFloat;
}

const std::array<std::int32_t, 102>& lts_window_for_delta(const float delta) noexcept {
    // Sony Initialize: a1+40 = BASE, a1+48 = LO, a1+56 = HI.
    // ApplyTimeDomainFilter: |Δc| < 0.25 → LO, 0.25 ≤ |Δc| < 0.5 → BASE, else HI.
    if (delta < 0.25F) {
        return native_lts_windows::kLtsLoWindow102;
    }
    if (delta < 0.5F) {
        return native_lts_windows::kLtsWindow102;
    }
    return native_lts_windows::kLtsHiWindow102;
}

} // namespace

bool ace_lts_process_f32(
    AceLtsHistory& state,
    std::vector<float>& samples,
    const bool enabled,
    const std::uint32_t lag,
    const std::uint32_t filter_index) noexcept {
    if (samples.empty()) {
        return false;
    }
    const std::uint32_t curr_index = enabled ? filter_index : 0U;
    const std::uint32_t curr_lag = enabled ? lag : 0U;
    if (curr_index >= 29U) {
        return false;
    }
    const std::uint32_t prev_index = state.filter_index;
    const std::uint32_t prev_lag = static_cast<std::uint32_t>(state.lag);
    const bool curr_on = curr_index != 0U;
    const bool prev_on = prev_index != 0U;

    std::vector<float> buffer;
    buffer.reserve(state.pcm.size() + samples.size());
    buffer.insert(buffer.end(), state.pcm.begin(), state.pcm.end());
    const std::ptrdiff_t origin = static_cast<std::ptrdiff_t>(buffer.size());
    buffer.insert(buffer.end(), samples.begin(), samples.end());

    float c_new = 0.0F;
    float a_new = 0.0F;
    float o_new = 0.0F;
    float c_old = 0.0F;
    float a_old = 0.0F;
    float o_old = 0.0F;
    lts_coeffs(curr_index, c_new, a_new, o_new);
    lts_coeffs(prev_index, c_old, a_old, o_old);

    const bool same_lag = prev_on && curr_on && prev_lag == curr_lag
        && curr_lag != 15U;
    const bool head_window = !same_lag || prev_index != curr_index;
    const auto& window = lts_window_for_delta(std::fabs(c_new - c_old));
    const std::size_t head = head_window
        ? (std::min)(kLtsWindow, samples.size()) : 0U;

    for (std::size_t n = 0U; n < samples.size(); ++n) {
        const std::ptrdiff_t index = origin + static_cast<std::ptrdiff_t>(n);
        float value = buffer[static_cast<std::size_t>(index)];
        if (curr_on || prev_on) {
            if (n < head) {
                const float mix = static_cast<float>(window[n]) * kQ30ToFloat;
                if (curr_on && prev_on && !same_lag) {
                    const float old_y = lts_fir(
                        buffer, index, prev_lag, c_old, a_old, o_old);
                    const float new_y = lts_fir(
                        buffer, index, curr_lag, c_new, a_new, o_new);
                    value = old_y + mix * (new_y - old_y);
                } else if (curr_on && prev_on) {
                    const float c = c_old + (c_new - c_old) * mix;
                    const float a = a_old + (a_new - a_old) * mix;
                    const float o = o_old + (o_new - o_old) * mix;
                    value = lts_fir(buffer, index, curr_lag, c, a, o);
                } else if (curr_on) {
                    value = lts_fir(
                        buffer, index, curr_lag,
                        c_new * mix, a_new * mix, o_new * mix);
                } else {
                    const float dry = 1.0F - mix;
                    value = lts_fir(
                        buffer, index, prev_lag,
                        c_old * dry, a_old * dry, o_old * dry);
                }
            } else if (curr_on) {
                value = lts_fir(
                    buffer, index, curr_lag, c_new, a_new, o_new);
            }
        }
        buffer[static_cast<std::size_t>(index)] = value;
        samples[n] = value;
    }

    const std::size_t keep = (std::min)(kLtsFrame, buffer.size());
    state.pcm.assign(buffer.end() - static_cast<std::ptrdiff_t>(keep),
                     buffer.end());
    state.lag = curr_on ? static_cast<std::size_t>(curr_lag) : 0U;
    state.filter_index = curr_index;
    std::int32_t center = 0;
    std::int32_t adjacent = 0;
    std::int32_t outer = 0;
    (void)ace_lts_filter_coefficients(curr_index, center, adjacent, outer);
    state.center_q31 = center;
    state.adjacent_q31 = adjacent;
    state.outer_q31 = outer;
    return true;
}

} // namespace dtsx
