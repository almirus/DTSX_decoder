#include "dtsx/ace_mdct.hpp"
#include "dtsx/ace_mdct_idle_table.generated.hpp"
#include "dtsx/ace_mdct_swap_table.generated.hpp"
#include "dtsx/ace_lts_window_table.generated.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>

namespace dtsx {

bool ace_mdct_set_window_size(
    const std::size_t window_size,
    std::uint32_t& exponent) noexcept {
    if (window_size < 128U || window_size > 1024U) {
        return false;
    }
    std::size_t value = window_size;
    exponent = 0U;
    do {
        ++exponent;
        value >>= 1U;
    } while (value != 0U);
    --exponent;
    return true;
}

void AceMdctHistoryBank::reset(const std::size_t channel_count) noexcept {
    channels_.assign(channel_count, {});
    float_channels_.assign(channel_count, {});
    previous_transform_size_ = 1024U;
}

float* AceMdctHistoryBank::float_plane(const std::size_t index) noexcept {
    if (index >= float_channels_.size()) {
        return nullptr;
    }
    return float_channels_[index].data();
}

std::size_t AceMdctHistoryBank::channel_count() const noexcept {
    return channels_.size();
}

std::int32_t* AceMdctHistoryBank::channel(
    const std::size_t index) noexcept {
    if (index >= channels_.size()) {
        return nullptr;
    }
    return channels_[index].data();
}

const std::int32_t* AceMdctHistoryBank::channel(
    const std::size_t index) const noexcept {
    if (index >= channels_.size()) {
        return nullptr;
    }
    return channels_[index].data();
}

std::size_t AceMdctHistoryBank::previous_transform_size() const noexcept {
    return previous_transform_size_;
}

void AceMdctHistoryBank::set_transform_size(const std::size_t size) noexcept {
    previous_transform_size_ = size;
}

bool AceMdctHistoryBank::overlap_add_q31(
    const std::size_t channel_index,
    const std::size_t history_offset,
    std::vector<std::int32_t>& transformed,
    const std::vector<std::int32_t>& low_window,
    const std::vector<std::int32_t>& high_window) noexcept {
    if (channel_index >= channels_.size()
        || history_offset > kHistorySamples
        || transformed.size() != low_window.size()
        || transformed.size() != high_window.size()
        || transformed.empty()
        || transformed.size() > kHistorySamples - history_offset) {
        return false;
    }
    std::vector<std::int32_t> history(
        channels_[channel_index].begin()
            + static_cast<std::ptrdiff_t>(history_offset),
        channels_[channel_index].begin()
            + static_cast<std::ptrdiff_t>(history_offset
                + transformed.size()));
    if (!ace_overlap_add_q31_reference(
            history, transformed, low_window, high_window)) {
        return false;
    }
    std::copy(history.begin(), history.end(),
        channels_[channel_index].begin()
            + static_cast<std::ptrdiff_t>(history_offset));
    return true;
}

bool ace_mdct_window_coefficients(
    const AceMdctWindowSize size,
    std::vector<std::int32_t>& coefficients) noexcept {
    using namespace native_lts_windows;
    switch (size) {
    case AceMdctWindowSize::Size128:
        coefficients.assign(kMdctWindow128.begin(), kMdctWindow128.end());
        return true;
    case AceMdctWindowSize::Size256:
        coefficients.assign(kMdctWindow256.begin(), kMdctWindow256.end());
        return true;
    case AceMdctWindowSize::Size512:
        coefficients.assign(kMdctWindow512.begin(), kMdctWindow512.end());
        return true;
    case AceMdctWindowSize::Size1024:
        coefficients.assign(kMdctWindow1024.begin(), kMdctWindow1024.end());
        return true;
    }
    coefficients.clear();
    return false;
}

bool ace_mdct_overlap_window_halves(
    const AceMdctWindowSize size,
    std::vector<std::int32_t>& low,
    std::vector<std::int32_t>& high) noexcept {
    std::vector<std::int32_t> table;
    if (!ace_mdct_window_coefficients(size, table)
        || table.empty() || (table.size() & 1U) != 0U) {
        low.clear();
        high.clear();
        return false;
    }
    const std::size_t half = table.size() / 2U;
    low.assign(table.begin(), table.begin()
        + static_cast<std::ptrdiff_t>(half));
    high.resize(half);
    for (std::size_t index = 0U; index < half; ++index) {
        high[index] = table[half - index - 1U];
    }
    return true;
}

bool ace_mdct_bitreversal_permutation_reference(
    std::vector<std::int32_t>& values,
    const std::uint32_t exponent) noexcept {
    if (values.size() != 1024U || exponent < 7U || exponent > 10U) {
        return false;
    }
    const std::size_t table_index = exponent - 7U;
    using namespace native_mdct_swap_tables;
    const auto apply = [&values](const auto& table) noexcept {
        for (std::size_t i = 0U; i < table.size(); i += 4U) {
            const std::size_t a = table[i];
            const std::size_t b = table[i + 1U];
            const std::size_t c = table[i + 2U];
            const std::size_t d = table[i + 3U];
            if (a >= values.size() || b >= values.size()
                || c >= values.size() || d >= values.size()) {
                return false;
            }
            const std::int32_t va = values[a];
            const std::int32_t vc = values[c];
            values[a] = values[b];
            values[c] = values[d];
            values[b] = va;
            values[d] = vc;
        }
        return true;
    };
    switch (table_index) {
    case 0U:
        return apply(kSwapTable0);
    case 1U:
        return apply(kSwapTable1);
    case 2U:
        return apply(kSwapTable2);
    default:
        return apply(kSwapTable3);
    }
}

bool ace_mdct_walsh_transform_reference(
    std::vector<std::int32_t>& values) noexcept {
    const std::size_t length = values.size();
    if (length < 2U || length > 1024U
        || (length & (length - 1U)) != 0U) {
        return false;
    }
    const auto wrap_add = [](const std::int32_t left,
                             const std::int32_t right) noexcept {
        return static_cast<std::int32_t>(
            static_cast<std::uint32_t>(left)
            + static_cast<std::uint32_t>(right));
    };
    const auto wrap_sub = [](const std::int32_t left,
                             const std::int32_t right) noexcept {
        return static_cast<std::int32_t>(
            static_cast<std::uint32_t>(left)
            - static_cast<std::uint32_t>(right));
    };
    for (std::size_t span = 1U; span < length; span <<= 1U) {
        const std::size_t step = span << 1U;
        for (std::size_t base = 0U; base < length; base += step) {
            for (std::size_t offset = 0U; offset < span; ++offset) {
                const std::int32_t first = values[base + offset];
                const std::int32_t second = values[base + span + offset];
                values[base + offset] = wrap_add(first, second);
                values[base + span + offset] = wrap_sub(first, second);
            }
        }
    }
    return true;
}

bool ace_mdct_walsh_native_reference(
    std::vector<std::int32_t>& values,
    const std::uint32_t exponent) noexcept {
    if (values.size() != 1024U || exponent < 7U || exponent > 10U) {
        return false;
    }
    using Vec = std::array<std::int32_t, 4U>;
    const auto add = [](const Vec& left, const Vec& right) noexcept {
        Vec result{};
        for (std::size_t i = 0U; i < result.size(); ++i) {
            result[i] = static_cast<std::int32_t>(
                static_cast<std::uint32_t>(left[i])
                + static_cast<std::uint32_t>(right[i]));
        }
        return result;
    };
    const auto sub = [](const Vec& left, const Vec& right) noexcept {
        Vec result{};
        for (std::size_t i = 0U; i < result.size(); ++i) {
            result[i] = static_cast<std::int32_t>(
                static_cast<std::uint32_t>(left[i])
                - static_cast<std::uint32_t>(right[i]));
        }
        return result;
    };
    const auto load = [&values](const std::size_t offset) noexcept {
        return Vec{values[offset], values[offset + 1U], values[offset + 2U],
            values[offset + 3U]};
    };
    const auto store = [&values](const std::size_t offset, const Vec& value) {
        for (std::size_t i = 0U; i < value.size(); ++i) {
            values[offset + i] = value[i];
        }
    };
    const auto butterfly_vectors = [&](const std::size_t low,
                                       const std::size_t high) noexcept {
        const Vec left = load(low);
        const Vec right = load(high);
        store(low, add(left, right));
        store(high, sub(left, right));
    };

    std::uint32_t outer_count = 1U;
    std::size_t span = std::size_t{1U} << (exponent - 1U);
    for (std::uint32_t stage = 0U; stage < exponent;
         ++stage, outer_count <<= 1U, span >>= 1U) {
        if (span >= 16U) {
            // First large pass: even outer lanes. Native v8 starts at word
            // 12, and each lane advances by 4*span words.
            for (std::uint32_t outer = 0U; outer < outer_count; outer += 2U) {
                const std::size_t lane = 12U
                    + static_cast<std::size_t>(outer / 2U) * 4U * span;
                for (std::size_t group = 0U; group < span / 16U; ++group) {
                    const std::size_t base = lane + group * 16U;
                    for (std::size_t vector = 0U; vector < 4U; ++vector) {
                        butterfly_vectors(
                            base - 12U + vector * 4U,
                            base - 12U + span + vector * 4U);
                    }
                }
            }
            for (std::uint32_t outer = 1U; outer < outer_count; outer += 2U) {
                const std::size_t lane = static_cast<std::size_t>((outer - 1U) / 2U)
                    * 4U * span;
                for (std::size_t group = 0U; group < span / 16U; ++group) {
                    const std::size_t base = lane + group * 16U;
                    for (std::size_t vector = 0U; vector < 4U; ++vector) {
                        butterfly_vectors(
                            base + 2U * span + vector * 4U,
                            base + 3U * span + vector * 4U);
                    }
                }
            }
        } else if (span == 8U) {
            // v6 == 8, even lanes: two vectors in each 8-word group.
            for (std::uint32_t outer = 0U; outer < outer_count; outer += 2U) {
                const std::size_t base = static_cast<std::size_t>(outer / 2U)
                    * 4U * span;
                for (std::size_t offset = 0U; offset < 8U; offset += 4U) {
                    butterfly_vectors(base + offset, base + span + offset);
                }
            }
            // v6 == 8, odd lanes use the packed +16-word quadrant.
            for (std::uint32_t outer = 1U; outer < outer_count; outer += 2U) {
                const std::size_t base = 16U
                    + static_cast<std::size_t>((outer - 1U) / 2U)
                        * 32U;
                butterfly_vectors(base, base + 8U);
                butterfly_vectors(base + 4U, base + 12U);
            }
        } else if (span == 4U) {
            for (std::uint32_t outer = 0U; outer < outer_count; outer += 2U) {
                const std::size_t base = static_cast<std::size_t>(outer / 2U)
                    * 4U * span;
                butterfly_vectors(base, base + span);
            }
            for (std::uint32_t outer = 1U; outer < outer_count; outer += 2U) {
                const std::size_t base = 8U
                    + static_cast<std::size_t>((outer - 1U) / 2U) * 16U;
                const Vec left = load(base);
                const Vec right = load(base + 4U);
                store(base, sub(left, right));
                store(base + 4U, add(right, left));
            }
        } else {
            if (span == 0U) {
                return false;
            }
            for (std::uint32_t outer = 0U; outer < outer_count; outer += 2U) {
                const std::size_t base = static_cast<std::size_t>(outer / 2U)
                    * 4U * span;
                for (std::size_t offset = 0U; offset < span; ++offset) {
                    const std::int32_t left = values[base + offset];
                    const std::int32_t right = values[base + span + offset];
                    values[base + offset] = static_cast<std::int32_t>(
                        static_cast<std::uint32_t>(left)
                        + static_cast<std::uint32_t>(right));
                    values[base + span + offset] = static_cast<std::int32_t>(
                        static_cast<std::uint32_t>(left)
                        - static_cast<std::uint32_t>(right));
                }
            }
            for (std::uint32_t outer = 1U; outer < outer_count; outer += 2U) {
                const std::size_t base = 2U * span
                    + static_cast<std::size_t>((outer - 1U) / 2U)
                        * 4U * span;
                for (std::size_t offset = 0U; offset < span; ++offset) {
                    const std::int32_t left = values[base + offset];
                    const std::int32_t right = values[base + span + offset];
                    values[base + offset] = static_cast<std::int32_t>(
                        static_cast<std::uint32_t>(left)
                        - static_cast<std::uint32_t>(right));
                    values[base + span + offset] = static_cast<std::int32_t>(
                        static_cast<std::uint32_t>(right)
                        + static_cast<std::uint32_t>(left));
                }
            }
        }
    }
    return true;
}

bool ace_mdct_walsh_tcl_scalar_reference(
    std::vector<std::int32_t>& values,
    const std::uint32_t exponent) noexcept {
    if (values.size() != 1024U || exponent < 7U || exponent > 10U) {
        return false;
    }
    const auto add = [](const std::int32_t left,
                        const std::int32_t right) noexcept {
        return static_cast<std::int32_t>(
            static_cast<std::uint32_t>(left)
            + static_cast<std::uint32_t>(right));
    };
    const auto sub = [](const std::int32_t left,
                        const std::int32_t right) noexcept {
        return static_cast<std::int32_t>(
            static_cast<std::uint32_t>(left)
            - static_cast<std::uint32_t>(right));
    };
    // Literal scalar form of the TCL ARM Walsh body.  The NEON branches use
    // the same packed addresses; this keeps lane order explicit.
    std::uint32_t outer_count = 1U;
    for (std::size_t span = std::size_t{1U} << (exponent - 1U);
         span != 0U; span >>= 1U, outer_count <<= 1U) {
        const std::size_t lane_stride = 4U * span;
        for (std::uint32_t outer = 0U; outer < outer_count; outer += 2U) {
            const std::size_t base = static_cast<std::size_t>(outer / 2U)
                * lane_stride;
            for (std::size_t offset = 0U; offset < span; ++offset) {
                const std::size_t low = base + offset;
                const std::size_t high = low + span;
                const std::int32_t left = values[low];
                const std::int32_t right = values[high];
                values[low] = add(left, right);
                values[high] = sub(left, right);
            }
        }
        for (std::uint32_t outer = 1U; outer < outer_count; outer += 2U) {
            const std::size_t base = 2U * span
                + static_cast<std::size_t>((outer - 1U) / 2U)
                    * lane_stride;
            for (std::size_t offset = 0U; offset < span; ++offset) {
                const std::size_t low = base + offset;
                const std::size_t high = low + span;
                const std::int32_t left = values[low];
                const std::int32_t right = values[high];
                values[low] = sub(left, right);
                values[high] = add(right, left);
            }
        }
    }
    return true;
}

bool ace_mdct_walsh_first_large_stage_native_reference(
    std::vector<std::int32_t>& workspace,
    const std::uint32_t exponent) noexcept {
    if (workspace.size() != 1024U || exponent < 7U || exponent > 10U) {
        return false;
    }
    const std::size_t span = std::size_t{1U} << (exponent - 1U);
    if (span < 16U) return false;
    const auto add = [](const std::int32_t left, const std::int32_t right) {
        return static_cast<std::int32_t>(static_cast<std::uint32_t>(left)
            + static_cast<std::uint32_t>(right));
    };
    const auto sub = [](const std::int32_t left, const std::int32_t right) {
        return static_cast<std::int32_t>(static_cast<std::uint32_t>(left)
            - static_cast<std::uint32_t>(right));
    };
    // Homatic WalshTransform, v6 >= 16, first outer pass (v5 == 1):
    // v13/v12 begin at a2 + 48 bytes and advance by one int32x4_t quartet.
    const std::size_t groups = span >> 4U;
    for (std::size_t group = 0U; group < groups; ++group) {
        const std::size_t pos = 12U + group * 16U;
        // v9 is 4 * v6 bytes, not words.
        const std::size_t high = pos + span;
        if (high >= workspace.size() || pos < 12U) return false;
        const std::array<std::int32_t, 4U> low{
            workspace[pos - 12U], workspace[pos - 8U],
            workspace[pos - 4U], workspace[pos]};
        const std::array<std::int32_t, 4U> upper{
            workspace[high - 12U], workspace[high - 8U],
            workspace[high - 4U], workspace[high]};
        workspace[pos - 12U] = add(upper[0], low[0]);
        workspace[pos - 8U] = add(upper[1], low[1]);
        workspace[pos - 4U] = add(upper[2], low[2]);
        workspace[pos] = add(upper[3], low[3]);
        workspace[high - 12U] = sub(low[0], upper[0]);
        workspace[high - 8U] = sub(low[1], upper[1]);
        workspace[high - 4U] = sub(low[2], upper[2]);
        workspace[high] = sub(low[3], upper[3]);
    }
    return true;
}

bool ace_mdct_transform_ordering_reference(
    std::vector<std::int32_t>& values) noexcept {
    if (!ace_mdct_walsh_transform_reference(values)) {
        return false;
    }
    for (std::size_t left = 0U, right = values.size() - 1U;
         left < values.size() / 2U; ++left, --right) {
        std::swap(values[left], values[right]);
    }
    return true;
}

std::int32_t ace_qrdmulh_s32(
    const std::int32_t left,
    const std::int32_t right) noexcept {
    constexpr std::int64_t kMin =
        static_cast<std::int64_t>((std::numeric_limits<std::int32_t>::min)());
    constexpr std::int64_t kMax =
        static_cast<std::int64_t>((std::numeric_limits<std::int32_t>::max)());
    // SQrdMulH saturates the only overflowing doubled product.
    if (left == kMin && right == kMin) {
        return static_cast<std::int32_t>(kMax);
    }
    const std::int64_t product = static_cast<std::int64_t>(left)
        * static_cast<std::int64_t>(right);
    const std::int64_t rounded = (product * 2LL + 0x80000000LL) >> 32U;
    return static_cast<std::int32_t>((std::max)(kMin,
        (std::min)(kMax, rounded)));
}

bool ace_mdct_idle_tables(
    const std::uint32_t exponent,
    std::vector<std::int32_t>& ltwidle,
    std::vector<std::int32_t>& utwidle) noexcept {
    using namespace native_mdct_idle_tables;
    ltwidle.clear();
    utwidle.clear();
    switch (exponent) {
    case 1U:
        ltwidle.assign(kLtwidleOrder1.begin(), kLtwidleOrder1.end());
        utwidle.assign(kUtwidleOrder1.begin(), kUtwidleOrder1.end());
        break;
    case 2U:
        ltwidle.assign(kLtwidleOrder2.begin(), kLtwidleOrder2.end());
        utwidle.assign(kUtwidleOrder2.begin(), kUtwidleOrder2.end());
        break;
    case 3U:
        ltwidle.assign(kLtwidleOrder3.begin(), kLtwidleOrder3.end());
        utwidle.assign(kUtwidleOrder3.begin(), kUtwidleOrder3.end());
        break;
    case 4U:
        ltwidle.assign(kLtwidleOrder4.begin(), kLtwidleOrder4.end());
        utwidle.assign(kUtwidleOrder4.begin(), kUtwidleOrder4.end());
        break;
    case 5U:
        ltwidle.assign(kLtwidleOrder5.begin(), kLtwidleOrder5.end());
        utwidle.assign(kUtwidleOrder5.begin(), kUtwidleOrder5.end());
        break;
    case 6U:
        ltwidle.assign(kLtwidleOrder6.begin(), kLtwidleOrder6.end());
        utwidle.assign(kUtwidleOrder6.begin(), kUtwidleOrder6.end());
        break;
    case 7U:
        ltwidle.assign(kLtwidleOrder7.begin(), kLtwidleOrder7.end());
        utwidle.assign(kUtwidleOrder7.begin(), kUtwidleOrder7.end());
        break;
    case 8U:
        ltwidle.assign(kLtwidleOrder8.begin(), kLtwidleOrder8.end());
        utwidle.assign(kUtwidleOrder8.begin(), kUtwidleOrder8.end());
        break;
    case 9U:
        ltwidle.assign(kLtwidleOrder9.begin(), kLtwidleOrder9.end());
        utwidle.assign(kUtwidleOrder9.begin(), kUtwidleOrder9.end());
        break;
    case 10U:
        ltwidle.assign(kLtwidleOrder10.begin(), kLtwidleOrder10.end());
        utwidle.assign(kUtwidleOrder10.begin(), kUtwidleOrder10.end());
        break;
    default:
        return false;
    }
    return ltwidle.size() == (std::size_t{1U} << (exponent - 1U))
        && utwidle.size() == ltwidle.size();
}

namespace {

struct MdctPair final {
    std::int32_t first = 0;
    std::int32_t second = 0;
};

MdctPair mdct_pair_add(const MdctPair a, const MdctPair b) noexcept {
    return {static_cast<std::int32_t>(
                static_cast<std::uint32_t>(a.first)
                + static_cast<std::uint32_t>(b.first)),
        static_cast<std::int32_t>(static_cast<std::uint32_t>(a.second)
                                   + static_cast<std::uint32_t>(b.second))};
}

MdctPair mdct_pair_sub(const MdctPair a, const MdctPair b) noexcept {
    return {static_cast<std::int32_t>(
                static_cast<std::uint32_t>(a.first)
                - static_cast<std::uint32_t>(b.first)),
        static_cast<std::int32_t>(static_cast<std::uint32_t>(a.second)
                                   - static_cast<std::uint32_t>(b.second))};
}

MdctPair mdct_pair_mul(const MdctPair a, const MdctPair b) noexcept {
    return {ace_qrdmulh_s32(a.first, b.first),
        ace_qrdmulh_s32(a.second, b.second)};
}

MdctPair mdct_pair_reverse(const MdctPair value) noexcept {
    return {value.second, value.first};
}

MdctPair mdct_pair_even_transpose(
    const MdctPair a, const MdctPair b) noexcept {
    return {a.first, b.first};
}

MdctPair mdct_pair_odd_transpose(
    const MdctPair a, const MdctPair b) noexcept {
    return {a.second, b.second};
}

MdctPair mdct_pair_load(
    const std::vector<std::int32_t>& values,
    const std::size_t offset) noexcept {
    return {values[offset], values[offset + 1U]};
}

void mdct_pair_store(
    std::vector<std::int32_t>& values,
    const std::size_t offset,
    const MdctPair value) noexcept {
    values[offset] = value.first;
    values[offset + 1U] = value.second;
}

} // namespace

bool ace_mdct_backward_rotate_initial_stage(
    std::vector<std::int32_t>& values,
    const std::uint32_t exponent) noexcept {
    if (exponent < 3U || exponent > 10U
        || (values.size() != (std::size_t{1U} << exponent)
            && values.size() != 1024U)) {
        return false;
    }
    std::vector<std::int32_t> ltw1_table;
    std::vector<std::int32_t> utw1_table;
    std::vector<std::int32_t> ltw2_table;
    std::vector<std::int32_t> utw2_table;
    if (!ace_mdct_idle_tables(1U, ltw1_table, utw1_table)
        || !ace_mdct_idle_tables(2U, ltw2_table, utw2_table)) {
        return false;
    }
    std::vector<std::int32_t> ltw3;
    std::vector<std::int32_t> utw3;
    if (!ace_mdct_idle_tables(3U, ltw3, utw3)) {
        return false;
    }
    const MdctPair ltw1{ltw1_table[0], ltw1_table[0]};
    const MdctPair utw1{utw1_table[0], utw1_table[0]};
    const MdctPair ltw2 = mdct_pair_reverse(
        {ltw2_table[0], ltw2_table[1]});
    const MdctPair utw2 = mdct_pair_reverse(
        {utw2_table[0], utw2_table[1]});
    const MdctPair ltw3_first = mdct_pair_reverse({ltw3[0], ltw3[1]});
    const MdctPair ltw3_second{ltw3[2], ltw3[3]};
    const MdctPair utw3_first = mdct_pair_reverse({utw3[0], utw3[1]});
    const MdctPair utw3_second{utw3[2], utw3[3]};
    // The initial native loop uses order-1 constants and the first pairs of
    // orders 2/3.  Its body is kept in a dedicated helper until the middle
    // radix stages are connected to the inverse pipeline.
    // The native routine always receives the 1024-word workspace, but the
    // exponent selects the active transform prefix.  In the ARM code the
    // loop bound is derived from the exponent, not from the allocation size.
    const std::size_t active_size = std::size_t{1U} << exponent;
    const std::size_t blocks = active_size / 8U;
    for (std::size_t block = 0U; block < blocks; ++block) {
        const std::size_t base = block * 8U;
        const MdctPair x0{values[base], values[base + 2U]};
        const MdctPair x1{values[base + 1U], values[base + 3U]};
        const MdctPair x2{values[base + 4U], values[base + 6U]};
        const MdctPair x3{values[base + 5U], values[base + 7U]};
        const MdctPair p = mdct_pair_sub(x1, mdct_pair_mul(x0, ltw1));
        const MdctPair q = mdct_pair_sub(x3, mdct_pair_mul(x2, ltw1));
        const MdctPair a = mdct_pair_add(mdct_pair_mul(p, utw1), x0);
        const MdctPair b = mdct_pair_add(mdct_pair_mul(q, utw1), x2);
        const MdctPair r = mdct_pair_sub(p, mdct_pair_mul(a, ltw1));
        const MdctPair s = mdct_pair_sub(q, mdct_pair_mul(b, ltw1));
        const MdctPair t = mdct_pair_even_transpose(r, a);
        const MdctPair u = mdct_pair_odd_transpose(a, r);
        const MdctPair v = mdct_pair_even_transpose(s, b);
        const MdctPair w = mdct_pair_odd_transpose(b, s);
        const MdctPair e = mdct_pair_sub(u, mdct_pair_mul(t, ltw2));
        const MdctPair f = mdct_pair_sub(w, mdct_pair_mul(v, ltw2));
        const MdctPair y = mdct_pair_add(mdct_pair_mul(e, utw2), t);
        const MdctPair x = mdct_pair_add(mdct_pair_mul(f, utw2), v);
        const MdctPair i = mdct_pair_sub(e, mdct_pair_mul(y, ltw2));
        const MdctPair j = mdct_pair_sub(
            mdct_pair_sub(f, mdct_pair_mul(x, ltw2)),
            mdct_pair_mul(y, ltw3_first));
        const MdctPair x_final = mdct_pair_sub(
            x, mdct_pair_mul(i, ltw3_second));
        const MdctPair y_final = mdct_pair_add(
            mdct_pair_mul(j, utw3_first), y);
        const MdctPair v31 = mdct_pair_add(
            mdct_pair_mul(x_final, utw3_second), i);
        mdct_pair_store(values, base, mdct_pair_reverse(y_final));
        // Homatic BackwardRotate stores v31 at v10[1] before writing the
        // reversed residual at v21 (v10 + 2).
        mdct_pair_store(values, base + 2U, v31);
        mdct_pair_store(values, base + 4U, mdct_pair_reverse(
            mdct_pair_sub(x_final, mdct_pair_mul(v31, ltw3_second))));
        mdct_pair_store(values, base + 6U, mdct_pair_sub(
            j, mdct_pair_mul(y_final, ltw3_first)));
    }
    return true;
}

bool reverse_ordering_prefix_native_reference(
    std::vector<std::int32_t>& values,
    const std::size_t length) noexcept {
    if (length < 16U || length > 1024U || (length & 15U) != 0U) {
        return false;
    }
    if (length > values.size()) {
        return false;
    }
    for (std::size_t left = 0U, right = length - 1U;
         left < right; ++left, --right) {
        std::swap(values[left], values[right]);
    }
    return true;
}

bool ace_mdct_reverse_ordering_native_reference(
    std::vector<std::int32_t>& values) noexcept {
    return reverse_ordering_prefix_native_reference(values, values.size());
}

bool ace_mdct_backward_rotate_native_reference(
    std::vector<std::int32_t>& values,
    const std::uint32_t exponent) noexcept {
    if (!ace_mdct_backward_rotate_initial_stage(values, exponent)) {
        return false;
    }
    if (exponent < 3U || exponent > 10U) {
        return false;
    }
    std::vector<std::int32_t> ltw;
    std::vector<std::int32_t> utw;
    const auto get_tables = [&ltw, &utw](const std::uint32_t order) {
        return ace_mdct_idle_tables(order, ltw, utw);
    };
    bool valid_access = true;
    const auto load = [&values, &valid_access](
                          const std::int64_t byte_offset) noexcept {
        if (byte_offset < 0 || (byte_offset & 3LL) != 0
            || static_cast<std::uint64_t>(byte_offset / 4 + 1)
                   >= values.size()) {
            valid_access = false;
            return MdctPair{};
        }
        const std::size_t index = static_cast<std::size_t>(byte_offset / 4);
        return MdctPair{values[index], values[index + 1U]};
    };
    const auto store = [&values, &valid_access](
                           const std::int64_t byte_offset,
                           const MdctPair pair) noexcept {
        if (byte_offset < 0 || (byte_offset & 3LL) != 0
            || static_cast<std::uint64_t>(byte_offset / 4 + 1)
                   >= values.size()) {
            valid_access = false;
            return;
        }
        const std::size_t index = static_cast<std::size_t>(byte_offset / 4);
        values[index] = pair.first;
        values[index + 1U] = pair.second;
    };
    const std::uint32_t half = 1U << (exponent - 1U);
    std::uint32_t v33 = half >> 3U;
    std::uint32_t v34 = 16U;
    std::uint32_t v107 = 3U;
    const std::uint32_t v102 = exponent - 2U;
    while (v107 < v102) {
        if (v33 != 0U && v34 >= 16U) {
            const std::uint32_t groups = v33;
            const std::uint32_t inner = v34 >> 4U;
            if (!get_tables(v107 + 1U)) {
                return false;
            }
            for (std::uint32_t group = 0U; group < groups; ++group) {
                std::int64_t v38 = static_cast<std::int64_t>(4U * v34)
                    + static_cast<std::int64_t>(group) *
                        static_cast<std::int64_t>(4U * v34);
                const std::int64_t v37 =
                    static_cast<std::int64_t>(group) * 4LL
                    * static_cast<std::int64_t>(v34);
                for (std::uint32_t item = 0U; item < inner; ++item) {
                    const std::int64_t a2_offset = v37
                        + static_cast<std::int64_t>(item) * 32LL;
                    const std::int64_t v47_offset = a2_offset;
                    const std::int64_t v51_offset = a2_offset + 16LL;
                    const std::int64_t v48_offset =
                        -32LL + v38;
                    const std::int64_t v45_offset =
                        -8LL + v38;
                    const MdctPair v52 = load(v47_offset);
                    const MdctPair v58 = load(v51_offset);
                    const MdctPair v56 = load(v47_offset + 8LL);
                    const MdctPair v57 = load(v47_offset + 24LL);
                    const std::size_t tw = static_cast<std::size_t>(item) * 8U;
                    if (tw + 7U >= ltw.size() || tw + 7U >= utw.size()) {
                        return false;
                    }
                    const MdctPair v59 = mdct_pair_sub(
                        mdct_pair_reverse(load(v48_offset + 24LL)),
                        mdct_pair_mul(v52, {ltw[tw], ltw[tw + 1U]}));
                    const MdctPair v61 = mdct_pair_sub(
                        mdct_pair_reverse(load(v48_offset + 16LL)),
                        mdct_pair_mul(v56, {ltw[tw + 2U], ltw[tw + 3U]}));
                    const MdctPair v65 = mdct_pair_sub(
                        mdct_pair_reverse(load(v48_offset)),
                        mdct_pair_mul(v57, {ltw[tw + 6U], ltw[tw + 7U]}));
                    const MdctPair v67 = mdct_pair_sub(
                        mdct_pair_reverse(load(v48_offset + 8LL)),
                        mdct_pair_mul(v58, {ltw[tw + 4U], ltw[tw + 5U]}));
                    const MdctPair v63 = mdct_pair_mul(v59,
                        {utw[tw], utw[tw + 1U]});
                    const MdctPair v68 = mdct_pair_add(v63, v52);
                    const MdctPair v69 = mdct_pair_add(
                        mdct_pair_mul(v65, {utw[tw + 6U], utw[tw + 7U]}), v57);
                    const MdctPair v70 = mdct_pair_add(
                        mdct_pair_mul(v61, {utw[tw + 2U], utw[tw + 3U]}), v56);
                    const MdctPair v71 = mdct_pair_mul(v69,
                        {ltw[tw + 6U], ltw[tw + 7U]});
                    const MdctPair v72 = mdct_pair_add(
                        mdct_pair_mul(v67, {utw[tw + 4U], utw[tw + 5U]}), v58);
                    store(v47_offset, v68);
                    store(v47_offset + 8LL, v70);
                    store(v47_offset + 24LL, v69);
                    store(v51_offset, v72);
                    store(v45_offset, mdct_pair_reverse(
                        mdct_pair_sub(v59, mdct_pair_mul(v68,
                            {ltw[tw], ltw[tw + 1U]}))));
                    store(v45_offset - 8LL, mdct_pair_reverse(
                        mdct_pair_sub(v61, mdct_pair_mul(v70,
                            {ltw[tw + 2U], ltw[tw + 3U]}))));
                    store(v45_offset - 16LL, mdct_pair_reverse(
                        mdct_pair_sub(v67, mdct_pair_mul(v72,
                            {ltw[tw + 4U], ltw[tw + 5U]}))));
                    store(v45_offset - 24LL, mdct_pair_reverse(
                        mdct_pair_sub(v65, v71)));
                    v38 -= 32LL;
                }
            }
        }
        v33 >>= 1U;
        ++v107;
        v34 <<= 1U;
    }
    // `v34` is doubled once per completed middle radix stage.  In the
    // Homatic body the tail receives 64 for exponent 7 (not 32): the native
    // seed is 16 and the loop executes through v107 == v102 - 1.
    v34 = (v102 < 4U ? 16U : (1U << (exponent - 1U)));
    if ((v34 >> 2U) == 0U) {
        return true;
    }
    std::vector<std::int32_t> ltw_next;
    std::vector<std::int32_t> utw_next;
    // The native tail indexes DTS_{L,U}TWIDLE at [v102] and [v102 + 1].
    // The exported table-pointer array is zero-based while
    // ace_mdct_idle_tables is named by the one-based transform order.
    if (!get_tables(v102 + 1U) || ltw.size() < 2U || utw.size() < 2U
        || !ace_mdct_idle_tables(v102 + 2U, ltw_next, utw_next)
        || ltw_next.size() < 8U || utw_next.size() < 8U) {
        return false;
    }
    const std::uint32_t count = v34 >> 2U;
    for (std::uint32_t index = 0U; index < count; ++index) {
        const std::int64_t offset = static_cast<std::int64_t>(index) * 8LL;
        const std::int64_t front_hi =
            static_cast<std::int64_t>(8U * v34 - 8U) - offset;
        const std::int64_t front_lo =
            static_cast<std::int64_t>(4U * v34 - 8U) - offset;
        const std::int64_t out_hi = static_cast<std::int64_t>(4U * v34)
            + offset;
        const std::size_t twiddle_offset =
            static_cast<std::size_t>(offset / 4LL);
        const std::size_t front_twiddle_offset =
            static_cast<std::size_t>(front_lo / 4LL);
        const MdctPair v87 = load(offset);
        const MdctPair v91 = mdct_pair_load(ltw, twiddle_offset);
        const MdctPair v83 = mdct_pair_load(utw, twiddle_offset);
        const MdctPair ltw1_1 = mdct_pair_load(ltw_next, twiddle_offset);
        const MdctPair utw1_1 = mdct_pair_load(utw_next, twiddle_offset);
        const MdctPair v90 = mdct_pair_reverse(
            mdct_pair_load(ltw_next, front_twiddle_offset));
        const MdctPair v92 = mdct_pair_reverse(
            mdct_pair_load(utw_next, front_twiddle_offset));
        const MdctPair v93 = mdct_pair_sub(mdct_pair_reverse(load(front_lo)),
                                            mdct_pair_mul(v87, v91));
        const MdctPair v94 = mdct_pair_sub(mdct_pair_reverse(load(front_hi)),
                                            mdct_pair_mul(load(out_hi), v91));
        const MdctPair v95 = mdct_pair_add(mdct_pair_mul(v93, v83), v87);
        const MdctPair v96 = mdct_pair_add(mdct_pair_mul(v94, v83),
                                           load(out_hi));
        const MdctPair v97 = mdct_pair_sub(v93, mdct_pair_mul(v95, v91));
        const MdctPair v98 = mdct_pair_sub(
            mdct_pair_sub(v94, mdct_pair_mul(v96, v91)),
            mdct_pair_mul(v95, ltw1_1));
        const MdctPair v99 = mdct_pair_sub(v96,
            mdct_pair_mul(v97, v90));
        const MdctPair v100 = mdct_pair_add(
            mdct_pair_mul(v98, utw1_1), v95);
        const MdctPair v101 = mdct_pair_add(
            mdct_pair_mul(v99, v92), v97);
        store(offset, v100);
        store(front_lo, mdct_pair_reverse(v101));
        store(out_hi, mdct_pair_sub(v99,
            mdct_pair_mul(v101, v90)));
        store(front_hi, mdct_pair_reverse(mdct_pair_sub(v98,
            mdct_pair_mul(v100, ltw1_1))));
    }
    return valid_access;
}

bool ace_mdct_native_transform_reference(
    std::vector<std::int32_t>& values,
    const std::uint32_t exponent) noexcept {
    if (exponent < 7U || exponent > 10U
        || values.size() != 1024U) {
        return false;
    }
    if (!ace_mdct_bitreversal_permutation_reference(values, exponent)) {
        return false;
    }
    if (!ace_mdct_walsh_tcl_scalar_reference(values, exponent)
        || !ace_mdct_backward_rotate_native_reference(values, exponent)
        || !reverse_ordering_prefix_native_reference(
            values, std::size_t{1U} << exponent)) {
        return false;
    }
    return true;
}

float ace_inverse_sqrt_window(
    const std::size_t transform_size) noexcept {
    switch (transform_size) {
    case 128U:
        return 0.08838835F;
    case 256U:
        return 0.0625F;
    case 512U:
        return 0.04419417F;
    case 1024U:
        return 0.03125F;
    default:
        return 0.0F;
    }
}

void ace_apply_native_pcm_scale(
    std::vector<float>& samples) noexcept {
    constexpr float kNativePcmScale = 3.0517578125e-5F;
    for (float& sample : samples) {
        sample *= kNativePcmScale;
    }
}

void ace_sat_left_shift_native(
    std::vector<std::int32_t>& samples,
    const std::uint32_t shift) noexcept {
    const auto saturate = [](const std::int64_t value) noexcept {
        return static_cast<std::int32_t>((std::max)(
            static_cast<std::int64_t>((std::numeric_limits<std::int32_t>::min)()),
            (std::min)(static_cast<std::int64_t>(
                           (std::numeric_limits<std::int32_t>::max)()),
                value)));
    };
    const std::uint32_t bounded = (std::min)(shift, 31U);
    for (std::int32_t& sample : samples) {
        const std::int64_t max_value =
            (std::numeric_limits<std::int32_t>::max)();
        const std::int64_t min_value =
            (std::numeric_limits<std::int32_t>::min)();
        const std::int64_t widened = bounded == 0U
            ? static_cast<std::int64_t>(sample)
            : sample > (max_value >> bounded)
                ? max_value
                : sample < (min_value >> bounded)
                    ? min_value
                    : static_cast<std::int64_t>(sample) << bounded;
        const std::int64_t rounded = widened >= 0
            ? (widened + 128LL) >> 8U
            : -(((-widened) + 128LL) >> 8U);
        const std::int64_t restored = rounded > (max_value >> 8U)
            ? max_value
            : rounded < (min_value >> 8U)
                ? min_value
                : rounded << 8U;
        sample = saturate(restored >> 8U);
    }
}

namespace {

void apply_single_window(
    const std::vector<float>& window,
    std::vector<float>& vector) noexcept {
    const std::size_t length = window.size();
    for (std::size_t n = 0U; n < length / 2U; ++n) {
        const float v0 = window[length - n - 1U] * vector[n]
            - window[n] * vector[length - n - 1U];
        const float v1 = window[n] * vector[n]
            + window[length - n - 1U] * vector[length - n - 1U];
        vector[n] = v0;
        vector[length - n - 1U] = v1;
    }
}

} // namespace

bool ace_make_window(
    const std::size_t length,
    const std::size_t transition,
    std::vector<float>& window) noexcept {
    if (length == 0U || transition > length
        || (length & 1U) != 0U || (transition & 1U) != 0U) {
        return false;
    }
    window.assign(length, 0.0F);
    const std::size_t boundary = (length - transition) / 2U;
    for (std::size_t k = 0U; k < boundary; ++k) {
        window[k] = 0.0F;
        window[length - k - 1U] = 1.0F;
    }
    const double pi_over_two = 1.57079632679489661923132169164;
    for (std::size_t k = 0U; k < transition; ++k) {
        const double w = std::sin(
            pi_over_two * (static_cast<double>(k) + 0.5)
            / static_cast<double>(transition));
        window[boundary + k] = static_cast<float>(
            std::sin(pi_over_two * w * w));
    }
    return true;
}

bool ace_inverse_transform_reference(
    const std::vector<float>& coefficients,
    std::vector<float>& samples) noexcept {
    // DTSAceMdct_SetWindowSize accepts the four native powers of two
    // (128/256/512/1024).  The previous implementation accepted only the
    // long block, which made the short-transform branch impossible to wire
    // into the internal decoder even though its DCT4 stage is independent of
    // the overlap/history geometry.
    const std::size_t transform_size = coefficients.size();
    if (transform_size != 128U && transform_size != 256U
        && transform_size != 512U && transform_size != 1024U) {
        samples.clear();
        return false;
    }
    // DTSAceMdct_Inverse_Process scales every 0x400-float input block by
    // DTS_INVSQRTWINDOW_LENGTH[3] (0x3d000000 = 1/sqrt(1024)) before the
    // Walsh/rotation/reverse MDCT pipeline.  Keep that native scale here;
    // omitting it produces a deterministic 32x amplitude error even though
    // the transform shape looks correct.
    std::vector<float> scaled(coefficients.size());
    const float inverse_sqrt_window = ace_inverse_sqrt_window(
        transform_size);
    if (inverse_sqrt_window == 0.0F) {
        samples.clear();
        return false;
    }
    for (std::size_t index = 0U; index < coefficients.size(); ++index) {
        scaled[index] = coefficients[index] * inverse_sqrt_window;
    }
    // Closed-form DCT-IV is the current synthesis kernel: it matches the
    // overlap time order of Sony DTS_ACE_MDCT_TRANSFORM.  The literal float
    // Walsh+BackwardRotate+ReverseOrdering port is behind
    // DTSX_P2_NATIVE_MDCT_F32=1 until its Kinsetsu |corr| meets or exceeds
    // DCT-IV (2s: 0.344 vs 0.416).
    if (const char* value = std::getenv("DTSX_P2_NATIVE_MDCT_F32")) {
        if (value[0] == '1') {
            std::uint32_t exponent = 0U;
            if (!ace_mdct_set_window_size(transform_size, exponent)) {
                samples.clear();
                return false;
            }
            std::vector<float> workspace(1024U, 0.0F);
            std::copy(scaled.begin(), scaled.end(), workspace.begin());
            if (!ace_mdct_native_transform_f32(workspace, exponent)) {
                samples.clear();
                return false;
            }
            samples.assign(workspace.begin(),
                workspace.begin()
                    + static_cast<std::ptrdiff_t>(transform_size));
            return true;
        }
    }
    return ace_dct4_reference(scaled, samples);
}

namespace {

std::int32_t mdct_float_to_q31(const float value) noexcept {
    if (!std::isfinite(value)) {
        return 0;
    }
    const float clamped = (std::max)(-1.0F, (std::min)(1.0F, value));
    const double scaled = static_cast<double>(clamped) * 2147483648.0;
    if (scaled >= 2147483647.0) {
        return 2147483647;
    }
    if (scaled <= -2147483648.0) {
        return static_cast<std::int32_t>(-2147483647 - 1);
    }
    return static_cast<std::int32_t>(std::lrint(scaled));
}

} // namespace

bool ace_inverse_transform_to_q31(
    const std::vector<float>& coefficients,
    std::vector<std::int32_t>& samples) noexcept {
    const std::size_t transform_size = coefficients.size();
    const float inverse_sqrt_window = ace_inverse_sqrt_window(
        transform_size);
    std::uint32_t exponent = 0U;
    if (inverse_sqrt_window == 0.0F
        || !ace_mdct_set_window_size(transform_size, exponent)) {
        samples.clear();
        return false;
    }
    std::vector<std::int32_t> workspace(1024U, 0);
    for (std::size_t index = 0U; index < transform_size; ++index) {
        workspace[index] = mdct_float_to_q31(
            coefficients[index] * inverse_sqrt_window);
    }
    if (!ace_mdct_native_transform_reference(workspace, exponent)) {
        samples.clear();
        return false;
    }
    samples.assign(workspace.begin(),
        workspace.begin() + static_cast<std::ptrdiff_t>(transform_size));
    return true;
}

bool ace_inverse_transform_packed_to_q31(
    const std::vector<float>& packed,
    const std::size_t block_size,
    std::vector<std::int32_t>& samples) noexcept {
    samples.clear();
    std::vector<std::vector<float>> blocks;
    if (!ace_mdct_deinterleave_blocks(packed, block_size, blocks)) {
        return false;
    }
    samples.reserve(packed.size());
    for (const auto& block : blocks) {
        std::vector<std::int32_t> transformed;
        if (!ace_inverse_transform_to_q31(block, transformed)) {
            samples.clear();
            return false;
        }
        samples.insert(samples.end(), transformed.begin(), transformed.end());
    }
    return samples.size() == packed.size();
}

bool ace_inverse_transform_native_candidate(
    const std::vector<float>& coefficients,
    std::vector<float>& samples) noexcept {
    const std::size_t transform_size = coefficients.size();
    if (transform_size != 128U && transform_size != 256U
        && transform_size != 512U && transform_size != 1024U) {
        samples.clear();
        return false;
    }
    const float inverse_sqrt_window = ace_inverse_sqrt_window(transform_size);
    if (inverse_sqrt_window == 0.0F) {
        samples.clear();
        return false;
    }
    // The native caller receives an int32 coefficient buffer and applies its
    // inverse-window scalar afterwards.  Do not invent a Q15 conversion at
    // this boundary; retain it only as an explicit diagnostic override.
    std::int32_t input_shift = 0;
    if (const char* value = std::getenv("DTSX_P2_NATIVE_MDCT_INPUT_SHIFT")) {
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 0 && parsed <= 30) {
            input_shift = static_cast<std::int32_t>(parsed);
        }
    }
    const double input_scale = static_cast<double>(
        std::uint64_t{1U} << static_cast<std::uint32_t>(input_shift));
    double input_multiplier = 1.0;
    if (const char* value = std::getenv(
            "DTSX_P2_NATIVE_MDCT_INPUT_MULTIPLIER")) {
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end != value && *end == '\0' && std::isfinite(parsed)
            && parsed > 0.0 && parsed <= 64.0) {
            input_multiplier = parsed;
        }
    }
    std::vector<std::int32_t> values(1024U, 0);
    for (std::size_t index = 0U; index < transform_size; ++index) {
        const double scaled = static_cast<double>(coefficients[index])
            * input_scale * input_multiplier
            * static_cast<double>(inverse_sqrt_window);
        if (!std::isfinite(scaled)
            || scaled > static_cast<double>((std::numeric_limits<std::int32_t>::max)())
            || scaled < static_cast<double>((std::numeric_limits<std::int32_t>::min)())) {
            samples.clear();
            return false;
        }
        values[index] = static_cast<std::int32_t>(std::lrint(scaled));
    }
    if (std::getenv("DTSX_P2_NATIVE_MDCT_APPLY_PRESCALE") != nullptr) {
        const std::int32_t scalar = transform_size == 128U
            ? 1035273459 : transform_size == 256U
                ? 1031798784 : transform_size == 512U
                    ? 1026884851 : 1023410176;
        constexpr std::int64_t rounding = INT64_C(1) << 27U;
        for (std::size_t index = 0U; index < transform_size; ++index) {
            const std::int64_t product = static_cast<std::int64_t>(
                values[index]) * scalar;
            values[index] = static_cast<std::int32_t>(product >= 0
                ? (product + rounding) >> 28U
                : -((-product + rounding) >> 28U));
        }
    }
    std::uint32_t exponent = 0U;
    for (std::size_t value = transform_size; value > 1U; value >>= 1U) {
        ++exponent;
    }
    if (!ace_mdct_native_transform_reference(values, exponent)) {
        samples.clear();
        return false;
    }
    samples.resize(transform_size);
    for (std::size_t index = 0U; index < transform_size; ++index) {
        // DTSAceStreamDecoder_Process converts the integer MDCT workspace to
        // float immediately before UnNormalize/THF with the native 1/1024
        // scale (0x3A800000).  Keep the candidate in the same normalized
        // domain consumed by the overlap/LTS stages.
        samples[index] = static_cast<float>(values[index]) * 0.0009765625F;
    }
    return true;
}

bool ace_mdct_deinterleave_blocks(
    const std::vector<float>& packed,
    const std::size_t block_size,
    std::vector<std::vector<float>>& blocks) noexcept {
    blocks.clear();
    if (packed.size() != 1024U || block_size == 0U
        || block_size > packed.size()
        || packed.size() % block_size != 0U) {
        return false;
    }
    const std::size_t block_count = packed.size() / block_size;
    if ((block_count & (block_count - 1U)) != 0U) {
        return false;
    }
    blocks.assign(block_count, std::vector<float>(block_size));
    // Matches the native loop in DTSAceMdct_Inverse_Process: source advances
    // by 4*block_count bytes while the destination block is contiguous.
    for (std::size_t block = 0U; block < block_count; ++block) {
        for (std::size_t sample = 0U; sample < block_size; ++sample) {
            blocks[block][sample] = packed[block + sample * block_count];
        }
    }
    return true;
}

bool ace_inverse_transform_packed_blocks(
    const std::vector<float>& packed,
    const std::size_t block_size,
    std::vector<float>& samples) noexcept {
    samples.clear();
    std::vector<std::vector<float>> blocks;
    if (!ace_mdct_deinterleave_blocks(packed, block_size, blocks)) {
        return false;
    }
    samples.reserve(packed.size());
    for (const auto& block : blocks) {
        std::vector<float> transformed;
        if (!ace_inverse_transform_reference(block, transformed)) {
            samples.clear();
            return false;
        }
        samples.insert(samples.end(), transformed.begin(), transformed.end());
    }
    return samples.size() == packed.size();
}

bool ace_inverse_transform_packed_blocks_native_candidate(
    const std::vector<float>& packed,
    const std::size_t block_size,
    std::vector<float>& samples) noexcept {
    samples.clear();
    std::vector<std::vector<float>> blocks;
    if (!ace_mdct_deinterleave_blocks(packed, block_size, blocks)) {
        return false;
    }
    samples.reserve(packed.size());
    for (const auto& block : blocks) {
        std::vector<float> transformed;
        if (!ace_inverse_transform_native_candidate(block, transformed)) {
            samples.clear();
            return false;
        }
        samples.insert(samples.end(), transformed.begin(), transformed.end());
    }
    return samples.size() == packed.size();
}

bool ace_windowed_imdct_reference(
    const std::size_t length,
    const std::size_t previous_transform,
    const std::size_t current_transform,
    std::vector<float>& previous,
    std::vector<float>& current) noexcept {
    if (length == 0U || previous.size() != length
        || current.size() != length || previous_transform == 0U
        || current_transform == 0U || previous_transform > length
        || current_transform > length || (previous_transform & 1U) != 0U
        || (current_transform & 1U) != 0U
        // The native short/long transition uses a separate iwindow routine
        // (Table 9-115).  Do not feed a mixed-size pair through the regular
        // window path: its history layout is different and cannot be inferred
        // from the single-window formula without the native differential.
        || previous_transform != current_transform
        || length % current_transform != 0U) {
        return false;
    }
    const std::size_t transition =
        (std::min)(previous_transform, current_transform);
    std::vector<float> previous_window;
    std::vector<float> current_window;
    if (!ace_make_window(previous_transform, transition, previous_window)
        || !ace_make_window(current_transform, transition, current_window)) {
        return false;
    }
    std::vector<float> transformed;
    const std::size_t blocks = length / current_transform;
    for (std::size_t block = 0U; block < blocks; ++block) {
        std::vector<float> source(
            current.begin() + static_cast<std::ptrdiff_t>(
                block * current_transform),
            current.begin() + static_cast<std::ptrdiff_t>(
                (block + 1U) * current_transform));
        if (!ace_dct4_reference(source, transformed)) {
            return false;
        }
        std::copy(transformed.begin(), transformed.end(),
            current.begin() + static_cast<std::ptrdiff_t>(
                block * current_transform));
    }
    const std::size_t half = current_transform / 2U;
    // Table 9-115 constructs exactly one transform-length boundary vector:
    // previous tail followed by the first half of the current transform.
    std::vector<float> boundary(current_transform, 0.0F);
    std::copy(previous.end() - static_cast<std::ptrdiff_t>(half),
        previous.end(), boundary.begin());
    std::copy(current.begin(), current.begin()
        + static_cast<std::ptrdiff_t>(half),
        boundary.begin() + static_cast<std::ptrdiff_t>(half));
    apply_single_window(previous_window, boundary);
    for (std::size_t n = 0U; n < half; ++n) {
        previous[previous.size() - half + n] = boundary[n];
        current[n] = boundary[half + n];
    }
    for (std::size_t block = 1U; block < blocks; ++block) {
        std::vector<float> segment(
            current.begin() + static_cast<std::ptrdiff_t>(
                block * current_transform),
            current.begin() + static_cast<std::ptrdiff_t>(
                (block + 1U) * current_transform));
        apply_single_window(current_window, segment);
        std::copy(segment.begin(), segment.end(), current.begin()
            + static_cast<std::ptrdiff_t>(block * current_transform));
    }
    return true;
}

bool ace_dct4_reference(
    const std::vector<float>& input,
    std::vector<float>& output) noexcept {
    const std::size_t length = input.size();
    if (length < 2U || length > 1024U
        || (length & (length - 1U)) != 0U) {
        return false;
    }
    output.assign(length, 0.0F);
    const double pi_over_four =
        0.7853981633974483096156608458199;
    // DTS_ACE_MDCT_TRANSFORM is BitreversalPermutation (inside Walsh) +
    // WalshTransform + BackwardRotate + ReverseOrdering, which composes to
    // an unnormalized DCT-IV.  Walsh butterflies are plain add/sub with no
    // 1/sqrt(2) per stage.  Inverse_Process already multiplies the 1024-bin
    // plane by DTS_INVSQRTWINDOW_LENGTH = 1/sqrt(N) before that pipeline, so
    // the orthonormal sqrt(2/N) factor must not be applied here: it stacked
    // a second 1/sqrt(N) and understated amplitude by sqrt(N/2).
    for (std::size_t m = 0U; m < length; ++m) {
        double sum = 0.0;
        for (std::size_t n = 0U; n < length; ++n) {
            sum += std::cos(
                (pi_over_four / length)
                * (2.0 * static_cast<double>(m) + 1.0)
                * (2.0 * static_cast<double>(n) + 1.0))
                * static_cast<double>(input[n]);
        }
        output[m] = static_cast<float>(sum);
        if (!std::isfinite(output[m])) {
            output.clear();
            return false;
        }
    }
    return true;
}

bool ace_reverse_ordering_reference(
    std::vector<float>& values) noexcept {
    if (values.empty() || (values.size() > 1U
                           && (values.size() & 1U) != 0U)) {
        return false;
    }
    // tcl_lib_dtsX.so.c: DTS_ACE_MDCT_ReverseOrdering exchanges
    // values[0..N/2) with values[N-1..N/2] in-place.  It is deliberately
    // kept separate from DCT4 because the native transform applies it at a
    // specific point relative to Walsh/rotation stages.
    for (std::size_t left = 0U, right = values.size() - 1U;
         left < values.size() / 2U; ++left, --right) {
        std::swap(values[left], values[right]);
    }
    return true;
}

bool ace_overlap_add_q31_reference(
    std::vector<std::int32_t>& history,
    std::vector<std::int32_t>& output,
    const std::vector<std::int32_t>& low_window,
    const std::vector<std::int32_t>& high_window) noexcept {
    if (history.empty() || history.size() != output.size()
        || history.size() != low_window.size()
        || history.size() != high_window.size()) {
        return false;
    }
    const auto multiply_q31 = [](const std::int32_t left,
                                 const std::int32_t right) noexcept {
        return ace_qrdmulh_s32(left, right);
    };
    for (std::size_t index = 0U; index < output.size(); ++index) {
        const std::int32_t transformed = output[index];
        const std::int32_t previous = history[index];
        const std::int32_t first = previous
            - multiply_q31(transformed, low_window[index]);
        const std::int32_t second = transformed
            + multiply_q31(first, high_window[index]);
        output[index] = first
            - multiply_q31(second, low_window[index]);
        history[index] = second;
    }
    return true;
}

bool ace_overlap_add_q31_native_pairwise_reference(
    std::vector<std::int32_t>& history,
    std::vector<std::int32_t>& transformed,
    const std::vector<std::int32_t>& window) noexcept {
    const std::size_t n = transformed.size();
    if (n < 16U || (n & 15U) != 0U || history.size() != n / 2U
        || window.size() != n) {
        return false;
    }
    using Pair = std::array<std::int32_t, 2U>;
    const auto q = [](const Pair& a, const Pair& b) noexcept {
        return Pair{ace_qrdmulh_s32(a[0], b[0]),
                    ace_qrdmulh_s32(a[1], b[1])};
    };
    const auto sub = [](const Pair& a, const Pair& b) noexcept {
        return Pair{
            static_cast<std::int32_t>(static_cast<std::uint32_t>(a[0])
                - static_cast<std::uint32_t>(b[0])),
            static_cast<std::int32_t>(static_cast<std::uint32_t>(a[1])
                - static_cast<std::uint32_t>(b[1]))};
    };
    const auto add = [](const Pair& a, const Pair& b) noexcept {
        return Pair{
            static_cast<std::int32_t>(static_cast<std::uint32_t>(a[0])
                + static_cast<std::uint32_t>(b[0])),
            static_cast<std::int32_t>(static_cast<std::uint32_t>(a[1])
                + static_cast<std::uint32_t>(b[1]))};
    };
    const auto rev = [](Pair a) noexcept {
        std::swap(a[0], a[1]);
        return a;
    };
    const auto load = [](const std::vector<std::int32_t>& v,
                         const std::size_t at) noexcept {
        return Pair{v[at], v[at + 1U]};
    };
    const auto store = [](std::vector<std::int32_t>& v,
                          const std::size_t at, const Pair& p) noexcept {
        v[at] = p[0];
        v[at + 1U] = p[1];
    };

    for (std::size_t group = 0U; group < n / 16U; ++group) {
        const std::size_t low = group * 8U;
        const std::size_t tail = n - 2U - group * 8U;
        const Pair h0 = load(history, low + 0U);
        const Pair h1 = load(history, low + 2U);
        const Pair h2 = load(history, low + 4U);
        const Pair h3 = load(history, low + 6U);
        const Pair l0 = load(window, low + 0U);
        const Pair l1 = load(window, low + 2U);
        const Pair l2 = load(window, low + 4U);
        const Pair l3 = load(window, low + 6U);
        const Pair t0 = load(transformed, tail);
        const Pair t1 = load(transformed, tail - 2U);
        const Pair t2 = load(transformed, tail - 4U);
        const Pair t3 = load(transformed, tail - 6U);
        const std::size_t high = n - 4U - group * 8U;
        const Pair g0 = load(window, high - 4U);
        const Pair g1 = load(window, high);
        const Pair g2 = load(window, high - 2U);
        const Pair g3 = load(window, high + 2U);

        const Pair v21 = q(h0, l0);
        const Pair v23 = sub(rev(t1), q(h1, l1));
        const Pair v25 = sub(rev(t3), q(h3, l3));
        const Pair v28 = sub(rev(t0), v21);
        const Pair v32 = sub(rev(t2), q(h2, l2));
        const Pair v29 = q(v25, rev(g0));
        const Pair v33 = add(v29, h3);
        const Pair v34 = add(q(v23, rev(g1)), h1);
        const Pair v35 = add(q(v28, rev(g3)), h0);
        const Pair v36 = add(q(v32, rev(g2)), h2);

        store(transformed, tail - 2U, rev(sub(v23, q(v34, l1))));
        store(transformed, tail, rev(sub(v28, q(v35, l0))));
        store(transformed, tail - 4U, rev(sub(v32, q(v36, l2))));
        store(transformed, tail - 6U, rev(sub(v25, q(v33, l3))));
        store(history, low + 0U, v35);
        store(history, low + 2U, v34);
        store(history, low + 4U, v36);
        store(history, low + 6U, v33);
    }
    return true;
}

bool ace_overlap_add_f32_native(
    float* const block,
    float* const history,
    const float* const window,
    const std::size_t overlap_size) noexcept {
    if (block == nullptr || history == nullptr || window == nullptr
        || overlap_size < 32U || (overlap_size & 31U) != 0U) {
        return false;
    }
    using Quad = std::array<float, 4U>;
    const auto load = [](const float* const values,
                         const std::size_t at) noexcept {
        return Quad{values[at], values[at + 1U], values[at + 2U],
            values[at + 3U]};
    };
    const auto store = [](float* const values, const std::size_t at,
                          const Quad& value) noexcept {
        values[at] = value[0];
        values[at + 1U] = value[1];
        values[at + 2U] = value[2];
        values[at + 3U] = value[3];
    };
    // vrev64q_s32 followed by vextq_s8(.., 8) reverses the whole quad.
    const auto reverse = [](const Quad& value) noexcept {
        return Quad{value[3], value[2], value[1], value[0]};
    };
    const auto multiply_subtract = [](const Quad& left, const Quad& right,
                                      const Quad& factor) noexcept {
        Quad result{};
        for (std::size_t lane = 0U; lane < result.size(); ++lane) {
            result[lane] = left[lane] - right[lane] * factor[lane];
        }
        return result;
    };
    const auto multiply_add = [](const Quad& left, const Quad& right,
                                 const Quad& factor) noexcept {
        Quad result{};
        for (std::size_t lane = 0U; lane < result.size(); ++lane) {
            result[lane] = left[lane] + right[lane] * factor[lane];
        }
        return result;
    };
    for (std::size_t group = 0U; group < overlap_size / 32U; ++group) {
        const std::size_t low = group * 16U;
        const std::size_t front = overlap_size / 2U - 4U - group * 16U;
        const std::size_t high = overlap_size - 4U - group * 16U;
        const Quad h0 = load(history, low + 0U);
        const Quad h1 = load(history, low + 4U);
        const Quad h2 = load(history, low + 8U);
        const Quad h3 = load(history, low + 12U);
        const Quad l0 = load(window, low + 0U);
        const Quad l1 = load(window, low + 4U);
        const Quad l2 = load(window, low + 8U);
        const Quad l3 = load(window, low + 12U);
        const Quad g0 = reverse(load(window, high));
        const Quad g1 = reverse(load(window, high - 4U));
        const Quad g2 = reverse(load(window, high - 8U));
        const Quad g3 = reverse(load(window, high - 12U));
        const Quad v20 = multiply_subtract(
            reverse(load(block, front)), h0, l0);
        const Quad v22 = multiply_subtract(
            reverse(load(block, front - 4U)), h1, l1);
        const Quad v24 = multiply_subtract(
            reverse(load(block, front - 8U)), h2, l2);
        const Quad v29 = multiply_subtract(
            reverse(load(block, front - 12U)), h3, l3);
        const Quad r0 = multiply_add(h0, v20, g0);
        const Quad r1 = multiply_add(h1, v22, g1);
        const Quad r2 = multiply_add(h2, v24, g2);
        const Quad r3 = multiply_add(h3, v29, g3);
        store(block, front, reverse(multiply_subtract(v20, l0, r0)));
        store(block, front - 4U, reverse(multiply_subtract(v22, l1, r1)));
        store(block, front - 8U, reverse(multiply_subtract(v24, l2, r2)));
        store(block, front - 12U, reverse(multiply_subtract(v29, l3, r3)));
        store(history, low + 0U, r0);
        store(history, low + 4U, r1);
        store(history, low + 8U, r2);
        store(history, low + 12U, r3);
    }
    return true;
}

bool ace_overlap_add_q31_native_sized_reference(
    std::vector<std::int32_t>& history,
    std::vector<std::int32_t>& transformed,
    const std::vector<std::int32_t>& window,
    const std::size_t overlap_size) noexcept {
    const std::size_t n = transformed.size();
    if (n < 16U || (n & 15U) != 0U || overlap_size < 16U
        || (overlap_size & 15U) != 0U || overlap_size > n
        || history.size() != overlap_size / 2U
        || window.size() != overlap_size) {
        return false;
    }
    using Pair = std::array<std::int32_t, 2U>;
    const auto q = [](const Pair& a, const Pair& b) noexcept {
        return Pair{ace_qrdmulh_s32(a[0], b[0]),
                    ace_qrdmulh_s32(a[1], b[1])};
    };
    const auto sub = [](const Pair& a, const Pair& b) noexcept {
        return Pair{
            static_cast<std::int32_t>(static_cast<std::uint32_t>(a[0])
                - static_cast<std::uint32_t>(b[0])),
            static_cast<std::int32_t>(static_cast<std::uint32_t>(a[1])
                - static_cast<std::uint32_t>(b[1]))};
    };
    const auto add = [](const Pair& a, const Pair& b) noexcept {
        return Pair{
            static_cast<std::int32_t>(static_cast<std::uint32_t>(a[0])
                + static_cast<std::uint32_t>(b[0])),
            static_cast<std::int32_t>(static_cast<std::uint32_t>(a[1])
                + static_cast<std::uint32_t>(b[1]))};
    };
    const auto rev = [](Pair a) noexcept {
        std::swap(a[0], a[1]);
        return a;
    };
    const auto load = [](const std::vector<std::int32_t>& v,
                         const std::size_t at) noexcept {
        return Pair{v[at], v[at + 1U]};
    };
    const auto store = [](std::vector<std::int32_t>& v,
                          const std::size_t at, const Pair& p) noexcept {
        v[at] = p[0];
        v[at + 1U] = p[1];
    };
    for (std::size_t group = 0U; group < overlap_size / 16U; ++group) {
        const std::size_t low = group * 8U;
        // Native starts the transformed tail at
        // `a2 + ((2 * a5) & ~1) - 8` bytes.  In int32 indices this is
        // `2 * overlap_size - 2`, not the history-half offset.
        const std::size_t front = 2U * overlap_size - 2U - group * 8U;
        const std::size_t high = overlap_size - 8U - group * 8U;
        const Pair h0 = load(history, low + 0U);
        const Pair h1 = load(history, low + 2U);
        const Pair h2 = load(history, low + 4U);
        const Pair h3 = load(history, low + 6U);
        const Pair l0 = load(window, low + 0U);
        const Pair l1 = load(window, low + 2U);
        const Pair l2 = load(window, low + 4U);
        const Pair l3 = load(window, low + 6U);
        const Pair t0 = load(transformed, front);
        const Pair t1 = load(transformed, front - 2U);
        const Pair t2 = load(transformed, front - 4U);
        const Pair t3 = load(transformed, front - 6U);
        const Pair g0 = load(window, high);
        const Pair g1 = load(window, high + 4U);
        const Pair g2 = load(window, high + 2U);
        const Pair g3 = load(window, high + 6U);
        const Pair v21 = q(h0, l0);
        const Pair v23 = sub(rev(t1), q(h1, l1));
        const Pair v25 = sub(rev(t3), q(h3, l3));
        const Pair v28 = sub(rev(t0), v21);
        const Pair v32 = sub(rev(t2), q(h2, l2));
        const Pair v29 = q(v25, rev(g0));
        const Pair v33 = add(v29, h3);
        const Pair v34 = add(q(v23, rev(g1)), h1);
        const Pair v35 = add(q(v28, rev(g3)), h0);
        const Pair v36 = add(q(v32, rev(g2)), h2);
        store(transformed, front - 2U, rev(sub(v23, q(v34, l1))));
        store(transformed, front, rev(sub(v28, q(v35, l0))));
        store(transformed, front - 4U, rev(sub(v32, q(v36, l2))));
        store(transformed, front - 6U, rev(sub(v25, q(v33, l3))));
        store(history, low + 0U, v35);
        store(history, low + 2U, v34);
        store(history, low + 4U, v36);
        store(history, low + 6U, v33);
    }
    return true;
}

bool ace_overlap_add_q31_sony_float_reference(
    std::vector<std::int32_t>& history,
    std::vector<std::int32_t>& transformed,
    const std::vector<std::int32_t>& window,
    const std::size_t overlap_size) noexcept {
    const std::size_t n = transformed.size();
    if (n < 32U || (n & 15U) != 0U || overlap_size < 32U
        || (overlap_size & 31U) != 0U || overlap_size > n
        || history.size() != overlap_size / 2U
        || window.size() != overlap_size) {
        return false;
    }
    constexpr float kQ31 = 2147483648.0F;
    const auto to_float = [](const std::int32_t value) noexcept {
        return static_cast<float>(value) / 2147483648.0F;
    };
    const auto to_q31 = [](const float value) noexcept {
        const double scaled = static_cast<double>(value) * 2147483648.0;
        return static_cast<std::int32_t>((std::max)(
            -2147483648.0, (std::min)(2147483647.0,
                std::round(scaled))));
    };
    const auto full_reverse = [](const std::array<float, 4>& value) noexcept {
        return std::array<float, 4>{value[3], value[2], value[1], value[0]};
    };
    const auto load = [&](const std::vector<std::int32_t>& values,
                          const std::size_t offset) noexcept {
        return std::array<float, 4>{to_float(values[offset + 0U]),
            to_float(values[offset + 1U]), to_float(values[offset + 2U]),
            to_float(values[offset + 3U])};
    };
    const auto load_window = [&](const std::size_t offset) noexcept {
        return std::array<float, 4>{to_float(window[offset + 0U]),
            to_float(window[offset + 1U]), to_float(window[offset + 2U]),
            to_float(window[offset + 3U])};
    };
    const auto multiply = [](const std::array<float, 4>& left,
                             const std::array<float, 4>& right) noexcept {
        std::array<float, 4> result{};
        for (std::size_t index = 0U; index < result.size(); ++index) {
            result[index] = left[index] * right[index];
        }
        return result;
    };
    const auto subtract = [](const std::array<float, 4>& left,
                             const std::array<float, 4>& right) noexcept {
        std::array<float, 4> result{};
        for (std::size_t index = 0U; index < result.size(); ++index) {
            result[index] = left[index] - right[index];
        }
        return result;
    };
    const auto add = [](const std::array<float, 4>& left,
                        const std::array<float, 4>& right) noexcept {
        std::array<float, 4> result{};
        for (std::size_t index = 0U; index < result.size(); ++index) {
            result[index] = left[index] + right[index];
        }
        return result;
    };
    for (std::size_t group = 0U; group < overlap_size / 32U; ++group) {
        const std::size_t low = group * 16U;
        const std::size_t front = overlap_size / 2U - 4U - group * 16U;
        const std::size_t high = overlap_size - 4U - group * 16U;
        const auto h0 = load(history, low + 0U);
        const auto h1 = load(history, low + 4U);
        const auto h2 = load(history, low + 8U);
        const auto h3 = load(history, low + 12U);
        const auto l0 = load_window(low + 0U);
        const auto l1 = load_window(low + 4U);
        const auto l2 = load_window(low + 8U);
        const auto l3 = load_window(low + 12U);
        const auto t0 = load(transformed, front);
        const auto t1 = load(transformed, front - 4U);
        const auto t2 = load(transformed, front - 8U);
        const auto t3 = load(transformed, front - 12U);
        const auto g0 = load_window(high);
        const auto g1 = load_window(high - 4U);
        const auto g2 = load_window(high - 8U);
        const auto g3 = load_window(high - 12U);
        const auto v20 = subtract(full_reverse(t0), multiply(h0, l0));
        const auto v22 = subtract(full_reverse(t1), multiply(h1, l1));
        const auto v24 = subtract(full_reverse(t2), multiply(h2, l2));
        const auto v29 = subtract(full_reverse(t3), multiply(h3, l3));
        const auto result = add(h0, multiply(v20, full_reverse(g0)));
        const auto v31 = add(h1, multiply(v22, full_reverse(g1)));
        const auto v32 = add(h2, multiply(v24, full_reverse(g2)));
        const auto v33 = add(h3, multiply(v29, full_reverse(g3)));
        const auto v34 = full_reverse(subtract(v20, multiply(g0, result)));
        const auto v35 = full_reverse(subtract(v22, multiply(g1, v31)));
        const auto v36 = full_reverse(subtract(v24, multiply(g2, v32)));
        const auto v37 = full_reverse(subtract(v29, multiply(g3, v33)));
        const auto store = [&](const std::size_t offset,
                               const std::array<float, 4>& value) noexcept {
            for (std::size_t index = 0U; index < value.size(); ++index) {
                transformed[offset + index] = to_q31(value[index]);
            }
        };
        store(front - 4U, v35);
        store(front, v34);
        store(front - 12U, v37);
        store(front - 8U, v36);
        for (std::size_t index = 0U; index < 4U; ++index) {
            history[low + index] = to_q31(result[index]);
            history[low + 4U + index] = to_q31(v31[index]);
            history[low + 8U + index] = to_q31(v32[index]);
            history[low + 12U + index] = to_q31(v33[index]);
        }
    }
    (void)kQ31;
    return true;
}

} // namespace dtsx
