#include "dtsx/ace_mdct.hpp"
#include "dtsx/ace_mdct_swap_table.generated.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace dtsx {
namespace {

constexpr float kQ31ToFloat = 1.0F / 2147483648.0F;

float q31_twiddle_f32(const std::int32_t value) noexcept {
    return static_cast<float>(value) * kQ31ToFloat;
}

bool idle_tables_f32(
    const std::uint32_t order,
    std::vector<float>& ltw,
    std::vector<float>& utw) noexcept {
    std::vector<std::int32_t> ltw_q31;
    std::vector<std::int32_t> utw_q31;
    if (!ace_mdct_idle_tables(order, ltw_q31, utw_q31)) {
        return false;
    }
    ltw.resize(ltw_q31.size());
    utw.resize(utw_q31.size());
    for (std::size_t index = 0U; index < ltw_q31.size(); ++index) {
        ltw[index] = q31_twiddle_f32(ltw_q31[index]);
        utw[index] = q31_twiddle_f32(utw_q31[index]);
    }
    return true;
}

struct PairF final {
    float first = 0.0F;
    float second = 0.0F;
};

PairF add_f(const PairF a, const PairF b) noexcept {
    return {a.first + b.first, a.second + b.second};
}

PairF sub_f(const PairF a, const PairF b) noexcept {
    return {a.first - b.first, a.second - b.second};
}

PairF mul_f(const PairF a, const PairF b) noexcept {
    return {a.first * b.first, a.second * b.second};
}

PairF reverse_f(const PairF value) noexcept {
    return {value.second, value.first};
}

PairF even_f(const PairF a, const PairF b) noexcept {
    return {a.first, b.first};
}

PairF odd_f(const PairF a, const PairF b) noexcept {
    return {a.second, b.second};
}

PairF load_f(
    const std::vector<float>& values, const std::size_t offset) noexcept {
    return {values[offset], values[offset + 1U]};
}

void store_f(
    std::vector<float>& values,
    const std::size_t offset,
    const PairF value) noexcept {
    values[offset] = value.first;
    values[offset + 1U] = value.second;
}

bool bitreversal_f32(
    std::vector<float>& values, const std::uint32_t exponent) noexcept {
    if (values.size() != 1024U || exponent < 7U || exponent > 10U) {
        return false;
    }
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
            const float va = values[a];
            const float vc = values[c];
            values[a] = values[b];
            values[c] = values[d];
            values[b] = va;
            values[d] = vc;
        }
        return true;
    };
    switch (exponent - 7U) {
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

bool walsh_tcl_f32(
    std::vector<float>& values, const std::uint32_t exponent) noexcept {
    if (values.size() != 1024U || exponent < 7U || exponent > 10U) {
        return false;
    }
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
                const float left = values[low];
                const float right = values[high];
                values[low] = left + right;
                values[high] = left - right;
            }
        }
        for (std::uint32_t outer = 1U; outer < outer_count; outer += 2U) {
            const std::size_t base = 2U * span
                + static_cast<std::size_t>((outer - 1U) / 2U)
                    * lane_stride;
            for (std::size_t offset = 0U; offset < span; ++offset) {
                const std::size_t low = base + offset;
                const std::size_t high = low + span;
                const float left = values[low];
                const float right = values[high];
                values[low] = left - right;
                values[high] = right + left;
            }
        }
    }
    return true;
}

bool reverse_prefix_f32(
    std::vector<float>& values, const std::size_t length) noexcept {
    if (length < 16U || length > 1024U || (length & 15U) != 0U
        || length > values.size()) {
        return false;
    }
    for (std::size_t left = 0U, right = length - 1U;
         left < right; ++left, --right) {
        const float swap = values[left];
        values[left] = values[right];
        values[right] = swap;
    }
    return true;
}

bool backward_rotate_initial_f32(
    std::vector<float>& values, const std::uint32_t exponent) noexcept {
    if (exponent < 3U || exponent > 10U
        || (values.size() != (std::size_t{1U} << exponent)
            && values.size() != 1024U)) {
        return false;
    }
    std::vector<float> ltw1;
    std::vector<float> utw1;
    std::vector<float> ltw2;
    std::vector<float> utw2;
    std::vector<float> ltw3;
    std::vector<float> utw3;
    if (!idle_tables_f32(1U, ltw1, utw1)
        || !idle_tables_f32(2U, ltw2, utw2)
        || !idle_tables_f32(3U, ltw3, utw3)
        || ltw1.empty() || utw1.empty()
        || ltw2.size() < 2U || utw2.size() < 2U
        || ltw3.size() < 4U || utw3.size() < 4U) {
        return false;
    }
    const PairF l1{ltw1[0], ltw1[0]};
    const PairF u1{utw1[0], utw1[0]};
    const PairF l2 = reverse_f({ltw2[0], ltw2[1]});
    const PairF u2 = reverse_f({utw2[0], utw2[1]});
    const PairF l3a = reverse_f({ltw3[0], ltw3[1]});
    const PairF l3b{ltw3[2], ltw3[3]};
    const PairF u3a = reverse_f({utw3[0], utw3[1]});
    const PairF u3b{utw3[2], utw3[3]};
    const std::size_t blocks = (std::size_t{1U} << exponent) / 8U;
    for (std::size_t block = 0U; block < blocks; ++block) {
        const std::size_t base = block * 8U;
        const PairF x0{values[base], values[base + 2U]};
        const PairF x1{values[base + 1U], values[base + 3U]};
        const PairF x2{values[base + 4U], values[base + 6U]};
        const PairF x3{values[base + 5U], values[base + 7U]};
        const PairF p = sub_f(x1, mul_f(x0, l1));
        const PairF q = sub_f(x3, mul_f(x2, l1));
        const PairF a = add_f(mul_f(p, u1), x0);
        const PairF b = add_f(mul_f(q, u1), x2);
        const PairF r = sub_f(p, mul_f(a, l1));
        const PairF s = sub_f(q, mul_f(b, l1));
        const PairF t = even_f(r, a);
        const PairF u = odd_f(a, r);
        const PairF v = even_f(s, b);
        const PairF w = odd_f(b, s);
        const PairF e = sub_f(u, mul_f(t, l2));
        const PairF f = sub_f(w, mul_f(v, l2));
        const PairF y = add_f(mul_f(e, u2), t);
        const PairF x = add_f(mul_f(f, u2), v);
        const PairF i = sub_f(e, mul_f(y, l2));
        const PairF j = sub_f(sub_f(f, mul_f(x, l2)), mul_f(y, l3a));
        const PairF x_final = sub_f(x, mul_f(i, l3b));
        const PairF y_final = add_f(mul_f(j, u3a), y);
        const PairF v31 = add_f(mul_f(x_final, u3b), i);
        store_f(values, base, reverse_f(y_final));
        store_f(values, base + 2U, v31);
        store_f(values, base + 4U,
            reverse_f(sub_f(x_final, mul_f(v31, l3b))));
        store_f(values, base + 6U, sub_f(j, mul_f(y_final, l3a)));
    }
    return true;
}

bool backward_rotate_f32(
    std::vector<float>& values, const std::uint32_t exponent) noexcept {
    if (!backward_rotate_initial_f32(values, exponent)
        || exponent < 3U || exponent > 10U) {
        return false;
    }
    std::vector<float> ltw;
    std::vector<float> utw;
    const auto get_tables = [&ltw, &utw](const std::uint32_t order) {
        return idle_tables_f32(order, ltw, utw);
    };
    bool valid_access = true;
    const auto load = [&values, &valid_access](
                          const std::int64_t byte_offset) noexcept {
        if (byte_offset < 0 || (byte_offset & 3LL) != 0
            || static_cast<std::uint64_t>(byte_offset / 4 + 1)
                   >= values.size()) {
            valid_access = false;
            return PairF{};
        }
        const std::size_t index = static_cast<std::size_t>(byte_offset / 4);
        return PairF{values[index], values[index + 1U]};
    };
    const auto store = [&values, &valid_access](
                           const std::int64_t byte_offset,
                           const PairF pair) noexcept {
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
                    + static_cast<std::int64_t>(group)
                        * static_cast<std::int64_t>(4U * v34);
                const std::int64_t v37 =
                    static_cast<std::int64_t>(group) * 4LL
                    * static_cast<std::int64_t>(v34);
                for (std::uint32_t item = 0U; item < inner; ++item) {
                    const std::int64_t a2_offset = v37
                        + static_cast<std::int64_t>(item) * 32LL;
                    const std::int64_t v48_offset = -32LL + v38;
                    const std::int64_t v45_offset = -8LL + v38;
                    const PairF v52 = load(a2_offset);
                    const PairF v58 = load(a2_offset + 16LL);
                    const PairF v56 = load(a2_offset + 8LL);
                    const PairF v57 = load(a2_offset + 24LL);
                    const std::size_t tw = static_cast<std::size_t>(item) * 8U;
                    if (tw + 7U >= ltw.size() || tw + 7U >= utw.size()) {
                        return false;
                    }
                    const PairF v59 = sub_f(
                        reverse_f(load(v48_offset + 24LL)),
                        mul_f(v52, {ltw[tw], ltw[tw + 1U]}));
                    const PairF v61 = sub_f(
                        reverse_f(load(v48_offset + 16LL)),
                        mul_f(v56, {ltw[tw + 2U], ltw[tw + 3U]}));
                    const PairF v65 = sub_f(
                        reverse_f(load(v48_offset)),
                        mul_f(v57, {ltw[tw + 6U], ltw[tw + 7U]}));
                    const PairF v67 = sub_f(
                        reverse_f(load(v48_offset + 8LL)),
                        mul_f(v58, {ltw[tw + 4U], ltw[tw + 5U]}));
                    const PairF v68 = add_f(
                        mul_f(v59, {utw[tw], utw[tw + 1U]}), v52);
                    const PairF v69 = add_f(
                        mul_f(v65, {utw[tw + 6U], utw[tw + 7U]}), v57);
                    const PairF v70 = add_f(
                        mul_f(v61, {utw[tw + 2U], utw[tw + 3U]}), v56);
                    const PairF v71 = mul_f(
                        v69, {ltw[tw + 6U], ltw[tw + 7U]});
                    const PairF v72 = add_f(
                        mul_f(v67, {utw[tw + 4U], utw[tw + 5U]}), v58);
                    store(a2_offset, v68);
                    store(a2_offset + 8LL, v70);
                    store(a2_offset + 24LL, v69);
                    store(a2_offset + 16LL, v72);
                    store(v45_offset, reverse_f(sub_f(v59,
                        mul_f(v68, {ltw[tw], ltw[tw + 1U]}))));
                    store(v45_offset - 8LL, reverse_f(sub_f(v61,
                        mul_f(v70, {ltw[tw + 2U], ltw[tw + 3U]}))));
                    store(v45_offset - 16LL, reverse_f(sub_f(v67,
                        mul_f(v72, {ltw[tw + 4U], ltw[tw + 5U]}))));
                    store(v45_offset - 24LL, reverse_f(sub_f(v65, v71)));
                    v38 -= 32LL;
                }
            }
        }
        v33 >>= 1U;
        ++v107;
        v34 <<= 1U;
    }
    v34 = (v102 < 4U ? 16U : (1U << (exponent - 1U)));
    if ((v34 >> 2U) == 0U) {
        return true;
    }
    std::vector<float> ltw_next;
    std::vector<float> utw_next;
    if (!get_tables(v102 + 1U) || ltw.size() < 2U || utw.size() < 2U
        || !idle_tables_f32(v102 + 2U, ltw_next, utw_next)
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
        const PairF v87 = load(offset);
        const PairF v91 = load_f(ltw, twiddle_offset);
        const PairF v83 = load_f(utw, twiddle_offset);
        const PairF ltw1_1 = load_f(ltw_next, twiddle_offset);
        const PairF utw1_1 = load_f(utw_next, twiddle_offset);
        const PairF v90 = reverse_f(load_f(ltw_next, front_twiddle_offset));
        const PairF v92 = reverse_f(load_f(utw_next, front_twiddle_offset));
        const PairF v93 = sub_f(reverse_f(load(front_lo)), mul_f(v87, v91));
        const PairF v94 = sub_f(
            reverse_f(load(front_hi)), mul_f(load(out_hi), v91));
        const PairF v95 = add_f(mul_f(v93, v83), v87);
        const PairF v96 = add_f(mul_f(v94, v83), load(out_hi));
        const PairF v97 = sub_f(v93, mul_f(v95, v91));
        const PairF v98 = sub_f(
            sub_f(v94, mul_f(v96, v91)), mul_f(v95, ltw1_1));
        const PairF v99 = sub_f(v96, mul_f(v97, v90));
        const PairF v100 = add_f(mul_f(v98, utw1_1), v95);
        const PairF v101 = add_f(mul_f(v99, v92), v97);
        store(offset, v100);
        store(front_lo, reverse_f(v101));
        store(out_hi, sub_f(v99, mul_f(v101, v90)));
        store(front_hi, reverse_f(sub_f(v98, mul_f(v100, ltw1_1))));
    }
    return valid_access;
}

} // namespace

bool ace_mdct_native_transform_f32(
    std::vector<float>& values, const std::uint32_t exponent) noexcept {
    if (exponent < 7U || exponent > 10U || values.size() != 1024U) {
        return false;
    }
    if (!bitreversal_f32(values, exponent)
        || !walsh_tcl_f32(values, exponent)
        || !backward_rotate_f32(values, exponent)
        || !reverse_prefix_f32(values, std::size_t{1U} << exponent)) {
        return false;
    }
    for (std::size_t index = 0U; index < (std::size_t{1U} << exponent);
         ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

} // namespace dtsx
