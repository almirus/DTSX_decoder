#include "dtsx/ace_vq.hpp"
#include "dtsx/ace_beta_table.generated.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace dtsx {
namespace {

constexpr std::uint32_t kPyramidMaximum =
    std::numeric_limits<std::uint32_t>::max();

// ETSI TS 103 491 Table 10-16; native symbol DTS_ACE_MAX_BITS_FOR_N.
// Entry zero is N=2, matching the native N-2 indexing.
constexpr std::array<std::uint8_t, 207> kMaxBitsForN{{
    11U, 19U, 26U, 33U, 38U, 39U, 40U, 40U, 41U, 41U, 41U, 41U, 42U, 42U, 42U, 42U,
    41U, 41U, 42U, 41U, 41U, 41U, 41U, 41U, 40U, 41U, 41U, 40U, 41U, 41U, 41U, 39U,
    39U, 40U, 40U, 41U, 39U, 39U, 39U, 39U, 40U, 40U, 40U, 37U, 37U, 37U, 38U, 38U,
    39U, 39U, 40U, 40U, 40U, 40U, 40U, 37U, 37U, 37U, 37U, 37U, 37U, 38U, 38U, 38U,
    38U, 38U, 38U, 38U, 39U, 39U, 39U, 39U, 39U, 39U, 39U, 39U, 35U, 35U, 35U, 35U,
    35U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 37U, 37U, 37U, 37U,
    37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U, 38U, 38U, 38U, 38U, 38U, 38U, 38U, 38U,
    38U, 38U, 38U, 38U, 38U, 33U, 33U, 33U, 33U, 33U, 33U, 34U, 34U, 34U, 34U, 34U,
    34U, 34U, 34U, 34U, 34U, 34U, 34U, 34U, 34U, 34U, 34U, 34U, 34U, 34U, 35U, 35U,
    35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U, 35U,
    35U, 35U, 35U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U,
    36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 36U, 37U, 37U, 37U, 37U,
    37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U, 37U,
}};

} // namespace

namespace {

[[nodiscard]] float bits_to_float(std::uint32_t bits) noexcept {
    float value = 0.0F;
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

[[nodiscard]] std::uint32_t ace_centered_inverse_map(
    std::uint32_t alphabet,
    std::uint32_t centre,
    std::uint32_t value) noexcept {
    if (centre > ((alphabet - 1U) >> 1U)) {
        return alphabet - ace_centered_inverse_map(
            alphabet, alphabet - centre - 1U, value) - 1U;
    }
    if (value > centre * 2U) {
        return value;
    }
    if ((value & 1U) == 0U) {
        return centre - (value >> 1U);
    }
    const std::uint32_t mapped = centre + ((value + 1U) >> 1U);
    return mapped < alphabet ? mapped : value;
}

[[nodiscard]] bool native_ratio_from_codeword(
    std::uint32_t alphabet,
    std::uint32_t codeword,
    std::uint32_t channel_count,
    AceBandRatio& ratio) noexcept {
    if (alphabet == 0U || codeword > alphabet || channel_count == 0U) {
        return false;
    }

    const std::uint32_t offset = alphabet < detail::kAceNativeBetaOffsets.size()
        ? detail::kAceNativeBetaOffsets[alphabet]
        : std::numeric_limits<std::uint32_t>::max();
    if (offset == std::numeric_limits<std::uint32_t>::max()
        || offset + codeword >= detail::kAceNativeBetaEntries.size()) {
        return false;
    }

    const detail::AceNativeBetaEntry& entry =
        detail::kAceNativeBetaEntries[offset + codeword];
    ratio.beta_q15 = entry.beta;
    ratio.left_norm = bits_to_float(entry.left_bits);
    ratio.right_norm = bits_to_float(entry.right_bits);
    if (ratio.beta_q15 == -32767) {
        ratio.mid_side_angle_q15 = 0x4000;
    } else if (ratio.beta_q15 == 32767) {
        ratio.mid_side_angle_q15 = -0x4000;
    } else {
        // DTSAceBandDequant_RatioDecoderBetaCodec passes the size of its
        // right partition (a3) to dtsAce_ComputeNormsFromBeta(), not the
        // complete parent vector.  For an N-component parent that partition
        // is floor(N/2); using N here sends recursion to a different bit
        // budget even though the BETA table entry itself is correct.
        const std::int32_t right_components = static_cast<std::int32_t>(
            channel_count >> 1U);
        ratio.mid_side_angle_q15 = (1024 - ratio.beta_q15
            * (right_components - 1)) >> 11;
    }
    return true;
}

[[nodiscard]] bool native_ratio_from_beta(
    const std::int32_t beta_q15,
    const std::uint32_t angle_components,
    AceBandRatio& ratio) noexcept {
    if (angle_components == 0U) {
        return false;
    }
    for (const detail::AceNativeBetaEntry& entry : detail::kAceNativeBetaEntries) {
        if (entry.beta != beta_q15) {
            continue;
        }
        ratio.beta_q15 = entry.beta;
        ratio.left_norm = bits_to_float(entry.left_bits);
        ratio.right_norm = bits_to_float(entry.right_bits);
        if (ratio.beta_q15 == -32767) {
            ratio.mid_side_angle_q15 = 0x4000;
        } else if (ratio.beta_q15 == 32767) {
            ratio.mid_side_angle_q15 = -0x4000;
        } else {
            ratio.mid_side_angle_q15 = (1024 - ratio.beta_q15
                * (static_cast<std::int32_t>(angle_components) - 1)) >> 11;
        }
        return true;
    }
    return false;
}

bool consume_split_vector(
    AceBitReader& source,
    std::uint32_t allocated_bits,
    std::uint32_t remaining_bits,
    std::uint32_t components,
    std::uint32_t num_blocks,
    std::int32_t log_n_q4,
    std::uint32_t depth,
    bool high_resolution,
    std::uint32_t component_offset,
    float gain,
    AceVqPayloadResult& result) noexcept {
    if (components == 0U) {
        return false;
    }
    const std::uint32_t partition_bits =
        (std::min)(allocated_bits, remaining_bits);
    if (!ace_vq_split_required(
            partition_bits, components, depth, high_resolution)) {
        const std::uint32_t pulses = ace_vq_select_pulses(
            components, partition_bits, remaining_bits, high_resolution);
        std::vector<float> discarded;
        AceVqDecodeResult leaf{};
        if (!ace_vq_decode_signed_pyramid(
                source, components, pulses, gain, discarded, leaf)) {
            return false;
        }
        result.bits_consumed += leaf.bits_consumed;
        ++result.leaf_count;
        result.leaves.push_back(AceVqLeaf{
            component_offset, components, pulses, gain,
            std::move(leaf.signed_pulses), std::move(discarded)});
        return true;
    }

    const std::uint32_t alpha = ace_vq_ratio_quantization_level(
        partition_bits, components, log_n_q4);
    AceVqSplit split{};
    if (!ace_vq_read_split(
            source, alpha, components, partition_bits, remaining_bits,
            num_blocks > 1U, split)) {
        return false;
    }
    ++result.split_count;
    result.bits_consumed += split.bits_consumed;
    const std::uint32_t after_ratio_remaining = remaining_bits
        - split.bits_consumed;
    const std::uint32_t left_components = (components + 1U) >> 1U;
    const std::uint32_t right_components = components - left_components;
    const std::uint32_t child_blocks = (num_blocks + 1U) >> 1U;
    const std::int32_t child_log_n_q4 = log_n_q4 - 16;

    auto decode_child = [&](std::uint32_t child_allocated,
                            std::uint32_t child_remaining,
                            std::uint32_t child_components,
                            std::uint32_t child_offset,
                            float child_gain,
                            std::uint32_t& used) noexcept {
        const std::uint32_t before = result.bits_consumed;
        if (!consume_split_vector(
                source, child_allocated, child_remaining,
                child_components, child_blocks, child_log_n_q4, depth + 1U,
                high_resolution, child_offset, child_gain, result)) {
            return false;
        }
        used = result.bits_consumed - before;
        return true;
    };

    const bool left_first = split.left_bits >= split.right_bits;
    const std::uint32_t first_allocated = left_first
        ? split.left_bits : split.right_bits;
    const std::uint32_t first_components = left_first
        ? left_components : right_components;
    const std::uint32_t first_offset = left_first
        ? component_offset : component_offset + left_components;
    const float first_gain = gain * (left_first
        ? split.ratio.left_norm : split.ratio.right_norm);
    std::uint32_t first_used = 0U;
    if (!decode_child(
            first_allocated, after_ratio_remaining, first_components,
            first_offset, first_gain, first_used)) {
        return false;
    }
    if (first_used > after_ratio_remaining) {
        return false;
    }
    const std::uint32_t unused = first_used < first_allocated
        ? first_allocated - first_used : 0U;
    std::uint32_t second_allocated = left_first
        ? split.right_bits : split.left_bits;
    if (unused > 3U && !split.final_partition) {
        second_allocated += unused - 3U;
    }
    const std::uint32_t second_remaining = after_ratio_remaining - first_used;
    second_allocated = (std::min)(second_allocated, second_remaining);
    const std::uint32_t second_components = left_first
        ? right_components : left_components;
    const std::uint32_t second_offset = left_first
        ? component_offset + left_components : component_offset;
    const float second_gain = gain * (left_first
        ? split.ratio.right_norm : split.ratio.left_norm);
    std::uint32_t second_used = 0U;
    return decode_child(
               second_allocated, second_remaining, second_components,
               second_offset, second_gain, second_used)
        && second_used <= second_remaining;
}

} // namespace

bool ace_vq_split_required(
    std::uint32_t allocated_bits,
    std::uint32_t components,
    std::uint32_t depth,
    bool high_resolution) noexcept {
    constexpr std::uint32_t kMaxDepthLowResolution = 3U;
    const std::uint32_t maximum_depth = kMaxDepthLowResolution
        + (high_resolution ? 1U : 0U);
    if (depth > maximum_depth || components < 4U
        || components > kMaxBitsForN.size() + 1U) {
        return false;
    }
    return allocated_bits > kMaxBitsForN[components - 2U];
}

std::uint32_t ace_vq_ratio_quantization_level(
    std::uint32_t allocated_bits,
    std::uint32_t components,
    std::int32_t log_n_q4) noexcept {
    // DTSAceBandDequant_RatioDecoderBetaCodec: the two candidate exponents
    // are evaluated in the native Q4 domain before the exponential-table
    // lookup. Keep all operations integral; this value determines both the
    // exact bounded-code alphabet and its bit consumption.
    if (components < 2U) {
        return 1U;
    }
    const std::int64_t denominator = static_cast<std::int64_t>(components) - 1;
    const std::int64_t first = 16LL * allocated_bits - 32LL;
    const std::int64_t second = (16LL * allocated_bits
        + ((static_cast<std::int64_t>(log_n_q4) >> 1U) - 16LL) * denominator)
        / denominator;
    const std::int64_t exponent = (std::min)(first, second);
    if (exponent <= 7LL) {
        return 1U;
    }
    if (exponent >= 129LL) {
        return 256U;
    }

    const std::uint32_t scale = static_cast<std::uint32_t>(exponent);
    const std::uint32_t high = scale >> 4U;
    const std::uint32_t table_value =
        detail::kAceExponentialTable[8U * (scale & 0xFU)];
    const std::uint32_t raw = scale < 96U
        ? (table_value >> (9U - high)) + 1U
        : ((table_value << (high - 5U)) >> 4U) + 1U;
    const std::uint32_t level = raw & ~std::uint32_t{1U};
    return level == 0U ? 1U : level;
}

std::uint32_t ace_vq_select_pulses(
    std::uint32_t components,
    std::uint32_t partition_bits,
    std::uint32_t band_bits,
    bool high_resolution) noexcept {
    if (components < 2U || components > detail::kAceMaxKForN.size()
        || partition_bits == 0U || band_bits == 0U) {
        return 0U;
    }

    std::uint32_t bits = (std::min)(partition_bits, band_bits);
    std::uint32_t maximum_k = detail::kAceMaxKForN[components - 1U];
    if (maximum_k > 128U && !high_resolution) {
        maximum_k = 128U;
    }

    std::uint32_t bits_limit = 0U;
    if (components <= 6U && !high_resolution) {
        bits_limit = detail::kAceLowResolutionMaxBits[components - 2U];
    } else if (components <= 5U) {
        bits_limit = kMaxBitsForN[components - 2U];
    } else {
        bits_limit = (std::min)(maximum_k, components) + 32U;
    }
    bits = (std::min)(bits, bits_limit);

    const detail::AceKEstimationCoefficient coefficient =
        detail::kAceKEstimationCoefficients[components - 2U];
    const std::uint32_t exponent = (bits << 9U) / (components - 1U);
    const std::uint64_t product = static_cast<std::uint64_t>(
        detail::kAceExponentialTable[(exponent >> 2U) & 0x7FU])
        * coefficient.offset;
    const std::uint32_t scaled = exponent > 4607U
        ? static_cast<std::uint32_t>(product << ((exponent >> 9U) - 9U))
        : static_cast<std::uint32_t>(product >> (9U - (exponent >> 9U)));
    const std::int64_t estimated = static_cast<std::int64_t>(scaled)
        - coefficient.scale;
    std::uint32_t pulses = estimated >= 256
        ? (std::min)(static_cast<std::uint32_t>((estimated + 256) >> 9U), maximum_k)
        : 1U;

    const std::uint32_t remaining = band_bits - bits;
    if (remaining > 7U) {
        // Native branches directly to LABEL_53 here: the estimated K is
        // valid, only the subsequent pyramid-alignment search is skipped.
        return pulses;
    }
    if (bits < 6U) {
        if (components <= 7U && remaining > ((components - 1U) >> 1U)) {
            return pulses;
        }
    } else if (remaining > ((78U * bits + 1249U) >> 9U)) {
        return pulses;
    }

    const std::uint32_t limited_pulses = (std::min)(pulses, components);
    std::uint32_t used_pulses = band_bits - 1U;
    if (limited_pulses < band_bits) {
        used_pulses = limited_pulses;
    } else {
        pulses = band_bits - 1U;
    }
    const std::uint32_t alignment_bits = band_bits - used_pulses;
    if (alignment_bits > 31U) {
        return pulses;
    }
    std::uint32_t mask = alignment_bits == 0U
        ? 0U : ~(std::numeric_limits<std::uint32_t>::max()
                  >> (32U - alignment_bits));
    while (pulses > components
        && ((ace_unsigned_pyramid_size(components, pulses) - 1U) & mask) != 0U) {
        --pulses;
    }
    while (pulses >= 1U
        && ((ace_unsigned_pyramid_size(components, pulses) - 1U) & mask) != 0U) {
        --pulses;
        mask <<= 1U;
    }
    return pulses;
}

std::uint32_t ace_unsigned_pyramid_size(
    std::uint32_t components,
    std::uint32_t pulses) noexcept {
    if (components == 0U) {
        return 0U;
    }

    // DTS_ACE_PYRAMID_VECTOR_MATRIX[n - 1][k] is C(n + k - 1, k), saturated
    // at the uint32 storage limit.  Computing recurrence terms in this order
    // keeps every non-saturated intermediate exact.
    const std::uint32_t terms = std::min(components - 1U, pulses);
    std::uint64_t value = 1U;
    for (std::uint32_t index = 1U; index <= terms; ++index) {
        const std::uint64_t numerator = components + pulses - index;
        const std::uint64_t denominator = index;
        if (value > (static_cast<std::uint64_t>(kPyramidMaximum)
                     * denominator) / numerator) {
            return kPyramidMaximum;
        }
        value = (value * numerator) / denominator;
    }
    return static_cast<std::uint32_t>(value);
}

bool ace_unsigned_pyramid_unrank(
    std::uint32_t components,
    std::uint32_t pulses,
    std::uint32_t codeword,
    std::vector<std::int32_t>& vector) noexcept {
    const std::uint32_t alphabet = ace_unsigned_pyramid_size(
        components, pulses);
    if (components == 0U || codeword >= alphabet) {
        return false;
    }

    vector.assign(components, 0);
    std::uint32_t remaining_components = components;
    std::uint32_t remaining_pulses = pulses;
    for (std::uint32_t index = 0U; index + 1U < components; ++index) {
        // The native decoder subtracts one cumulative pyramid region at a
        // time.  This is the equivalent direct form: the first coefficient
        // is enumerated K, K-1, ..., 0.
        std::uint32_t coefficient = remaining_pulses;
        while (true) {
            const std::uint32_t tail_size = ace_unsigned_pyramid_size(
                remaining_components - 1U,
                remaining_pulses - coefficient);
            if (codeword < tail_size) {
                vector[index] = static_cast<std::int32_t>(coefficient);
                remaining_pulses -= coefficient;
                --remaining_components;
                break;
            }
            codeword -= tail_size;
            if (coefficient == 0U) {
                return false;
            }
            --coefficient;
        }
    }
    vector.back() = static_cast<std::int32_t>(remaining_pulses);
    return true;
}

bool ace_vq_decode_signed_pyramid(
    AceBitReader& source,
    std::uint32_t components,
    std::uint32_t pulses,
    float gain,
    std::vector<float>& vector,
    AceVqDecodeResult& result) noexcept {
    result = {};
    vector.clear();
    if (components == 0U) {
        return false;
    }

    const std::size_t begin = source.position();
    const std::uint32_t alphabet = ace_unsigned_pyramid_size(
        components, pulses);
    std::uint32_t codeword = 0U;
    if (!read_ace_uniform(source, alphabet, codeword)) {
        return false;
    }

    std::vector<std::int32_t> pulses_vector;
    if (!ace_unsigned_pyramid_unrank(
            components, pulses, codeword, pulses_vector)) {
        return false;
    }

    for (const std::int32_t coefficient : pulses_vector) {
        if (coefficient != 0) {
            ++result.nonzero_components;
        }
    }
    std::uint32_t sign_mask = 0U;
    if (!source.read(result.nonzero_components, sign_mask)) {
        return false;
    }

    // The native walks from the end, shifting the raw sign mask right after
    // each nonzero component.  This produces the same sign association as
    // taking the bit stream MSB-first in forward coefficient order.
    for (std::int32_t& coefficient : pulses_vector) {
        if (coefficient == 0) {
            continue;
        }
        const std::uint32_t bit = (sign_mask
            >> (result.nonzero_components - 1U)) & 1U;
        --result.nonzero_components;
        if (bit != 0U) {
            coefficient = -coefficient;
        }
    }

    std::uint64_t norm_square = 0U;
    for (const std::int32_t coefficient : pulses_vector) {
        norm_square += static_cast<std::uint64_t>(coefficient * coefficient);
    }
    if (norm_square == 0U) {
        vector.assign(components, 0.0F);
    } else {
        const float scale = gain / std::sqrt(static_cast<float>(norm_square));
        vector.resize(components);
        for (std::uint32_t index = 0U; index < components; ++index) {
            vector[index] = scale * static_cast<float>(pulses_vector[index]);
        }
    }
    result.nonzero_components = static_cast<std::uint32_t>(
        std::count_if(pulses_vector.begin(), pulses_vector.end(),
            [](std::int32_t value) { return value != 0; }));
    result.signed_pulses = pulses_vector;
    result.bits_consumed = static_cast<std::uint32_t>(source.position() - begin);
    return true;
}

bool ace_vq_consume_split_vector(
    AceBitReader& source,
    std::uint32_t allocated_bits,
    std::uint32_t remaining_bits,
    std::uint32_t components,
    std::uint32_t num_blocks,
    std::int32_t log_n_q4,
    std::uint32_t depth,
    bool high_resolution,
    AceVqPayloadResult& result) noexcept {
    result = {};
    if (num_blocks == 0U) {
        return false;
    }
    return consume_split_vector(
        source, allocated_bits, remaining_bits, components, num_blocks,
        log_n_q4, depth, high_resolution, 0U, 1.0F, result);
}

bool ace_vq_assemble_vector(
    const AceVqPayloadResult& payload,
    std::uint32_t components,
    std::vector<float>& vector) noexcept {
    vector.assign(components, 0.0F);
    std::vector<bool> covered(components, false);
    for (const AceVqLeaf& leaf : payload.leaves) {
        if (leaf.component_offset > components
            || leaf.components > components - leaf.component_offset
            || leaf.normalized_pulses.size() != leaf.components) {
            vector.clear();
            return false;
        }
        for (std::uint32_t index = 0U; index < leaf.components; ++index) {
            const std::uint32_t destination = leaf.component_offset + index;
            if (covered[destination]) {
                vector.clear();
                return false;
            }
            covered[destination] = true;
            vector[destination] = leaf.normalized_pulses[index];
        }
    }
    if (std::find(covered.begin(), covered.end(), false) != covered.end()) {
        vector.clear();
        return false;
    }
    return true;
}

bool ace_stereo_mid_side_revert(
    std::vector<float>& mid,
    std::vector<float>& side,
    float mid_norm,
    float side_norm) noexcept {
    if (mid.size() != side.size()
        || !std::isfinite(mid_norm) || !std::isfinite(side_norm)) {
        return false;
    }

    // DTSAceBandDequant_StereoDecMidSideRevert(): rotate each coefficient,
    // accumulate float L2 sums, then scale by 1/(sqrt(e)+5.421e-20).
    float left_energy = 0.0F;
    float right_energy = 0.0F;
    for (std::size_t index = 0U; index < mid.size(); ++index) {
        const float left = mid[index] * mid_norm + side[index] * side_norm;
        const float right = mid[index] * mid_norm - side[index] * side_norm;
        mid[index] = left;
        side[index] = right;
        left_energy += left * left;
        right_energy += right * right;
    }
    constexpr float kEps = 5.421e-20F;
    const float left_scale = 1.0F / (std::sqrt(left_energy) + kEps);
    const float right_scale = 1.0F / (std::sqrt(right_energy) + kEps);
    for (float& value : mid) {
        value *= left_scale;
    }
    for (float& value : side) {
        value *= right_scale;
    }
    return true;
}

bool ace_decode_band_ratio(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t channel_count,
    bool raw_codeword,
    AceBandRatio& ratio) noexcept {
    if (alphabet == 0U || channel_count == 0U) {
        return false;
    }
    std::uint32_t codeword = 0U;
    if (!read_ace_uniform(source, alphabet + 1U, codeword)) {
        return false;
    }
    if (!raw_codeword) {
        codeword = ace_centered_inverse_map(
            alphabet + 1U, alphabet >> 1U, codeword);
    }
    return native_ratio_from_codeword(alphabet, codeword, channel_count, ratio);
}

bool ace_band_ratio_from_beta(
    const std::int32_t beta_q15,
    const std::uint32_t angle_components,
    AceBandRatio& ratio) noexcept {
    return native_ratio_from_beta(beta_q15, angle_components, ratio);
}

bool ace_vq_read_split(
    AceBitReader& source,
    std::uint32_t alpha,
    std::uint32_t channel_count,
    std::uint32_t allocated_bits,
    std::uint32_t remaining_bits,
    bool raw_codeword,
    AceVqSplit& split) noexcept {
    split = {};
    if (alpha == 0U) {
        return false;
    }
    const std::size_t begin = source.position();
    if (!ace_decode_band_ratio(
            source, alpha, channel_count, raw_codeword, split.ratio)) {
        return false;
    }
    split.bits_consumed = static_cast<std::uint32_t>(source.position() - begin);
    if (split.bits_consumed > allocated_bits
        || split.bits_consumed > remaining_bits) {
        return false;
    }

    // Native: v30 = allocated - ratio_bits; v34 = (v30 + 1 - delta) >> 1;
    // v35 = max(min(v34, v30), 0); v36 = v30 - v35.  The first half uses
    // left_norm and is stored at the lower vector address.
    const std::int64_t available = static_cast<std::int64_t>(allocated_bits)
        - split.bits_consumed;
    std::int64_t left = (available + 1 - split.ratio.mid_side_angle_q15) >> 1;
    if (available <= left) {
        left = available;
    }
    if (left < 0) {
        left = 0;
    }
    split.left_bits = static_cast<std::uint32_t>(left);
    split.right_bits = static_cast<std::uint32_t>(available - left);
    split.final_partition = split.ratio.beta_q15 == -32767
        || split.ratio.beta_q15 == 32767;
    return true;
}

} // namespace dtsx
