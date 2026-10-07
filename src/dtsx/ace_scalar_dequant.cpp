#include "dtsx/ace_scalar_dequant.hpp"

#include "dtsx/ace_pow2_table.generated.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace dtsx {
namespace {

constexpr std::array<std::int32_t, 22> kPredictiveFQ10{
    614, 594, 579, 563, 550, 538, 527, 517, 512, 512, 512,
    512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512};
constexpr std::array<std::int32_t, 22> kPredictiveBQ10{
    0, 1024, 768, 512, 512, 461, 410, 410, 358, 512, 512,
    512, 614, 614, 614, 614, 614, 614, 717, 768, 819, 819};
constexpr std::array<std::int32_t, 22> kIndependentBQ10{
    0, 947, 973, 717, 717, 614, 512, 512, 461, 614, 640,
    640, 794, 819, 819, 819, 819, 845, 947, 1024, 1024, 1024};

bool valid_control(const AceScalarDequantControl& control) noexcept {
    return control.channel_count != 0U
        && control.first_channel < 2U
        && control.channel_count <= 2U - control.first_channel
        && control.band_count <= 22U;
}

} // namespace

std::int32_t ace_pow2_i32_native(
    const std::int32_t value,
    const std::uint8_t input_fraction,
    const std::int32_t output_fraction) noexcept {
    using namespace native_pow2;
    const std::uint32_t fraction_bits =
        (std::min)(static_cast<std::uint32_t>(input_fraction), 31U);
    const std::int32_t integral = value >> fraction_bits;
    const std::int32_t remainder = static_cast<std::int32_t>(
        static_cast<std::int64_t>(value)
        - (static_cast<std::int64_t>(integral) << fraction_bits));
    const std::int32_t interpolation = fraction_bits == 31U
        ? 0 : static_cast<std::int32_t>(
            static_cast<std::uint32_t>(remainder)
            << (31U - fraction_bits));
    const std::uint32_t table_pair =
        (static_cast<std::uint32_t>(interpolation) >> 24U) & 0x7FU;
    const std::uint32_t table_index = table_pair * 2U;
    const std::uint64_t product = static_cast<std::uint64_t>(
        kTable[table_index + 1U]) * static_cast<std::uint64_t>(
            interpolation & 0x00FFFFFFU);
    const std::uint32_t high_word = static_cast<std::uint32_t>(
        (product - 0x80000000ULL) >> 32U);
    const std::int32_t interpolated = static_cast<std::int32_t>(
        static_cast<std::uint32_t>(kTable[table_index]) + high_word);
    const std::int64_t exponent = static_cast<std::int64_t>(
        output_fraction) + integral;
    const std::int64_t shift = exponent - 31LL;
    if (shift < -2LL) {
        if (shift < -33LL) {
            return 0;
        }
        const std::uint32_t right = static_cast<std::uint32_t>(
            -3LL - shift);
        const std::int64_t biased = static_cast<std::int64_t>(interpolated)
            + (1LL << right);
        return static_cast<std::int32_t>(biased >> (right + 1U));
    }
    const std::int64_t left = exponent - 29LL;
    if (left >= 0LL && left < 31LL) {
        const std::int64_t shifted =
            static_cast<std::int64_t>(interpolated) << left;
        if (shifted <= (std::numeric_limits<std::int32_t>::max)()
            && shifted >= (std::numeric_limits<std::int32_t>::min)()) {
            return static_cast<std::int32_t>(shifted);
        }
    }
    return interpolated < 0 ? (std::numeric_limits<std::int32_t>::min)()
                            : (std::numeric_limits<std::int32_t>::max)();
}

bool ace_scalar_dequant_coarse(
    const AceScalarMatrix& residual,
    const AceScalarMatrix* previous,
    const AceScalarDequantControl& control,
    AceScalarMatrix& output) noexcept {
    if (!valid_control(control)) {
        return false;
    }
    const auto& b = control.predictive ? kPredictiveBQ10 : kIndependentBQ10;
    for (std::uint32_t channel = control.first_channel;
         channel < control.first_channel + control.channel_count;
         ++channel) {
        std::int32_t previous_band_residual_q10 = 0;
        std::int32_t previous_band_term_q10 = 0;
        for (std::uint32_t band = 0U; band < control.band_count; ++band) {
            previous_band_term_q10 += static_cast<std::int32_t>(
                (static_cast<std::int64_t>(b[band])
                    * static_cast<std::int16_t>(
                        previous_band_residual_q10)) >> 10U);
            const std::int32_t current_residual_q10 =
                residual[channel][band] << 10U;
            const std::int32_t previous_frame_q10 = previous != nullptr
                ? std::max((*previous)[channel][band], -16384)
                : 0;
            const std::int32_t frame_term_q10 = control.predictive
                ? static_cast<std::int32_t>(
                    (static_cast<std::int64_t>(kPredictiveFQ10[band])
                        * static_cast<std::int16_t>(previous_frame_q10))
                    >> 10U)
                : 0;
            output[channel][band] = previous_band_term_q10
                + current_residual_q10 + frame_term_q10;
            previous_band_residual_q10 = current_residual_q10;
        }
    }
    return true;
}

bool ace_scalar_dequant_fine(
    AceScalarMatrix& values,
    const AceScalarRefinementCodes& codes,
    const std::array<std::uint32_t, 22>& bit_allocation,
    const AceScalarDequantControl& control) noexcept {
    if (!valid_control(control)) {
        return false;
    }
    for (std::uint32_t channel = control.first_channel;
         channel < control.first_channel + control.channel_count;
         ++channel) {
        for (std::uint32_t band = 0U; band < control.band_count; ++band) {
            const std::uint32_t bits = bit_allocation[band];
            if (bits == 0U) {
                continue;
            }
            if (bits > 16U) {
                return false;
            }
            const std::uint32_t centered =
                2U * static_cast<std::uint32_t>(codes[channel][band]) + 1U;
            const std::uint32_t quantized = bits + 1U <= 10U
                ? centered << (9U - bits)
                : centered >> (bits - 9U);
            values[channel][band] +=
                static_cast<std::int32_t>(quantized) - 512;
        }
    }
    return true;
}

bool ace_scalar_dequant_finalize(
    AceScalarMatrix& values,
    const AceScalarRefinementCodes& final_codes,
    const std::array<std::uint32_t, 22>& fine_bit_allocation,
    const std::array<std::uint32_t, 22>& final_bit_allocation,
    const AceScalarDequantControl& control) noexcept {
    if (!valid_control(control)) {
        return false;
    }
    for (std::uint32_t channel = control.first_channel;
         channel < control.first_channel + control.channel_count;
         ++channel) {
        for (std::uint32_t band = 0U; band < control.band_count; ++band) {
            const std::uint32_t final_bits = final_bit_allocation[band];
            if (final_bits == 0U) {
                continue;
            }
            const std::uint32_t fine_bits = fine_bit_allocation[band];
            // Native v15 is `fine_width + 1 + final_width`: the implicit
            // centre bit is part of the final scalar quantizer width.
            const std::uint32_t total_bits = fine_bits + 1U + final_bits;
            if (fine_bits > 10U || total_bits > 31U) {
                return false;
            }
            const std::uint32_t centered = 2U * static_cast<std::uint32_t>(
                final_codes[channel][band]) + 1U;
            const std::int32_t quantized = total_bits <= 10U
                ? static_cast<std::int32_t>(
                    centered << (10U - total_bits))
                : static_cast<std::int32_t>(
                    centered >> (total_bits - 10U));
            const std::int32_t bias = fine_bits <= 10U
                ? -(1 << (9U - fine_bits))
                : 0;
            values[channel][band] += bias + quantized;
        }
    }
    return true;
}

} // namespace dtsx
