#include "render/gain_interpolator.hpp"

namespace dtsx_decode {
namespace {

std::int32_t arithmetic_shift(std::int64_t value,
                              std::uint32_t bits) noexcept {
    if (bits == 0U) {
        return static_cast<std::int32_t>(value);
    }
    if (bits >= 63U) {
        return value < 0 ? -1 : 0;
    }
    return static_cast<std::int32_t>(value >> bits);
}

std::int32_t wrapping_left_shift(std::int32_t value,
                                 std::uint32_t bits) noexcept {
    return static_cast<std::int32_t>(
        static_cast<std::uint32_t>(value) << bits);
}

} // namespace

bool apply_native_gain_ramp(
    NativeGainRamp& ramp,
    std::int32_t destination_gain,
    const std::int32_t* input,
    std::int32_t* output,
    std::size_t sample_count,
    std::uint8_t pcm_fractional_bits,
    GainApplyMode mode,
    bool snap_gain_to_zero_db) noexcept {
    // libdtsx.so: dts_3d_complex_channel_renderer_t_render_apply,
    // 0xe7138..0xe739c.
    if (input == nullptr || output == nullptr
        || ramp.fractional_bits >= 31U
        || pcm_fractional_bits == 0U
        || pcm_fractional_bits >= 31U) {
        return false;
    }
    std::uint32_t gain_bits = ramp.fractional_bits;
    std::int32_t unity = static_cast<std::int32_t>(1U << gain_bits);
    std::int32_t base_gain = ramp.base_gain;
    std::int32_t current = ramp.accumulator;
    std::uint32_t gain_downshift = 0U;
    if (gain_bits <= 22U) {
        gain_downshift = 23U - gain_bits;
        gain_bits = 23U;
        base_gain = wrapping_left_shift(base_gain, gain_downshift);
        unity = wrapping_left_shift(unity, gain_downshift);
        // dts_3d_complex_channel_renderer_t_render_apply keeps *a1,
        // the persistent accumulator, in the promoted Q23 domain. Only
        // the new destination gain (*a4) is promoted on every call.
        destination_gain = wrapping_left_shift(
            destination_gain, gain_downshift);
    }
    const std::int32_t delta = unity - base_gain;
    const std::int64_t interpolation_base =
        static_cast<std::int64_t>(destination_gain) * base_gain;
    const std::int64_t rounding =
        static_cast<std::int64_t>(1U << (pcm_fractional_bits - 1U));
    std::int32_t last_gain = current;
    std::int32_t previous_gain = static_cast<std::int32_t>(sample_count);
    for (std::size_t index = 0; index < sample_count; ++index) {
        previous_gain = last_gain;
        last_gain = arithmetic_shift(
            static_cast<std::int64_t>(delta) * last_gain
                + interpolation_base,
            gain_bits);
        const std::int32_t applied_gain =
            last_gain >> gain_downshift;
        const std::int32_t sample = arithmetic_shift(
            static_cast<std::int64_t>(applied_gain) * input[index]
                + rounding,
            pcm_fractional_bits);
        if (mode == GainApplyMode::Subtract) {
            output[index] = static_cast<std::int32_t>(
                static_cast<std::uint32_t>(output[index])
                - static_cast<std::uint32_t>(sample));
        } else {
            output[index] = static_cast<std::int32_t>(
                static_cast<std::uint32_t>(output[index])
                + static_cast<std::uint32_t>(sample));
        }
    }
    if (snap_gain_to_zero_db
        && last_gain == previous_gain
        && destination_gain == unity
        && last_gain != destination_gain) {
        last_gain = destination_gain;
    }
    ramp.accumulator = last_gain;
    return true;
}

} // namespace dtsx_decode
