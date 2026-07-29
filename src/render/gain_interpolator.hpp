#pragma once

#include <cstddef>
#include <cstdint>

namespace dtsx_decode {

struct NativeGainRamp final {
    std::int32_t accumulator = 0;
    std::int32_t base_gain = 0;
    std::uint8_t fractional_bits = 0;
};

enum class GainApplyMode {
    Add = 0,
    Subtract = 1,
};

[[nodiscard]] bool apply_native_gain_ramp(
    NativeGainRamp& ramp,
    std::int32_t destination_gain,
    const std::int32_t* input,
    std::int32_t* output,
    std::size_t sample_count,
    std::uint8_t pcm_fractional_bits,
    GainApplyMode mode,
    bool snap_gain_to_zero_db) noexcept;

} // namespace dtsx_decode
