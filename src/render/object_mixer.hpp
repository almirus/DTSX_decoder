#pragma once

#include "render/gain_interpolator.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx_decode {

struct DestinationGainState final {
    NativeGainRamp ramp;
    std::int32_t destination_gain = 0;
};

struct WaveformRenderBlock final {
    const std::int32_t* samples = nullptr;
    std::size_t sample_count = 0;
    std::vector<DestinationGainState> destination_gains;
};

[[nodiscard]] bool render_object_waveforms(
    std::vector<WaveformRenderBlock>& waveforms,
    std::vector<std::vector<std::int32_t>>& destination_channels,
    std::uint8_t pcm_fractional_bits,
    GainApplyMode mode,
    bool clamp_output,
    bool snap_gain_to_zero_db) noexcept;

} // namespace dtsx_decode
