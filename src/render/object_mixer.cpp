#include "render/object_mixer.hpp"

#include <algorithm>

namespace dtsx_decode {

bool render_object_waveforms(
    std::vector<WaveformRenderBlock>& waveforms,
    std::vector<std::vector<std::int32_t>>& destination_channels,
    std::uint8_t pcm_fractional_bits,
    GainApplyMode mode,
    bool clamp_output,
    bool snap_gain_to_zero_db) noexcept {
    // libdtsx.so: dts_3d_complex_channel_renderer_t_render,
    // 0xe73a0..0xe7628.
    if (destination_channels.empty()) {
        return waveforms.empty();
    }
    const std::size_t sample_count = destination_channels.front().size();
    for (const auto& channel : destination_channels) {
        if (channel.size() != sample_count) {
            return false;
        }
    }
    for (WaveformRenderBlock& waveform : waveforms) {
        if (waveform.samples == nullptr || waveform.sample_count != sample_count
            || waveform.destination_gains.size()
                != destination_channels.size()) {
            return false;
        }
        for (std::size_t channel = 0;
             channel < destination_channels.size();
             ++channel) {
            DestinationGainState& gain = waveform.destination_gains[channel];
            if (!apply_native_gain_ramp(
                    gain.ramp,
                    gain.destination_gain,
                    waveform.samples,
                    destination_channels[channel].data(),
                    sample_count,
                    pcm_fractional_bits,
                    mode,
                    snap_gain_to_zero_db)) {
                return false;
            }
        }
    }
    if (clamp_output) {
        constexpr std::int32_t kMinimum = -0x800000;
        constexpr std::int32_t kMaximum = 0x7FFFFF;
        for (auto& channel : destination_channels) {
            for (std::int32_t& sample : channel) {
                sample = std::max(kMinimum, std::min(kMaximum, sample));
            }
        }
    }
    return true;
}

} // namespace dtsx_decode
