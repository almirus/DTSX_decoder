#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct SixBitUpdate final {
    std::uint8_t index = 0;
    std::uint8_t value = 0;
};

struct WaveformMetadataUpdate final {
    bool first_set_present = false;
    bool second_set_present = false;
    std::uint32_t first_mask = 0;
    std::uint32_t second_mask = 0;
    std::vector<SixBitUpdate> first_values;
    std::vector<SixBitUpdate> second_values;
};

struct ModeThreeWaveformValue final {
    std::uint8_t gain_code = 61;
    std::uint32_t selected_index = 0;
    std::vector<std::uint8_t> values;
};

struct ModeThreeMetadata final {
    bool per_waveform_gain_present = false;
    std::vector<ModeThreeWaveformValue> waveforms;
    bool fallback_gain_present = false;
    std::uint8_t fallback_gain_code = 61;
};

[[nodiscard]] std::vector<WaveformMetadataUpdate> unpack_waveform_metadata_updates(
    bitstream::Cursor& source,
    std::uint8_t waveform_count,
    std::uint8_t update_value_count);

[[nodiscard]] ModeThreeMetadata unpack_mode_three_metadata(
    bitstream::Cursor& source,
    std::uint8_t waveform_count,
    std::uint8_t update_value_count);

} // namespace dtsx
