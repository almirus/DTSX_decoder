#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <array>
#include <cstdint>

namespace dtsx {

struct CoreSubstreamInfo final {
    std::uint8_t extension_type = 0;
    std::uint8_t dialog_normalization = 0;
    std::uint8_t representation_type = 0;
    std::uint8_t channel_set_count = 0;
    std::uint32_t samples_per_block = 0;
    std::uint32_t audio_frame_size = 0;
    std::uint32_t sample_rate = 0;
    std::uint8_t sample_rate_index = 0;
    std::uint32_t subframe_count = 0;
    std::uint32_t output_block_samples = 0;
    std::uint32_t channel_table_count = 0;
    std::array<std::uint8_t, 32> channel_active{};
    std::array<std::uint8_t, 32> channel_mode{};
    std::array<std::uint8_t, 32> channel_bandwidth{};
};

[[nodiscard]] bool parse_core_substream_header(
    bitstream::Cursor& source, CoreSubstreamInfo& info) noexcept;

} // namespace dtsx
