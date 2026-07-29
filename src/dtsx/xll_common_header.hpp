#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <cstdint>

namespace dtsx {

constexpr std::uint32_t kXllSyncLegacy = 0x34388C4FU;
constexpr std::uint32_t kXllSync = 0x41A29547U;

struct XllCommonHeader final {
    std::uint32_t sync = 0;
    std::uint8_t version = 0;
    std::uint32_t header_size = 0;
    std::uint8_t frame_size_bits = 0;
    std::uint32_t frame_size = 0;
    std::uint8_t channel_set_count = 0;
    std::uint32_t segments_per_frame = 0;
    std::uint32_t samples_per_segment = 0;
    std::uint8_t segment_size_bits = 0;
    std::uint8_t band_crc_present = 0;
    bool scalable_lsb = false;
    std::uint8_t channel_set_header_size_bits = 0;
    std::uint8_t scalable_resolution = 0;
    bool legacy_sync = false;
    bool legacy_flag = false;
    bool crc_valid = false;
};

[[nodiscard]] bool unpack_xll_common_header(
    bitstream::Cursor& source, XllCommonHeader& header) noexcept;

} // namespace dtsx
