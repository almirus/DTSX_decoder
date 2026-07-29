#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct XllNavigationEntry final {
    std::uint32_t size_bytes = 0;
    std::uint32_t byte_offset = 0;
};

struct XllNavigationTable final {
    bool crc_valid = false;
    std::uint32_t byte_size = 0;
    std::uint8_t band_count = 0;
    std::uint32_t segment_count = 0;
    std::uint8_t channel_set_count = 0;
    std::vector<XllNavigationEntry> entries;

    [[nodiscard]] const XllNavigationEntry* find(
        std::uint8_t band,
        std::uint32_t segment,
        std::uint8_t channel_set) const noexcept;
};

[[nodiscard]] bool unpack_xll_navigation_table(
    bitstream::Cursor& source,
    std::uint8_t segment_size_bits,
    std::uint32_t segment_count,
    const std::vector<std::uint8_t>& channel_set_band_counts,
    XllNavigationTable& table) noexcept;

} // namespace dtsx
