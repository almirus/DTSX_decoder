#include "dtsx/xll_navigation.hpp"

#include "dtsx/crc16.hpp"

#include <algorithm>
#include <limits>

namespace dtsx {
namespace {

std::size_t entry_index(
    const XllNavigationTable& table,
    std::uint8_t band,
    std::uint32_t segment,
    std::uint8_t channel_set) noexcept {
    return (static_cast<std::size_t>(band) * table.segment_count + segment)
        * table.channel_set_count + channel_set;
}

} // namespace

const XllNavigationEntry* XllNavigationTable::find(
    std::uint8_t band,
    std::uint32_t segment,
    std::uint8_t channel_set) const noexcept {
    if (band >= band_count || segment >= segment_count
        || channel_set >= channel_set_count) {
        return nullptr;
    }
    return &entries[entry_index(*this, band, segment, channel_set)];
}

bool unpack_xll_navigation_table(
    bitstream::Cursor& source,
    std::uint8_t segment_size_bits,
    std::uint32_t segment_count,
    const std::vector<std::uint8_t>& channel_set_band_counts,
    XllNavigationTable& table) noexcept {
    table = {};
    if (segment_size_bits == 0U || segment_size_bits > 32U
        || segment_count == 0U || channel_set_band_counts.empty()
        || channel_set_band_counts.size()
            > std::numeric_limits<std::uint8_t>::max()) {
        return false;
    }

    table.band_count = *std::max_element(
        channel_set_band_counts.begin(), channel_set_band_counts.end());
    table.segment_count = segment_count;
    table.channel_set_count = static_cast<std::uint8_t>(
        channel_set_band_counts.size());
    if (table.band_count == 0U) {
        return false;
    }

    const std::size_t entry_count =
        static_cast<std::size_t>(table.band_count) * segment_count
        * table.channel_set_count;
    table.entries.resize(entry_count);
    const bitstream::Cursor table_start = source;
    std::uint32_t cumulative_offset = 0U;
    std::uint64_t encoded_bits = 16U;

    for (std::uint8_t band = 0; band < table.band_count; ++band) {
        for (std::uint32_t segment = 0;
             segment < segment_count;
             ++segment) {
            for (std::uint8_t channel_set = 0;
                 channel_set < table.channel_set_count;
                 ++channel_set) {
                XllNavigationEntry& entry = table.entries[
                    entry_index(table, band, segment, channel_set)];
                entry.byte_offset = cumulative_offset;
                if (channel_set_band_counts[channel_set] <= band) {
                    continue;
                }
                if (source.remaining_bits() < segment_size_bits) {
                    table = {};
                    return false;
                }
                entry.size_bytes =
                    source.extract_unsigned(segment_size_bits) + 1U;
                cumulative_offset += entry.size_bytes;
                encoded_bits += segment_size_bits;
            }
        }
    }

    const std::uint32_t alignment =
        (8U - (source.bit_offset() & 7U)) & 7U;
    source.fast_forward(static_cast<std::int32_t>(alignment));
    if (source.remaining_bits() < 16U) {
        table = {};
        return false;
    }
    source.fast_forward(16);

    table.byte_size = static_cast<std::uint32_t>(
        (encoded_bits + 7U) >> 3U);
    bitstream::Cursor crc_source = table_start;
    table.crc_valid =
        valid_crc16(crc_source, 8U * table.byte_size);
    return table.crc_valid;
}

} // namespace dtsx
