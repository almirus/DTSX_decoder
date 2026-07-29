#include "dtsx/xll_common_header.hpp"

#include "dtsx/crc16.hpp"

namespace dtsx {

bool unpack_xll_common_header(
    bitstream::Cursor& source, XllCommonHeader& header) noexcept {
    // libdtsx.so: dtsx_decodeXLLCommonHeader, 0xb40d0.
    const bitstream::Cursor frame_start = source;
    header = {};
    header.sync = source.extract_unsigned(32U);
    if (header.sync != kXllSyncLegacy && header.sync != kXllSync) {
        return false;
    }

    header.version = static_cast<std::uint8_t>(
        source.extract_unsigned(4U) + 1U);
    header.header_size = source.extract_unsigned(8U) + 1U;
    header.frame_size_bits = static_cast<std::uint8_t>(
        source.extract_unsigned(5U) + 1U);
    header.frame_size =
        source.extract_unsigned(header.frame_size_bits) + 1U;
    header.channel_set_count = static_cast<std::uint8_t>(
        source.extract_unsigned(4U) + 1U);
    header.segments_per_frame = 1U << source.extract_unsigned(4U);
    header.samples_per_segment = 1U << source.extract_unsigned(4U);
    header.segment_size_bits = static_cast<std::uint8_t>(
        source.extract_unsigned(5U) + 1U);
    header.band_crc_present =
        static_cast<std::uint8_t>(source.extract_unsigned(2U));
    header.scalable_lsb = source.extract_unsigned(1U) != 0U;
    header.channel_set_header_size_bits = static_cast<std::uint8_t>(
        source.extract_unsigned(5U) + 1U);
    if (header.scalable_lsb) {
        header.scalable_resolution =
            static_cast<std::uint8_t>(source.extract_unsigned(4U));
    }
    header.legacy_sync = header.sync == kXllSyncLegacy;
    if (header.legacy_sync) {
        header.legacy_flag = source.extract_unsigned(1U) != 0U;
    }

    const std::uint32_t available_bytes =
        frame_start.remaining_bits() / 8U;
    if (header.header_size < 4U
        || header.header_size > available_bytes
        || header.frame_size < header.header_size
        || header.frame_size > available_bytes) {
        return false;
    }
    bitstream::Cursor crc_source = frame_start;
    crc_source.fast_forward(32);
    header.crc_valid =
        valid_crc16(crc_source, 8U * (header.header_size - 4U));
    source = frame_start;
    source.fast_forward(
        static_cast<std::int32_t>(8U * header.header_size));
    return header.crc_valid;
}

} // namespace dtsx
