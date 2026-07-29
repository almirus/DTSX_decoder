#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <cstdint>

namespace dtsx {

struct LbrChunkHeader final {
    std::uint8_t chunk_id = 0;
    std::uint8_t header_bytes = 0;
    std::uint16_t payload_bytes = 0;
};

[[nodiscard]] LbrChunkHeader unpack_lbr_chunk_header(
    bitstream::Cursor& source) noexcept;

[[nodiscard]] bool parse_lbr_channels(bitstream::Cursor& source,
                                       std::uint32_t& speaker_count,
                                       std::uint32_t& speaker_mask,
                                       bool& has_extension) noexcept;

} // namespace dtsx
