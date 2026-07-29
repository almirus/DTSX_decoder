#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <cstdint>

namespace dtsx {

[[nodiscard]] bool valid_crc16(bitstream::Cursor& source,
                               std::uint32_t bit_count) noexcept;

} // namespace dtsx
