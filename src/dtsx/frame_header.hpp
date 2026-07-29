#pragma once

#include <array>
#include <cstdint>

#include "bitstream/dtsx_bitstream.hpp"

namespace dtsx {

struct ExtensionFrameSizes final {
    std::uint32_t header_size = 0;
    std::uint32_t frame_size = 0;
    std::uint8_t index = 0;
};

[[nodiscard]] std::uint32_t unpack_core_frame_size(
    const std::array<std::uint8_t, 8>& header) noexcept;

[[nodiscard]] std::uint32_t unpack_core_14bit_frame_size(
    const std::array<std::uint8_t, 10>& header,
    bool little_endian) noexcept;

[[nodiscard]] ExtensionFrameSizes unpack_extension_frame_sizes(
    const std::array<std::uint8_t, 12>& header) noexcept;

[[nodiscard]] bool validate_core_frame_header(bitstream::Cursor& source,
                                               std::uint32_t& frame_size)
    noexcept;

} // namespace dtsx
