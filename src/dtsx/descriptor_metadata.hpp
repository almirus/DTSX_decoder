#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <cstdint>

namespace dtsx {

struct DescriptorMetadata final {
    std::uint32_t enabled = 0;
    std::uint8_t descriptor_class = 0;
};

[[nodiscard]] bool unpack_descriptor_metadata(bitstream::Cursor& source,
                                              DescriptorMetadata& metadata) noexcept;

} // namespace dtsx
