#pragma once

#include <cstdint>

namespace dtsx {

struct RawInfoFields final {
    std::uint32_t flags0 = 0;
    std::uint8_t value4 = 0;
    std::uint8_t value8 = 0;
    std::uint8_t value12 = 0;
    std::uint32_t value16 = 0;
    std::uint32_t value20 = 0;
    std::uint8_t value24 = 0;
    std::uint8_t value28 = 0;
    std::uint16_t value32 = 0;
    std::uint16_t value36 = 0;
    std::uint8_t value56 = 0;
    std::uint32_t value68 = 0;
    std::uint8_t value72 = 0;
    std::uint8_t value80 = 0;
};

struct RawInfoMasks final {
    std::uint32_t first = 0;
    std::uint32_t second = 0;
};

[[nodiscard]] RawInfoMasks set_raw_info(const RawInfoFields& fields) noexcept;

} // namespace dtsx
