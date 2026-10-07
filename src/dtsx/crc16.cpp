#include "dtsx/crc16.hpp"

#include <array>

namespace dtsx {
namespace {

constexpr std::array<std::uint16_t, 16> kCrcTable = {
    0x0000U,
    0x1021U,
    0x2042U,
    0x3063U,
    0x4084U,
    0x50A5U,
    0x60C6U,
    0x70E7U,
    0x8108U,
    0x9129U,
    0xA14AU,
    0xB16BU,
    0xC18CU,
    0xD1ADU,
    0xE1CEU,
    0xF1EFU,
};

} // namespace

bool valid_crc16(bitstream::Cursor& source, std::uint32_t bit_count) noexcept {
    if (bit_count == 0U) {
        return false;
    }

    std::uint16_t crc = 0xFFFFU;
    std::uint32_t consumed = 0;
    do {
        const std::uint8_t value =
            static_cast<std::uint8_t>(source.extract_unsigned(8U));
        consumed += 8U;
        const std::uint16_t high_step = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(crc << 4U))
            ^ kCrcTable[(value >> 4U) ^ (crc >> 12U)]);
        crc = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(high_step << 4U)
            ^ kCrcTable[(value & 0x0FU) ^ (high_step >> 12U)]);
    } while (consumed < bit_count);
    return crc == 0U;
}

} // namespace dtsx
