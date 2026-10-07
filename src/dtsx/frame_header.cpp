#include "dtsx/frame_header.hpp"

namespace dtsx {
namespace {

std::uint32_t load_u32_le(const std::uint8_t* bytes) noexcept {
    return static_cast<std::uint32_t>(bytes[0])
        | (static_cast<std::uint32_t>(bytes[1]) << 8U)
        | (static_cast<std::uint32_t>(bytes[2]) << 16U)
        | (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

std::uint32_t byte_swap_u32(std::uint32_t value) noexcept {
    return (value >> 24U) | ((value >> 8U) & 0x0000FF00U)
        | ((value << 8U) & 0x00FF0000U) | (value << 24U);
}

std::uint32_t extract_14bit_header_bits(
    const std::array<std::uint8_t, 10>& header,
    bool little_endian,
    std::uint32_t bit_offset,
    std::uint32_t bit_count) noexcept {
    std::uint32_t result = 0U;
    while (bit_count != 0U) {
        const std::uint32_t word_index = bit_offset / 14U;
        const std::uint32_t offset_in_word = bit_offset % 14U;
        const std::uint32_t available = 14U - offset_in_word;
        const std::uint32_t take =
            bit_count < available ? bit_count : available;
        const std::size_t byte_index =
            static_cast<std::size_t>(word_index) * 2U;
        const std::uint16_t word = little_endian
            ? static_cast<std::uint16_t>(
                  header[byte_index]
                  | (static_cast<std::uint16_t>(
                         header[byte_index + 1U])
                     << 8U))
            : static_cast<std::uint16_t>(
                  (static_cast<std::uint16_t>(
                       header[byte_index])
                   << 8U)
                  | header[byte_index + 1U]);
        const std::uint32_t shift = available - take;
        const std::uint32_t mask = (1U << take) - 1U;
        result = (result << take)
            | ((static_cast<std::uint32_t>(word) >> shift) & mask);
        bit_offset += take;
        bit_count -= take;
    }
    return result;
}

} // namespace

std::uint32_t unpack_core_frame_size(
    const std::array<std::uint8_t, 8>& header) noexcept {
    const bool big_endian = header[0] == 0x7FU;
    const std::uint32_t byte_4_or_5 = big_endian ? header[5] : header[4];
    const std::uint32_t byte_5_or_4 = big_endian ? header[4] : header[5];
    const std::uint32_t byte_6_or_7 = big_endian ? header[7] : header[6];
    const std::uint32_t byte_7_or_6 = big_endian ? header[6] : header[7];
    const std::uint32_t packed = (byte_4_or_5 << 16U)
        | (byte_5_or_4 << 24U) | byte_6_or_7 | (byte_7_or_6 << 8U);
    return ((packed >> 4U) & 0x3FFFU) + 1U;
}

std::uint32_t unpack_core_14bit_frame_size(
    const std::array<std::uint8_t, 10>& header,
    bool little_endian) noexcept {
    if (extract_14bit_header_bits(
            header, little_endian, 32U, 6U)
        != 63U) {
        return 0U;
    }
    const std::uint32_t raw_size = extract_14bit_header_bits(
        header, little_endian, 46U, 14U);
    if (raw_size < 95U || raw_size > 0x3FFFU) {
        return 0U;
    }

    // dtsForwardBits advances one physical 16-bit word for every fourteen
    // logical bits. Capture therefore ends after the word containing the
    // last logical frame bit.
    const std::uint32_t logical_bits = (raw_size + 1U) * 8U;
    return ((logical_bits + 13U) / 14U) * 2U;
}

ExtensionFrameSizes unpack_extension_frame_sizes(
    const std::array<std::uint8_t, 12>& header) noexcept {
    const bool little_endian = header[0] == 0x64U;
    const std::uint32_t byte_10_or_11 =
        little_endian ? header[10] : header[11];
    const std::uint32_t upper_bytes = little_endian
        ? (static_cast<std::uint32_t>(header[9]) << 16U)
            | (static_cast<std::uint32_t>(header[8]) << 24U)
        : (static_cast<std::uint32_t>(header[8]) << 16U)
            | (static_cast<std::uint32_t>(header[9]) << 24U);
    const std::uint32_t byte_11_or_10 =
        little_endian ? header[11] : header[10];
    const std::uint32_t first_word = little_endian
        ? byte_swap_u32(load_u32_le(header.data() + 4))
        : (static_cast<std::uint32_t>(header[4]) << 16U)
            | (static_cast<std::uint32_t>(header[5]) << 24U)
            | static_cast<std::uint32_t>(header[6])
            | (static_cast<std::uint32_t>(header[7]) << 8U);
    std::uint32_t second_word =
        upper_bytes | byte_11_or_10 | (byte_10_or_11 << 8U);

    ExtensionFrameSizes result;
    result.index = static_cast<std::uint8_t>((first_word >> 22U) & 3U);
    std::uint32_t frame_size_minus_one = 0;
    std::uint32_t header_size_minus_one = 0;
    if ((first_word & 0x200000U) != 0U) {
        frame_size_minus_one = (first_word << 11U) | (second_word >> 21U);
        frame_size_minus_one &= 0xFFFFFU;
        header_size_minus_one = (first_word >> 9U) & 0xFFFU;
    } else {
        second_word = 8U * first_word | (second_word >> 29U);
        frame_size_minus_one = static_cast<std::uint16_t>(second_word);
        header_size_minus_one = static_cast<std::uint8_t>(first_word >> 13U);
    }
    result.header_size = header_size_minus_one + 1U;
    result.frame_size = frame_size_minus_one + 1U;
    return result;
}

bool validate_core_frame_header(bitstream::Cursor& source,
                                std::uint32_t& frame_size) noexcept {
    const bitstream::Cursor saved = source;
    source.fast_forward(32);
    if (source.extract_unsigned(6U) != 63U) {
        source = saved;
        return false;
    }
    source.fast_forward(8);
    const std::uint32_t raw_size = source.extract_unsigned(14U);
    if (raw_size < 95U || raw_size - 95U > 0x3FA0U) {
        source = saved;
        return false;
    }
    frame_size = raw_size + 1U;
    source = saved;
    return true;
}

} // namespace dtsx
