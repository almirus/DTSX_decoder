#include "bitstream/dtsx_word_buffer.hpp"

#include <limits>
#include <stdexcept>

namespace dtsx::bitstream {

std::optional<BitstreamInit> initialize_bitstream(
    const std::vector<std::uint8_t>& bytes) noexcept {
    if (bytes.size() < 4U) {
        return std::nullopt;
    }
    const std::uint32_t word =
        (static_cast<std::uint32_t>(bytes[0]) << 24U)
        | (static_cast<std::uint32_t>(bytes[1]) << 16U)
        | (static_cast<std::uint32_t>(bytes[2]) << 8U)
        | bytes[3];
    switch (word) {
    case 0x7FFE8001U:
    case 0xFE7F0180U:
    case 0x58642520U:
        return BitstreamInit{InputPacking::Core16Bit, 16U,
                              word == 0xFE7F0180U};
    case 0x1FFFE800U:
    case 0xFF1F00E8U:
        return BitstreamInit{InputPacking::Core14Bit, 14U,
                              word == 0xFF1F00E8U};
    case 0x64582025U:
        return BitstreamInit{InputPacking::Extension32Bit, 32U, false};
    default:
        return std::nullopt;
    }
}

WordBuffer::WordBuffer(
    const std::vector<std::uint8_t>& bytes, bool swap_byte_pairs) {
    if (bytes.empty()) {
        throw std::invalid_argument("DTS bitstream buffer is empty");
    }
    if (bytes.size()
        > static_cast<std::size_t>(
            std::numeric_limits<std::uint32_t>::max() / 8U)) {
        throw std::length_error("DTS bitstream buffer is too large");
    }

    bit_count_ = static_cast<std::uint32_t>(bytes.size() * 8U);
    // The native bitstream reader performs a two-word lookahead even for a
    // request that ends in the final word.  Keep an explicit zero sentinel so
    // truncated/header-limited reads cannot dereference past the buffer.
    words_.assign((bytes.size() + 3U) / 4U + 1U, 0U);
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        std::size_t source_index = index;
        if (swap_byte_pairs) {
            const std::size_t paired_index = index ^ 1U;
            if (paired_index < bytes.size()) {
                source_index = paired_index;
            }
        }
        const std::size_t word_index = index / 4U;
        const std::uint32_t shift =
            24U - 8U * static_cast<std::uint32_t>(index & 3U);
        words_[word_index] |=
            static_cast<std::uint32_t>(bytes[source_index]) << shift;
    }
}

Cursor WordBuffer::cursor() const noexcept {
    // dtsxBitstreamExtractBitsUnsigned consumes the most-significant bit of
    // each 32-bit word first. The pair swap follows the big/little sync-word
    // pairs accepted by the frame-capture parser.
    return Cursor(words_.data(), bit_count_);
}

std::uint32_t WordBuffer::bit_count() const noexcept {
    return bit_count_;
}

} // namespace dtsx::bitstream
