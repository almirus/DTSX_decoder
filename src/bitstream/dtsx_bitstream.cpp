#include "bitstream/dtsx_bitstream.hpp"

#include <algorithm>

namespace dtsx::bitstream {
namespace {

std::uint32_t merge_words(const std::uint32_t* words,
                          std::uint32_t bit_offset,
                          std::uint32_t requested_bits) noexcept {
    const std::uint32_t first = words[0] << bit_offset;
    if (requested_bits <= 32U - bit_offset) {
        return first;
    }

    return first | (words[1] >> (32U - bit_offset));
}

std::uint32_t high_bits(std::uint32_t value, std::uint32_t count) noexcept {
    return count == 0U ? 0U : value >> (32U - count);
}

} // namespace

Cursor::Cursor(const std::uint32_t* words, std::uint32_t remaining_bits) noexcept
    : current_word_(words),
      bit_offset_(0),
      remaining_bits_(remaining_bits),
      valid_(true) {}

std::uint32_t Cursor::extract_unsigned(std::uint32_t requested_bits) noexcept {
    // libdtsx.so: dtsxBitstreamExtractBitsUnsigned, 0x642ac.
    if (requested_bits > remaining_bits_) {
        valid_ = false;
    }
    const std::uint32_t extracted_bits = std::min(requested_bits, remaining_bits_);
    const std::uint32_t merged = merge_words(current_word_, bit_offset_, extracted_bits);
    const std::uint32_t new_offset = bit_offset_ + extracted_bits;

    if (new_offset >= 32U) {
        ++current_word_;
        bit_offset_ = new_offset - 32U;
    } else {
        bit_offset_ = new_offset;
    }
    remaining_bits_ -= extracted_bits;
    return high_bits(merged, extracted_bits);
}

std::int32_t Cursor::extract_signed(std::uint32_t requested_bits) noexcept {
    // libdtsx.so: dtsxBitstreamExtractBitsSigned, 0x64300.
    if (requested_bits > remaining_bits_) {
        valid_ = false;
    }
    const std::uint32_t extracted_bits =
        std::min(requested_bits, remaining_bits_);
    const std::uint32_t value =
        extract_unsigned(extracted_bits);
    if (extracted_bits == 0U || extracted_bits == 32U
        || (value & (1U << (extracted_bits - 1U))) == 0U) {
        return static_cast<std::int32_t>(value);
    }
    return static_cast<std::int32_t>(
        value | (~0U << extracted_bits));
}

bool Cursor::attempt_extract_signed(std::uint32_t requested_bits,
                                    std::int32_t& value) noexcept {
    // libdtsx.so: dtsxBitstreamAttemptToExtractBitsSigned, 0x95f58.
    if (requested_bits > remaining_bits_) {
        value = 0;
        return false;
    }
    value = extract_signed(requested_bits);
    return true;
}

std::uint32_t Cursor::lookahead_unsigned(std::uint32_t requested_bits) const noexcept {
    // libdtsx.so: dtsxBitstreamLookaheadBitsUnsigned, 0x95fe4.
    const std::uint32_t merged = merge_words(current_word_, bit_offset_, requested_bits);
    return high_bits(merged, requested_bits);
}

Cursor Cursor::limited(std::uint32_t bit_count) const noexcept {
    Cursor result = *this;
    if (bit_count > remaining_bits_) {
        result.valid_ = false;
    }
    result.remaining_bits_ = std::min(bit_count, remaining_bits_);
    return result;
}

void Cursor::fast_forward(std::int32_t requested_bits) noexcept {
    // libdtsx.so: dtsxBitstreamFastForwardBits, 0x96010.
    const std::uint32_t nonnegative_bits =
        requested_bits < 0 ? 0U : static_cast<std::uint32_t>(requested_bits);
    if (nonnegative_bits > remaining_bits_) {
        valid_ = false;
    }
    const std::uint32_t forwarded_bits = std::min(nonnegative_bits, remaining_bits_);
    const std::uint32_t new_offset = bit_offset_ + forwarded_bits;

    remaining_bits_ -= forwarded_bits;
    bit_offset_ = new_offset & 31U;
    current_word_ += new_offset >> 5U;
}

const std::uint32_t* Cursor::current_word() const noexcept {
    return current_word_;
}

std::uint32_t Cursor::bit_offset() const noexcept {
    return bit_offset_;
}

std::uint32_t Cursor::remaining_bits() const noexcept {
    return remaining_bits_;
}

bool Cursor::valid() const noexcept {
    return valid_;
}

} // namespace dtsx::bitstream
