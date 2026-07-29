#include "bitstream/dtsx_segment.hpp"

#include <algorithm>

namespace dtsx::bitstream {
namespace {

std::uint32_t position_bits(Position position) noexcept {
    return position.word_offset * 32U + position.bit_offset;
}

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

void Segment::init(const std::uint32_t* words,
                   std::uint32_t segment_size) noexcept {
    // libdtsx.so: dtsxBitstreamInitSegment, 0x95180.
    current_ = words;
    base_ = words;
    segment_size_ = segment_size;
    current_bit_ = 0;
    remaining_bits_ = 0;
    start_word_ = 0;
    start_bit_ = 0;
    end_word_ = 0;
    end_bit_ = 0;
    mode_ = 0;
}

void Segment::reset() noexcept {
    // libdtsx.so: dtsxBitstreamReset, 0x9549c.
    current_ = base_;
    current_bit_ = 0;
    remaining_bits_ = 0;
    end_word_ = start_word_;
    end_bit_ = start_bit_;
}

void Segment::clone_from(const Segment& source) noexcept {
    // libdtsx.so: dtsxBitstreamClone, 0x95610.
    *this = source;
}

bool Segment::is_valid() const noexcept {
    // libdtsx.so: dtsxBitstreamIsSegmentValid, 0x9515c.
    return current_ != nullptr && base_ != nullptr;
}

Position Segment::current_position() const noexcept {
    // libdtsx.so: dtsxBitstreamGetCurrentBitPosition, 0x95318.
    const auto word_delta = static_cast<std::uint32_t>(current_ - base_);
    return {word_delta, static_cast<std::uint8_t>(current_bit_)};
}

Position Segment::start_position() const noexcept {
    return {start_word_, static_cast<std::uint8_t>(start_bit_)};
}

Position Segment::end_position() const noexcept {
    return {end_word_, static_cast<std::uint8_t>(end_bit_)};
}

void Segment::set_start_to_position(Position position) noexcept {
    // libdtsx.so: dtsxBitstreamSetStartToPosition, 0x956a8.
    const Position current = current_position();
    const Position end = end_position();
    const std::uint32_t requested = position_bits(position);
    const std::uint32_t current_bits = position_bits(current);
    const std::uint32_t end_bits = position_bits(end);
    start_word_ = position.word_offset;
    start_bit_ = position.bit_offset;
    if (requested > current_bits) {
        current_ = base_ + position.word_offset;
        current_bit_ = position.bit_offset;
        remaining_bits_ = end_bits - requested;
    }
    if (requested > end_bits) {
        end_word_ = position.word_offset;
        end_bit_ = position.bit_offset;
        remaining_bits_ = requested - current_bits;
    }
}

void Segment::set_end_to_position(Position position) noexcept {
    // libdtsx.so: dtsxBitstreamSetEndToPosition, 0x957f0.
    const Position current = current_position();
    const Position start = start_position();
    const std::uint32_t requested = position_bits(position);
    const std::uint32_t current_bits = position_bits(current);
    const std::uint32_t start_bits = position_bits(start);
    end_word_ = position.word_offset;
    end_bit_ = position.bit_offset;
    if (requested < current_bits) {
        current_ = base_ + position.word_offset;
        current_bit_ = position.bit_offset;
        remaining_bits_ = 0;
    } else {
        remaining_bits_ = requested - current_bits;
    }
    if (requested < start_bits) {
        start_word_ = end_word_;
        start_bit_ = end_bit_;
    }
}

void Segment::move_to_start() noexcept {
    // libdtsx.so: dtsxBitstreamMoveToStart, 0x95a3c.
    const std::uint32_t end_bits = end_word_ * 32U + end_bit_;
    const std::uint32_t start_bits = start_word_ * 32U + start_bit_;
    current_ = base_ + start_word_;
    current_bit_ = start_bit_;
    remaining_bits_ = end_bits - start_bits;
}

void Segment::fast_forward(std::int32_t bits) noexcept {
    // libdtsx.so: dtsxBitstreamFastForwardBits, 0x96010.
    const std::uint32_t nonnegative =
        bits < 0 ? 0U : static_cast<std::uint32_t>(bits);
    const std::uint32_t amount = std::min(nonnegative, remaining_bits_);
    const std::uint32_t absolute = current_bit_ + amount;
    current_ += absolute >> 5U;
    current_bit_ = absolute & 31U;
    remaining_bits_ -= amount;
}

void Segment::rewind_bits(std::uint32_t bits) noexcept {
    // libdtsx.so: dtsxBitstreamRewindBits, 0x95228.
    const Position current = current_position();
    const Position start = start_position();
    const Position end = end_position();
    const std::uint32_t current_bits = position_bits(current);
    const std::uint32_t start_bits = position_bits(start);
    if (current_bits - start_bits < bits) {
        current_ = base_;
        current_bit_ = 0;
        remaining_bits_ = position_bits(end) - start_bits;
        return;
    }
    const std::uint32_t new_position = current_bits - bits;
    current_ = base_ + (new_position >> 5U);
    current_bit_ = new_position & 31U;
    remaining_bits_ += bits;
}

void Segment::move_to_32_bit_boundary() noexcept {
    // libdtsx.so: dtsxBitstreamMoveTo32BitBoundary, 0x953ac.
    if (current_bit_ == 0U) {
        return;
    }
    current_ += 1;
    remaining_bits_ = remaining_bits_ + current_bit_ - 32U;
    current_bit_ = 0;
}

void Segment::move_to_8_bit_boundary() noexcept {
    // libdtsx.so: dtsxBitstreamMoveTo8BitBoundary, 0x9541c.
    if ((current_bit_ & 7U) == 0U) {
        return;
    }
    const std::uint32_t rounded = (current_bit_ + 7U) & ~7U;
    const bool crosses_word = rounded > 31U;
    const std::uint32_t absolute = current_bit_ + remaining_bits_;
    remaining_bits_ = absolute - rounded;
    if (crosses_word) {
        current_ = current_ + 1;
        current_bit_ = 0;
    } else {
        current_bit_ = rounded;
    }
}

void Segment::align_to_previous_32_bit_boundary() noexcept {
    // libdtsx.so: dtsxBitstreamAlignToPrevious32BitBoundary, 0x95f00.
    remaining_bits_ += current_bit_;
    current_bit_ = 0;
}

bool Segment::move_to_position(Position position) noexcept {
    // libdtsx.so: dtsxBitstreamMoveToPosition, 0x95930.
    const std::uint32_t requested = position_bits(position);
    const std::uint32_t start = position_bits(start_position());
    const std::uint32_t end = position_bits(end_position());
    if (requested < start || requested > end) {
        return false;
    }
    current_ = base_ + position.word_offset;
    current_bit_ = position.bit_offset;
    remaining_bits_ = end - requested;
    return true;
}

bool Segment::at_start() const noexcept {
    // libdtsx.so: dtsxBitstreamAtStart, 0x95d50.
    return position_bits(current_position()) == position_bits(start_position());
}

bool Segment::at_end() const noexcept {
    // libdtsx.so: dtsxBitstreamAtEnd, 0x95ce8.
    return position_bits(current_position()) == position_bits(end_position());
}

void Segment::set_mode_14_bit() noexcept {
    // libdtsx.so: dtsxBitstreamSetMode14bit, 0x95db8.
    mode_ = 1U;
}

void Segment::set_mode_16_bit() noexcept {
    // libdtsx.so: dtsxBitstreamSetMode16bit, 0x95e0c.
    mode_ = 0U;
}

bool Segment::in_14_bit_mode() const noexcept {
    // libdtsx.so: dtsxBitstreamIn14BitMode, 0x95eb4.
    return mode_ != 0U;
}

std::uint32_t Segment::extract_unsigned(
    std::uint32_t requested_bits) noexcept {
    const std::uint32_t amount = std::min(requested_bits, remaining_bits_);
    const std::uint32_t merged = merge_words(current_, current_bit_, amount);
    const std::uint32_t absolute = current_bit_ + amount;
    current_ += absolute >> 5U;
    current_bit_ = absolute & 31U;
    remaining_bits_ -= amount;
    return high_bits(merged, amount);
}

std::uint32_t Segment::remaining_bits() const noexcept {
    return remaining_bits_;
}

std::uint32_t Segment::words_to_end() const noexcept {
    // libdtsx.so: dtsxBitstream32bitWordsToEnd, 0x95bac.
    return end_word_ - static_cast<std::uint32_t>(current_ - base_);
}

std::uint32_t Segment::current_size_in_words() const noexcept {
    // libdtsx.so: dtsxBitstreamGetCurrentSizeIn32BitWords, 0x95c00.
    return end_word_ - start_word_;
}

std::int32_t Segment::bit_distance(Position first,
                                   Position second) const noexcept {
    // libdtsx.so: dtsxBitstreamBitDistanceBetweenPositions, 0x9604c.
    return static_cast<std::int32_t>(position_bits(second))
        - static_cast<std::int32_t>(position_bits(first));
}

} // namespace dtsx::bitstream
