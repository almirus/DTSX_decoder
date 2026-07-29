#pragma once

#include <cstdint>

namespace dtsx::bitstream {

struct Position final {
    std::uint32_t word_offset = 0;
    std::uint8_t bit_offset = 0;
};

class Segment final {
public:
    Segment() noexcept = default;

    void init(const std::uint32_t* words, std::uint32_t segment_size) noexcept;
    void reset() noexcept;
    void clone_from(const Segment& source) noexcept;

    [[nodiscard]] bool is_valid() const noexcept;
    [[nodiscard]] Position current_position() const noexcept;
    [[nodiscard]] Position start_position() const noexcept;
    [[nodiscard]] Position end_position() const noexcept;

    void set_start_to_position(Position position) noexcept;
    void set_end_to_position(Position position) noexcept;
    void move_to_start() noexcept;
    void fast_forward(std::int32_t bits) noexcept;
    void rewind_bits(std::uint32_t bits) noexcept;
    void move_to_32_bit_boundary() noexcept;
    void move_to_8_bit_boundary() noexcept;
    void align_to_previous_32_bit_boundary() noexcept;
    [[nodiscard]] bool move_to_position(Position position) noexcept;
    void set_mode_14_bit() noexcept;
    void set_mode_16_bit() noexcept;
    [[nodiscard]] bool in_14_bit_mode() const noexcept;

    [[nodiscard]] bool at_start() const noexcept;
    [[nodiscard]] bool at_end() const noexcept;

    [[nodiscard]] std::uint32_t extract_unsigned(
        std::uint32_t requested_bits) noexcept;
    [[nodiscard]] std::uint32_t remaining_bits() const noexcept;
    [[nodiscard]] std::uint32_t words_to_end() const noexcept;
    [[nodiscard]] std::uint32_t current_size_in_words() const noexcept;
    [[nodiscard]] std::int32_t bit_distance(Position first,
                                            Position second) const noexcept;

private:
    const std::uint32_t* current_ = nullptr;
    std::uint32_t current_bit_ = 0;
    std::uint32_t remaining_bits_ = 0;
    std::uint32_t start_word_ = 0;
    std::uint32_t start_bit_ = 0;
    std::uint32_t end_word_ = 0;
    std::uint32_t end_bit_ = 0;
    std::uint32_t segment_size_ = 0;
    const std::uint32_t* base_ = nullptr;
    std::uint32_t mode_ = 0;
};

} // namespace dtsx::bitstream
