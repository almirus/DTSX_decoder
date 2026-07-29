#pragma once

#include <cstdint>

namespace dtsx::bitstream {

class Cursor final {
public:
    Cursor(const std::uint32_t* words, std::uint32_t remaining_bits) noexcept;

    [[nodiscard]] std::uint32_t extract_unsigned(std::uint32_t requested_bits) noexcept;
    [[nodiscard]] std::int32_t extract_signed(std::uint32_t requested_bits) noexcept;
    [[nodiscard]] bool attempt_extract_signed(
        std::uint32_t requested_bits, std::int32_t& value) noexcept;
    [[nodiscard]] std::uint32_t lookahead_unsigned(std::uint32_t requested_bits) const noexcept;
    [[nodiscard]] Cursor limited(std::uint32_t bit_count) const noexcept;
    void fast_forward(std::int32_t requested_bits) noexcept;

    [[nodiscard]] const std::uint32_t* current_word() const noexcept;
    [[nodiscard]] std::uint32_t bit_offset() const noexcept;
    [[nodiscard]] std::uint32_t remaining_bits() const noexcept;
    [[nodiscard]] bool valid() const noexcept;

private:
    const std::uint32_t* current_word_;
    std::uint32_t bit_offset_;
    std::uint32_t remaining_bits_;
    bool valid_;
};

} // namespace dtsx::bitstream
