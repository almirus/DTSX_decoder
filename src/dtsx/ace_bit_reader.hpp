#pragma once

#include <cstddef>
#include <cstdint>

namespace dtsx {

class AceBitReader final {
public:
    AceBitReader(const std::uint8_t* bytes, std::size_t size) noexcept;

    [[nodiscard]] bool read(
        std::uint32_t count,
        std::uint32_t& value) noexcept;
    [[nodiscard]] bool align() noexcept;
    [[nodiscard]] bool seek(std::size_t bit_position) noexcept;
    [[nodiscard]] std::size_t position() const noexcept;
    [[nodiscard]] std::size_t bits_left() const noexcept;
    [[nodiscard]] bool valid() const noexcept;

private:
    const std::uint8_t* bytes_ = nullptr;
    std::size_t bit_limit_ = 0U;
    std::size_t position_ = 0U;
    bool valid_ = true;
};

[[nodiscard]] std::uint32_t ace_num_bits(std::uint32_t value) noexcept;

[[nodiscard]] bool read_ace_unary(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t& value) noexcept;

[[nodiscard]] bool read_ace_unary_ones(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t& value) noexcept;

[[nodiscard]] bool read_ace_uniform(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t& value) noexcept;

[[nodiscard]] bool read_ace_golomb(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t parameter,
    std::uint32_t& value) noexcept;

[[nodiscard]] bool read_ace_golomb_limited(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t parameter,
    std::uint32_t limit,
    std::uint32_t& value) noexcept;

[[nodiscard]] bool read_ace_vlc(
    AceBitReader& source,
    const std::uint8_t* primary,
    std::size_t primary_size,
    const std::uint8_t* escape,
    std::size_t escape_size,
    std::uint32_t& value) noexcept;

[[nodiscard]] bool read_ace_limits_vlc(
    AceBitReader& source,
    const std::uint8_t* widths,
    std::size_t width_count,
    std::uint32_t& value) noexcept;

[[nodiscard]] bool read_ace_non_uniform_five_ten(
    AceBitReader& source,
    std::uint32_t& value) noexcept;

} // namespace dtsx
