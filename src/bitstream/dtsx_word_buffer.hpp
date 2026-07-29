#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace dtsx::bitstream {

enum class InputPacking {
    Core16Bit,
    Core14Bit,
    Extension32Bit,
};

struct BitstreamInit final {
    InputPacking packing = InputPacking::Extension32Bit;
    std::uint32_t bits_per_word = 0;
    bool swap_byte_pairs = false;
};

[[nodiscard]] std::optional<BitstreamInit> initialize_bitstream(
    const std::vector<std::uint8_t>& bytes) noexcept;

class WordBuffer final {
public:
    WordBuffer(const std::vector<std::uint8_t>& bytes, bool swap_byte_pairs);

    [[nodiscard]] Cursor cursor() const noexcept;
    [[nodiscard]] std::uint32_t bit_count() const noexcept;

private:
    std::vector<std::uint32_t> words_;
    std::uint32_t bit_count_ = 0;
};

} // namespace dtsx::bitstream
