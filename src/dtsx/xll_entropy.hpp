#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace dtsx {

struct XllMsbCoding final {
    bool rice_coding = false;
    std::uint8_t escape_width = 0;
    std::uint8_t initial_parameter = 0;
    std::uint32_t initial_sample_count = 0;
    std::uint8_t rice_parameter = 0;
};

struct XllDecimatorHistory final {
    std::uint8_t bit_width = 0;
    std::vector<std::array<std::int32_t, 7>> channels;
};

[[nodiscard]] bool unpack_xll_msb_binary(
    bitstream::Cursor& source,
    std::uint32_t sample_count,
    std::uint8_t bit_width,
    std::vector<std::int32_t>& values);

[[nodiscard]] bool unpack_xll_msb_rice(
    bitstream::Cursor& source,
    const std::vector<std::uint8_t>& escape_flags,
    std::uint8_t escape_width,
    std::vector<std::int32_t>& values);

[[nodiscard]] bool unpack_xll_msb_rice_binary(
    bitstream::Cursor& source,
    const std::vector<std::uint8_t>& escape_flags,
    std::uint8_t rice_bits,
    std::uint8_t escape_width,
    std::vector<std::int32_t>& values);

[[nodiscard]] bool unpack_xll_lsb_core(
    bitstream::Cursor& source,
    std::uint32_t sample_count,
    std::uint8_t bit_width,
    std::vector<std::uint32_t>& values);

[[nodiscard]] bool unpack_xll_msb(
    bitstream::Cursor& source,
    std::uint32_t sample_count,
    const XllMsbCoding& coding,
    std::vector<std::int32_t>& values);

[[nodiscard]] bool combine_xll_msb_lsb(
    const std::vector<std::int32_t>& msb,
    const std::vector<std::uint32_t>& lsb,
    std::uint8_t msb_shift,
    std::uint8_t lsb_shift,
    std::vector<std::int32_t>& samples);

[[nodiscard]] bool unpack_xll_decimator_history(
    bitstream::Cursor& source,
    std::uint32_t segment_index,
    std::uint8_t mode,
    std::uint8_t channel_count,
    std::uint8_t first_channel,
    XllDecimatorHistory& history);

} // namespace dtsx
