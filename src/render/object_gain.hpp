#pragma once

#include <cstdint>

namespace dtsx_decode {

[[nodiscard]] std::int32_t decode_object_presentation_gain_q23(
    std::uint8_t gain_code) noexcept;

[[nodiscard]] std::int32_t
decode_object_alternative_presentation_gain_q23(
    std::uint8_t gain_code) noexcept;

[[nodiscard]] std::int32_t decode_object_point_gain_q23(
    std::uint8_t gain_code,
    std::int32_t object_gain_q15 = 0x8000,
    std::int32_t presentation_gain_q23 = 0x800000) noexcept;

[[nodiscard]] std::int32_t decode_object_source_gain_q23(
    std::uint8_t object_gain_code,
    std::uint8_t object_gain_exponent,
    std::uint8_t point_gain_code,
    std::int32_t presentation_gain_q23 = 0x800000) noexcept;

[[nodiscard]] std::int32_t decode_object_destination_gain_q23(
    std::uint8_t gain_code,
    std::int32_t presentation_gain_q23 = 0x800000) noexcept;

} // namespace dtsx_decode
