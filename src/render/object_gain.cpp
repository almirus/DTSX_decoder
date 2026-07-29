#include "render/object_gain.hpp"

#include <algorithm>
#include <array>

namespace dtsx_decode {
namespace {

constexpr std::array<std::int32_t, 61> kScaleCoefficientTable = {
    0x0000, 0x0029, 0x0034, 0x0041, 0x0052, 0x0068, 0x0082,
    0x00A4, 0x00CF, 0x0104, 0x0148, 0x019D, 0x0207, 0x028E,
    0x0337, 0x040C, 0x048B, 0x0519, 0x05B8, 0x066A, 0x0733,
    0x0814, 0x0910, 0x0A2B, 0x0B68, 0x0CCD, 0x0E5D, 0x101D,
    0x1215, 0x1449, 0x16C3, 0x181C, 0x198A, 0x1B0D, 0x1CA8,
    0x1E5B, 0x2027, 0x220F, 0x2413, 0x2637, 0x287A, 0x2AE0,
    0x2D6B, 0x301B, 0x32F5, 0x35FA, 0x392D, 0x3C90, 0x4027,
    0x43F4, 0x47FB, 0x4C3F, 0x50C3, 0x558C, 0x5A82, 0x5FFD,
    0x65AD, 0x6BB3, 0x7215, 0x78D7, 0x8000,
};

constexpr std::array<std::int32_t, 100> kDownmixCoefficientTable = {
    0, 35, 37, 39, 41, 44, 46, 49, 52, 55,
    58, 62, 65, 69, 73, 78, 82, 87, 92, 98,
    104, 110, 116, 123, 130, 138, 146, 155, 164, 174,
    184, 195, 207, 219, 232, 246, 260, 276, 292, 309,
    328, 347, 368, 389, 413, 437, 463, 490, 519, 550,
    583, 617, 654, 693, 734, 777, 823, 872, 924, 978,
    1036, 1066, 1098, 1130, 1163, 1197, 1232, 1268, 1305, 1343,
    1382, 1422, 1464, 1506, 1550, 1596, 1642, 1690, 1740, 1790,
    1843, 1896, 1952, 2009, 2068, 2128, 2190, 2254, 2320, 2388,
    2457, 2529, 2603, 2679, 2757, 2838, 2920, 3006, 3093, 3184,
};

} // namespace

std::int32_t decode_object_presentation_gain_q23(
    std::uint8_t gain_code) noexcept {
    // libdtsx.so: dtsLookUpScaleCoeffTable(code, 1), 0x9e6c8,
    // followed by the Q15 x Q23 products in sub_5F12C,
    // sub_5F810, sub_5FE60, and sub_60444.
    std::size_t table_index = 0U;
    if (gain_code > 1U) {
        table_index = std::min<std::size_t>(
            static_cast<std::size_t>(gain_code - 1U),
            kScaleCoefficientTable.size() - 1U);
    }
    return kScaleCoefficientTable[table_index] << 8U;
}

std::int32_t decode_object_point_gain_q23(
    std::uint8_t gain_code,
    std::int32_t object_gain_q15,
    std::int32_t presentation_gain_q23) noexcept {
    // libdtsx.so: dtsPlayerObjectRenderer_SetupObjectRenderer,
    // 0x5f3a0, point/extended-source gain calculation.
    if (gain_code == 0U || gain_code > kScaleCoefficientTable.size()) {
        return 0;
    }
    const std::int64_t object_point_gain =
        (static_cast<std::int64_t>(object_gain_q15)
             * kScaleCoefficientTable[gain_code - 1U]
         + 0x4000LL)
        >> 15U;
    return static_cast<std::int32_t>(
        (static_cast<std::int64_t>(presentation_gain_q23)
             * object_point_gain
         + 0x4000LL)
        >> 15U);
}

std::int32_t decode_object_source_gain_q23(
    std::uint8_t object_gain_code,
    std::uint8_t object_gain_exponent,
    std::uint8_t point_gain_code,
    std::int32_t presentation_gain_q23) noexcept {
    // libdtsx.so: dtsParseExSSChunks object gain at 0xa1dc0 and
    // dtsPlayerObjectRenderer_SetupObjectRenderer at 0x5f3a0.
    if (object_gain_code == 0U
        || object_gain_code > kScaleCoefficientTable.size()
        || object_gain_exponent > 3U) {
        return 0;
    }
    const std::int32_t object_gain_q15 =
        kScaleCoefficientTable[object_gain_code - 1U]
        << object_gain_exponent;
    return decode_object_point_gain_q23(
        point_gain_code,
        object_gain_q15,
        presentation_gain_q23);
}

std::int32_t decode_object_destination_gain_q23(
    std::uint8_t gain_code,
    std::int32_t presentation_gain_q23) noexcept {
    // libdtsx.so: sub_5F810/sub_5FE60, 0x5f810/0x5fe60.
    // Codes zero and one select entry zero. Every larger code selects
    // every fourth dtsx_dmixCoeffTable entry starting at entry four.
    const std::size_t table_index = gain_code > 1U
        ? 4U * static_cast<std::size_t>(gain_code - 1U)
        : 0U;
    if (table_index >= kDownmixCoefficientTable.size()) {
        return 0;
    }
    return static_cast<std::int32_t>(
        (static_cast<std::int64_t>(presentation_gain_q23)
             * kDownmixCoefficientTable[table_index]
         + 0x4000LL)
        >> 15U);
}

} // namespace dtsx_decode
