#include "dtsx/ace_coarse_residual.hpp"

#include <array>

namespace dtsx {
namespace {

struct GolombCode final {
    std::uint32_t parameter = 0U;
    std::uint32_t limit = 0U;
};

constexpr GolombCode kA{0U, 15U};
constexpr GolombCode kB{1U, 31U};
constexpr GolombCode kC{2U, 31U};
constexpr GolombCode kD{3U, 31U};
constexpr std::array<GolombCode, 12> kLowBandCodes{{
    kB, kA, kA, kB, kA, kA, kC, kB, kA, kD, kA, kA,
}};

[[nodiscard]] bool valid(const AceCoarseResidualControl& control) noexcept {
    return control.channel_count != 0U
        && control.first_channel < 2U
        && control.channel_count <= 2U - control.first_channel
        && control.band_count <= 22U
        && control.selector_a < 2U && control.selector_b < 2U;
}

} // namespace

std::int32_t ace_integer_map_invmap_pos(
    const std::uint32_t value) noexcept {
    const std::uint32_t magnitude = (value + 1U) >> 1U;
    return (value & 1U) != 0U
        ? static_cast<std::int32_t>(magnitude)
        : -static_cast<std::int32_t>(value >> 1U);
}

std::int32_t ace_integer_map_invmap_neg(
    const std::uint32_t value) noexcept {
    return (value & 1U) != 0U
        ? -static_cast<std::int32_t>((value + 1U) >> 1U)
        : static_cast<std::int32_t>(value >> 1U);
}

bool unpack_ace_coarse_residuals(
    AceBitReader& source,
    const AceCoarseResidualControl& control,
    AceScalarMatrix& residuals,
    std::uint32_t& initial_code,
    std::size_t& bits_consumed) noexcept {
    initial_code = 0U;
    bits_consumed = 0U;
    if (!valid(control)) {
        return false;
    }
    const std::size_t begin = source.position();
    if (!read_ace_golomb_limited(source, 8U, 0U, 7U, initial_code)) {
        return false;
    }
    for (std::uint32_t band = 0U; band < control.band_count; ++band) {
        const GolombCode code = band >= 3U ? kA : kLowBandCodes[
            6U * control.selector_a + 3U * control.selector_b + band];
        for (std::uint32_t channel = control.first_channel;
             channel < control.first_channel + control.channel_count;
             ++channel) {
            std::uint32_t raw = 0U;
            if (!read_ace_golomb_limited(
                    source, 64U, code.parameter + initial_code,
                    code.limit, raw)) {
                return false;
            }
            residuals[channel][band] = ace_integer_map_invmap_pos(raw);
        }
    }
    bits_consumed = source.position() - begin;
    return true;
}

} // namespace dtsx
