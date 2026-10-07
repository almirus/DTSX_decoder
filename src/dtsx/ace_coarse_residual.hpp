#pragma once

#include "dtsx/ace_bit_reader.hpp"
#include "dtsx/ace_scalar_dequant.hpp"

#include <cstddef>
#include <cstdint>

namespace dtsx {

struct AceCoarseResidualControl final {
    std::uint32_t first_channel = 0U;
    std::uint32_t channel_count = 0U;
    std::uint32_t band_count = 0U;
    std::uint32_t selector_a = 0U;
    std::uint32_t selector_b = 0U;
};

// Native dtsAce_IntegerMapInvmapPos from the homatic/sony ACE decoder.
// The decoded Golomb symbol is mapped to a signed residual before it is
// written to the per-channel/per-band scalar matrix.
[[nodiscard]] std::int32_t ace_integer_map_invmap_pos(
    std::uint32_t value) noexcept;

// Native dtsAce_IntegerMapInvmapNeg used by signed-mapped Golomb readers.
[[nodiscard]] std::int32_t ace_integer_map_invmap_neg(
    std::uint32_t value) noexcept;

// Native counterpart: DTSAceBitStreamUnpacker_UnpackCoarseResiduals and
// DTSAceBitStreamDecoder_GolombDecode. Output is pre-ScalarDequant.
[[nodiscard]] bool unpack_ace_coarse_residuals(
    AceBitReader& source,
    const AceCoarseResidualControl& control,
    AceScalarMatrix& residuals,
    std::uint32_t& initial_code,
    std::size_t& bits_consumed) noexcept;

} // namespace dtsx
