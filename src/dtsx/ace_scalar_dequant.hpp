#pragma once

#include <array>
#include <cstdint>

namespace dtsx {

// Native DTSAceScalarDequant control subset used by mono/stereo ACE streams.
// Values are Q10 and the channel axis is always the native two-channel
// stream-local axis, never a physical speaker index.
struct AceScalarDequantControl final {
    bool predictive = false;
    std::uint32_t first_channel = 0U;
    std::uint32_t channel_count = 0U;
    std::uint32_t band_count = 0U;
};

// Native dts_flib_math_pow2_i32(a1, a2, a3), used by ACE UnNormalize.
// Inputs/outputs are signed fixed-point values; the implementation retains
// the native 256-entry interpolation table and saturation branches.
[[nodiscard]] std::int32_t ace_pow2_i32_native(
    std::int32_t value,
    std::uint8_t input_fraction,
    std::int32_t output_fraction) noexcept;

using AceScalarMatrix = std::array<std::array<std::int32_t, 22>, 2>;
using AceScalarRefinementCodes =
    std::array<std::array<std::uint16_t, 22>, 2>;

// Port of DTSAceScalarDequant_CoarseDecode. `previous` is the persistent
// stream-local dequant output from the preceding ACE frame.
[[nodiscard]] bool ace_scalar_dequant_coarse(
    const AceScalarMatrix& residual,
    const AceScalarMatrix* previous,
    const AceScalarDequantControl& control,
    AceScalarMatrix& output) noexcept;

// Port of DTSAceScalarDequant_FineDecode for the signalled log-normal
// refinement codes. It updates the Q10 coarse result in place.
[[nodiscard]] bool ace_scalar_dequant_fine(
    AceScalarMatrix& values,
    const AceScalarRefinementCodes& codes,
    const std::array<std::uint32_t, 22>& bit_allocation,
    const AceScalarDequantControl& control) noexcept;

// Port of DTSAceScalarDequant_Finalize. `fine_bit_allocation` is the already
// decoded refinement width and `final_bit_allocation` is assigned by the
// native UnpackFinalRefinement tail allocator. Both are per band and the
// result remains native Q10 lognorm, before band dequant/MDCT.
[[nodiscard]] bool ace_scalar_dequant_finalize(
    AceScalarMatrix& values,
    const AceScalarRefinementCodes& final_codes,
    const std::array<std::uint32_t, 22>& fine_bit_allocation,
    const std::array<std::uint32_t, 22>& final_bit_allocation,
    const AceScalarDequantControl& control) noexcept;

} // namespace dtsx
