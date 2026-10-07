#pragma once

#include "dtsx/ace_stream.hpp"
#include "dtsx/ace_scalar_dequant.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dtsx {

struct AceFinalRefinement final {
    std::array<std::uint32_t, 22> allocation{};
    AceScalarRefinementCodes codes{};
    std::size_t bits_consumed = 0U;
    std::size_t final_bit_offset = 0U;
};

// Native counterpart: DTSAceBitStreamUnpacker_UnpackFinalRefinement.  The
// caller provides BandDequant a4 (v19+176): DWORD 0/1 leftover flags, not VQ
// bit allocation.  Sony writes 1 iff HR VQ is set, the band underspent its
// allocation, and (signalled - consumed + running balance) >= channel count.
[[nodiscard]] bool unpack_ace_final_refinement(
    const std::uint8_t* bytes,
    std::size_t size,
    const AceStreamPrefix& prefix,
    std::size_t bit_offset,
    const std::array<std::uint32_t, 22>& initial_allocation,
    AceFinalRefinement& output) noexcept;

} // namespace dtsx
