#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx {

// Exact control portion of DTSAceBandDequant_Process().  The native decoder
// assigns each coded band its signalled allocation plus a share of unused VQ
// bits from preceding bands.  This state is deliberately independent from
// coefficient reconstruction so the ACE payload reader can use the original
// scheduling rules before PCM synthesis is attached.
struct AceBandScheduleInput final {
    std::uint32_t total_bits = 0U;
    std::uint32_t initial_used_bits = 0U;
    std::uint32_t coded_bands = 0U;
    std::int32_t initial_running_balance = 0;
    std::uint32_t reserve_bits = 0U;
    bool reserve_enabled = false;
    const std::uint32_t* signalled_allocation = nullptr;
    const std::uint32_t* consumed_bits = nullptr;
    std::size_t band_count = 0U;
};

struct AceBandScheduleEntry final {
    std::uint32_t allocation = 0U;
    std::uint32_t remaining_bits_before = 0U;
    std::uint32_t consumed_bits = 0U;
    std::int32_t running_balance_after = 0;
    bool reserve_deferred = false;
};

struct AceBandScheduleState final {
    std::uint32_t total_bits = 0U;
    std::uint32_t used_bits = 0U;
    std::uint32_t coded_bands = 0U;
    std::uint32_t current_band = 0U;
    std::uint32_t deferred_reserve_bits = 0U;
    std::int32_t running_balance = 0;
    std::uint32_t reserve_bits = 0U;
    bool reserve_enabled = false;
};

// Native counterpart: DTSAceBandDequant_DistributeRunningBalance().
[[nodiscard]] std::int32_t ace_distribute_running_balance(
    std::int32_t balance,
    std::uint32_t remaining_coded_bands) noexcept;

// One native loop iteration. `signalled_allocation` is the allocation from
// BitAllocation, and `consumed_bits` is returned by the VQ band reader.
[[nodiscard]] bool ace_schedule_next_band(
    AceBandScheduleState& state,
    std::uint32_t signalled_allocation,
    std::uint32_t consumed_bits,
    AceBandScheduleEntry& entry) noexcept;

// Native counterpart: the running-allocation loop in
// DTSAceBandDequant_Process(). `consumed_bits` is supplied by the completed
// mono/stereo VQ band reader; using it here preserves native next-band budget
// decisions without inventing a bitrate allocation heuristic.
[[nodiscard]] bool ace_schedule_band_allocations(
    const AceBandScheduleInput& input,
    std::vector<AceBandScheduleEntry>& schedule) noexcept;

// DTSAceBandDequant_Process writes a4[band] as the DWORD boolean
// `allocation > consumed && a1[20] && (signalled - consumed + balance) >= a11`.
// SetControlParams copies v19+1332: a1[20] is high-resolution VQ (dword 334);
// Process a11 is v64 = effective_channel_count.  UnpackFinalRefinement reads
// that array as v10[44] (v19+176), not the VQ bit-allocation vector.
[[nodiscard]] std::array<std::uint32_t, 22> ace_band_dequant_tail_allocation(
    const std::vector<AceBandScheduleEntry>& schedule) noexcept;

} // namespace dtsx
