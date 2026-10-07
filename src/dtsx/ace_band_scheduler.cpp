#include "dtsx/ace_band_scheduler.hpp"

#include <algorithm>
#include <limits>

namespace dtsx {

std::int32_t ace_distribute_running_balance(
    std::int32_t balance,
    std::uint32_t remaining_coded_bands) noexcept {
    if (remaining_coded_bands == 0U) {
        return 0;
    }
    if (remaining_coded_bands < 5U) {
        return balance / static_cast<std::int32_t>(remaining_coded_bands);
    }
    const std::int64_t magnitude = balance < 0
        ? -static_cast<std::int64_t>(balance)
        : static_cast<std::int64_t>(balance);
    const std::int32_t distributed = static_cast<std::int32_t>(
        (7LL * magnitude) >> 5U);
    return balance < 0 ? -distributed : distributed;
}

bool ace_schedule_next_band(
    AceBandScheduleState& state,
    std::uint32_t signalled_allocation,
    std::uint32_t consumed_bits,
    AceBandScheduleEntry& entry) noexcept {
    entry = {};
    if (state.current_band >= state.coded_bands
        || state.used_bits > state.total_bits
        || state.deferred_reserve_bits > state.total_bits - state.used_bits) {
        return false;
    }
    const std::uint32_t remaining = state.total_bits - state.used_bits
        - state.deferred_reserve_bits;
    const std::int64_t desired = static_cast<std::int64_t>(
        signalled_allocation) + ace_distribute_running_balance(
            state.running_balance, state.coded_bands - state.current_band);
    const std::uint32_t allocation = desired <= 0 ? 0U
        : static_cast<std::uint32_t>(std::min<std::int64_t>(desired, remaining));
    // DTSAceBandDequant_Process accepts a codeword that consumes past its
    // provisional allocation: the following band receives the resulting
    // negative running balance.  Only the physical payload bound is fatal.
    if (consumed_bits > remaining) {
        return false;
    }
    const std::int64_t next_balance64 = static_cast<std::int64_t>(
        signalled_allocation) - static_cast<std::int64_t>(consumed_bits)
        + state.running_balance;
    if (next_balance64 < std::numeric_limits<std::int32_t>::min()
        || next_balance64 > std::numeric_limits<std::int32_t>::max()) {
        return false;
    }
    // Sony DTSAceBandDequant_Process: a4[band] and the reserved-bit subtract
    // use the same three conditions; there is no extra remaining-size gate
    // on the write.  Insufficient leftover fails on the next band via
    // `a1[28] < used + deferred`.
    const bool reserve_deferred = state.reserve_enabled
        && allocation > consumed_bits
        && next_balance64 >= static_cast<std::int64_t>(state.reserve_bits);
    state.running_balance = static_cast<std::int32_t>(next_balance64)
        - static_cast<std::int32_t>(reserve_deferred ? state.reserve_bits : 0U);
    state.deferred_reserve_bits += reserve_deferred ? state.reserve_bits : 0U;
    state.used_bits += consumed_bits;
    ++state.current_band;
    entry = {allocation, remaining, consumed_bits, state.running_balance,
             reserve_deferred};
    return true;
}

bool ace_schedule_band_allocations(
    const AceBandScheduleInput& input,
    std::vector<AceBandScheduleEntry>& schedule) noexcept {
    schedule.clear();
    if (input.signalled_allocation == nullptr || input.consumed_bits == nullptr
        || input.band_count == 0U || input.coded_bands > input.band_count
        || input.initial_used_bits > input.total_bits) {
        return false;
    }

    schedule.reserve(input.band_count);
    AceBandScheduleState state{
        input.total_bits, input.initial_used_bits, input.coded_bands, 0U,
        0U, input.initial_running_balance, input.reserve_bits,
        input.reserve_enabled};

    for (std::size_t band = 0U; band < input.band_count; ++band) {
        if (band >= input.coded_bands) {
            if (input.consumed_bits[band] != 0U) {
                return false;
            }
            schedule.push_back({});
            continue;
        }
        AceBandScheduleEntry entry{};
        if (!ace_schedule_next_band(
                state, input.signalled_allocation[band],
                input.consumed_bits[band], entry)) {
            return false;
        }
        schedule.push_back(entry);
    }
    return true;
}

std::array<std::uint32_t, 22> ace_band_dequant_tail_allocation(
    const std::vector<AceBandScheduleEntry>& schedule) noexcept {
    std::array<std::uint32_t, 22> tail{};
    const std::size_t count = (std::min)(schedule.size(), tail.size());
    for (std::size_t band = 0U; band < count; ++band) {
        // Native stores the compare as a DWORD 0/1, then UnpackFinalRefinement
        // copies it into v10[132] as the initial per-band refinement width.
        tail[band] = schedule[band].reserve_deferred ? 1U : 0U;
    }
    return tail;
}

} // namespace dtsx
