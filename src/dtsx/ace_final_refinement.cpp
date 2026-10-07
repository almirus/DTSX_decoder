#include "dtsx/ace_final_refinement.hpp"

#include "dtsx/ace_bit_reader.hpp"

#include <algorithm>

namespace dtsx {

bool unpack_ace_final_refinement(
    const std::uint8_t* bytes,
    std::size_t size,
    const AceStreamPrefix& prefix,
    std::size_t bit_offset,
    const std::array<std::uint32_t, 22>& initial_allocation,
    AceFinalRefinement& output) noexcept {
    output = {};
    if (bytes == nullptr || prefix.coding_mode == AceCodingMode::Off
        || prefix.effective_channel_count == 0U
        || prefix.first_effective_channel >= 2U
        || prefix.effective_channel_count > 2U - prefix.first_effective_channel
        || prefix.effective_bands > 22U || bit_offset > size * 8U) {
        return false;
    }
    AceBitReader source(bytes, size);
    if (!source.seek(bit_offset)) {
        return false;
    }
    const std::uint32_t channels = prefix.effective_channel_count;
    const std::size_t remaining = source.bits_left();
    // Native: `(remaining / channels) * channels`.  It deliberately leaves
    // a trailing non-divisible pad untouched.
    std::size_t budget = remaining - (remaining % channels);
    for (std::uint32_t band = 0U; band < prefix.effective_bands; ++band) {
        const std::uint32_t fine = prefix.refinement_allocation[band];
        const std::uint32_t initial = initial_allocation[band];
        if (fine > 8U || initial > 8U) {
            return false;
        }
        if (fine < 8U) {
            output.allocation[band] = initial;
            const std::size_t cost = static_cast<std::size_t>(channels)
                * initial;
            if (cost > budget) {
                return false;
            }
            budget -= cost;
        }
    }

    // Native outer loop first assigns a bit only to bands with no initial
    // tail allocation, then repeatedly cycles all eligible bands.  It stops
    // when a complete pass cannot reduce the budget.
    std::size_t previous_budget = 0U;
    for (std::uint32_t pass = 0U; budget != 0U; ++pass) {
        if (pass >= 2U && previous_budget == budget) {
            break;
        }
        previous_budget = budget;
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands && budget != 0U;
             ++band) {
            const std::uint32_t fine = prefix.refinement_allocation[band];
            const std::uint32_t initial = initial_allocation[band];
            if (fine > 8U
                || output.allocation[band] + fine >= 8U
                || (pass == 0U && initial != 0U)) {
                continue;
            }
            ++output.allocation[band];
            budget -= channels;
        }
    }

    for (std::uint32_t band = 0U; band < prefix.effective_bands; ++band) {
        const std::uint32_t width = output.allocation[band];
        if (width > 8U) {
            return false;
        }
        for (std::uint32_t channel = prefix.first_effective_channel;
             channel < prefix.first_effective_channel
                 + prefix.effective_channel_count;
             ++channel) {
            std::uint32_t code = 0U;
            if (!source.read(width, code)) {
                return false;
            }
            output.codes[channel][band] =
                static_cast<std::uint16_t>(code);
        }
    }
    output.bits_consumed = source.position() - bit_offset;
    output.final_bit_offset = source.position();
    return true;
}

} // namespace dtsx
