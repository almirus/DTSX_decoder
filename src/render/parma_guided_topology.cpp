#include "render/parma_guided_topology.hpp"

#include "render/parma_blind_config.hpp"
#include "render/parma_layout.hpp"

#include <cmath>

namespace dtsx_decode {
namespace {

std::uint8_t nth_enabled_main_slot(
    std::uint32_t channel_mask,
    std::uint32_t ordinal) noexcept {
    const std::uint32_t main_mask =
        parma_disable_lfe_channels(channel_mask);
    for (std::uint32_t slot = 0U; slot < 32U; ++slot) {
        if ((main_mask & (1U << slot)) == 0U) {
            continue;
        }
        if (ordinal-- == 0U) {
            return static_cast<std::uint8_t>(slot);
        }
    }
    return kParmaUnusedChannel;
}

bool equal_coefficients(
    const std::array<float, 4U>& coefficients,
    std::uint32_t count) noexcept {
    if (count < 3U || count > coefficients.size()) {
        return false;
    }
    // SetGuided uses exact float equality, not an epsilon comparison.
    for (std::uint32_t index = 1U; index < count; ++index) {
        if (coefficients[index] != coefficients[0U]) {
            return false;
        }
    }
    return true;
}

template <std::size_t SourceCount>
std::uint32_t unique_group_count(
    const ParmaGuidedTopology& topology,
    std::uint8_t mode) noexcept {
    std::uint32_t count = 0U;
    for (std::uint32_t node = 0U;
         node < topology.node_count;
         ++node) {
        if (topology.nodes[node].mode != mode) {
            continue;
        }
        bool already_seen = false;
        for (std::uint32_t previous = 0U;
             previous < node;
             ++previous) {
            if (topology.nodes[previous].mode != mode) {
                continue;
            }
            already_seen = true;
            for (std::size_t source = 0U;
                 source < SourceCount;
                 ++source) {
                already_seen = already_seen
                    && topology.nodes[previous].sources[source]
                        == topology.nodes[node].sources[source];
            }
            if (already_seen) {
                break;
            }
        }
        if (!already_seen) {
            ++count;
        }
    }
    return count;
}

} // namespace

bool build_parma_guided_topology(
    const ParmaGuidedControls& controls,
    ParmaGuidedTopology& topology) noexcept {
    topology = {};
    if (controls.downmix_main_channel_count == 0U
        || controls.downmix_main_channel_count > 11U
        || controls.upmix_main_channel_count
            < controls.downmix_main_channel_count
        || controls.upmix_main_channel_count > 11U
        || controls.added_output_count != controls.upmix_main_channel_count
            - controls.downmix_main_channel_count) {
        return false;
    }

    // SetGuided first enters all direct input-layout outputs.  Their source
    // channel is the identical enabled PARMA slot.
    for (std::uint32_t ordinal = 0U;
         ordinal < controls.downmix_main_channel_count;
         ++ordinal) {
        const std::uint8_t slot = nth_enabled_main_slot(
            controls.downmix_channel_mask, ordinal);
        if (slot == kParmaUnusedChannel || topology.node_count >= 11U) {
            topology = {};
            return false;
        }
        ParmaGuidedNode& node = topology.nodes[topology.node_count++];
        node.mode = 0U;
        node.target = slot;
        node.sources[0U] = slot;
    }

    for (std::uint32_t row = 0U;
         row < controls.added_output_count;
         ++row) {
        const std::uint8_t target = nth_enabled_main_slot(
            parma_disable_lfe_channels(controls.upmix_channel_mask)
                & ~parma_disable_lfe_channels(controls.downmix_channel_mask),
            row);
        if (target == kParmaUnusedChannel || topology.node_count >= 11U) {
            topology = {};
            return false;
        }
        ParmaGuidedNode& node = topology.nodes[topology.node_count++];
        node.target = target;
        std::array<float, 4U> source_coefficients{};
        std::uint32_t source_count = 0U;
        for (std::uint32_t column = 0U;
             column < controls.downmix_main_channel_count;
             ++column) {
            const float coefficient =
                controls.custom_encoder_coefficients[row][column];
            if (coefficient <= 0.0F) {
                continue;
            }
            if (source_count >= node.sources.size()) {
                topology = {};
                return false;
            }
            const std::uint8_t source = nth_enabled_main_slot(
                controls.downmix_channel_mask, column);
            if (source == kParmaUnusedChannel) {
                topology = {};
                return false;
            }
            node.sources[source_count] = source;
            source_coefficients[source_count++] = coefficient;
        }

        // DTS_ParmaDec_SetGuided accepts only pair, equal-coefficient triplet
        // or equal-coefficient quadruplet reconstruction groups.  A one-source
        // added channel is not a legal guided upmix group.
        if (source_count == 2U) {
            node.mode = 1U;
            const float first = source_coefficients[0U];
            const float second = source_coefficients[1U];
            // DTS_ParmaDec_SetGuided: acosf(sqrt(a^2 / (a^2 + b^2)))
            // normalized by pi/2.  The outer acos is essential: equal
            // coefficients produce 0.5, the native pairwise midpoint.
            node.decoding_angle = std::acos(std::sqrt(
                (first * first)
                / (first * first + second * second + 1.0e-12F)))
                * 0.63662F;
        } else if (source_count == 3U
                   && equal_coefficients(source_coefficients, source_count)) {
            node.mode = 2U;
            node.decoding_angle = 0.5F;
        } else if (source_count == 4U
                   && equal_coefficients(source_coefficients, source_count)) {
            node.mode = 3U;
            node.decoding_angle = 0.5F;
        } else {
            topology = {};
            return false;
        }
    }
    if (topology.node_count != controls.upmix_main_channel_count) {
        topology = {};
        return false;
    }

    // UpdateChannelInfo stores fixed native group tables: two quadruplets,
    // four triplets and sixteen pairs per layer. SetControls can accept a
    // larger matrix, but DTS_ParmaDec_Process then returns -10012.
    if (unique_group_count<4U>(topology, 3U) > 2U
        || unique_group_count<3U>(topology, 2U) > 4U
        || unique_group_count<2U>(topology, 1U) > 16U) {
        topology = {};
        return false;
    }
    return true;
}

} // namespace dtsx_decode
