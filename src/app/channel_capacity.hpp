#pragma once

#include "audio/layout.hpp"
#include "dtsx/speaker_mask.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace dtsx_decode {

inline void add_activity_speakers(
    std::uint32_t activity_mask,
    std::uint32_t& physical_mask) {
    for (const std::uint32_t speaker :
         dtsx::expand_speaker_activity_mask(activity_mask)) {
        physical_mask |= speaker;
    }
}

inline bool layout_speaker_is_available(
    const std::string& name,
    std::uint32_t physical_mask) {
    std::uint32_t speaker = 0U;
    if (!dtsx::standard_speaker_mask(name, speaker)) {
        return false;
    }
    if ((physical_mask & speaker) != 0U) {
        return true;
    }
    if (name == "SL") {
        return (physical_mask & (1U << 9U)) != 0U;
    }
    if (name == "SR") {
        return (physical_mask & (1U << 10U)) != 0U;
    }
    return false;
}

inline std::vector<std::string> missing_layout_channels(
    const ChannelLayout& layout,
    std::uint32_t physical_mask) {
    std::vector<std::string> missing;
    const bool has_side_pair =
        std::find(
            layout.channels.begin(),
            layout.channels.end(),
            "SL") != layout.channels.end();
    for (const std::string& channel : layout.channels) {
        bool available =
            layout_speaker_is_available(channel, physical_mask);
        if (!has_side_pair && channel == "BL") {
            available = available
                || (physical_mask & (1U << 3U)) != 0U;
        } else if (!has_side_pair && channel == "BR") {
            available = available
                || (physical_mask & (1U << 4U)) != 0U;
        }
        if (!available) {
            missing.push_back(channel);
        }
    }
    return missing;
}

inline std::string channel_capacity_warning(
    const ChannelLayout& layout,
    std::uint32_t physical_mask,
    bool dynamic_objects_available) {
    if (physical_mask == 0U || dynamic_objects_available) {
        return {};
    }
    const std::vector<std::string> missing =
        missing_layout_channels(layout, physical_mask);
    if (missing.empty()) {
        return {};
    }
    std::string warning =
        "Warning: stream provides "
        + std::to_string(layout.channels.size() - missing.size())
        + "/"
        + std::to_string(layout.channels.size())
        + " requested channels; missing: ";
    for (std::size_t index = 0U; index < missing.size(); ++index) {
        if (index != 0U) {
            warning += ", ";
        }
        warning += missing[index];
    }
    warning += '.';
    return warning;
}

} // namespace dtsx_decode
