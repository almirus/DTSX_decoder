#include "layout.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace dtsx_decode {
namespace {

std::string lower_ascii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

const std::vector<ChannelLayout>& layouts() {
    static const std::vector<ChannelLayout> value = {
        {"mono", "mono", {"FC"}, 0x00000004u},
        {"stereo", "stereo", {"FL", "FR"}, 0x00000003u},
        {"5.1", "5.1", {"FL", "FR", "FC", "LFE", "BL", "BR"}, 0x0000003Fu},
        {"5.1(side)",
         "5.1(side)",
         {"FL", "FR", "FC", "LFE", "SL", "SR"},
         0x0000060Fu},
        {"7.1",
         "7.1",
         {"FL", "FR", "FC", "LFE", "BL", "BR", "SL", "SR"},
         0x0000063Fu},
        {"5.1.2",
         "5.1.2",
         {"FL", "FR", "FC", "LFE", "BL", "BR", "TFL", "TFR"},
         0x0000503Fu},
        {"5.1.4",
         "5.1.4",
         {"FL", "FR", "FC", "LFE", "BL", "BR", "TFL", "TFR", "TBL", "TBR"},
         0x0002D03Fu},
        {"7.1.2",
         "7.1.2",
         {"FL", "FR", "FC", "LFE", "BL", "BR", "SL", "SR", "TFL", "TFR"},
         0x0000563Fu},
        {"7.1.4",
         "7.1.4",
         {"FL", "FR", "FC", "LFE", "BL", "BR", "SL", "SR", "TFL", "TFR", "TBL", "TBR"},
         0x0002D63Fu},
    };
    return value;
}

} // namespace

std::optional<ChannelLayout> find_layout(const std::string& name) {
    std::string key = lower_ascii(name);
    if (key == "1.0") {
        key = "mono";
    } else if (key == "2.0") {
        key = "stereo";
    }
    for (const ChannelLayout& layout : layouts()) {
        if (lower_ascii(layout.name) == key) {
            return layout;
        }
    }
    return std::nullopt;
}

std::optional<ChannelLayout> find_layout_by_ffmpeg_name(const std::string& name) {
    const std::string key = lower_ascii(name);
    for (const ChannelLayout& layout : layouts()) {
        if (lower_ascii(layout.ffmpeg_name) == key) {
            return layout;
        }
    }
    return std::nullopt;
}

std::string supported_layouts_text() {
    std::ostringstream out;
    bool first = true;
    for (const ChannelLayout& layout : layouts()) {
        if (!first) {
            out << ", ";
        }
        first = false;
        out << layout.name;
    }
    return out.str();
}

std::string join_channel_names(const ChannelLayout& layout) {
    std::ostringstream out;
    for (std::size_t i = 0; i < layout.channels.size(); ++i) {
        if (i != 0) {
            out << ',';
        }
        out << layout.channels[i];
    }
    return out.str();
}

ChannelLayout dolby_ordered_layout(const ChannelLayout& layout) {
    if (layout.name != "7.1"
        && layout.name != "7.1.2"
        && layout.name != "7.1.4") {
        throw std::invalid_argument(
            "--dolby-output supports only 7.1, 7.1.2 and 7.1.4 layouts");
    }

    static constexpr std::array<std::string_view, 12U> kDolbyOrder = {{
        "FL", "FR", "FC", "LFE",
        "SL", "SR", "BL", "BR",
        "TFL", "TFR", "TBL", "TBR",
    }};
    ChannelLayout result = layout;
    result.channels.clear();
    result.channels.reserve(layout.channels.size());
    for (const std::string_view name : kDolbyOrder) {
        const auto found = std::find(
            layout.channels.begin(), layout.channels.end(), name);
        if (found != layout.channels.end()) {
            result.channels.push_back(*found);
        }
    }
    for (const std::string& name : layout.channels) {
        if (std::find(
                result.channels.begin(), result.channels.end(), name)
            == result.channels.end()) {
            result.channels.push_back(name);
        }
    }
    return result;
}

std::uint32_t wave_mask_for_channel(
    std::string_view channel) noexcept {
    static constexpr std::array<
        std::pair<std::string_view, std::uint32_t>, 12U>
        kWaveMasks = {{
            {"FL", 0x00000001U},
            {"FR", 0x00000002U},
            {"FC", 0x00000004U},
            {"LFE", 0x00000008U},
            {"BL", 0x00000010U},
            {"BR", 0x00000020U},
            {"SL", 0x00000200U},
            {"SR", 0x00000400U},
            {"TFL", 0x00001000U},
            {"TFR", 0x00004000U},
            {"TBL", 0x00008000U},
            {"TBR", 0x00020000U},
        }};
    for (const auto& entry : kWaveMasks) {
        if (entry.first == channel) {
            return entry.second;
        }
    }
    return 0U;
}

} // namespace dtsx_decode
