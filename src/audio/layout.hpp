#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace dtsx_decode {

struct ChannelLayout {
    std::string name;
    std::string ffmpeg_name;
    std::vector<std::string> channels;
    std::uint32_t wave_mask = 0;
};

std::optional<ChannelLayout> find_layout(const std::string& name);
std::optional<ChannelLayout> find_layout_by_ffmpeg_name(const std::string& name);
std::string supported_layouts_text();
std::string join_channel_names(const ChannelLayout& layout);
ChannelLayout dolby_ordered_layout(const ChannelLayout& layout);
std::uint32_t wave_mask_for_channel(std::string_view channel) noexcept;

} // namespace dtsx_decode
