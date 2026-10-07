#include "dtsx/ace_frame.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <utility>

namespace dtsx {
namespace {

class AceBitReader final {
public:
    AceBitReader(const std::uint8_t* bytes, std::size_t size) noexcept
        : bytes_(bytes), bit_limit_(size * 8U) {}

    [[nodiscard]] bool read(
        std::uint32_t count,
        std::uint32_t& value) noexcept {
        if (count > 32U || position_ > bit_limit_
            || count > bit_limit_ - position_) {
            valid_ = false;
            value = 0U;
            return false;
        }
        value = 0U;
        for (std::uint32_t bit = 0U; bit < count; ++bit) {
            value = (value << 1U)
                | ((bytes_[position_ >> 3U]
                    >> (7U - static_cast<unsigned>(position_ & 7U)))
                   & 1U);
            ++position_;
        }
        return true;
    }

    [[nodiscard]] bool align() noexcept {
        const std::size_t aligned = (position_ + 7U) & ~std::size_t{7U};
        if (aligned > bit_limit_) {
            valid_ = false;
            return false;
        }
        position_ = aligned;
        return true;
    }

    [[nodiscard]] std::size_t position() const noexcept {
        return position_;
    }

    [[nodiscard]] bool valid() const noexcept {
        return valid_;
    }

private:
    const std::uint8_t* bytes_ = nullptr;
    std::size_t bit_limit_ = 0U;
    std::size_t position_ = 0U;
    bool valid_ = true;
};

bool read_unary(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (alphabet <= 1U) {
        return true;
    }
    while (value + 1U < alphabet) {
        std::uint32_t bit = 0U;
        if (!source.read(1U, bit)) {
            return false;
        }
        if (bit != 0U) {
            break;
        }
        ++value;
    }
    return true;
}

bool read_unary_ones(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (alphabet <= 1U) {
        return true;
    }
    while (value + 1U < alphabet) {
        std::uint32_t bit = 0U;
        if (!source.read(1U, bit)) {
            return false;
        }
        if (bit == 0U) {
            break;
        }
        ++value;
    }
    return true;
}

bool read_limits_vlc(
    AceBitReader& source,
    std::uint32_t& value) noexcept {
    static constexpr std::array<std::uint8_t, 8> kWidths{
        1U, 1U, 2U, 2U, 2U, 4U, 4U, 16U};
    std::uint64_t total = 0U;
    for (const std::uint8_t width : kWidths) {
        std::uint32_t part = 0U;
        if (!source.read(width, part)) {
            return false;
        }
        total += part;
        if (total > std::numeric_limits<std::uint32_t>::max()) {
            return false;
        }
        if (part + 1U < (1U << width)) {
            value = static_cast<std::uint32_t>(total);
            return true;
        }
    }
    value = static_cast<std::uint32_t>(total);
    return true;
}

template <std::size_t PrimarySize, std::size_t EscapeSize>
bool read_vlc(
    AceBitReader& source,
    const std::array<std::uint8_t, PrimarySize>& primary,
    const std::array<std::uint8_t, EscapeSize>& escape,
    std::uint32_t& value) noexcept {
    std::uint32_t primary_index = 0U;
    if (!read_unary(source, PrimarySize, primary_index)
        || primary_index >= PrimarySize) {
        return false;
    }
    std::uint64_t primary_base = 0U;
    for (std::uint32_t index = 0U;
         index < primary_index;
         ++index) {
        primary_base += std::uint64_t{1U} << primary[index];
    }
    std::uint32_t part = 0U;
    if (!source.read(primary[primary_index], part)) {
        return false;
    }
    std::uint64_t result = primary_base + part;
    std::uint64_t primary_alpha = primary_base
        + (std::uint64_t{1U} << primary[primary_index]);
    for (std::size_t index = primary_index + 1U;
         index < PrimarySize;
         ++index) {
        primary_alpha += std::uint64_t{1U} << primary[index];
    }
    if constexpr (EscapeSize != 0U) {
        if (result + EscapeSize >= primary_alpha) {
            const std::uint64_t escape_index =
                result + EscapeSize - primary_alpha;
            if (escape_index >= EscapeSize) {
                return false;
            }
            std::uint64_t escape_base = 0U;
            for (std::size_t index = 0U;
                 index < escape_index;
                 ++index) {
                escape_base += std::uint64_t{1U} << escape[index];
            }
            if (!source.read(escape[escape_index], part)) {
                return false;
            }
            result = primary_alpha + escape_base + part;
        }
    }
    if (result > std::numeric_limits<std::uint32_t>::max()) {
        return false;
    }
    value = static_cast<std::uint32_t>(result);
    return true;
}

bool read_native_vlc(
    AceBitReader& source,
    std::uint32_t mode,
    std::uint32_t& value) noexcept {
    switch (mode) {
    case 0U:
        return read_vlc(
            source,
            std::array<std::uint8_t, 3>{0U, 2U, 4U},
            std::array<std::uint8_t, 2>{8U, 16U},
            value);
    case 1U:
        return read_vlc(
            source,
            std::array<std::uint8_t, 3>{0U, 4U, 10U},
            std::array<std::uint8_t, 2>{26U, 42U},
            value);
    case 2U:
        return read_vlc(
            source,
            std::array<std::uint8_t, 4>{2U, 7U, 9U, 13U},
            std::array<std::uint8_t, 2>{24U, 32U},
            value);
    default:
        return false;
    }
}

bool read_non_uniform_five_ten(
    AceBitReader& source,
    std::uint32_t& value) noexcept {
    if (!source.read(5U, value)) {
        return false;
    }
    if (value > 23U) {
        std::uint32_t tail = 0U;
        if (!source.read(5U, tail)) {
            return false;
        }
        value = ((value - 24U) << 5U) + tail;
    }
    if (value == 279U) {
        std::uint32_t tail = 0U;
        if (!source.read(22U, tail)) {
            return false;
        }
        value += tail;
    }
    return true;
}

const AceStreamSetHeader* find_previous(
    const AceFrameParserState& state,
    std::uint32_t id) noexcept {
    const auto found = std::find_if(
        state.stream_sets.begin(),
        state.stream_sets.end(),
        [id](const AceStreamSetHeader& stream_set) {
            return stream_set.id == id;
        });
    return found != state.stream_sets.end() ? &*found : nullptr;
}

bool read_stream_composition(
    AceBitReader& source,
    AceStreamSetHeader& stream_set) noexcept {
    std::uint32_t code = 0U;
    if (!read_limits_vlc(source, code)) {
        return false;
    }
    std::uint32_t lfe_count = 0U;
    std::uint32_t mono_count = 0U;
    std::uint32_t stereo_count = 0U;
    switch (code) {
    case 0U: mono_count = 1U; break;
    case 1U: stereo_count = 1U; break;
    case 2U: lfe_count = 1U; mono_count = 1U; stereo_count = 2U; break;
    case 3U: lfe_count = 1U; mono_count = 1U; stereo_count = 3U; break;
    case 4U: lfe_count = 1U; mono_count = 1U; stereo_count = 5U; break;
    case 5U: mono_count = 1U; stereo_count = 2U; break;
    case 6U: stereo_count = 4U; break;
    case 7U: lfe_count = 1U; mono_count = 1U; stereo_count = 4U; break;
    case 8U: lfe_count = 1U; mono_count = 2U; stereo_count = 3U; break;
    case 9U: lfe_count = 1U; mono_count = 2U; stereo_count = 4U; break;
    case 10U:
        if (!read_limits_vlc(source, lfe_count)
            || lfe_count > 32U) {
            return false;
        }
        stream_set.lfe_channels.resize(lfe_count);
        for (std::uint32_t& channels : stream_set.lfe_channels) {
            if (!read_limits_vlc(source, channels)) {
                return false;
            }
            ++channels;
        }
        if (!read_limits_vlc(source, mono_count)
            || !read_limits_vlc(source, stereo_count)) {
            return false;
        }
        break;
    default:
        return false;
    }
    if (lfe_count > 32U || mono_count > 32U || stereo_count > 32U
        || lfe_count + mono_count + stereo_count > 32U) {
        return false;
    }
    if (stream_set.lfe_channels.empty()) {
        stream_set.lfe_channels.assign(lfe_count, 1U);
    }
    stream_set.mono_bandwidth_modes.assign(mono_count, 0U);
    stream_set.stereo_bandwidth_modes.assign(stereo_count, 0U);
    return true;
}

bool read_bandwidth_modes(
    AceBitReader& source,
    AceStreamSetHeader& stream_set) noexcept {
    const std::size_t mono_count =
        stream_set.mono_bandwidth_modes.size();
    const std::size_t stereo_count =
        stream_set.stereo_bandwidth_modes.size();
    std::uint32_t value = 0U;
    if (mono_count + stereo_count == 0U) {
        return true;
    }
    if (mono_count + stereo_count == 1U) {
        if (!source.read(2U, value)) {
            return false;
        }
        if (mono_count != 0U) {
            stream_set.mono_bandwidth_modes[0] = value;
        } else {
            stream_set.stereo_bandwidth_modes[0] = value;
        }
        return true;
    }
    std::uint32_t per_stream = 0U;
    if (!source.read(1U, per_stream)) {
        return false;
    }
    if (per_stream == 0U) {
        if (mono_count != 0U) {
            if (!source.read(2U, value)) {
                return false;
            }
            std::fill(
                stream_set.mono_bandwidth_modes.begin(),
                stream_set.mono_bandwidth_modes.end(),
                value);
        }
        if (stereo_count != 0U) {
            if (!source.read(2U, value)) {
                return false;
            }
            std::fill(
                stream_set.stereo_bandwidth_modes.begin(),
                stream_set.stereo_bandwidth_modes.end(),
                value);
        }
        return true;
    }
    for (std::uint32_t& mode : stream_set.mono_bandwidth_modes) {
        if (!source.read(2U, mode)) {
            return false;
        }
    }
    for (std::uint32_t& mode : stream_set.stereo_bandwidth_modes) {
        if (!source.read(2U, mode)) {
            return false;
        }
    }
    return true;
}

bool read_payload_sizes(
    AceBitReader& source,
    AceStreamSetHeader& stream_set) noexcept {
    stream_set.lfe_payload_sizes.resize(stream_set.lfe_channels.size());
    for (std::uint32_t& size : stream_set.lfe_payload_sizes) {
        if (!read_non_uniform_five_ten(source, size)) {
            return false;
        }
    }
    static constexpr std::array<std::uint8_t, 4> kPrimary{
        2U, 7U, 9U, 13U};
    static constexpr std::array<std::uint8_t, 2> kEscape{24U, 32U};
    stream_set.mono_payload_sizes.resize(
        stream_set.mono_bandwidth_modes.size());
    for (std::uint32_t& size : stream_set.mono_payload_sizes) {
        if (!read_vlc(source, kPrimary, kEscape, size)) {
            return false;
        }
    }
    stream_set.stereo_payload_sizes.resize(
        stream_set.stereo_bandwidth_modes.size());
    for (std::uint32_t& size : stream_set.stereo_payload_sizes) {
        if (!read_vlc(source, kPrimary, kEscape, size)) {
            return false;
        }
    }
    return true;
}

} // namespace

std::uint32_t AceStreamSetHeader::channel_count() const noexcept {
    std::uint64_t channels = mono_bandwidth_modes.size()
        + 2U * stereo_bandwidth_modes.size();
    for (const std::uint32_t count : lfe_channels) {
        channels += count;
    }
    return channels <= std::numeric_limits<std::uint32_t>::max()
        ? static_cast<std::uint32_t>(channels)
        : 0U;
}

std::uint64_t AceStreamSetHeader::payload_size() const noexcept {
    std::uint64_t size = 0U;
    for (const std::uint32_t value : lfe_payload_sizes) {
        size += value;
    }
    for (const std::uint32_t value : mono_payload_sizes) {
        size += value;
    }
    for (const std::uint32_t value : stereo_payload_sizes) {
        size += value;
    }
    return size;
}

std::uint32_t AceFrameHeader::channel_count() const noexcept {
    std::uint64_t channels = 0U;
    for (const AceStreamSetHeader& stream_set : stream_sets) {
        channels += stream_set.channel_count();
    }
    return channels <= std::numeric_limits<std::uint32_t>::max()
        ? static_cast<std::uint32_t>(channels)
        : 0U;
}

std::uint64_t AceFrameHeader::payload_size() const noexcept {
    std::uint64_t size = padding_size;
    for (const AceStreamSetHeader& stream_set : stream_sets) {
        size += stream_set.payload_size();
    }
    return size;
}

AceFrameParseResult parse_ace_frame_header(
    const std::uint8_t* bytes,
    std::size_t size,
    AceFrameParserState& state,
    AceFrameHeader& header) noexcept {
    header = {};
    if (bytes == nullptr || size == 0U
        || size > std::numeric_limits<std::uint32_t>::max()) {
        return AceFrameParseResult::Invalid;
    }
    AceBitReader source(bytes, size);
    if (!source.align()) {
        return AceFrameParseResult::Invalid;
    }
    std::uint32_t value = 0U;
    if (!source.read(1U, value)) {
        return AceFrameParseResult::Invalid;
    }
    header.sync_frame = value != 0U;
    if (!header.sync_frame && !state.initialized) {
        return AceFrameParseResult::Invalid;
    }
    if (header.sync_frame) {
        if (!read_unary_ones(source, 4U, value) || value > 1U) {
            return AceFrameParseResult::Unsupported;
        }
        header.deemphasis_enabled = value == 0U;
        if (!read_unary(source, 2U, value) || value != 0U) {
            return AceFrameParseResult::Unsupported;
        }
        header.frame_duration = 1024U;
        if (!read_unary(source, 2U, value)) {
            return AceFrameParseResult::Invalid;
        }
        header.sample_rate = value == 0U ? 48000U : 44100U;
    } else {
        header.deemphasis_enabled = state.deemphasis_enabled;
        header.frame_duration = state.frame_duration;
        header.sample_rate = state.sample_rate;
    }
    std::uint32_t stream_set_count = 0U;
    if (!read_native_vlc(source, 1U, stream_set_count)
        || stream_set_count >= 32U) {
        return AceFrameParseResult::Invalid;
    }
    ++stream_set_count;
    if (!source.read(1U, value)) {
        return AceFrameParseResult::Invalid;
    }
    header.padding_present = value != 0U;
    header.stream_sets.reserve(stream_set_count);
    for (std::uint32_t index = 0U;
         index < stream_set_count;
         ++index) {
        AceStreamSetHeader stream_set;
        if (!read_native_vlc(source, 0U, stream_set.id)) {
            return AceFrameParseResult::Invalid;
        }
        const AceStreamSetHeader* previous = header.sync_frame
            ? nullptr : find_previous(state, stream_set.id);
        stream_set.predictive = previous != nullptr;
        if (previous != nullptr) {
            stream_set.lfe_channels = previous->lfe_channels;
            stream_set.mono_bandwidth_modes =
                previous->mono_bandwidth_modes;
            stream_set.stereo_bandwidth_modes =
                previous->stereo_bandwidth_modes;
        } else if (!read_stream_composition(source, stream_set)
                   || !read_bandwidth_modes(source, stream_set)) {
            return AceFrameParseResult::Invalid;
        }
        if (!read_payload_sizes(source, stream_set)) {
            return AceFrameParseResult::Invalid;
        }
        header.stream_sets.push_back(std::move(stream_set));
    }
    if (!source.align()) {
        return AceFrameParseResult::Invalid;
    }
    header.payload_bit_offset = source.position();
    const std::uint64_t header_bytes = header.payload_bit_offset >> 3U;
    const std::uint64_t audio_bytes = [&header]() noexcept {
        std::uint64_t total = 0U;
        for (const AceStreamSetHeader& stream_set : header.stream_sets) {
            total += stream_set.payload_size();
        }
        return total;
    }();
    if (!source.valid() || header_bytes + audio_bytes > size) {
        return AceFrameParseResult::Invalid;
    }
    std::uint64_t payload_offset = header_bytes;
    for (std::size_t set_index = 0U;
         set_index < header.stream_sets.size();
         ++set_index) {
        const AceStreamSetHeader& stream_set = header.stream_sets[set_index];
        for (std::size_t stream = 0U;
             stream < stream_set.lfe_payload_sizes.size();
             ++stream) {
            const std::uint32_t payload_size =
                stream_set.lfe_payload_sizes[stream];
            header.stream_payloads.push_back(AceStreamPayload{
                AceStreamType::Lfe,
                static_cast<std::uint32_t>(set_index),
                static_cast<std::uint32_t>(stream),
                stream_set.lfe_channels[stream],
                0U,
                static_cast<std::uint32_t>(payload_offset),
                payload_size});
            payload_offset += payload_size;
        }
        for (std::size_t stream = 0U;
             stream < stream_set.mono_payload_sizes.size();
             ++stream) {
            const std::uint32_t payload_size =
                stream_set.mono_payload_sizes[stream];
            header.stream_payloads.push_back(AceStreamPayload{
                AceStreamType::Mono,
                static_cast<std::uint32_t>(set_index),
                static_cast<std::uint32_t>(stream),
                1U,
                stream_set.mono_bandwidth_modes[stream],
                static_cast<std::uint32_t>(payload_offset),
                payload_size});
            payload_offset += payload_size;
        }
        for (std::size_t stream = 0U;
             stream < stream_set.stereo_payload_sizes.size();
             ++stream) {
            const std::uint32_t payload_size =
                stream_set.stereo_payload_sizes[stream];
            header.stream_payloads.push_back(AceStreamPayload{
                AceStreamType::Stereo,
                static_cast<std::uint32_t>(set_index),
                static_cast<std::uint32_t>(stream),
                2U,
                stream_set.stereo_bandwidth_modes[stream],
                static_cast<std::uint32_t>(payload_offset),
                payload_size});
            payload_offset += payload_size;
        }
    }
    if (payload_offset != header_bytes + audio_bytes
        || payload_offset > size) {
        return AceFrameParseResult::Invalid;
    }
    const std::uint64_t trailing = size - header_bytes - audio_bytes;
    if (header.padding_present) {
        if (trailing > std::numeric_limits<std::uint32_t>::max()) {
            return AceFrameParseResult::Invalid;
        }
        header.padding_size = static_cast<std::uint32_t>(trailing);
    } else if (trailing != 0U) {
        return AceFrameParseResult::Invalid;
    }
    state.initialized = true;
    state.sample_rate = header.sample_rate;
    state.frame_duration = header.frame_duration;
    state.deemphasis_enabled = header.deemphasis_enabled;
    state.stream_sets = header.stream_sets;
    return AceFrameParseResult::Complete;
}

} // namespace dtsx
