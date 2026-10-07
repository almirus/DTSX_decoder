#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx {

enum class AceStreamType {
    Lfe,
    Mono,
    Stereo,
};

struct AceStreamPayload final {
    AceStreamType type = AceStreamType::Mono;
    std::uint32_t stream_set_index = 0U;
    std::uint32_t stream_index = 0U;
    std::uint32_t channel_count = 0U;
    std::uint32_t bandwidth_mode = 0U;
    std::uint32_t offset = 0U;
    std::uint32_t size = 0U;
};

struct AceStreamSetHeader final {
    std::uint32_t id = 0U;
    bool predictive = false;
    std::vector<std::uint32_t> lfe_channels;
    std::vector<std::uint32_t> mono_bandwidth_modes;
    std::vector<std::uint32_t> stereo_bandwidth_modes;
    std::vector<std::uint32_t> lfe_payload_sizes;
    std::vector<std::uint32_t> mono_payload_sizes;
    std::vector<std::uint32_t> stereo_payload_sizes;

    [[nodiscard]] std::uint32_t channel_count() const noexcept;
    [[nodiscard]] std::uint64_t payload_size() const noexcept;
};

struct AceFrameHeader final {
    bool sync_frame = false;
    bool deemphasis_enabled = false;
    std::uint32_t sample_rate = 0U;
    std::uint32_t frame_duration = 0U;
    bool padding_present = false;
    std::uint32_t padding_size = 0U;
    std::size_t payload_bit_offset = 0U;
    std::vector<AceStreamSetHeader> stream_sets;
    std::vector<AceStreamPayload> stream_payloads;

    [[nodiscard]] std::uint32_t channel_count() const noexcept;
    [[nodiscard]] std::uint64_t payload_size() const noexcept;
};

struct AceFrameParserState final {
    bool initialized = false;
    std::uint32_t sample_rate = 0U;
    std::uint32_t frame_duration = 0U;
    bool deemphasis_enabled = false;
    std::vector<AceStreamSetHeader> stream_sets;
};

enum class AceFrameParseResult {
    Complete,
    Invalid,
    Unsupported,
};

[[nodiscard]] AceFrameParseResult parse_ace_frame_header(
    const std::uint8_t* bytes,
    std::size_t size,
    AceFrameParserState& state,
    AceFrameHeader& header) noexcept;

} // namespace dtsx
