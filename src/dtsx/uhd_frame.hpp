#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx {

constexpr std::uint32_t kUhdSyncFrameWord = 0x40411BF2U;
constexpr std::uint32_t kUhdNonSyncFrameWord = 0x71C442E8U;

struct UhdChunk final {
    std::uint32_t index = 0U;
    std::uint32_t id = 0U;
    std::uint32_t offset = 0U;
    std::uint32_t size = 0U;
    bool crc_present = false;
};

struct UhdFrameHeader final {
    bool sync_frame = false;
    bool full_channel_based_mix = false;
    std::uint32_t major_version = 0U;
    std::uint32_t minor_version = 0U;
    std::uint32_t ftoc_size = 0U;
    std::uint32_t frame_size = 0U;
    std::uint32_t base_duration = 0U;
    std::uint32_t frame_duration = 0U;
    std::uint32_t clock_rate = 0U;
    std::uint32_t sample_rate = 0U;
    std::uint32_t samples_per_channel = 0U;
    std::uint32_t speaker_activity_mask = 0U;
    std::vector<UhdChunk> metadata_chunks;
    std::vector<UhdChunk> audio_chunks;
};

struct UhdFrameParserState final {
    bool initialized = false;
    bool full_channel_based_mix = false;
    std::uint32_t major_version = 0U;
    std::uint32_t minor_version = 0U;
    std::uint32_t base_duration = 0U;
    std::uint32_t frame_duration = 0U;
    std::uint32_t clock_rate = 0U;
    std::uint32_t sample_rate = 0U;
    std::vector<std::uint32_t> audio_chunk_ids;
};

enum class UhdHeaderParseResult {
    NeedMoreData,
    Complete,
    Invalid,
    Unsupported,
};

[[nodiscard]] bool parse_uhd_full_mix_metadata(
    const std::vector<std::uint8_t>& frame,
    UhdFrameHeader& header) noexcept;

[[nodiscard]] UhdHeaderParseResult parse_uhd_frame_header(
    const std::vector<std::uint8_t>& bytes,
    UhdFrameParserState& state,
    UhdFrameHeader& header) noexcept;

} // namespace dtsx
