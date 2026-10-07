#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
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
    struct MetadataObjectAssociation final {
        std::uint32_t object_id = 0U;
        std::uint32_t representation_type = 0U;
        std::uint32_t audio_chunk_index = 0U;
        std::uint32_t navigation_index = 0U;
        std::uint32_t channel_layout_index = 0U;
        std::uint32_t channel_activity_mask = 0U;
        // Native dtsx2 object state fields used by ACEW registration.  These
        // are derived only from the parsed channel-layout record; waveform
        // payload identity remains separate until MDE object descriptors are
        // decoded.
        std::uint32_t registration_channel_count = 0U;
        std::uint32_t registration_layout_mask = 0U;
        std::array<std::uint32_t, 3U> registration_descriptor{};
    };
    struct ThreeDObjectMetadata final {
        std::uint32_t object_id = 0U;
        std::uint32_t layout_mask = 0U;
        std::vector<std::uint32_t> speaker_indices;
    };
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
    std::uint32_t channel_layout_index = 0U;
    std::uint32_t speaker_activity_mask = 0U;
    bool type1_certified_content = false;
    std::vector<UhdChunk> metadata_chunks;
    std::vector<UhdChunk> audio_chunks;
    // Raw metadata chunk payloads are retained for the P2 MDE/object pass.
    // The FTOC only describes their extents; MDE unpacking consumes the
    // payload bitstream separately from ACE audio chunks.
    std::vector<std::vector<std::uint8_t>> metadata_payloads;
    // Object IDs from native P2 metadata chunk type 1 (Table 6-2/6-6).
    // Empty means that the chunk was absent or its object-list fields were
    // not complete in the available frame, not that the stream has zero
    // objects.
    std::uint32_t metadata_audio_presentation_index = 0U;
    std::vector<std::uint32_t> metadata_object_ids;
    std::vector<MetadataObjectAssociation> metadata_object_associations;
    // Representation types 4/5 use the native 3D-object metadata branch;
    // they do not carry the channel-mask association fields below.
    bool has_3d_object_metadata = false;
    std::vector<ThreeDObjectMetadata> three_d_object_metadata;
    // Associations whose MDE audio-chunk index cannot be resolved by the
    // native two-entry DTSX2_MDE_GetAudioChunk contract are retained for
    // diagnostics instead of being silently treated as decoded PCM.
    std::vector<std::uint32_t> unresolved_object_audio_chunk_indices;
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
    std::uint32_t audio_presentation_count = 0U;
    bool interact_obj_limits_present = false;
    std::vector<bool> presentation_selectable;
    std::vector<std::uint32_t> presentation_explicit_list_masks;
    struct AudioChunk final {
        std::uint32_t index = 0U;
        std::uint32_t id = 0U;
        bool present = false;
    };
    std::vector<AudioChunk> audio_chunks;
    bool pbr_smoothing_enabled = false;
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
