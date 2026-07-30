#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/object_metadata_block.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct PreliminaryMetadataHeader final {
    std::uint8_t chunk_id = 0;
    std::uint8_t raw_flags = 0;
    std::uint8_t association_type = 0;
    std::uint8_t association_index = 0;
    std::uint8_t selector = 0;
    bool alternate_association = false;
    bool primary = false;
    bool short_form = false;
};

[[nodiscard]] PreliminaryMetadataHeader unpack_preliminary_metadata_header(
    bitstream::Cursor& source, std::uint8_t association_mode) noexcept;

struct AudioPresentationMetadata final {
    std::uint8_t presentation_index = 0;
    bool selectable = false;
    std::uint8_t presentation_count = 0;
    bool metadata_present = false;
    std::uint8_t object_count = 0;
    bool object_count_extended = false;
    std::uint8_t object_count_extension = 0;
    bool channel_mask_present = false;
    std::uint32_t speaker_activity_mask = 0;
    std::uint8_t speaker_count = 0;
    bool alternative_render_gain_present = false;
    std::uint8_t alternative_render_gain_code = 61;
    std::uint8_t render_gain_code = 61;
    bool object_groups_present = false;
    std::uint8_t object_group_count = 0;
    bool sequential_waveform_offsets = false;
    std::uint8_t waveform_offset_bits = 0;
    bool alternative_rendering_metadata_present = false;
    std::vector<ObjectMetadataBlock> objects;
};

struct CombinedMixMetadata final {
    std::uint32_t reference_speaker_activity_mask = 0U;
    std::uint32_t output_speaker_activity_mask = 0U;
    std::uint32_t added_speaker_activity_mask = 0U;
    std::vector<std::uint32_t> reference_speaker_masks;
    std::vector<std::uint32_t> added_speaker_masks;
    std::vector<std::uint32_t> lossy_reference_speaker_masks;
    // Transposed native matrix: reference channel major, added channel minor.
    // Each value is the six-bit dtsx_dmixCoeffTable code from the stream.
    std::vector<std::uint8_t> downmix_coefficient_codes;
};

[[nodiscard]] bool unpack_combined_mix_metadata(
    bitstream::Cursor& source,
    std::uint8_t association_mode,
    std::uint32_t fallback_reference_speaker_activity_mask,
    CombinedMixMetadata& metadata) noexcept;

[[nodiscard]] bool unpack_audio_presentation_metadata(
    bitstream::Cursor& source,
    const PreliminaryMetadataHeader& preliminary,
    bool short_form,
    AudioPresentationMetadata& metadata) noexcept;

[[nodiscard]] bool unpack_audio_presentation_object_bodies(
    bitstream::Cursor& source,
    AudioPresentationMetadata& metadata,
    const std::vector<bool>* waveform_decoder_available = nullptr);

} // namespace dtsx
