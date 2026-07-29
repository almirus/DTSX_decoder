#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/object_metadata_header.hpp"
#include "dtsx/object_metadata_updates.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct AlternativeRenderSet final {
    std::uint32_t speaker_activity_mask = 0;
    bool subset_allowed = false;
    std::vector<std::vector<std::uint8_t>> waveform_gain_codes;
};

struct ObjectMetadataBlock final {
    bool metadata_present = false;
    bool object_id_available = false;
    std::uint32_t object_id = 0;
    std::uint32_t waveform_channel_base_offset = 0;
    bool waveform_id_available = false;
    std::uint8_t waveform_id = 0;
    std::vector<std::uint8_t> waveform_channel_offsets;
    ObjectMetadataPreamble preamble;
    bool group_assignment_present = false;
    std::uint8_t group_assignment = 0;
    bool group_assignment_coherent = false;
    bool spatial_group_present = false;
    std::uint8_t spatial_group = 15;
    ObjectSpatialHeader spatial_header;
    std::vector<PointSourceMetadata> points;
    std::vector<WaveformMetadataUpdate> updates;
    ModeThreeMetadata mode_three;
    bool alternative_rendering_present = false;
    std::uint8_t alternative_speaker_mask_bits = 0;
    std::vector<AlternativeRenderSet> alternative_render_sets;
    bool body_parsed = false;
};

[[nodiscard]] bool unpack_object_metadata_bodies(
    bitstream::Cursor& source,
    bool object_groups_present,
    std::uint8_t update_value_count,
    bool sequential_waveform_offsets,
    std::uint8_t waveform_offset_bits,
    std::vector<ObjectMetadataBlock>& blocks);

} // namespace dtsx
