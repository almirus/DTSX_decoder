#pragma once

#include "dtsx/object_metadata_block.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct ObjectWaveformRequest final {
    std::uint32_t object_id = 0;
    std::uint8_t waveform_id = 0;
};

struct DecodedWaveformSlot final {
    std::uint8_t waveform_id = 0;
    std::uint32_t decoder_slot = 0;
};

struct ObjectWaveformMapping final {
    std::uint32_t object_id = 0;
    std::uint8_t waveform_id = 0;
    std::uint32_t decoder_slot = 0;
};

[[nodiscard]] bool map_objects_to_decoded_waveforms(
    const std::vector<ObjectWaveformRequest>& objects,
    const std::vector<DecodedWaveformSlot>& decoded,
    std::vector<ObjectWaveformMapping>& mappings);

[[nodiscard]] bool object_waveform_channel_indices(
    const ObjectMetadataBlock& object,
    std::vector<std::uint32_t>& channel_indices,
    const std::vector<std::uint32_t>* waveform_base_by_id =
        nullptr) noexcept;

} // namespace dtsx
