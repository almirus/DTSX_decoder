#include "dtsx/object_waveform_map.hpp"

#include <limits>

namespace dtsx {

bool map_objects_to_decoded_waveforms(
    const std::vector<ObjectWaveformRequest>& objects,
    const std::vector<DecodedWaveformSlot>& decoded,
    std::vector<ObjectWaveformMapping>& mappings) {
    // libdtsx.so: dtsPlayerObjectRenderer_MapObjectsToDecoders, 0x5eb04.
    mappings.clear();
    mappings.reserve(objects.size());
    for (const ObjectWaveformRequest& object : objects) {
        if (decoded.empty()) {
            mappings.clear();
            return false;
        }

        std::size_t decoder_index = 0U;
        bool found = decoded.front().waveform_id == object.waveform_id;
        for (std::size_t index = 0;
             !found && index + 1U < decoded.size();
             ++index) {
            const std::uint8_t current = decoded[index].waveform_id;
            const std::uint8_t next = decoded[index + 1U].waveform_id;
            if (next == object.waveform_id) {
                decoder_index = index + 1U;
                found = true;
            } else if (object.waveform_id > current
                       && next > object.waveform_id) {
                decoder_index = index;
                found = true;
            }
        }
        if (!found
            && object.waveform_id
                > decoded.back().waveform_id) {
            // libdtsx(v2).so.c:
            // dtsPlayerObjectRenderer_MapObjectsToDecoders. Decoder IDs
            // delimit ranges: an object above the final decoder ID is
            // assigned to that final decoder.
            decoder_index = decoded.size() - 1U;
            found = true;
        }
        if (!found) {
            mappings.clear();
            return false;
        }
        const DecodedWaveformSlot& match = decoded[decoder_index];
        mappings.push_back(ObjectWaveformMapping{
            object.object_id, object.waveform_id, match.decoder_slot});
    }
    return true;
}

bool object_waveform_channel_indices(
    const ObjectMetadataBlock& object,
    std::vector<std::uint32_t>& channel_indices,
    const std::vector<std::uint32_t>*
        waveform_base_by_id) noexcept {
    // libdtsx.so: dtsPlayerObjectRenderer_SetupObjectRenderer,
    // 0x5f3a0, waveform sample-array lookup using object bytes
    // 104, 105, and 106+n.
    channel_indices.clear();
    if (!object.metadata_present || !object.waveform_id_available
        || object.waveform_channel_offsets.size()
            != object.preamble.waveform_count) {
        return false;
    }
    std::uint32_t waveform_base =
        static_cast<std::uint32_t>(object.waveform_id);
    if (waveform_base_by_id != nullptr
        && object.waveform_id
            < waveform_base_by_id->size()) {
        waveform_base =
            (*waveform_base_by_id)[object.waveform_id];
        if (waveform_base
            == std::numeric_limits<std::uint32_t>::max()) {
            return false;
        }
    }
    channel_indices.reserve(object.waveform_channel_offsets.size());
    for (const std::uint8_t waveform_offset :
         object.waveform_channel_offsets) {
        // The native setup path performs pointer arithmetic only after the
        // waveform base and the per-object offsets have been resolved.  Keep
        // the equivalent operation bounded here: a malformed metadata block
        // must not wrap a 32-bit channel index back into an earlier decoded
        // waveform slot.
        const std::uint64_t channel_index =
            static_cast<std::uint64_t>(waveform_base)
            + static_cast<std::uint64_t>(
                object.waveform_channel_base_offset)
            + static_cast<std::uint64_t>(waveform_offset);
        if (channel_index >
            static_cast<std::uint64_t>(
                std::numeric_limits<std::uint32_t>::max())) {
            channel_indices.clear();
            return false;
        }
        channel_indices.push_back(
            static_cast<std::uint32_t>(channel_index));
    }
    return true;
}

} // namespace dtsx
