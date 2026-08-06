#pragma once

#include "dtsx/frame_assembler.hpp"
#include "dtsx/object_metadata_block.hpp"
#include "dtsx/preliminary_metadata.hpp"
#include "dtsx/xll_frame_decoder.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace dtsx_decode {

struct DecodedObjectAudioFrame final {
    std::uint32_t sample_rate = 0;
    std::uint32_t samples_per_channel = 0;
    bool imax_enhanced = false;
    std::uint32_t dtsx_extension_sync_word = 0;
    std::uint32_t ignored_unmapped_objects = 0;
    std::vector<std::vector<std::int32_t>> waveform_channels;
    std::vector<std::uint32_t> waveform_speaker_masks;
    std::vector<std::uint32_t> waveform_source_activity_masks;
    // True for physical supplemental channels: legacy type-69 XLL channel
    // sets and the unreferenced private type-68 upper-layer decoder selected
    // by type-247 renderer metadata. Type-241 object waveforms remain false.
    std::vector<bool> waveform_is_supplemental;
    std::vector<dtsx::XllEmbeddedDownmixOutput>
        supplemental_downmix_outputs;
    std::vector<dtsx::XllHierarchicalDownmixOutput>
        bed_downmix_outputs;
    // XLL channel-set bed decoded by the native path.  This is kept
    // separate from object waveforms so a 7.1.4 bed can be rendered without
    // asking FFmpeg to decode or upmix DTS:X.
    std::vector<std::vector<std::int32_t>> bed_channels;
    std::uint32_t bed_speaker_activity_mask = 0;
    std::vector<std::uint32_t> waveform_base_by_id;
    std::uint32_t metadata_speaker_activity_mask = 0;
    std::uint32_t supplemental_speaker_activity_mask = 0;
    bool alternative_presentation_gain_present = false;
    std::uint8_t alternative_presentation_gain_code = 61;
    std::uint8_t presentation_gain_code = 61;
    std::uint32_t metadata_presentation_headers = 0;
    std::uint32_t metadata_body_parse_failures = 0;
    std::uint32_t raw_metadata_envelopes = 0;
    std::uint32_t raw_metadata_crc_failures = 0;
    std::uint32_t raw_metadata_elements = 0;
    std::vector<dtsx::ObjectMetadataBlock> objects;
};

enum class ObjectFrameDecodeResult {
    Ignored,
    Decoded,
    Malformed,
};

class ObjectFrameDecoder final {
public:
    [[nodiscard]] ObjectFrameDecodeResult decode(
        const dtsx::ElementaryFrame& elementary,
        DecodedObjectAudioFrame& decoded,
        const std::vector<dtsx::XllLossyBaseChannel>&
            lossy_base_channels = {});
    [[nodiscard]] const std::string& last_error() const noexcept {
        return last_error_;
    }

private:
    std::vector<dtsx::XllFrameDecoder> xll_decoders_;
    std::vector<std::vector<dtsx::XllFrameDecoder>>
        uhd_xll_decoders_;
    std::vector<std::vector<std::uint8_t>> xll_pbr_buffers_;
    std::vector<dtsx::CombinedMixMetadata>
        combined_mix_metadata_state_;
    std::vector<bool> combined_mix_metadata_valid_;
    std::vector<std::uint32_t> waveform_base_by_id_state_;
    std::vector<dtsx::ObjectMetadataBlock> object_state_;
    std::uint32_t metadata_speaker_activity_mask_ = 0;
    bool alternative_presentation_gain_present_ = false;
    std::uint8_t alternative_presentation_gain_code_ = 61;
    std::uint8_t presentation_gain_code_ = 61;
    std::string last_error_;
};

} // namespace dtsx_decode
