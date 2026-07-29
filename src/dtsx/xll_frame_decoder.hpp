#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/xll_channel_decoder.hpp"
#include "dtsx/xll_channel_set.hpp"
#include "dtsx/xll_common_header.hpp"
#include "dtsx/xll_navigation.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace dtsx {

struct XllExtension final {
    std::uint32_t sync_word = 0;
    std::uint32_t frame_offset = 0;
    std::vector<std::uint8_t> payload;
    bool present = false;
};

struct XllLossyBaseChannel final {
    std::uint32_t speaker_mask = 0U;
    const std::vector<std::int32_t>* samples = nullptr;
};

struct XllSupplementalChannelSet final {
    bitstream::Cursor source;
    std::uint32_t speaker_activity_mask = 0U;
    std::vector<std::uint32_t> reference_speaker_masks;
    std::vector<std::uint32_t> lossy_reference_speaker_masks;
    std::vector<std::int32_t> downmix_coefficients;
};

struct XllEmbeddedDownmixOutput final {
    std::uint32_t source_channel_offset = 0U;
    std::uint32_t source_channel_count = 0U;
    std::vector<std::uint32_t> reference_speaker_masks;
    std::vector<std::uint32_t> source_speaker_masks;
    std::vector<std::int32_t> current_coefficients;
    std::vector<std::int32_t> previous_coefficients;
    std::vector<std::uint8_t> reference_storage_bit_depths;
    std::uint8_t source_storage_bit_depth = 0U;
    bool single_frequency_band = false;
};

struct XllHierarchicalDownmixOutput final {
    std::vector<std::uint32_t> speaker_masks;
    std::vector<std::vector<std::int32_t>> planar_channels;
};

struct XllDecodedFrame final {
    XllCommonHeader common;
    std::vector<XllChannelSetHeader> channel_sets;
    XllNavigationTable navigation;
    std::vector<XllNavigationTable> supplemental_navigation;
    XllExtension extension;
    std::vector<std::vector<std::int32_t>> planar_channels;
    std::vector<XllEmbeddedDownmixOutput> embedded_downmix_outputs;
    std::vector<XllHierarchicalDownmixOutput>
        hierarchical_downmix_outputs;
    std::uint32_t main_planar_channel_count = 0;
    std::uint32_t sample_rate = 0;
    std::uint32_t samples_per_channel = 0;
    bool msb_complete = false;
};

[[nodiscard]] std::int32_t
xll_metadata_downmix_coefficient(
    std::uint8_t code) noexcept;

class XllFrameDecoder final {
public:
    [[nodiscard]] bool decode_msb_frame(
        bitstream::Cursor source,
        XllDecodedFrame& frame,
        bool one_to_one_mapping = false,
        const std::vector<XllSupplementalChannelSet>&
            supplemental_channel_sets = {},
        const std::vector<XllLossyBaseChannel>&
            lossy_base_channels = {});

    [[nodiscard]] const std::string& last_error() const noexcept {
        return last_error_;
    }

private:
    std::vector<std::vector<XllChannelSetDecoder>>
        channel_decoders_;
    std::vector<std::vector<XllChannelParameters>>
        channel_parameters_;
    std::vector<std::vector<std::int32_t>>
        downmix_coefficients_;
    std::vector<std::vector<std::int32_t>>
        downmix_inverse_scales_;
    std::vector<XllChannelSetHeader>
        previous_raw_channel_set_headers_;
    std::string last_error_;
};

} // namespace dtsx
