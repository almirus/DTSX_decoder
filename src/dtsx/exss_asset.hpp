#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/exss_header.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace dtsx {

struct ExssAssetSummary final {
    std::uint32_t header_size = 0;
    std::uint8_t asset_index = 0;
    bool content_type_present = false;
    std::uint8_t content_type = 0;
    bool type1_certified_content = false;
    std::uint8_t object_audio_type = 0;
    std::uint32_t sample_rate = 0;
    std::uint32_t channel_count = 0;
    bool one_to_one_mapping = false;
    bool embedded_stereo = false;
    bool embedded_six_channel = false;
    bool speaker_mask_present = false;
    std::uint32_t speaker_activity_mask = 0;
    std::uint32_t speaker_count = 0;
    std::uint8_t representation_type = 0;
    std::uint8_t coding_mode = 0;
    bool coding_mode_available = false;
    std::uint16_t coding_components = 0;
    std::array<std::uint32_t, 12> component_size_bytes{};
    std::array<std::uint32_t, 12> component_byte_offsets{};
    bool xll_sync_present = false;
    std::uint8_t xll_smoothing_buffer_kbytes = 0;
    std::uint8_t xll_initial_delay_bits = 0;
    std::uint32_t xll_initial_delay_frames = 0;
    std::uint32_t xll_sync_offset = 0;
    std::uint8_t dts_hd_stream_id = 0;
    bool decode_in_secondary_decoder = false;
    bool xll_metadata_present = false;
    bool xll_object_metadata_present = false;
    std::uint32_t xll_metadata_offset = 0;
    std::vector<std::uint8_t> xll_metadata_chunk_sizes;
    std::vector<std::uint8_t> xll_associated_chunk_types;
    std::vector<std::uint16_t> xll_associated_chunk_extents;
    std::uint32_t xll_navigation_bit_offset = 0;
    std::uint32_t xll_object_sizes_bit_offset = 0;
    std::uint32_t descriptor_bits_used = 0;
};

[[nodiscard]] bool unpack_exss_asset_summary(
    bitstream::Cursor& source,
    const ExssHeader& exss,
    ExssAssetSummary& asset,
    std::uint32_t asset_ordinal = 0U) noexcept;

} // namespace dtsx
