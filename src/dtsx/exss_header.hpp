#pragma once

#include "bitstream/dtsx_bitstream.hpp"

#include <cstdint>
#include <array>

namespace dtsx {

struct ExssHeader final {
    std::uint8_t stream_index = 0;
    bool long_size_fields = false;
    std::uint8_t size_field_bits = 0;
    std::uint32_t header_size = 0;
    std::uint32_t frame_size = 0;
    bool static_fields_present = false;
    std::uint8_t reference_clock_code = 0;
    std::uint32_t frame_duration = 0;
    std::uint32_t asset_count = 0;
    std::uint32_t waveform_count = 0;
    std::uint32_t total_asset_bytes = 0;
    std::uint32_t presentation_id = 0;
    std::uint32_t speaker_mask_bits = 0;
    std::uint32_t speaker_mask_count = 0;
    std::uint8_t speaker_metadata_present = 0;
    std::uint8_t selected_waveform = 0;
    std::uint8_t waveform_metadata_mode = 0;
    std::uint8_t waveform_metadata_type = 0;
    std::array<std::uint8_t, 8> waveform_present{};
    std::array<std::uint32_t, 8> asset_sizes{};
    std::array<std::uint32_t, 8> asset_header_sizes{};
    std::array<std::uint32_t, 8> asset_header_bit_offsets{};
    std::array<std::uint8_t, 8> asset_types{};
    std::array<std::uint8_t, 8> asset_object_audio{};
    std::array<std::uint8_t, 8> asset_selected{};
    std::array<std::uint8_t, 8> speaker_counts{};
    std::array<std::uint8_t, 8> presentation_asset_masks{};
};

[[nodiscard]] bool unpack_exss_header(bitstream::Cursor& source,
                                      ExssHeader& header,
                                      std::uint32_t selected_waveform_index = 0U) noexcept;

[[nodiscard]] bool validate_exss_frame(bitstream::Cursor source) noexcept;

} // namespace dtsx
