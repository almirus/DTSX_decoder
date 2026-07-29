#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "bitstream/dtsx_word_buffer.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace dtsx {

constexpr std::uint32_t kMetadataChunkSync = 0x3A429B0AU;

struct MetadataElementHeader final {
    std::uint8_t chunk_id = 0;
    std::uint8_t flags = 0;
    std::uint8_t payload_size = 0;
    std::uint32_t byte_offset = 0;
    bool primary = false;
    bool short_form = false;
    bool alternate_association = false;
    std::uint8_t association_type = 0;
    std::uint8_t association_index = 0;
    std::uint8_t payload_first_byte = 0;
};

struct MetadataChunkEnvelope final {
    std::vector<std::uint8_t> element_sizes;
    std::uint32_t element_payload_size = 0;
    std::uint32_t crc_region_size = 0;
    bool crc_valid = false;
    std::vector<MetadataElementHeader> elements;
};

struct MetadataChunkLocation final {
    std::uint32_t byte_offset = 0;
    std::uint32_t element_prefix_bytes = 0;
    MetadataChunkEnvelope envelope;
    std::shared_ptr<const bitstream::WordBuffer> source_words;
};

[[nodiscard]] bool unpack_metadata_chunk_envelope(
    bitstream::Cursor& source,
    MetadataChunkEnvelope& envelope,
    std::uint8_t association_mode = 1U);

[[nodiscard]] bool unpack_metadata_chunk_payload(
    bitstream::Cursor& source,
    const std::vector<std::uint8_t>& element_sizes,
    MetadataChunkEnvelope& envelope,
    std::uint8_t association_mode = 1U);

[[nodiscard]] std::vector<MetadataChunkLocation> scan_metadata_chunks(
    bitstream::Cursor source,
    std::uint32_t frame_bytes,
    std::uint8_t association_mode = 1U);

[[nodiscard]] const MetadataElementHeader* find_metadata_element_for_asset(
    const MetadataChunkEnvelope& envelope,
    std::uint8_t chunk_id,
    std::uint8_t asset_index,
    bool primary) noexcept;

} // namespace dtsx
