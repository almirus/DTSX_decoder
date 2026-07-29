#include "dtsx/metadata_chunk.hpp"

#include "dtsx/crc16.hpp"
#include "dtsx/preliminary_metadata.hpp"

#include <utility>

namespace dtsx {

bool unpack_metadata_chunk_payload(
    bitstream::Cursor& source,
    const std::vector<std::uint8_t>& element_sizes,
    MetadataChunkEnvelope& envelope,
    std::uint8_t association_mode) {
    // libdtsx.so: sub_C5B9C, 0xc5b9c.  DTS:X XLL assets carry the
    // element-size vector in the asset descriptor, so the in-band chunk
    // starts directly with element headers and ends with its CRC16.
    envelope = {};
    envelope.element_sizes = element_sizes;
    for (const std::uint8_t size : element_sizes) {
        envelope.element_payload_size += size;
    }
    if (element_sizes.empty()) {
        return false;
    }
    const bitstream::Cursor chunk_start = source;
    const std::uint32_t overhead =
        2U * static_cast<std::uint32_t>(element_sizes.size()) + 2U;
    const std::uint32_t available_bytes =
        source.remaining_bits() / 8U;
    const auto validate_region =
        [&chunk_start, available_bytes](
            std::uint32_t bytes) noexcept {
            if (bytes > available_bytes) {
                return false;
            }
            bitstream::Cursor crc_source = chunk_start;
            return valid_crc16(crc_source, 8U * bytes);
        };
    envelope.crc_region_size =
        envelope.element_payload_size + overhead;
    envelope.crc_valid =
        validate_region(envelope.crc_region_size);
    if (!envelope.crc_valid) {
        // libdtsx.so receives these byte counts from the private XLL
        // navigation arrays.  Some descriptors continue beyond their
        // nominal header and an unavailable look-ahead word can corrupt the
        // final count while the in-band CRC remains authoritative.  Recover
        // only that final count, keeping every preceding element boundary.
        const std::uint32_t prefix_payload_size =
            envelope.element_payload_size
            - envelope.element_sizes.back();
        for (std::uint32_t last_size = 0U;
             last_size <= 0xFFU;
             ++last_size) {
            const std::uint32_t region_size =
                prefix_payload_size + last_size + overhead;
            if (!validate_region(region_size)) {
                continue;
            }
            envelope.element_sizes.back() =
                static_cast<std::uint8_t>(last_size);
            envelope.element_payload_size =
                prefix_payload_size + last_size;
            envelope.crc_region_size = region_size;
            envelope.crc_valid = true;
            break;
        }
    }
    if (envelope.crc_region_size > available_bytes) {
        return false;
    }
    bitstream::Cursor element_source = source;
    std::uint32_t element_offset = 0U;
    envelope.elements.reserve(element_sizes.size());
    for (const std::uint8_t size : element_sizes) {
        bitstream::Cursor preliminary_source = element_source;
        const PreliminaryMetadataHeader preliminary =
            unpack_preliminary_metadata_header(
                preliminary_source, association_mode);
        const std::uint8_t payload_first_byte =
            size == 0U ? 0U : static_cast<std::uint8_t>(
                preliminary_source.lookahead_unsigned(8U));
        envelope.elements.push_back(MetadataElementHeader{
            preliminary.chunk_id,
            preliminary.raw_flags,
            size,
            element_offset,
            preliminary.primary,
            preliminary.short_form,
            preliminary.alternate_association,
            preliminary.association_type,
            preliminary.association_index,
            payload_first_byte});
        element_source.fast_forward(
            static_cast<std::int32_t>(8U * (2U + size)));
        element_offset += 2U + size;
    }
    source.fast_forward(static_cast<std::int32_t>(
        8U * envelope.crc_region_size));
    return source.valid() && element_source.valid();
}

bool unpack_metadata_chunk_envelope(bitstream::Cursor& source,
                                    MetadataChunkEnvelope& envelope,
                                    std::uint8_t association_mode) {
    // libdtsx.so: dtsCheckValidMetaDataChunk, 0x2f0cc..0x2f138.
    if (source.remaining_bits() < 40U
        || source.extract_unsigned(32U) != kMetadataChunkSync) {
        return false;
    }

    const std::uint32_t element_count = source.extract_unsigned(8U) + 1U;
    if (source.remaining_bits() / 8U < element_count) {
        return false;
    }
    envelope = {};
    envelope.element_sizes.reserve(element_count);
    for (std::uint32_t index = 0; index < element_count; ++index) {
        const std::uint8_t size =
            static_cast<std::uint8_t>(source.extract_unsigned(8U));
        envelope.element_sizes.push_back(size);
        envelope.element_payload_size += size;
    }

    envelope.crc_region_size =
        envelope.element_payload_size + 2U * element_count + 2U;
    if (envelope.crc_region_size > source.remaining_bits() / 8U) {
        return false;
    }
    bitstream::Cursor element_source = source;
    std::uint32_t element_offset = 0U;
    envelope.elements.reserve(element_count);
    for (std::uint32_t index = 0; index < element_count; ++index) {
        bitstream::Cursor preliminary_source = element_source;
        const PreliminaryMetadataHeader preliminary =
            unpack_preliminary_metadata_header(
                preliminary_source, association_mode);
        const std::uint8_t payload_first_byte =
            envelope.element_sizes[index] == 0U
            ? 0U
            : static_cast<std::uint8_t>(
                preliminary_source.lookahead_unsigned(8U));
        envelope.elements.push_back(MetadataElementHeader{
            preliminary.chunk_id,
            preliminary.raw_flags,
            envelope.element_sizes[index],
            element_offset,
            preliminary.primary,
            preliminary.short_form,
            preliminary.alternate_association,
            preliminary.association_type,
            preliminary.association_index,
            payload_first_byte});
        element_source.fast_forward(static_cast<std::int32_t>(
            8U * (2U + envelope.element_sizes[index])));
        element_offset += 2U + envelope.element_sizes[index];
    }
    envelope.crc_valid = valid_crc16(
        source, 8U * envelope.crc_region_size);
    // libdtsx.so stores the CRC result in the chunk state but does not reject
    // an otherwise structurally valid chunk here (dtsCheckValidMetaDataChunk,
    // 0x2f05c..0x2f138).  The caller may still expose crc_valid diagnostically.
    return source.valid() && element_source.valid();
}

std::vector<MetadataChunkLocation> scan_metadata_chunks(
    bitstream::Cursor source,
    std::uint32_t frame_bytes,
    std::uint8_t association_mode) {
    // libdtsx.so: dtsCheckValidMetaDataChunk, 0x2f05c..0x2f138.
    std::vector<MetadataChunkLocation> chunks;
    std::uint32_t byte_offset = 0;
    while (byte_offset + 4U <= frame_bytes
           && source.remaining_bits() >= 32U) {
        bitstream::Cursor candidate = source;
        if (candidate.extract_unsigned(32U) == kMetadataChunkSync) {
            candidate = source;
            MetadataChunkEnvelope envelope;
            if (unpack_metadata_chunk_envelope(
                    candidate, envelope, association_mode)) {
                chunks.push_back(MetadataChunkLocation{
                    byte_offset,
                    5U + static_cast<std::uint32_t>(
                        envelope.element_sizes.size()),
                    std::move(envelope)});
            }
        }
        source.fast_forward(32);
        byte_offset += 4U;
    }
    return chunks;
}

const MetadataElementHeader* find_metadata_element_for_asset(
    const MetadataChunkEnvelope& envelope,
    std::uint8_t chunk_id,
    std::uint8_t asset_index,
    bool primary) noexcept {
    // libdtsx.so: dtsFindMDChunkOfAsset, 0xa0460.
    for (const MetadataElementHeader& element : envelope.elements) {
        if (element.chunk_id != chunk_id) {
            continue;
        }
        if (chunk_id > 4U) {
            if (chunk_id == 241U && element.primary == primary
                && (!primary || element.payload_first_byte == asset_index)) {
                return &element;
            }
            continue;
        }
        if (chunk_id >= 2U) {
            if (element.payload_first_byte == asset_index) {
                return &element;
            }
            continue;
        }
        if (chunk_id == 1U) {
            return &element;
        }
    }
    return nullptr;
}

} // namespace dtsx
