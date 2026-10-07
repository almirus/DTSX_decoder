#include "dtsx/lbr_chunk.hpp"

#include "dtsx/speaker_mask.hpp"

namespace dtsx {

LbrChunkHeader unpack_lbr_chunk_header(
    bitstream::Cursor& source) noexcept {
    LbrChunkHeader result;
    result.chunk_id = static_cast<std::uint8_t>(source.extract_unsigned(8U));
    const std::uint8_t second_byte =
        static_cast<std::uint8_t>(source.extract_unsigned(8U));
    if ((result.chunk_id & 0x80U) != 0U) {
        result.payload_bytes = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(second_byte) << 8U)
            | source.extract_unsigned(8U));
        result.chunk_id &= 0x7FU;
        result.header_bytes = 3U;
    } else {
        result.payload_bytes = second_byte;
        result.header_bytes = 2U;
    }
    return result;
}

bool parse_lbr_channels(bitstream::Cursor& source,
                        std::uint32_t& speaker_count,
                        std::uint32_t& speaker_mask,
                        bool& has_extension) noexcept {
    if (source.extract_unsigned(32U) != 0x0A801921U) {
        return false;
    }
    const std::uint8_t version = static_cast<std::uint8_t>(
        source.extract_unsigned(8U));
    if (version == 0U || (version != 1U && version != 2U)) {
        return false;
    }

    std::uint32_t mask = speaker_mask;
    if (version == 2U) {
        source.fast_forward(8);
        const std::uint16_t swapped = static_cast<std::uint16_t>(
            source.extract_unsigned(16U));
        mask = static_cast<std::uint16_t>((swapped << 8U) | (swapped >> 8U));
        source.fast_forward(64);
    }
    if (speaker_count == 0U) {
        speaker_count = speaker_count_from_activity_mask(mask);
    }

    const LbrChunkHeader first = unpack_lbr_chunk_header(source);
    if ((first.chunk_id & 0xFDU) != 4U) {
        return false;
    }
    std::int32_t remaining = first.payload_bytes;
    if (first.chunk_id == 4U) {
        const bitstream::Cursor payload_start = source;
        const std::uint16_t value = static_cast<std::uint16_t>(
            (source.extract_unsigned(8U) << 8U)
            | source.extract_unsigned(8U));
        std::uint32_t sum = 0U;
        for (std::uint32_t i = 0; i < first.header_bytes; ++i) {
            sum += source.extract_unsigned(8U);
        }
        if (sum != value) {
            return false;
        }
        source = payload_start;
        remaining -= 2;
    }

    while (remaining > 0) {
        const LbrChunkHeader chunk = unpack_lbr_chunk_header(source);
        remaining -= static_cast<std::int32_t>(
            chunk.header_bytes + chunk.payload_bytes);
        if (chunk.chunk_id == 1U) {
            break;
        }
        if (chunk.chunk_id != 0U) {
            source.fast_forward(static_cast<std::int32_t>(
                8U * chunk.payload_bytes));
        }
    }
    if (remaining <= 0) {
        return false;
    }

    const std::uint8_t flags = static_cast<std::uint8_t>(
        source.extract_unsigned(8U));
    if ((flags & 1U) != 0U) {
        const std::uint8_t band = static_cast<std::uint8_t>(
            source.extract_unsigned(3U));
        static constexpr std::uint32_t kBandCount[7] = {1U, 2U, 2U, 3U,
                                                         3U, 4U, 4U};
        if (band <= 6U) {
            const std::uint32_t groups = kBandCount[band];
            bool present[32] = {};
            for (std::uint32_t group = 0; group < groups; ++group) {
                for (std::uint32_t channel = 0; channel < speaker_count
                                               && channel < 32U; ++channel) {
                    const std::uint32_t index = group * speaker_count + channel;
                    const bool value = source.extract_unsigned(1U) != 0U;
                    if (index < 32U) {
                        present[index] = value;
                    }
                }
            }
            for (std::uint32_t group = 0; group < groups; ++group) {
                for (std::uint32_t channel = 0; channel < speaker_count
                                               && channel < 32U; ++channel) {
                    const std::uint32_t index = group * speaker_count + channel;
                    if (index < 32U && present[index]) {
                        source.fast_forward(5);
                        if (band == 2U) {
                            source.fast_forward(1);
                        }
                    }
                }
            }
        }
    }
    if ((flags & 2U) != 0U) {
        const std::uint8_t update = static_cast<std::uint8_t>(
            source.extract_unsigned(8U));
        has_extension = true;
        if ((update & 1U) != 0U) {
            mask |= 0x10U;
        }
        if ((update & 2U) != 0U) {
            mask |= 0x20U;
        }
        if ((update & 4U) != 0U) {
            mask = (update & 0x80U) != 0U ? (mask & ~0x5U) | 0x840U
                                         : mask | 0x40U;
        }
        if ((update & 8U) != 0U) {
            mask |= 0x80U;
        }
        if ((update & 0x10U) != 0U) {
            mask |= 0x100U;
        }
        if ((update & 0x20U) != 0U) {
            mask |= 0x200U;
        }
        if ((update & 0x40U) != 0U) {
            mask |= 0x400U;
        }
        speaker_count = speaker_count_from_activity_mask(mask);
    }
    speaker_mask = mask;
    speaker_count = speaker_count_from_activity_mask(mask);
    return true;
}

} // namespace dtsx
