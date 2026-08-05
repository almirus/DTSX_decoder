#include "dtsx/uhd_frame.hpp"

#include <array>
#include <limits>

namespace dtsx {
namespace {

class ByteBitReader final {
public:
    explicit ByteBitReader(
        const std::vector<std::uint8_t>& bytes,
        std::size_t bit_limit = std::numeric_limits<std::size_t>::max())
        : bytes_(bytes),
          bit_limit_(
              bit_limit < bytes.size() * 8U
                  ? bit_limit
                  : bytes.size() * 8U) {}

    [[nodiscard]] bool read(
        std::uint32_t bit_count,
        std::uint32_t& value) noexcept {
        if (bit_count > 32U
            || bit_position_ + bit_count > bit_limit_) {
            valid_ = false;
            value = 0U;
            return false;
        }
        value = 0U;
        for (std::uint32_t bit = 0U; bit < bit_count; ++bit) {
            const std::size_t byte_index = bit_position_ >> 3U;
            const std::uint32_t shift =
                7U - static_cast<std::uint32_t>(bit_position_ & 7U);
            value = (value << 1U)
                | ((bytes_[byte_index] >> shift) & 1U);
            ++bit_position_;
        }
        return true;
    }

    [[nodiscard]] bool skip(std::size_t bit_count) noexcept {
        if (bit_position_ + bit_count > bit_limit_) {
            valid_ = false;
            return false;
        }
        bit_position_ += bit_count;
        return true;
    }

    [[nodiscard]] std::size_t position() const noexcept {
        return bit_position_;
    }

    [[nodiscard]] bool valid() const noexcept {
        return valid_;
    }

private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t bit_limit_;
    std::size_t bit_position_ = 0U;
    bool valid_ = true;
};

template <std::size_t Size>
bool read_variable_length(
    ByteBitReader& source,
    const std::array<std::uint8_t, Size>& widths,
    std::uint32_t& value,
    bool add_previous_ranges = true) noexcept {
    static_assert(Size == 4U);
    std::uint32_t prefix = 0U;
    if (!source.read(1U, prefix)) {
        return false;
    }
    std::uint32_t table_index = 0U;
    if (prefix != 0U) {
        if (!source.read(1U, prefix)) {
            return false;
        }
        table_index = 1U;
        if (prefix != 0U) {
            if (!source.read(1U, prefix)) {
                return false;
            }
            table_index = prefix != 0U ? 3U : 2U;
        }
    }

    std::uint32_t extracted = 0U;
    if (!source.read(widths[table_index], extracted)) {
        return false;
    }
    value = extracted;
    if (add_previous_ranges) {
        for (std::uint32_t index = 0U;
             index < table_index;
             ++index) {
            value += 1U << widths[index];
        }
    }
    return true;
}

std::uint16_t crc16_ccitt(
    const std::uint8_t* bytes,
    std::size_t size) noexcept {
    static constexpr std::array<std::uint16_t, 16> kTable = {
        0x0000U, 0x1021U, 0x2042U, 0x3063U,
        0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
        0x8108U, 0x9129U, 0xA14AU, 0xB16BU,
        0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    };
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t index = 0U; index < size; ++index) {
        const std::uint8_t value = bytes[index];
        const std::uint16_t high = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(crc << 4U)
            ^ kTable[(value >> 4U) ^ (crc >> 12U)]);
        crc = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(high << 4U)
            ^ kTable[(value & 0x0FU) ^ (high >> 12U)]);
    }
    return crc;
}

bool read_timestamp_update(
    ByteBitReader& source,
    bool sync_frame) noexcept {
    std::uint32_t present = 0U;
    if (!source.read(1U, present)) {
        return false;
    }
    if (present != 0U) {
        return source.skip(sync_frame ? 36U : 36U);
    }
    return true;
}

} // namespace

bool parse_uhd_full_mix_metadata(
    const std::vector<std::uint8_t>& frame,
    UhdFrameHeader& header) noexcept {
    // DTSXDecoder.dll (sdk-dtsx-p2): sub_1001C250 after the FTOC
    // cursor is advanced to the type-0x01 metadata chunk.
    if (!header.full_channel_based_mix
        || header.metadata_chunks.size() != 1U) {
        return false;
    }
    const UhdChunk& chunk = header.metadata_chunks.front();
    if (chunk.size < 2U
        || chunk.offset > frame.size()
        || chunk.size > frame.size() - chunk.offset) {
        return false;
    }

    ByteBitReader source(
        frame,
        static_cast<std::size_t>(chunk.offset + chunk.size) * 8U);
    if (!source.skip(static_cast<std::size_t>(chunk.offset) * 8U)) {
        return false;
    }
    std::uint32_t value = 0U;
    if (!source.read(8U, value) || value != 0x01U) {
        return false;
    }

    if (!read_variable_length(
            source,
            std::array<std::uint8_t, 4>{0U, 2U, 4U, 4U},
            value)) {
        return false;
    }
    for (std::uint32_t index = 0U; index < 4U; ++index) {
        if (!source.read(1U, value)) {
            return false;
        }
        if (value != 0U && !source.skip(5U)) {
            return false;
        }
    }

    if (!source.read(1U, value)) {
        return false;
    }
    if (value != 0U) {
        if (!source.skip(8U)) {
            return false;
        }
        for (std::uint32_t index = 0U; index < 3U; ++index) {
            if (!source.read(1U, value)) {
                return false;
            }
            if (value != 0U) {
                if (!source.read(4U, value)) {
                    return false;
                }
                if (value == 15U && !source.skip(15U)) {
                    return false;
                }
            }
            if (!source.read(1U, value)) {
                return false;
            }
            if (value != 0U && !source.skip(36U)) {
                return false;
            }
        }
    }

    std::uint32_t representation = 0U;
    if (!source.read(3U, representation) || representation >= 4U) {
        return false;
    }
    std::uint32_t mask_index = 1U;
    if (representation != 3U
        && !source.read(4U, mask_index)) {
        return false;
    }
    header.channel_layout_index = mask_index;
    static constexpr std::array<std::uint32_t, 14> kLayoutMasks = {
        0x00000001U,
        0x00000002U,
        0x00000006U,
        0x0000000FU,
        0x0000001FU,
        0x0000084BU,
        0x0000002FU,
        0x0000802FU,
        0x000088ABU,
        0x0000886BU,
        0x0003FBFBU,
        0x00000003U,
        0x00000007U,
        0x00000843U,
    };
    if (mask_index < kLayoutMasks.size()) {
        header.speaker_activity_mask = kLayoutMasks[mask_index];
    } else if (mask_index == 14U) {
        if (!source.read(16U, header.speaker_activity_mask)) {
            return false;
        }
    } else if (mask_index == 15U) {
        if (!source.read(32U, header.speaker_activity_mask)) {
            return false;
        }
    } else {
        return false;
    }
    // DTS-UHD Profile 2 T1 certified content. MediaInfoLib's
    // File_DtsUhd::ExtractObjectInfo applies the same Table 7-21 rule:
    // an explicitly coded 16-bit layout (index 14) with one of the three
    // certified channel activity masks identifies IMAX Enhanced content.
    header.type1_certified_content =
        mask_index == 14U
        && (header.speaker_activity_mask == 0x0000000FU
            || header.speaker_activity_mask == 0x0000002FU
            || header.speaker_activity_mask == 0x0000802FU);
    return source.valid() && header.speaker_activity_mask != 0U;
}

UhdHeaderParseResult parse_uhd_frame_header(
    const std::vector<std::uint8_t>& bytes,
    UhdFrameParserState& state,
    UhdFrameHeader& header) noexcept {
    // DTSXDecoder.dll (sdk-dtsx-p2): sub_1001C250 and
    // ETSI TS 103 491 clauses 6.4.3, 6.4.5 and 6.4.13.
    if (bytes.size() < 5U) {
        return UhdHeaderParseResult::NeedMoreData;
    }

    ByteBitReader initial(bytes);
    std::uint32_t sync_word = 0U;
    if (!initial.read(32U, sync_word)) {
        return UhdHeaderParseResult::NeedMoreData;
    }
    if (sync_word != kUhdSyncFrameWord
        && sync_word != kUhdNonSyncFrameWord) {
        return UhdHeaderParseResult::Invalid;
    }
    header = {};
    header.sync_frame = sync_word == kUhdSyncFrameWord;

    std::uint32_t ftoc_size_minus_one = 0U;
    if (!read_variable_length(
            initial,
            std::array<std::uint8_t, 4>{5U, 8U, 10U, 12U},
            ftoc_size_minus_one)) {
        return UhdHeaderParseResult::NeedMoreData;
    }
    header.ftoc_size = ftoc_size_minus_one + 1U;
    if (header.ftoc_size < 2U) {
        return UhdHeaderParseResult::Invalid;
    }
    if (bytes.size() < header.ftoc_size) {
        return UhdHeaderParseResult::NeedMoreData;
    }

    ByteBitReader source(bytes, header.ftoc_size * 8U);
    if (!source.skip(initial.position())) {
        return UhdHeaderParseResult::Invalid;
    }
    UhdFrameParserState next_state = state;
    std::uint32_t value = 0U;
    if (header.sync_frame) {
        if (!source.read(1U, value)) {
            return UhdHeaderParseResult::Invalid;
        }
        next_state.full_channel_based_mix = value != 0U;
        if (!next_state.full_channel_based_mix) {
            return UhdHeaderParseResult::Unsupported;
        }

        next_state.major_version = 2U;
        next_state.minor_version = 0U;
        if (!source.read(2U, value) || value >= 3U) {
            return UhdHeaderParseResult::Invalid;
        }
        static constexpr std::array<std::uint32_t, 3> kBaseDuration = {
            512U, 480U, 384U};
        next_state.base_duration = kBaseDuration[value];
        if (!source.read(3U, value)) {
            return UhdHeaderParseResult::Invalid;
        }
        next_state.frame_duration =
            next_state.base_duration * (value + 1U);
        if (!source.read(2U, value) || value >= 3U) {
            return UhdHeaderParseResult::Invalid;
        }
        static constexpr std::array<std::uint32_t, 3> kClockRates = {
            32000U, 44100U, 48000U};
        next_state.clock_rate = kClockRates[value];
        if (!read_timestamp_update(source, true)
            || !source.read(2U, value)) {
            return UhdHeaderParseResult::Invalid;
        }
        next_state.sample_rate =
            next_state.clock_rate * (1U << value);
        next_state.audio_chunk_ids.clear();
        next_state.initialized = true;
    } else if (!next_state.initialized) {
        return UhdHeaderParseResult::Invalid;
    }

    if (!next_state.full_channel_based_mix) {
        return UhdHeaderParseResult::Unsupported;
    }
    header.full_channel_based_mix = true;
    header.major_version = next_state.major_version;
    header.minor_version = next_state.minor_version;
    header.base_duration = next_state.base_duration;
    header.frame_duration = next_state.frame_duration;
    header.clock_rate = next_state.clock_rate;
    header.sample_rate = next_state.sample_rate;
    header.samples_per_channel =
        header.clock_rate == 0U
        ? 0U
        : static_cast<std::uint32_t>(
              static_cast<std::uint64_t>(header.frame_duration)
              * header.sample_rate / header.clock_rate);

    std::uint64_t payload_size = header.ftoc_size;
    if (header.sync_frame) {
        std::uint32_t metadata_size = 0U;
        if (!read_variable_length(
                source,
                std::array<std::uint8_t, 4>{6U, 9U, 12U, 15U},
                metadata_size)) {
            return UhdHeaderParseResult::Invalid;
        }
        header.metadata_chunks.push_back(
            UhdChunk{0U, 0U, header.ftoc_size, metadata_size, false});
        payload_size += metadata_size;
    }

    std::uint32_t audio_chunk_id = 0U;
    if (header.sync_frame) {
        if (!read_variable_length(
                source,
                std::array<std::uint8_t, 4>{2U, 4U, 6U, 8U},
                audio_chunk_id)) {
            return UhdHeaderParseResult::Invalid;
        }
        next_state.audio_chunk_ids.assign(1U, audio_chunk_id);
    } else {
        if (next_state.audio_chunk_ids.empty()) {
            return UhdHeaderParseResult::Invalid;
        }
        audio_chunk_id = next_state.audio_chunk_ids.front();
    }

    std::uint32_t audio_chunk_size = 0U;
    if (audio_chunk_id != 0U
        && !read_variable_length(
            source,
            std::array<std::uint8_t, 4>{9U, 11U, 13U, 16U},
            audio_chunk_size)) {
        return UhdHeaderParseResult::Invalid;
    }
    header.audio_chunks.push_back(UhdChunk{
        0U,
        audio_chunk_id,
        static_cast<std::uint32_t>(payload_size),
        audio_chunk_size,
        false});
    payload_size += audio_chunk_size;

    const bool crc_required = header.sync_frame;
    if (crc_required
        && crc16_ccitt(bytes.data(), header.ftoc_size) != 0U) {
        return UhdHeaderParseResult::Invalid;
    }
    if (!source.valid()
        || payload_size > std::numeric_limits<std::uint32_t>::max()
        || payload_size < header.ftoc_size) {
        return UhdHeaderParseResult::Invalid;
    }
    header.frame_size = static_cast<std::uint32_t>(payload_size);
    state = std::move(next_state);
    return UhdHeaderParseResult::Complete;
}

} // namespace dtsx
