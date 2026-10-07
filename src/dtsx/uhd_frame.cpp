#include "dtsx/uhd_frame.hpp"
#include "dtsx/speaker_mask.hpp"

#include <algorithm>
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

bool read_presentation_navigation(
    ByteBitReader& source,
    bool sync_frame,
    bool full_channel_based_mix,
    UhdFrameParserState& state) noexcept {
    std::uint32_t value = 0U;
    if (sync_frame) {
        if (full_channel_based_mix) {
            state.audio_presentation_count = 1U;
            state.presentation_selectable.assign(1U, true);
            state.presentation_explicit_list_masks.assign(1U, 0U);
        } else {
            if (!read_variable_length(
                    source,
                    std::array<std::uint8_t, 4>{0U, 2U, 4U, 5U},
                    value)) {
                return false;
            }
            state.audio_presentation_count = value + 1U;
            state.presentation_selectable.assign(
                state.audio_presentation_count, false);
            state.presentation_explicit_list_masks.assign(
                state.audio_presentation_count, 0U);
        }
    }
    if (state.audio_presentation_count == 0U
        || state.audio_presentation_count > 32U
        || state.presentation_selectable.size()
            != state.audio_presentation_count
        || state.presentation_explicit_list_masks.size()
            != state.audio_presentation_count) {
        return false;
    }

    for (std::uint32_t presentation = 0U;
         presentation < state.audio_presentation_count;
         ++presentation) {
        if (sync_frame && !full_channel_based_mix) {
            if (!source.read(1U, value)) {
                return false;
            }
            state.presentation_selectable[presentation] = value != 0U;
        }
        if (!state.presentation_selectable[presentation]) {
            continue;
        }

        if (sync_frame) {
            std::uint32_t dependency_mask = 0U;
            if (presentation != 0U
                && !source.read(presentation, dependency_mask)) {
                return false;
            }
            std::uint32_t explicit_list_mask = 0U;
            for (std::uint32_t dependency = 0U;
                 dependency < presentation;
                 ++dependency) {
                if ((dependency_mask & (1U << dependency)) == 0U) {
                    continue;
                }
                if (!source.read(1U, value)) {
                    return false;
                }
                explicit_list_mask |= value << dependency;
            }
            state.presentation_explicit_list_masks[presentation] =
                explicit_list_mask;
        }

        const std::uint32_t explicit_list_mask =
            state.presentation_explicit_list_masks[presentation];
        for (std::uint32_t dependency = 0U;
             dependency < presentation;
             ++dependency) {
            if ((explicit_list_mask & (1U << dependency)) == 0U) {
                continue;
            }
            bool update = sync_frame;
            if (!sync_frame) {
                if (!source.read(1U, value)) {
                    return false;
                }
                update = value != 0U;
            }
            if (update
                && !read_variable_length(
                    source,
                    std::array<std::uint8_t, 4>{4U, 8U, 16U, 32U},
                    value)) {
                return false;
            }
        }
    }
    return true;
}

bool read_chunk_navigation(
    ByteBitReader& source,
    bool sync_frame,
    bool full_channel_based_mix,
    std::uint32_t ftoc_size,
    UhdFrameParserState& state,
    UhdFrameHeader& header,
    std::uint64_t& payload_size) noexcept {
    std::uint32_t value = 0U;
    std::uint32_t metadata_count = 0U;
    if (full_channel_based_mix) {
        metadata_count = sync_frame ? 1U : 0U;
    } else if (!read_variable_length(
                   source,
                   std::array<std::uint8_t, 4>{2U, 4U, 6U, 8U},
                   metadata_count)) {
        return false;
    }

    std::vector<std::uint32_t> metadata_sizes(metadata_count, 0U);
    std::vector<bool> metadata_crc(metadata_count, false);
    for (std::uint32_t index = 0U; index < metadata_count; ++index) {
        if (!read_variable_length(
                source,
                std::array<std::uint8_t, 4>{6U, 9U, 12U, 15U},
                metadata_sizes[index])) {
            return false;
        }
        if (!full_channel_based_mix) {
            if (!source.read(1U, value)) {
                return false;
            }
            metadata_crc[index] = value != 0U;
        }
    }

    std::uint32_t audio_count = 0U;
    if (full_channel_based_mix) {
        audio_count = 1U;
    } else if (!read_variable_length(
                   source,
                   std::array<std::uint8_t, 4>{2U, 4U, 6U, 8U},
                   audio_count)) {
        return false;
    }

    for (auto& chunk : state.audio_chunks) {
        chunk.present = false;
    }
    header.audio_chunks.reserve(audio_count);
    for (std::uint32_t packing_index = 0U;
         packing_index < audio_count;
         ++packing_index) {
        std::uint32_t chunk_index = 0U;
        if (!full_channel_based_mix
            && !read_variable_length(
                source,
                std::array<std::uint8_t, 4>{2U, 4U, 6U, 8U},
                chunk_index)) {
            return false;
        }
        auto found = state.audio_chunks.end();
        for (auto iterator = state.audio_chunks.begin();
             iterator != state.audio_chunks.end();
             ++iterator) {
            if (iterator->index == chunk_index) {
                found = iterator;
                break;
            }
        }
        if (found == state.audio_chunks.end()) {
            state.audio_chunks.push_back(
                UhdFrameParserState::AudioChunk{chunk_index, 256U, true});
            found = state.audio_chunks.end() - 1;
        }
        found->present = true;

        bool id_present = sync_frame;
        if (!sync_frame && !full_channel_based_mix) {
            if (!source.read(1U, value)) {
                return false;
            }
            id_present = value != 0U;
        }
        if (id_present) {
            if (!read_variable_length(
                    source,
                    std::array<std::uint8_t, 4>{2U, 4U, 6U, 8U},
                    found->id)) {
                return false;
            }
        } else if (found->id >= 256U) {
            return false;
        }

        std::uint32_t chunk_size = 0U;
        if (found->id != 0U
            && !read_variable_length(
                source,
                std::array<std::uint8_t, 4>{9U, 11U, 13U, 16U},
                chunk_size)) {
            return false;
        }
        header.audio_chunks.push_back(UhdChunk{
            chunk_index,
            found->id,
            0U,
            chunk_size,
            false});
    }
    state.audio_chunks.erase(
        std::remove_if(
            state.audio_chunks.begin(),
            state.audio_chunks.end(),
            [](const UhdFrameParserState::AudioChunk& chunk) {
                return !chunk.present;
            }),
        state.audio_chunks.end());

    std::uint64_t offset = ftoc_size;
    header.metadata_chunks.reserve(metadata_count);
    for (std::uint32_t index = 0U; index < metadata_count; ++index) {
        header.metadata_chunks.push_back(UhdChunk{
            index,
            0U,
            static_cast<std::uint32_t>(offset),
            metadata_sizes[index],
            metadata_crc[index]});
        offset += metadata_sizes[index];
    }
    for (UhdChunk& chunk : header.audio_chunks) {
        chunk.offset = static_cast<std::uint32_t>(offset);
        offset += chunk.size;
    }
    payload_size = offset;
    return true;
}

bool read_pbr_smoothing(
    ByteBitReader& source,
    bool sync_frame,
    UhdFrameParserState& state) noexcept {
    std::uint32_t value = 0U;
    if (sync_frame) {
        if (!source.read(1U, value)) {
            return false;
        }
        state.pbr_smoothing_enabled = value != 0U;
    }
    if (!state.pbr_smoothing_enabled) {
        return true;
    }
    if (!source.read(2U, value)) {
        return false;
    }
    const std::uint32_t payload_bits = 9U + 3U * value;
    if (!source.skip(payload_bits)
        || !source.read(1U, value)) {
        return false;
    }
    if (value != 0U
        && (!source.skip(2U + 8U)
            || !source.skip(payload_bits - 2U))) {
        return false;
    }
    return true;
}

std::uint32_t native_layout_speaker_count(
    const std::uint32_t mask) noexcept {
    if (mask == 0U) {
        return 0U;
    }
    std::uint32_t count = (mask & 3U);
    if ((mask & 4U) != 0U) count += 2U;
    count += ((mask >> 3U) & 1U) + ((mask >> 4U) & 1U);
    if ((mask & 0x20U) != 0U) count += 2U;
    if ((mask & 0x40U) != 0U) count += 2U;
    count += ((mask >> 7U) & 1U) + ((mask >> 8U) & 1U);
    if ((mask & 0x200U) != 0U) count += 2U;
    if ((mask & 0x400U) != 0U) count += 2U;
    if ((mask & 0x800U) != 0U) count += 2U;
    count += (mask >> 12U) & 1U;
    if ((mask & 0x2000U) != 0U) count += 2U;
    count += (mask >> 14U) & 1U;
    if ((mask & 0x8000U) != 0U) count += 2U;
    count += (mask >> 16U) & 1U;
    if ((mask & 0x20000U) != 0U) count += 2U;
    if ((mask & 0x40000U) != 0U) count += 2U;
    if ((mask & 0x80000U) != 0U) count += 2U;
    return count;
}

bool read_native_varlen_17(
    ByteBitReader& source,
    std::uint32_t& value) noexcept {
    return read_variable_length(
        source, std::array<std::uint8_t, 4>{12U, 16U, 20U, 32U},
        value, false);
}

bool parse_re_object_metadata(
    ByteBitReader& source,
    std::uint32_t object_id,
    bool initial,
    std::uint32_t selected_layout,
    std::uint32_t reference_layout,
    std::uint32_t selected_speaker_count,
    UhdFrameHeader& header) noexcept {
    // dtsx2ExtractREObjectMD, Sony/homatic implementation.  This parser only
    // records the representation's layout/value tuples; presentation and
    // interactive controls remain outside the P2 audio-bus scope.
    std::uint32_t value = 0U;
    if (!initial && (!source.read(1U, value) || value == 0U)) {
        return true;
    }
    if (reference_layout != 0U && !source.skip(1U)) {
        return false;
    }
    if (!source.read(2U, value)) {
        return false;
    }
    const std::uint32_t layout_count = value + 1U;
    if (layout_count > 4U) {
        return false;
    }
    struct Candidate final {
        bool selected = false;
        std::uint32_t layout = 0U;
        std::uint32_t speakers = 0U;
    };
    Candidate best{};
    std::uint32_t selected_channel_mask_flag = 0U;
    for (std::uint32_t index = 0U; index < layout_count; ++index) {
        std::uint32_t channel_mask_flag = 0U;
        std::uint32_t layout = 0U;
        if (!source.read(1U, channel_mask_flag)
            || !read_native_varlen_17(source, layout)
            || !source.read(1U, value)) {
            return false;
        }
        selected_channel_mask_flag = channel_mask_flag;
        const std::uint32_t speakers = native_layout_speaker_count(layout);
        const bool compatible = value != 0U
            ? (layout & ~selected_layout) == 0U
            : layout == selected_layout;
        // Native selection prefers a compatible layout, but before one is
        // found it retains the candidate with the largest speaker count.
        // This is the `v15`/`*a1 >= NumSpeakersInLayout` branch in
        // dtsx2ExtractREObjectMD, not a simple first-match search.
        if ((compatible && !best.selected)
            || (compatible && best.selected
                && best.layout != selected_layout
                && best.speakers < speakers)
            || (!compatible && !best.selected
                && best.speakers < speakers)) {
            best = {true, layout, speakers};
        }
        (void)channel_mask_flag;
    }
    UhdFrameHeader::ThreeDObjectMetadata record;
    record.object_id = object_id;
    record.layout_mask = best.selected ? best.layout : 0U;
    const std::uint32_t speaker_count = best.selected
        ? best.speakers : selected_speaker_count;
    for (std::uint32_t layout_index = 0U;
         layout_index < selected_speaker_count;
         ++layout_index) {
        if (speaker_count == 0U) {
            break;
        }
        std::uint32_t speaker_mask = 0U;
        if (!source.read(speaker_count, speaker_mask)) {
            return false;
        }
        for (std::uint32_t speaker = 0U; speaker < speaker_count; ++speaker) {
            if ((speaker_mask & (1U << speaker)) == 0U) {
                continue;
            }
            // `Bits` in the native function selects the optional per-speaker
            // update flag: tcl/homatic read one extra bit only when it is set.
            if ((selected_channel_mask_flag != 0U && !source.read(1U, value))
                || !source.read(6U, value)
                || value > 60U) {
                return false;
            }
            record.speaker_indices.push_back(value);
        }
    }
    header.three_d_object_metadata.push_back(std::move(record));
    return true;
}

void parse_mde_object_list(
    const std::vector<std::uint8_t>& payload,
    bool full_channel_based_mix,
    UhdFrameParserState& parser_state,
    UhdFrameHeader& header) noexcept {
    if (payload.size() < 2U || payload.front() != 1U) {
        return;
    }
    std::vector<std::uint8_t> body(
        payload.begin() + 1, payload.end());
    ByteBitReader source(body);
    std::uint32_t presentation = 0U;
    if (!read_variable_length(
            source, std::array<std::uint8_t, 4>{0U, 2U, 4U, 4U},
            presentation)
        || presentation > 255U) {
        return;
    }
    header.metadata_audio_presentation_index = presentation;
    if (full_channel_based_mix) {
        header.metadata_object_ids.assign(1U, 256U);
        return;
    }
    std::uint32_t count = 0U;
    if (!read_variable_length(
            source, std::array<std::uint8_t, 4>{3U, 4U, 6U, 8U}, count)
        || count > 256U) {
        header.metadata_object_ids.clear();
        return;
    }
    std::vector<std::uint32_t> ids;
    ids.reserve(count);
    for (std::uint32_t index = 0U; index < count; ++index) {
        std::uint32_t wide = 0U;
        std::uint32_t object_id = 0U;
        if (!source.read(1U, wide)
            || !source.read(wide != 0U ? 8U : 4U, object_id)) {
            header.metadata_object_ids.clear();
            return;
        }
        ids.push_back(object_id);
    }
    // MDE may be split over several FTOC metadata chunks.  Native object
    // extraction walks every chunk and keeps the object table; replacing it
    // with the last payload loses IDs before the object-decoder dispatch.
    for (const std::uint32_t object_id : ids) {
        if (std::find(header.metadata_object_ids.begin(),
                      header.metadata_object_ids.end(), object_id)
            == header.metadata_object_ids.end()) {
            header.metadata_object_ids.push_back(object_id);
        }
    }
    // The native MDE parser can expose more than one metadata payload in a
    // frame.  Keep the 3D-representation indication at frame scope; clearing
    // it here would let a later channel-based payload erase an already seen
    // dtsx2ExtractREObjectMD dispatch.
    if (full_channel_based_mix) {
        return;
    }
    const bool selectable = presentation
        < parser_state.presentation_selectable.size()
        && parser_state.presentation_selectable[presentation];
    if (selectable) {
        for (std::uint32_t index = 0U; index < 4U; ++index) {
            std::uint32_t present = 0U;
            if (!source.read(1U, present)
                || (present != 0U && !source.skip(5U))) {
                return;
            }
        }
        std::uint32_t multi_frame = 0U;
        if (!source.read(1U, multi_frame) || multi_frame != 0U) {
            return;
        }
    }
    std::uint32_t studio_present = 0U;
    if (!source.read(1U, studio_present)
        || (studio_present != 0U && !source.skip(11U))) {
        return;
    }
    for (const std::uint32_t object_id : header.metadata_object_ids) {
        // Native dtsx2ExtractMetadataForObjects resets the transient object
        // record and calls dtsx2ExtractObjectMetadata with a3=1 for every
        // ObjectID on every metadata frame.  There is no persistent
        // ObjStaticFlag bit in this dispatch; skipping IDs after the first
        // frame would leave dynamic metadata unread and shift the cursor.
        // Table 7-22 has ObjActiveFlag before ObjRepresTypeIndex for every
        // object except reserved ID 256. Native consumes this bit even when
        // the object is inactive.
        if (object_id != 256U && !source.skip(1U)) {
            header.metadata_object_associations.clear();
            return;
        }
        std::uint32_t representation = 0U;
        if (!source.read(3U, representation)) {
            header.metadata_object_associations.clear();
            return;
        }
        if (representation > 3U) {
            // Homatic's dtsx2ExtractObjectMetadata dispatches representation
            // types 4/5 to dtsx2ExtractREObjectMD.  That branch has a
            // different bit grammar and does not contain ObjAudioChunkIndex
            // or ObjNaviWithinACIndex.  Do not consume it as a channel-mask
            // record: doing so shifts the cursor and fabricates associations.
            header.has_3d_object_metadata = true;
            if (!parse_re_object_metadata(
                    source, object_id, true,
                    header.speaker_activity_mask,
                    header.speaker_activity_mask,
                    native_layout_speaker_count(
                        header.speaker_activity_mask),
                    header)) {
                header.three_d_object_metadata.clear();
                header.metadata_object_associations.clear();
                return;
            }
            // Native dtsx2ExtractMetadataForObjects continues with the next
            // ObjectID after the distinct RE-object grammar.  Do not return
            // from the enclosing object-list loop here.
            continue;
        }
        std::uint32_t type_description_present = 0U;
        if (!source.skip(3U)
            || !source.read(1U, type_description_present)) {
            header.metadata_object_associations.clear();
            return;
        }
        if (type_description_present != 0U) {
            std::uint32_t wide = 0U;
            if (!source.read(1U, wide)
                || !source.skip(wide != 0U ? 3U : 5U)) {
                header.metadata_object_associations.clear();
                return;
            }
        }
        std::uint32_t audio_chunk = 0U;
        std::uint32_t navigation = 0U;
        if (!read_variable_length(
                source, std::array<std::uint8_t, 4>{1U, 4U, 4U, 8U},
                audio_chunk)
            || !read_variable_length(
                source, std::array<std::uint8_t, 4>{3U, 3U, 4U, 8U},
                navigation)) {
            header.metadata_object_associations.clear();
            return;
        }
        // Table 7-22 optional fields for channel-mask representations.
        // Object3DMetaDataPresent is false for representation types 0..3.
        std::uint32_t present = 0U;
        if (!source.read(1U, present)
            || (present != 0U && !source.skip(8U))) {
            header.metadata_object_associations.clear();
            return;
        }
        if (!source.read(1U, present)) {
            header.metadata_object_associations.clear();
            return;
        }
        if (present != 0U && parser_state.interact_obj_limits_present) {
            std::uint32_t limits = 0U;
            if (!source.read(1U, limits)
                || (limits != 0U && !source.skip(5U))) {
                header.metadata_object_associations.clear();
                return;
            }
        }
        // Table 7-26: channel-mask parameters are part of the native object
        // record used by ACEW registration.  They follow the association
        // fields; leaving them unread shifts the next object record.
        std::uint32_t channel_layout_index = 0U;
        std::uint32_t channel_activity_mask = 0U;
        if (representation == 3U) { // REP_TYPE_BINAURAL
            channel_layout_index = 1U;
        } else if (!source.read(4U, channel_layout_index)) {
            header.metadata_object_associations.clear();
            return;
        }
        static constexpr std::array<std::uint32_t, 14> kChannelMaskTable{
            0x000001U, 0x000002U, 0x000006U, 0x00000FU,
            0x00001FU, 0x00084BU, 0x00002FU, 0x00802FU,
            0x00486BU, 0x00886BU, 0x03FBFBU, 0x000003U,
            0x000007U, 0x000843U};
        if (channel_layout_index < kChannelMaskTable.size()) {
            channel_activity_mask =
                kChannelMaskTable[channel_layout_index];
        } else {
            const std::uint32_t mask_bits =
                16U << (channel_layout_index - 14U);
            if (mask_bits > 32U
                || !source.read(mask_bits, channel_activity_mask)) {
                header.metadata_object_associations.clear();
                return;
            }
        }
        header.metadata_object_associations.push_back({
            object_id, representation, audio_chunk, navigation,
            channel_layout_index, channel_activity_mask,
            speaker_count_from_activity_mask(channel_activity_mask),
            channel_activity_mask,
            // dtsx2ExtractObjectMetadata initializes native object state
            // field [7] to 7 only for reserved ObjectID 256; ordinary object
            // records retain zero.  This is the first ACEW registration
            // descriptor word and must not be synthesized from the layout.
            {object_id == 256U ? 7U : 0U,
             speaker_count_from_activity_mask(channel_activity_mask),
             channel_activity_mask}});
    }
}

} // namespace

bool parse_uhd_full_mix_metadata(
    const std::vector<std::uint8_t>& frame,
    UhdFrameHeader& header) noexcept {
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
    // tcl_lib_dtsX.so.c: dword_DA104 (DTSXP2 layout index table), including
    // private 5.1.4/IMAX masks 0x802f and 0x843.
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
        if (next_state.full_channel_based_mix) {
            next_state.major_version = 2U;
            next_state.minor_version = 0U;
        } else {
            std::uint32_t short_revision = 0U;
            if (!source.read(1U, short_revision)) {
                return UhdHeaderParseResult::Invalid;
            }
            const std::uint32_t half_width =
                short_revision != 0U ? 3U : 6U;
            if (!source.read(half_width * 2U, value)) {
                return UhdHeaderParseResult::Invalid;
            }
            next_state.major_version = (value >> half_width) + 2U;
            next_state.minor_version =
                value & ((1U << half_width) - 1U);
        }
        if (!source.read(2U, value) || value >= 3U) {
            return UhdHeaderParseResult::Invalid;
        }
        // tcl_lib_dtsX.so.c: dword_DA3D8 / DTSXP2_BaseDurationTable.
        // The native P2 scanner uses the same 512/480/384 base durations;
        // keep this table independent from the DTSX2 (non-P2) varlen tables.
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
        if (!next_state.full_channel_based_mix) {
            // Table 6-11: Reserved followed by InteractObjLimitsPresent.
            if (!source.skip(1U) || !source.read(1U, value)) {
                return UhdHeaderParseResult::Invalid;
            }
            next_state.interact_obj_limits_present = value != 0U;
        } else {
            next_state.interact_obj_limits_present = false;
        }
        next_state.audio_chunks.clear();
        next_state.initialized = true;
    } else if (!next_state.initialized) {
        return UhdHeaderParseResult::Invalid;
    }

    header.full_channel_based_mix =
        next_state.full_channel_based_mix;
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

    if (!read_presentation_navigation(
            source,
            header.sync_frame,
            header.full_channel_based_mix,
            next_state)) {
        return UhdHeaderParseResult::Invalid;
    }
    std::uint64_t payload_size = header.ftoc_size;
    if (!read_chunk_navigation(
            source,
            header.sync_frame,
            header.full_channel_based_mix,
            header.ftoc_size,
            next_state,
            header,
            payload_size)
        || !read_pbr_smoothing(
            source, header.sync_frame, next_state)) {
        return UhdHeaderParseResult::Invalid;
    }

    const bool crc_required =
        !header.full_channel_based_mix || header.sync_frame;
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
    header.metadata_payloads.clear();
    header.metadata_payloads.reserve(header.metadata_chunks.size());
    header.metadata_object_ids.clear();
    header.has_3d_object_metadata = false;
    header.three_d_object_metadata.clear();
    header.metadata_object_associations.clear();
    header.unresolved_object_audio_chunk_indices.clear();
    header.metadata_audio_presentation_index = 0U;
    for (const UhdChunk& chunk : header.metadata_chunks) {
        if (chunk.offset > bytes.size()
            || chunk.size > bytes.size() - chunk.offset) {
            // FTOC parsing is intentionally complete before the payload is
            // assembled (FrameAssembler needs frame_size at this point).
            // Keep the optional raw payload empty and do not delay the
            // header result; the final full-frame parse will capture it.
            header.metadata_payloads.clear();
            break;
        }
        header.metadata_payloads.emplace_back(
            bytes.begin() + static_cast<std::ptrdiff_t>(chunk.offset),
            bytes.begin() + static_cast<std::ptrdiff_t>(
                chunk.offset + chunk.size));
        parse_mde_object_list(
            header.metadata_payloads.back(),
            header.full_channel_based_mix,
            next_state,
            header);
    }
    for (const auto& association : header.metadata_object_associations) {
        const bool chunk_present = std::any_of(
            header.audio_chunks.begin(), header.audio_chunks.end(),
            [&association](const UhdChunk& chunk) {
                return chunk.index == association.audio_chunk_index;
            });
        if (!chunk_present) {
            header.unresolved_object_audio_chunk_indices.push_back(
                association.audio_chunk_index);
        }
    }
    state = std::move(next_state);
    return UhdHeaderParseResult::Complete;
}

} // namespace dtsx
