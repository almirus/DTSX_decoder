#include "dtsx/preliminary_metadata.hpp"

#include "dtsx/speaker_mask.hpp"

#include <algorithm>

namespace dtsx {
namespace {

constexpr std::uint8_t kSpeakerActivityChannelCounts[20] = {
    1U, 2U, 2U, 1U, 1U, 2U, 2U, 1U, 1U, 2U,
    2U, 2U, 1U, 2U, 1U, 2U, 1U, 2U, 2U, 2U,
};

std::uint8_t count_speaker_activity_channels(
    std::uint32_t mask) noexcept {
    std::uint8_t count = 0;
    for (std::uint32_t index = 0U;
         index < 20U;
         ++index) {
        if ((mask & (1U << index)) != 0U) {
            count = static_cast<std::uint8_t>(
                count + kSpeakerActivityChannelCounts[index]);
        }
    }
    return count;
}

} // namespace

bool unpack_combined_mix_metadata(
    bitstream::Cursor& source,
    std::uint8_t association_mode,
    std::uint32_t fallback_reference_speaker_activity_mask,
    CombinedMixMetadata& metadata) noexcept {
    metadata = {};
    const PreliminaryMetadataHeader preliminary =
        unpack_preliminary_metadata_header(source, association_mode);
    if (preliminary.chunk_id < 2U
        || preliminary.chunk_id > 4U) {
        return false;
    }

    (void)source.extract_unsigned(1U);
    (void)source.extract_unsigned(4U);
    (void)source.extract_unsigned(4U);
    if (source.extract_unsigned(1U) != 0U) {
        (void)source.extract_unsigned(1U);
    }

    const bool explicit_reference =
        source.extract_unsigned(1U) != 0U;
    if (explicit_reference) {
        (void)source.extract_unsigned(2U);
        (void)source.extract_unsigned(3U);
        const std::uint32_t mask_words =
            source.extract_unsigned(3U) + 1U;
        metadata.reference_speaker_activity_mask =
            source.extract_unsigned(4U * mask_words);
    } else {
        (void)source.extract_unsigned(4U);
        metadata.reference_speaker_activity_mask =
            fallback_reference_speaker_activity_mask;
    }

    if (source.extract_unsigned(1U) != 0U) {
        (void)source.extract_unsigned(6U);
        if (source.extract_unsigned(1U) != 0U) {
            (void)source.extract_unsigned(6U);
        }
    }
    const std::uint32_t output_mask_words =
        source.extract_unsigned(3U) + 1U;
    metadata.output_speaker_activity_mask =
        source.extract_unsigned(4U * output_mask_words);
    metadata.added_speaker_activity_mask =
        metadata.output_speaker_activity_mask
        & ~metadata.reference_speaker_activity_mask;

    bool first_gain_present = false;
    if (preliminary.chunk_id <= 3U) {
        (void)source.extract_unsigned(5U);
        (void)source.extract_unsigned(6U);
        first_gain_present =
            source.extract_unsigned(1U) != 0U;
        const bool gain_present =
            source.extract_unsigned(1U) != 0U;
        if (gain_present) {
            (void)source.extract_unsigned(6U);
            if (first_gain_present
                && source.extract_unsigned(1U) != 0U) {
                (void)source.extract_unsigned(6U);
            }
        }
    } else {
        (void)source.extract_unsigned(4U);
    }

    metadata.reference_speaker_masks =
        expand_speaker_activity_mask(
            metadata.reference_speaker_activity_mask);
    metadata.added_speaker_masks =
        expand_speaker_activity_mask(
            metadata.added_speaker_activity_mask);
    if (metadata.added_speaker_masks.empty()
        || metadata.reference_speaker_masks.empty()
        || metadata.reference_speaker_masks.size() > 32U
        || metadata.added_speaker_masks.size() > 32U) {
        return false;
    }

    const bool downmix_present =
        source.extract_unsigned(1U) != 0U;
    const bool coefficients_present =
        downmix_present
        || source.extract_unsigned(1U) != 0U;
    if (!coefficients_present) {
        return false;
    }
    (void)source.extract_unsigned(6U);
    const std::uint32_t common_gain_selector =
        source.extract_unsigned(first_gain_present ? 1U : 3U);
    if (common_gain_selector != 0U) {
        const std::uint32_t common_gain_count =
            1U << (common_gain_selector - 1U);
        for (std::uint32_t gain = 0U;
             gain < common_gain_count;
             ++gain) {
            (void)source.extract_unsigned(6U);
        }
    }

    const std::size_t reference_count =
        metadata.reference_speaker_masks.size();
    const std::size_t added_count =
        metadata.added_speaker_masks.size();
    metadata.lossy_reference_speaker_masks.assign(
        added_count, 0U);
    metadata.downmix_coefficient_codes.assign(
        reference_count * added_count, 0U);
    for (std::size_t added = 0U;
         added < added_count;
         ++added) {
        const std::uint32_t update_mask =
            source.extract_unsigned(
                static_cast<std::uint32_t>(
                    reference_count));
        for (std::size_t reference = 0U;
             reference < reference_count;
             ++reference) {
            if ((update_mask & (1U << reference)) == 0U) {
                continue;
            }
            const std::uint8_t code =
                static_cast<std::uint8_t>(
                    source.extract_unsigned(6U));
            if (code > 61U) {
                metadata = {};
                return false;
            }
            metadata.downmix_coefficient_codes[
                reference * added_count + added] = code;
            if (metadata.lossy_reference_speaker_masks[added]
                == 0U) {
                metadata.lossy_reference_speaker_masks[added] =
                    metadata.reference_speaker_masks[reference];
            }
        }
        if (metadata.lossy_reference_speaker_masks[added]
            == 0U) {
            metadata = {};
            return false;
        }
    }
    if (!source.valid()) {
        metadata = {};
        return false;
    }
    return true;
}

PreliminaryMetadataHeader unpack_preliminary_metadata_header(
    bitstream::Cursor& source, std::uint8_t association_mode) noexcept {
    PreliminaryMetadataHeader result;
    result.chunk_id = static_cast<std::uint8_t>(source.extract_unsigned(8U));
    result.raw_flags = static_cast<std::uint8_t>(source.extract_unsigned(8U));
    result.primary = true;
    std::uint8_t selector_bits = 3U;
    switch (association_mode) {
    case 1U:
        selector_bits = 1U;
        result.short_form = (result.raw_flags & 0x80U) != 0U;
        break;
    case 2U:
        selector_bits = 2U;
        result.alternate_association = (result.raw_flags & 0x80U) != 0U;
        result.short_form = (result.raw_flags & 0x40U) != 0U;
        break;
    case 3U:
    case 4U:
        selector_bits = 3U;
        result.alternate_association = (result.raw_flags >> 6U) != 0U;
        result.short_form = (result.raw_flags & 0x20U) != 0U;
        break;
    default:
        // The default native form stores raw_flags >> 5 and treats any
        // nonzero value as alternate association.
        result.alternate_association = (result.raw_flags >> 5U) != 0U;
        break;
    }
    if ((result.chunk_id & 0x80U) == 0U) {
        result.selector = static_cast<std::uint8_t>(
            result.raw_flags & (0xFFU >> selector_bits));
        return result;
    }
    bool associated = true;
    if (!result.alternate_association) {
        associated = (result.raw_flags
            & (1U << (7U - selector_bits))) != 0U;
        result.primary = associated;
        if (associated) {
            ++selector_bits;
        }
    }
    result.association_type = static_cast<std::uint8_t>(
        (result.raw_flags & (15U << (4U - selector_bits)))
        >> (4U - selector_bits));
    result.association_index = result.association_type;
    return result;
}

bool unpack_audio_presentation_metadata(
    bitstream::Cursor& source,
    const PreliminaryMetadataHeader& preliminary,
    bool short_form,
    AudioPresentationMetadata& metadata) noexcept {
    if (preliminary.chunk_id != 241U || !preliminary.primary) {
        return false;
    }
    metadata = {};
    metadata.presentation_index = static_cast<std::uint8_t>(
        source.extract_unsigned(4U));
    metadata.selectable = source.extract_unsigned(1U) != 0U;
    metadata.presentation_count = static_cast<std::uint8_t>(
        source.extract_unsigned(4U) + 1U);
    (void)source.extract_unsigned(1U);
    if (source.extract_unsigned(1U) != 0U) {
        source.fast_forward(1);
    }
    if (short_form) {
        (void)source.extract_unsigned(4U);
        return source.valid();
    }
    metadata.metadata_present = source.extract_unsigned(1U) != 0U;
    if (!metadata.metadata_present) {
        return source.valid();
    }
    metadata.object_count = static_cast<std::uint8_t>(
        source.extract_unsigned(4U) + 1U);
    metadata.object_count_extended = source.extract_unsigned(1U) != 0U;
    bool group_metadata = false;
    bool channel_metadata = false;
    if (metadata.object_count_extended) {
        metadata.object_count_extension = static_cast<std::uint8_t>(
            source.extract_unsigned(4U));
    } else {
        if (source.extract_unsigned(1U) != 0U) {
            source.fast_forward(11);
        }
        channel_metadata = source.extract_unsigned(1U) != 0U;
        metadata.channel_mask_present = channel_metadata;
        if (channel_metadata) {
            const bool compact_count = source.extract_unsigned(1U) != 0U;
            (void)source.extract_unsigned(2U);
            (void)source.extract_unsigned(compact_count ? 3U : 4U);
            const std::uint32_t mask_words = source.extract_unsigned(3U) + 1U;
            metadata.speaker_activity_mask = source.extract_unsigned(
                4U * mask_words);
            metadata.speaker_count =
                count_speaker_activity_channels(
                    metadata.speaker_activity_mask);
            metadata.alternative_render_gain_present =
                source.extract_unsigned(1U) != 0U;
            if (metadata.alternative_render_gain_present) {
                metadata.alternative_render_gain_code =
                    static_cast<std::uint8_t>(
                        source.extract_unsigned(6U));
            }
            if (source.extract_unsigned(1U) != 0U) {
                metadata.render_gain_code =
                    static_cast<std::uint8_t>(
                        source.extract_unsigned(6U));
            }
        } else if (source.extract_unsigned(1U) != 0U) {
            const std::uint32_t mask_words = source.extract_unsigned(3U) + 1U;
            metadata.speaker_activity_mask = source.extract_unsigned(
                4U * mask_words);
            metadata.speaker_count =
                count_speaker_activity_channels(
                    metadata.speaker_activity_mask);
        }
        group_metadata = source.extract_unsigned(1U) != 0U;
        metadata.object_groups_present = group_metadata;
        if (group_metadata) {
            metadata.object_group_count = static_cast<std::uint8_t>(
                source.extract_unsigned(3U) + 1U);
            const std::uint32_t relation_count = channel_metadata
                ? source.extract_unsigned(4U) + 1U : 0U;
            for (std::uint32_t index = 0;
                 index < metadata.object_group_count;
                 ++index) {
                source.fast_forward(3);
                const std::uint32_t relation_bits = channel_metadata
                    ? source.extract_unsigned(3U) : 0U;
                source.fast_forward(static_cast<std::int32_t>(relation_bits));
                source.fast_forward(static_cast<std::int32_t>(relation_bits));
                source.fast_forward(static_cast<std::int32_t>(
                    relation_count * relation_bits));
            }
        }
        if (channel_metadata && source.extract_unsigned(1U) != 0U) {
            source.fast_forward(6);
        }
    }

    const std::uint32_t index_bits_code = source.extract_unsigned(2U);
    const bool shared_indices = source.extract_unsigned(1U) != 0U;
    (void)source.extract_unsigned(1U);
    const std::uint32_t short_index_bits = index_bits_code + 2U;
    const std::uint32_t count_bits = index_bits_code + 1U;
    const std::uint32_t long_index_bits = index_bits_code + 3U;
    metadata.sequential_waveform_offsets =
        metadata.channel_mask_present;
    metadata.waveform_offset_bits =
        static_cast<std::uint8_t>(long_index_bits);
    metadata.objects.reserve(metadata.object_count);
    for (std::uint32_t index = 0; index < metadata.object_count; ++index) {
        ObjectMetadataBlock object;
        object.metadata_present = source.extract_unsigned(1U) != 0U;
        if (shared_indices) {
            object.object_id_available = true;
            object.object_id =
                source.extract_unsigned(2U * short_index_bits);
        }
        if (!shared_indices || object.metadata_present) {
            object.waveform_id_available = true;
            object.waveform_id = static_cast<std::uint8_t>(
                source.extract_unsigned(short_index_bits));
            object.waveform_channel_base_offset =
                source.extract_unsigned(long_index_bits);
        }
        if (!object.metadata_present) {
            metadata.objects.push_back(object);
            continue;
        }
        object.preamble = unpack_object_metadata_preamble(
            source, static_cast<std::uint8_t>(count_bits));
        if (group_metadata && source.extract_unsigned(1U) != 0U) {
            object.group_assignment_present = true;
            object.group_assignment =
                static_cast<std::uint8_t>(source.extract_unsigned(3U));
            object.group_assignment_coherent =
                source.extract_unsigned(1U) != 0U;
        }
        metadata.objects.push_back(object);
    }
    return source.valid();
}

bool unpack_audio_presentation_object_bodies(
    bitstream::Cursor& source,
    AudioPresentationMetadata& metadata,
    const std::vector<bool>* waveform_decoder_available) {
    if (waveform_decoder_available != nullptr
        && waveform_decoder_available->size()
            != metadata.objects.size()) {
        return false;
    }
    if (!unpack_object_metadata_bodies(
            source,
            metadata.object_groups_present,
            metadata.speaker_count,
            metadata.sequential_waveform_offsets,
            metadata.waveform_offset_bits,
            metadata.objects)) {
        return false;
    }

    metadata.alternative_rendering_metadata_present =
        source.extract_unsigned(1U) != 0U;
    if (!metadata.alternative_rendering_metadata_present) {
        return source.valid();
    }
    for (std::size_t object_index = 0U;
         object_index < metadata.objects.size();
         ++object_index) {
        ObjectMetadataBlock& object =
            metadata.objects[object_index];
        if (waveform_decoder_available != nullptr
            && !(*waveform_decoder_available)[object_index]) {
            continue;
        }
        object.alternative_rendering_present =
            source.extract_unsigned(1U) != 0U;
        if (!object.alternative_rendering_present
            || !object.metadata_present) {
            continue;
        }
        const std::uint32_t render_set_count =
            source.extract_unsigned(2U) + 1U;
        object.alternative_speaker_mask_bits =
            static_cast<std::uint8_t>(
                source.extract_unsigned(5U));
        object.alternative_render_sets.resize(
            render_set_count);
        for (AlternativeRenderSet& render_set :
             object.alternative_render_sets) {
            render_set.speaker_activity_mask =
                source.extract_unsigned(
                    object.alternative_speaker_mask_bits);
            const std::uint32_t speaker_count =
                count_speaker_activity_channels(
                    render_set.speaker_activity_mask);
            render_set.subset_allowed =
                source.extract_unsigned(1U) != 0U;
            render_set.waveform_gain_codes.assign(
                object.preamble.waveform_count,
                std::vector<std::uint8_t>(
                    speaker_count, 0U));
            for (std::vector<std::uint8_t>& waveform :
                 render_set.waveform_gain_codes) {
                const std::uint32_t update_mask =
                    source.extract_unsigned(speaker_count);
                for (std::uint32_t speaker = 0U;
                     speaker < speaker_count;
                     ++speaker) {
                    if ((update_mask & (1U << speaker)) != 0U) {
                        waveform[speaker] =
                            static_cast<std::uint8_t>(
                                source.extract_unsigned(6U));
                    }
                }
            }
        }
    }
    return source.valid();
}

} // namespace dtsx
