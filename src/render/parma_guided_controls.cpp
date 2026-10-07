#include "render/parma_guided_controls.hpp"

#include "dtsx/speaker_mask.hpp"
#include "dtsx/xll_frame_decoder.hpp"
#include "render/parma_layout.hpp"

#include <array>

namespace dtsx_decode {
namespace {

constexpr std::array<std::uint16_t, 64U> kDtsxDownmixCoefficientQ15 = {{
    0U, 35U, 37U, 39U, 41U, 44U, 46U, 49U,
    52U, 55U, 58U, 62U, 65U, 69U, 73U, 78U,
    82U, 87U, 92U, 98U, 104U, 110U, 116U, 123U,
    130U, 138U, 146U, 155U, 164U, 174U, 184U, 195U,
    207U, 219U, 232U, 246U, 260U, 276U, 292U, 309U,
    328U, 347U, 368U, 389U, 413U, 437U, 463U, 490U,
    519U, 550U, 583U, 617U, 654U, 693U, 734U, 777U,
    823U, 872U, 924U, 978U, 1036U, 1066U, 1098U, 1130U,
}};

std::int32_t guided_added_output_ordinal(
    std::uint32_t downmix_channel_mask,
    std::uint32_t upmix_channel_mask,
    std::uint32_t speaker_mask) noexcept {
    const std::int32_t slot =
        parma_channel_slot_from_speaker_mask(speaker_mask);
    if (slot < 0 || slot >= 32) {
        return -1;
    }
    const std::uint32_t added_mask =
        parma_disable_lfe_channels(upmix_channel_mask)
        & ~parma_disable_lfe_channels(downmix_channel_mask);
    const std::uint32_t bit = 1U << static_cast<std::uint32_t>(slot);
    if ((added_mask & bit) == 0U) {
        return -1;
    }
    std::int32_t ordinal = 0;
    for (std::uint32_t index = 0U;
         index < static_cast<std::uint32_t>(slot);
         ++index) {
        ordinal += static_cast<std::int32_t>((added_mask >> index) & 1U);
    }
    return ordinal;
}

} // namespace

bool derive_parma_guided_controls(
    const dtsx::CombinedMixMetadata& metadata,
    std::uint32_t decoded_bed_speaker_activity_mask,
    std::uint32_t requested_output_speaker_activity_mask,
    ParmaGuidedControls& controls) noexcept {
    controls = {};
    // DTS_ParmaDec_SetGuided accepts only the exact matrix whose reference
    // layout is the decoded bed and whose output layout is the presentation.
    if (metadata.reference_speaker_activity_mask
            != decoded_bed_speaker_activity_mask
        || metadata.output_speaker_activity_mask
            != requested_output_speaker_activity_mask
        || metadata.added_speaker_activity_mask
            != (requested_output_speaker_activity_mask
                & ~decoded_bed_speaker_activity_mask)) {
        return false;
    }
    const std::vector<std::uint32_t> output_speakers =
        dtsx::expand_speaker_activity_mask(
            requested_output_speaker_activity_mask);
    if (output_speakers.empty()
        || metadata.reference_speaker_masks.empty()
        || metadata.added_speaker_masks.empty()
        || metadata.downmix_coefficient_codes.size()
            != metadata.reference_speaker_masks.size()
                * metadata.added_speaker_masks.size()) {
        return false;
    }

    controls.downmix_speaker_activity_mask =
        decoded_bed_speaker_activity_mask;
    controls.upmix_speaker_activity_mask =
        requested_output_speaker_activity_mask;
    controls.downmix_channel_mask =
        parma_channel_mask_from_speaker_activity(
            decoded_bed_speaker_activity_mask);
    controls.upmix_channel_mask =
        parma_channel_mask_from_speaker_activity(
            requested_output_speaker_activity_mask);
    controls.downmix_main_channel_count =
        parma_main_channel_count(controls.downmix_channel_mask);
    controls.upmix_main_channel_count =
        parma_main_channel_count(controls.upmix_channel_mask);
    if (controls.downmix_main_channel_count == 0U
        || controls.upmix_main_channel_count == 0U
        || controls.downmix_main_channel_count
            > controls.upmix_main_channel_count
        || controls.downmix_main_channel_count > 11U
        || controls.upmix_main_channel_count > 11U) {
        controls = {};
        return false;
    }

    std::uint32_t added_main_count = 0U;
    for (std::size_t added = 0U;
         added < metadata.added_speaker_masks.size();
         ++added) {
        const std::int32_t added_ordinal = guided_added_output_ordinal(
            controls.downmix_channel_mask,
            controls.upmix_channel_mask,
            metadata.added_speaker_masks[added]);
        // A combined-mix matrix can describe LFE, but DTS_ParmaDec masks LFE
        // before SetGuided builds its pair/triplet/quad topology.
        if (parma_channel_slot_from_speaker_mask(
                metadata.added_speaker_masks[added]) == -2) {
            continue;
        }
        if (added_ordinal < 0) {
            controls = {};
            return false;
        }
        const std::uint32_t row = static_cast<std::uint32_t>(
            added_ordinal);
        if (row >= 11U) {
            controls = {};
            return false;
        }
        ++added_main_count;
        for (std::size_t reference = 0U;
             reference < metadata.reference_speaker_masks.size();
             ++reference) {
            const std::int32_t reference_ordinal =
                parma_main_channel_ordinal(
                    controls.downmix_channel_mask,
                    metadata.reference_speaker_masks[reference]);
            if (parma_channel_slot_from_speaker_mask(
                    metadata.reference_speaker_masks[reference]) == -2) {
                continue;
            }
            if (reference_ordinal < 0
                || static_cast<std::uint32_t>(reference_ordinal) >= 11U) {
                controls = {};
                return false;
            }
            const std::uint8_t code = metadata.downmix_coefficient_codes[
                reference * metadata.added_speaker_masks.size() + added];
            // The parser retained the six-bit dtsx_dmixCoeffTable index.
            // UpdateParmaCustomCoeffs receives the table's unsigned Q15
            // value and applies the exact 1 / 32768 conversion.
            controls.custom_encoder_coefficients[row][
                static_cast<std::size_t>(reference_ordinal)]
                = static_cast<float>(kDtsxDownmixCoefficientQ15[code])
                    / 32768.0F;
        }
    }
    controls.added_output_count = added_main_count;
    if (added_main_count == 0U
        || controls.downmix_main_channel_count + added_main_count
            != controls.upmix_main_channel_count) {
        controls = {};
        return false;
    }
    return true;
}

} // namespace dtsx_decode
