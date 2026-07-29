#include "dtsx/object_metadata_updates.hpp"

#include <utility>

namespace dtsx {
namespace {

std::vector<SixBitUpdate> unpack_update_values(bitstream::Cursor& source,
                                               std::uint32_t mask,
                                               std::uint8_t value_count) {
    std::vector<SixBitUpdate> result;
    for (std::uint32_t index = 0; index < value_count; ++index) {
        if ((mask & (1U << index)) == 0U) {
            continue;
        }
        result.push_back(
            {static_cast<std::uint8_t>(index),
             static_cast<std::uint8_t>(source.extract_unsigned(6U))});
    }
    return result;
}

} // namespace

std::vector<WaveformMetadataUpdate> unpack_waveform_metadata_updates(
    bitstream::Cursor& source,
    std::uint8_t waveform_count,
    std::uint8_t update_value_count) {
    // libdtsx.so: dtsParseExSSChunks, 0xa1440..0xa1598.
    std::vector<WaveformMetadataUpdate> result;
    result.reserve(waveform_count);
    for (std::uint32_t waveform = 0; waveform < waveform_count; ++waveform) {
        WaveformMetadataUpdate update;
        update.first_set_present = source.extract_unsigned(1U) != 0U;
        update.second_set_present = source.extract_unsigned(1U) != 0U;

        if (update.first_set_present) {
            update.first_mask = source.extract_unsigned(update_value_count);
            update.first_values =
                unpack_update_values(source, update.first_mask, update_value_count);
        }
        if (update.second_set_present) {
            update.second_mask = source.extract_unsigned(update_value_count);
            update.second_values =
                unpack_update_values(source, update.second_mask, update_value_count);
        }
        result.push_back(std::move(update));
    }
    return result;
}

ModeThreeMetadata unpack_mode_three_metadata(bitstream::Cursor& source,
                                             std::uint8_t waveform_count,
                                             std::uint8_t update_value_count) {
    // libdtsx.so: dtsParseExSSChunks, 0xa12c0..0xa2168.
    ModeThreeMetadata result;
    result.per_waveform_gain_present = source.extract_unsigned(1U) != 0U;
    if (update_value_count == 0U) {
        result.fallback_gain_present = source.extract_unsigned(1U) != 0U;
        // dtsParseExSSChunks gates the six-bit fallback coefficient by the
        // per-waveform flag (0xA1Fxx), even though it always reads the
        // fallback-presence bit first.
        if (result.per_waveform_gain_present) {
            result.fallback_gain_code =
                static_cast<std::uint8_t>(source.extract_unsigned(6U));
        }
        return result;
    }

    std::uint32_t selected_index_bits = 0;
    if (update_value_count != 1U) {
        for (std::uint32_t value = 1U; value < update_value_count; value *= 2U) {
            ++selected_index_bits;
        }
    }

    result.waveforms.reserve(waveform_count);
    for (std::uint32_t waveform = 0; waveform < waveform_count; ++waveform) {
        ModeThreeWaveformValue value;
        value.gain_code = 61U;
        if (result.per_waveform_gain_present) {
            value.gain_code =
                static_cast<std::uint8_t>(source.extract_unsigned(6U));
        }
        value.selected_index = selected_index_bits == 0U
            ? 0U
            : source.extract_unsigned(selected_index_bits);
        value.values.assign(update_value_count, 0U);
        if (value.selected_index < update_value_count) {
            value.values[value.selected_index] = value.gain_code;
        }
        result.waveforms.push_back(std::move(value));
    }
    return result;
}

} // namespace dtsx
