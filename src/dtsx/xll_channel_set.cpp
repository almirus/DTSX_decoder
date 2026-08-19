#include "dtsx/xll_channel_set.hpp"

#include "dtsx/crc16.hpp"

#include <algorithm>
#include <array>
#include <limits>

namespace dtsx {
namespace {

constexpr std::array<std::uint32_t, 16> kSampleRates = {
    8000U, 16000U, 32000U, 64000U, 128000U, 22050U,
    44100U, 88200U, 176400U, 352800U, 12000U, 24000U,
    48000U, 96000U, 192000U, 384000U,
};

std::uint8_t index_bits(std::uint32_t count) noexcept {
    std::uint8_t bits = 0U;
    for (std::uint32_t value = 1U; value < count; value <<= 1U) {
        ++bits;
    }
    return bits;
}

std::int32_t unfold_signed(std::uint32_t value) noexcept {
    return (value & 1U) != 0U
        ? static_cast<std::int32_t>(~(value >> 1U))
        : static_cast<std::int32_t>(value >> 1U);
}

std::uint32_t bit_count(std::uint32_t value) noexcept {
    std::uint32_t count = 0U;
    while (value != 0U) {
        count += value & 1U;
        value >>= 1U;
    }
    return count;
}

bool skip_speaker_mapping(
    bitstream::Cursor& source,
    std::uint8_t channel_count,
    std::uint8_t channel_mask_bits) noexcept {
    const std::uint8_t coefficient_bits = static_cast<std::uint8_t>(
        2U * (source.extract_unsigned(3U) + 3U));
    const std::uint32_t configuration_count =
        source.extract_unsigned(2U) + 1U;
    for (std::uint32_t configuration = 0U;
         configuration < configuration_count;
         ++configuration) {
        if (channel_count > 32U) {
            return false;
        }
        const std::uint32_t active_channel_mask =
            source.extract_unsigned(channel_count);
        const std::uint32_t speaker_count =
            source.extract_unsigned(6U) + 1U;
        const bool speaker_mask_present =
            source.extract_unsigned(1U) != 0U;
        const std::uint64_t speaker_description_bits =
            speaker_mask_present
            ? channel_mask_bits
            : 25ULL * speaker_count;
        const std::uint64_t coefficient_data_bits =
            static_cast<std::uint64_t>(
                bit_count(active_channel_mask))
            * coefficient_bits;
        const std::uint64_t skipped =
            speaker_description_bits + coefficient_data_bits;
        if (skipped > source.remaining_bits()) {
            return false;
        }
        source.fast_forward(static_cast<std::int32_t>(skipped));
    }
    return true;
}

bool unpack_downmix_coefficients(
    bitstream::Cursor& source,
    bool primary_channel_set,
    std::uint8_t downmix_type,
    std::uint32_t preceding_hierarchy_channels,
    std::uint8_t channel_count,
    XllChannelSetHeader& header) {
    static constexpr std::array<std::uint8_t, 7>
        kPrimaryDownmixChannels = {1U, 2U, 2U, 3U, 3U, 4U, 4U};
    if (primary_channel_set
        && downmix_type >= kPrimaryDownmixChannels.size()) {
        return false;
    }
    const std::uint32_t row_count =
        primary_channel_set
        ? kPrimaryDownmixChannels[downmix_type]
        : preceding_hierarchy_channels;
    const std::uint32_t values_per_row =
        channel_count + (primary_channel_set ? 0U : 1U);
    const std::uint64_t value_count =
        static_cast<std::uint64_t>(row_count) * values_per_row;
    if (9ULL * value_count > source.remaining_bits()
        || value_count
            > static_cast<std::uint64_t>(
                std::numeric_limits<std::size_t>::max())) {
        return false;
    }
    header.downmix_row_count = row_count;
    header.downmix_values_per_row = values_per_row;
    header.downmix_coefficients.resize(
        static_cast<std::size_t>(value_count));
    if (!primary_channel_set) {
        header.downmix_scale_codes.reserve(row_count);
    }
    std::size_t value = 0U;
    for (std::uint32_t row = 0U; row < row_count; ++row) {
        for (std::uint32_t column = 0U;
             column < values_per_row;
             ++column) {
            const std::uint16_t code =
                static_cast<std::uint16_t>(
                    source.extract_unsigned(9U));
            header.downmix_coefficients[value++] = code;
            if (!primary_channel_set && column == 0U) {
                header.downmix_scale_codes.push_back(code);
            }
        }
    }
    return true;
}

bool unpack_prediction_band(
    bitstream::Cursor& source,
    std::uint8_t channel_count,
    XllChannelSetBand& band) {
    band.joint_decorrelation =
        source.extract_unsigned(1U) != 0U;
    band.channel_order.resize(channel_count);
    const std::uint8_t channel_index_bits = index_bits(channel_count);
    if (band.joint_decorrelation) {
        for (std::uint8_t& channel : band.channel_order) {
            channel = static_cast<std::uint8_t>(
                source.extract_unsigned(channel_index_bits));
            if (channel >= channel_count) {
                return false;
            }
        }
        for (std::uint32_t pair = 0;
             pair < channel_count / 2U;
             ++pair) {
            std::int32_t coefficient = 0;
            if (source.extract_unsigned(1U) != 0U) {
                coefficient =
                    unfold_signed(source.extract_unsigned(7U));
            }
            band.joint_pairs.push_back({
                static_cast<std::uint8_t>(2U * pair),
                static_cast<std::uint8_t>(2U * pair + 1U),
                coefficient,
            });
        }
    } else {
        for (std::uint32_t channel = 0;
             channel < channel_count;
             ++channel) {
            band.channel_order[channel] =
                static_cast<std::uint8_t>(channel);
        }
    }

    band.prediction.resize(channel_count);
    std::vector<std::uint8_t> adaptive_orders(channel_count);
    for (std::uint8_t& order : adaptive_orders) {
        order = static_cast<std::uint8_t>(
            source.extract_unsigned(4U));
    }
    for (std::uint32_t channel = 0;
         channel < channel_count;
         ++channel) {
        XllChannelPrediction& prediction = band.prediction[channel];
        const std::uint8_t adaptive_order =
            adaptive_orders[channel];
        if (adaptive_order == 0U) {
            prediction.fixed_order = static_cast<std::uint8_t>(
                source.extract_unsigned(2U));
        }
    }
    for (std::uint32_t channel = 0;
         channel < channel_count;
         ++channel) {
        XllChannelPrediction& prediction = band.prediction[channel];
        const std::uint8_t adaptive_order =
            adaptive_orders[channel];
        if (adaptive_order == 0U) {
            continue;
        }
        prediction.adaptive_reflection_coefficients.reserve(
            adaptive_order);
        for (std::uint32_t coefficient = 0;
             coefficient < adaptive_order;
             ++coefficient) {
            prediction.adaptive_reflection_coefficients.push_back(
                unfold_signed(source.extract_unsigned(8U)));
        }
    }
    return true;
}

} // namespace

bool probe_xll_channel_set_header(
    bitstream::Cursor& source,
    bool reject_replacement_sets,
    XllChannelSetProbe& probe,
    std::uint8_t total_channel_sets,
    bool legacy_sync) noexcept {
    // libdtsx.so: dtsx_decodeTryXLLChSetHeader, 0xb1f48..0xb255c.
    const bitstream::Cursor header_start = source;
    probe = {};
    probe.header_size = source.extract_unsigned(10U) + 1U;
    if (probe.header_size > header_start.remaining_bits() / 8U) {
        return false;
    }
    bitstream::Cursor crc_source = header_start;
    probe.crc_valid =
        valid_crc16(crc_source, 8U * probe.header_size);
    if (!probe.crc_valid) {
        source = header_start;
        source.fast_forward(
            static_cast<std::int32_t>(8U * probe.header_size));
        return false;
    }

    const std::uint8_t channel_count_code =
        static_cast<std::uint8_t>(source.extract_unsigned(4U));
    probe.channel_count =
        static_cast<std::uint8_t>(channel_count_code + 1U);
    probe.channel_mask =
        source.extract_unsigned(probe.channel_count);
    probe.bit_depth = static_cast<std::uint8_t>(
        source.extract_unsigned(5U) + 1U);
    probe.storage_bit_depth = static_cast<std::uint8_t>(
        source.extract_unsigned(5U) + 1U);
    probe.parameter_bits = probe.storage_bit_depth <= 8U
        ? 3U
        : (probe.storage_bit_depth <= 16U ? 4U : 5U);
    probe.sample_rate =
        kSampleRates[source.extract_unsigned(4U)];
    if ((total_channel_sets > 1U || probe.sample_rate > 96000U)
        && probe.parameter_bits <= 4U) {
        ++probe.parameter_bits;
    }
    if (legacy_sync) {
        probe.reuse_previous_header =
            source.extract_unsigned(1U) != 0U;
        if (probe.reuse_previous_header) {
            source = header_start;
            source.fast_forward(static_cast<std::int32_t>(
                8U * probe.header_size));
            return true;
        }
    }
    probe.frequency_ratio = static_cast<std::uint8_t>(
        1U << source.extract_unsigned(2U));
    probe.replacement_set_index = static_cast<std::uint8_t>(
        source.extract_unsigned(2U));
    if (probe.replacement_set_index != 0U) {
        probe.default_replacement_set =
            source.extract_unsigned(1U) != 0U;
    }
    if (reject_replacement_sets
        && probe.replacement_set_index != 0U) {
        source = header_start;
        source.fast_forward(
            static_cast<std::int32_t>(8U * probe.header_size));
        return false;
    }
    if (probe.sample_rate <= 96000U) {
        probe.frequency_band_count_available = true;
        probe.frequency_band_count = 1U;
    }
    source = header_start;
    source.fast_forward(
        static_cast<std::int32_t>(8U * probe.header_size));
    return true;
}

bool unpack_xll_primary_channel_set_header(
    bitstream::Cursor& source,
    const XllCommonHeader& common,
    XllChannelSetHeader& header,
    bool one_to_one_mapping,
    std::uint32_t preceding_hierarchy_channels,
    const XllChannelSetHeader* previous_header,
    std::uint8_t alternate_prefix_bits) noexcept {
    // libdtsx.so: dtsx_decodeXLLChSetHeader primary-stream path,
    // 0xb263c..0xb2e2c.
    const bitstream::Cursor header_start = source;
    header = {};
    XllChannelSetProbe& probe = header.probe;
    probe.header_size = source.extract_unsigned(10U) + 1U;
    if (probe.header_size > header_start.remaining_bits() / 8U) {
        return false;
    }

    probe.channel_count = static_cast<std::uint8_t>(
        source.extract_unsigned(4U) + 1U);
    probe.channel_mask =
        source.extract_unsigned(probe.channel_count);
    probe.bit_depth = static_cast<std::uint8_t>(
        source.extract_unsigned(5U) + 1U);
    probe.storage_bit_depth = static_cast<std::uint8_t>(
        source.extract_unsigned(5U) + 1U);
    probe.parameter_bits = probe.storage_bit_depth <= 8U
        ? 3U
        : (probe.storage_bit_depth <= 16U ? 4U : 5U);
    probe.sample_rate =
        kSampleRates[source.extract_unsigned(4U)];
    if ((common.channel_set_count > 1U
         || probe.sample_rate > 96000U)
        && probe.parameter_bits <= 4U) {
        ++probe.parameter_bits;
    }
    if (common.legacy_sync) {
        probe.reuse_previous_header =
            source.extract_unsigned(1U) != 0U;
        if (probe.reuse_previous_header) {
            bitstream::Cursor crc_source = header_start;
            const bool crc_valid =
                valid_crc16(crc_source, 8U * probe.header_size);
            if (previous_header == nullptr
                || !crc_valid
                || !source.valid()) {
                source = header_start;
                source.fast_forward(static_cast<std::int32_t>(
                    8U * probe.header_size));
                return false;
            }
            const XllChannelSetProbe current_probe = probe;
            header = *previous_header;
            header.probe.header_size = current_probe.header_size;
            header.probe.channel_count =
                current_probe.channel_count;
            header.probe.channel_mask =
                current_probe.channel_mask;
            header.probe.bit_depth = current_probe.bit_depth;
            header.probe.storage_bit_depth =
                current_probe.storage_bit_depth;
            header.probe.parameter_bits =
                current_probe.parameter_bits;
            header.probe.sample_rate = current_probe.sample_rate;
            header.probe.reuse_previous_header = true;
            header.probe.crc_valid = true;
            source = header_start;
            source.fast_forward(static_cast<std::int32_t>(
                8U * current_probe.header_size));
            return true;
        }
    }
    probe.frequency_ratio = static_cast<std::uint8_t>(
        1U << source.extract_unsigned(2U));
    probe.replacement_set_index = static_cast<std::uint8_t>(
        source.extract_unsigned(2U));
    if (probe.replacement_set_index != 0U) {
        probe.default_replacement_set =
            source.extract_unsigned(1U) != 0U;
    }

    if (alternate_prefix_bits != 0U) {
        if (alternate_prefix_bits > source.remaining_bits()) {
            source = header_start;
            source.fast_forward(static_cast<std::int32_t>(
                8U * probe.header_size));
            return false;
        }
        source.fast_forward(alternate_prefix_bits);
        header.primary_channel_set = true;
        header.hierarchical_channel_set = true;
    } else if (one_to_one_mapping) {
        header.primary_channel_set =
            source.extract_unsigned(1U) != 0U;
        header.downmix_coefficients_present =
            source.extract_unsigned(1U) != 0U;
        header.embedded_downmix_present =
            header.downmix_coefficients_present
            && source.extract_unsigned(1U) != 0U;
        if (header.downmix_coefficients_present
            && header.primary_channel_set) {
            header.downmix_type = static_cast<std::uint8_t>(
                source.extract_unsigned(3U));
            if (header.downmix_type >= 7U) {
                source = header_start;
                source.fast_forward(static_cast<std::int32_t>(
                    8U * probe.header_size));
                return false;
            }
        }
        header.hierarchical_channel_set =
            source.extract_unsigned(1U) != 0U;
        if (header.downmix_coefficients_present
            && !unpack_downmix_coefficients(
                source,
                header.primary_channel_set,
                header.downmix_type,
                preceding_hierarchy_channels,
                probe.channel_count,
                header)) {
            source = header_start;
            source.fast_forward(static_cast<std::int32_t>(
                8U * probe.header_size));
            return false;
        }
        header.channel_mask_enabled =
            source.extract_unsigned(1U) != 0U;
        if (header.channel_mask_enabled) {
            header.speaker_channel_mask =
                source.extract_unsigned(
                    common.channel_set_header_size_bits);
            if (bit_count(header.speaker_channel_mask)
                != probe.channel_count) {
                source = header_start;
                source.fast_forward(static_cast<std::int32_t>(
                    8U * probe.header_size));
                return false;
            }
        } else {
            const std::uint64_t position_bits =
                25ULL * probe.channel_count;
            if (position_bits > source.remaining_bits()) {
                source = header_start;
                source.fast_forward(static_cast<std::int32_t>(
                    8U * probe.header_size));
                return false;
            }
            source.fast_forward(
                static_cast<std::int32_t>(position_bits));
        }
    } else {
        header.primary_channel_set = true;
        header.hierarchical_channel_set = true;
        const bool mapping_coefficients_present =
            source.extract_unsigned(1U) != 0U;
        if (mapping_coefficients_present
            && !skip_speaker_mapping(
                source,
                probe.channel_count,
                common.channel_set_header_size_bits)) {
            source = header_start;
            source.fast_forward(static_cast<std::int32_t>(
                8U * probe.header_size));
            return false;
        }
    }

    probe.frequency_band_count_available = true;
    probe.frequency_band_count = probe.sample_rate > 96000U
        ? static_cast<std::uint8_t>(
            source.extract_unsigned(1U) != 0U ? 4U : 2U)
        : 1U;
    header.bands.resize(probe.frequency_band_count);
    for (std::uint32_t band_index = 0;
         band_index < probe.frequency_band_count;
         ++band_index) {
        XllChannelSetBand& band = header.bands[band_index];
        if (!unpack_prediction_band(
                source, probe.channel_count, band)) {
            source = header_start;
            source.fast_forward(
                static_cast<std::int32_t>(8U * probe.header_size));
            return false;
        }

        band.embedded_downmix_present =
            header.embedded_downmix_present
            && (band_index == 0U
                || source.extract_unsigned(1U) != 0U);

        if (band_index == 0U && common.scalable_lsb) {
            band.primary_size_present = true;
            band.primary_size =
                source.extract_unsigned(common.segment_size_bits);
            if (band.primary_size != 0U
                && common.band_crc_present > 1U) {
                band.primary_size += 2U;
            }
            band.primary_widths.resize(probe.channel_count);
            band.secondary_widths.resize(probe.channel_count);
            for (std::uint8_t& width : band.primary_widths) {
                width = static_cast<std::uint8_t>(
                    source.extract_unsigned(4U));
            }
            for (std::uint8_t& width : band.secondary_widths) {
                width = static_cast<std::uint8_t>(
                    source.extract_unsigned(4U));
            }
        } else if (band_index != 0U) {
            const bool primary_widths_present =
                source.extract_unsigned(1U) != 0U;
            if (primary_widths_present) {
                band.primary_size_present = true;
                band.primary_size =
                    source.extract_unsigned(common.segment_size_bits);
                if (band.primary_size != 0U
                    && common.band_crc_present > 2U) {
                    band.primary_size += 2U;
                }
            }
            band.primary_widths.assign(probe.channel_count, 0U);
            if (primary_widths_present) {
                for (std::uint8_t& width :
                     band.primary_widths) {
                    width = static_cast<std::uint8_t>(
                        source.extract_unsigned(4U));
                }
            }
            const bool secondary_widths_present =
                source.extract_unsigned(1U) != 0U;
            band.secondary_widths.assign(
                probe.channel_count, 0U);
            if (secondary_widths_present) {
                for (std::uint8_t& width :
                     band.secondary_widths) {
                    width = static_cast<std::uint8_t>(
                        source.extract_unsigned(4U));
                }
            }
        } else {
            band.primary_widths.assign(
                probe.channel_count, 0U);
            band.secondary_widths.assign(
                probe.channel_count, 0U);
        }
    }

    const std::uint32_t parsed_bits =
        header_start.remaining_bits() - source.remaining_bits();
    if (parsed_bits > 8U * probe.header_size) {
        source = header_start;
        source.fast_forward(
            static_cast<std::int32_t>(8U * probe.header_size));
        return false;
    }
    bitstream::Cursor crc_source = header_start;
    probe.crc_valid =
        valid_crc16(crc_source, 8U * probe.header_size);
    source = header_start;
    source.fast_forward(
        static_cast<std::int32_t>(8U * probe.header_size));
    return probe.crc_valid;
}

} // namespace dtsx
