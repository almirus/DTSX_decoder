#include "dtsx/core_substream.hpp"

namespace dtsx {

bool parse_core_substream_header(bitstream::Cursor& source,
                                 CoreSubstreamInfo& info) noexcept {
    info = {};
    source.fast_forward(0x26);
    const std::uint32_t broadcast = source.extract_unsigned(1U);
    source.fast_forward(7);
    info.audio_frame_size = source.extract_unsigned(14U) + 1U;
    info.sample_rate_index = static_cast<std::uint8_t>(
        source.extract_unsigned(6U));
    info.dialog_normalization = static_cast<std::uint8_t>(
        source.extract_unsigned(4U));
    source.fast_forward(10);
    info.extension_type = static_cast<std::uint8_t>(
        source.extract_unsigned(3U));
    if (broadcast == 1U) {
        source.fast_forward(16);
    }
    info.representation_type = static_cast<std::uint8_t>(
        source.extract_unsigned(1U));
    source.fast_forward(1);
    const std::uint32_t coding_mode = source.extract_unsigned(2U);
    source.fast_forward(1);
    if (broadcast == 1U) {
        source.fast_forward(1);
    }
    source.fast_forward(7);
    const bool object_flag = (source.extract_unsigned(3U) & 1U) != 0U;
    source.fast_forward(6);
    const std::uint32_t table_count = source.extract_unsigned(4U) + 1U;
    info.channel_table_count = table_count;
    info.channel_set_count = static_cast<std::uint8_t>(
        source.extract_unsigned(3U) + 1U);
    const std::uint32_t entries = info.channel_set_count;
    std::array<std::array<std::uint8_t, 32>, 4> mode_columns{};
    std::array<std::array<std::uint8_t, 32>, 5> bandwidth_columns{};
    source.fast_forward(static_cast<std::int32_t>(21U * entries));

    for (std::uint32_t index = 0; index < entries && index < 32U; ++index) {
        info.channel_active[index] = static_cast<std::uint8_t>(
            source.extract_unsigned(1U));
    }
    for (std::uint32_t column = 0; column < 4U; ++column) {
        for (std::uint32_t index = 0; index < entries && index < 32U;
             ++index) {
            const std::uint32_t value = source.extract_unsigned(2U);
            if (column == 0U) {
                info.channel_mode[index] = static_cast<std::uint8_t>(value);
            }
            mode_columns[column][index] = static_cast<std::uint8_t>(value);
        }
    }
    for (std::uint32_t column = 0; column < 5U; ++column) {
        for (std::uint32_t index = 0; index < entries && index < 32U;
             ++index) {
            const std::uint32_t value = source.extract_unsigned(3U);
            if (column == 0U) {
                info.channel_bandwidth[index] = static_cast<std::uint8_t>(value);
                bandwidth_columns[column][index] =
                    static_cast<std::uint8_t>(value);
            }
        }
    }
    for (std::uint32_t index = 0; index < entries && index < 32U; ++index) {
        if (info.channel_active[index] == 0U) {
            source.fast_forward(2);
        }
        for (std::uint32_t column = 0; column < 4U; ++column) {
            if (mode_columns[column][index] <= 2U) {
                source.fast_forward(2);
            }
        }
        for (std::uint32_t column = 0; column < 5U; ++column) {
            if (bandwidth_columns[column][index] <= 6U) {
                source.fast_forward(2);
            }
        }
    }
    if (broadcast != 0U) {
        // DTSFrameScanner_ParseCoreSS advances over the broadcast-specific
        // 16-bit field after the channel-table descriptors and before the
        // subframe count.
        source.fast_forward(16);
    }
    info.representation_type = static_cast<std::uint8_t>(
        info.representation_type | (coding_mode << 1U)
        | (object_flag ? 0x80U : 0U));
    info.subframe_count = source.extract_unsigned(2U) + 1U;
    static constexpr std::uint32_t kSampleRates[16] = {
        0U, 8000U, 16000U, 32000U, 0U, 0U, 11025U, 22050U,
        44100U, 0U, 0U, 12000U, 24000U, 48000U, 0U, 0U,
    };
    const std::uint32_t rate_index = info.sample_rate_index;
    info.sample_rate = rate_index < 16U ? kSampleRates[rate_index] : 0U;
    info.output_block_samples = info.subframe_count * table_count;
    return source.valid();
}

} // namespace dtsx
