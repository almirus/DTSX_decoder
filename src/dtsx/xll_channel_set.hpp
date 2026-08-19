#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/xll_common_header.hpp"
#include "dtsx/xll_prediction.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct XllChannelSetProbe final {
    std::uint32_t header_size = 0;
    std::uint8_t channel_count = 0;
    std::uint32_t channel_mask = 0;
    std::uint8_t bit_depth = 0;
    std::uint8_t storage_bit_depth = 0;
    std::uint8_t parameter_bits = 0;
    std::uint32_t sample_rate = 0;
    std::uint8_t frequency_ratio = 1;
    std::uint8_t replacement_set_index = 0;
    bool default_replacement_set = false;
    bool frequency_band_count_available = false;
    std::uint8_t frequency_band_count = 0;
    bool reuse_previous_header = false;
    bool crc_valid = false;
};

struct XllChannelSetBand final {
    bool joint_decorrelation = false;
    bool embedded_downmix_present = false;
    std::vector<std::uint8_t> channel_order;
    std::vector<XllJointDecorrelationPair> joint_pairs;
    std::vector<XllChannelPrediction> prediction;
    bool primary_size_present = false;
    std::uint32_t primary_size = 0;
    std::vector<std::uint8_t> primary_widths;
    std::vector<std::uint8_t> secondary_widths;
};

struct XllChannelSetHeader final {
    XllChannelSetProbe probe;
    bool primary_channel_set = true;
    bool hierarchical_channel_set = true;
    bool downmix_coefficients_present = false;
    bool embedded_downmix_present = false;
    std::uint8_t downmix_type = 0;
    std::uint32_t downmix_row_count = 0;
    std::uint32_t downmix_values_per_row = 0;
    std::vector<std::uint16_t> downmix_coefficients;
    std::vector<std::uint16_t> downmix_scale_codes;
    std::vector<std::int32_t> external_downmix_coefficients;
    bool channel_mask_enabled = false;
    std::uint32_t speaker_channel_mask = 0;
    std::vector<XllChannelSetBand> bands;
};

[[nodiscard]] bool probe_xll_channel_set_header(
    bitstream::Cursor& source,
    bool reject_replacement_sets,
    XllChannelSetProbe& probe,
    std::uint8_t total_channel_sets = 1U,
    bool legacy_sync = false) noexcept;

[[nodiscard]] bool unpack_xll_primary_channel_set_header(
    bitstream::Cursor& source,
    const XllCommonHeader& common,
    XllChannelSetHeader& header,
    bool one_to_one_mapping = false,
    std::uint32_t preceding_hierarchy_channels = 0U,
    const XllChannelSetHeader* previous_header = nullptr,
    std::uint8_t alternate_prefix_bits = 0U) noexcept;

} // namespace dtsx
