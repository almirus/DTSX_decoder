#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/xll_channel_parameters.hpp"
#include "dtsx/xll_channel_set.hpp"
#include "dtsx/xll_entropy.hpp"
#include "dtsx/xll_prediction.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace dtsx {

struct XllDecodedChannelSet final {
    std::vector<std::vector<std::int32_t>> channels;
    XllDecimatorHistory decimator_history;
};

class XllChannelSetDecoder final {
public:
    [[nodiscard]] bool decode_msb_segment(
        bitstream::Cursor& source,
        std::uint32_t segment_index,
        std::uint32_t sample_count,
        const XllChannelParameters& parameters,
        const std::vector<XllChannelPrediction>& prediction,
        const std::vector<std::uint8_t>& channel_order,
        std::uint8_t decimator_mode,
        std::uint8_t first_decimator_channel,
        const std::vector<XllJointDecorrelationPair>& joint_pairs,
        XllDecodedChannelSet& decoded);

    [[nodiscard]] bool combine_lsb_segment(
        bitstream::Cursor& source,
        const XllChannelSetBand& band,
        const std::vector<std::uint8_t>& msb_shifts,
        XllDecodedChannelSet& decoded);

    [[nodiscard]] const std::string& last_error() const noexcept {
        return last_error_;
    }

private:
    std::vector<XllAdaptivePredictionState> adaptive_state_;
    std::vector<std::array<std::int32_t, 8>> fixed_state_;
    std::string last_error_;
};

} // namespace dtsx
