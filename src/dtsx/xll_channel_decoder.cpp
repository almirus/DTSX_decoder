#include "dtsx/xll_channel_decoder.hpp"

#include <utility>

namespace dtsx {

bool XllChannelSetDecoder::decode_msb_segment(
    bitstream::Cursor& source,
    std::uint32_t segment_index,
    std::uint32_t sample_count,
    const XllChannelParameters& parameters,
    const std::vector<XllChannelPrediction>& prediction,
    const std::vector<std::uint8_t>& channel_order,
    std::uint8_t decimator_mode,
    std::uint8_t first_decimator_channel,
    const std::vector<XllJointDecorrelationPair>& joint_pairs,
    XllDecodedChannelSet& decoded) {
    decoded = {};
    last_error_.clear();
    if (sample_count == 0U || prediction.empty()
        || channel_order.size() != prediction.size()
        || parameters.coding.empty()
        || (!parameters.shared
            && parameters.coding.size() != prediction.size())) {
        last_error_ = "invalid channel-set dimensions";
        return false;
    }

    const std::size_t channel_count = prediction.size();
    decoded.channels.resize(channel_count);
    for (std::size_t channel = 0; channel < channel_count; ++channel) {
        const std::size_t coding_index =
            parameters.shared ? 0U : channel;
        if (coding_index >= parameters.coding.size()
            || !unpack_xll_msb(
                source,
                sample_count,
                parameters.coding[coding_index],
                decoded.channels[channel])) {
            decoded = {};
            last_error_ = "MSB residual unpack failed at channel "
                + std::to_string(channel);
            return false;
        }
    }

    if (!unpack_xll_decimator_history(
            source,
            segment_index,
            decimator_mode,
            static_cast<std::uint8_t>(channel_count),
            first_decimator_channel,
            decoded.decimator_history)) {
        decoded = {};
        last_error_ = "decimator history unpack failed";
        return false;
    }

    if (adaptive_state_.size() != channel_count) {
        adaptive_state_.assign(
            channel_count, XllAdaptivePredictionState{});
        fixed_state_.assign(
            channel_count, std::array<std::int32_t, 8>{});
    }
    const bool first_segment = segment_index == 0U;
    for (std::size_t channel = 0; channel < channel_count; ++channel) {
        const XllChannelPrediction& channel_prediction =
            prediction[channel];
        if (!channel_prediction
                 .adaptive_reflection_coefficients.empty()) {
            if (!inverse_xll_adaptive_prediction(
                    decoded.channels[channel],
                    channel_prediction
                        .adaptive_reflection_coefficients,
                    first_segment,
                    adaptive_state_[channel])) {
                decoded = {};
                last_error_ =
                    "adaptive inverse prediction failed at channel "
                    + std::to_string(channel);
                return false;
            }
        } else if (channel_prediction.fixed_order != 0U
                   && !inverse_xll_fixed_prediction(
                       decoded.channels[channel],
                       channel_prediction.fixed_order,
                       first_segment,
                       fixed_state_[channel])) {
            decoded = {};
            last_error_ = "fixed inverse prediction failed at channel "
                + std::to_string(channel);
            return false;
        }
    }

    for (const XllJointDecorrelationPair& pair : joint_pairs) {
        if (pair.source_channel >= channel_count
            || pair.destination_channel >= channel_count
            || !inverse_xll_joint_channel_decorrelation(
                decoded.channels[pair.source_channel],
                decoded.channels[pair.destination_channel],
                pair.coefficient)) {
            decoded = {};
            last_error_ = "joint-channel decorrelation failed";
            return false;
        }
    }
    std::vector<std::vector<std::int32_t>> reordered(channel_count);
    std::vector<bool> assigned(channel_count, false);
    for (std::size_t channel = 0U;
         channel < channel_count;
         ++channel) {
        const std::uint8_t destination = channel_order[channel];
        if (destination >= channel_count || assigned[destination]) {
            decoded = {};
            last_error_ = "invalid joint-channel permutation";
            return false;
        }
        assigned[destination] = true;
        reordered[destination] =
            std::move(decoded.channels[channel]);
    }
    decoded.channels = std::move(reordered);
    return true;
}

bool XllChannelSetDecoder::combine_lsb_segment(
    bitstream::Cursor& source,
    const XllChannelSetBand& band,
    const std::vector<std::uint8_t>& msb_shifts,
    XllDecodedChannelSet& decoded) {
    const std::size_t channel_count = decoded.channels.size();
    last_error_.clear();
    if (channel_count == 0U
        || band.primary_widths.size() != channel_count
        || band.secondary_widths.size() != channel_count
        || msb_shifts.size() != channel_count) {
        last_error_ = "invalid MSB/LSB dimensions";
        return false;
    }

    for (std::size_t channel = 0;
         channel < channel_count;
         ++channel) {
        std::vector<std::uint32_t> lsb;
        if (!unpack_xll_lsb_core(
                source,
                static_cast<std::uint32_t>(
                    decoded.channels[channel].size()),
                band.primary_widths[channel],
                lsb)) {
            last_error_ = "LSB unpack failed at channel "
                + std::to_string(channel);
            return false;
        }
        std::vector<std::int32_t> combined;
        if (!combine_xll_msb_lsb(
                decoded.channels[channel],
                lsb,
                msb_shifts[channel],
                band.secondary_widths[channel],
                combined)) {
            last_error_ = "MSB/LSB combine failed at channel "
                + std::to_string(channel);
            return false;
        }
        decoded.channels[channel] = std::move(combined);
    }
    return true;
}

} // namespace dtsx
