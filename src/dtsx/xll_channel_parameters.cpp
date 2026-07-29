#include "dtsx/xll_channel_parameters.hpp"

#include <algorithm>

namespace dtsx {

bool unpack_xll_channel_parameters(
    bitstream::Cursor& source,
    std::uint32_t segment_index,
    std::uint8_t parameter_bits,
    const std::vector<std::uint8_t>& adaptive_prediction_orders,
    XllChannelParameters& parameters) noexcept {
    // libdtsx.so: dtsxXLLGetChannelParams, 0xb9ab8.
    if (parameter_bits == 0U || parameter_bits > 8U
        || adaptive_prediction_orders.empty()) {
        return false;
    }

    const std::size_t channel_count =
        adaptive_prediction_orders.size();
    if (segment_index != 0U
        && source.extract_unsigned(1U) != 0U) {
        if (parameters.coding.empty()
            || (!parameters.shared
                && parameters.coding.size() != channel_count)) {
            return false;
        }
        for (XllMsbCoding& coding : parameters.coding) {
            coding.initial_parameter = 0U;
            coding.initial_sample_count = 0U;
        }
        return true;
    }

    const bool single_code = source.extract_unsigned(1U) != 0U;
    const std::size_t parsed_coding_count =
        single_code ? 1U : channel_count;
    parameters = {};
    parameters.shared = single_code;
    parameters.coding.resize(parsed_coding_count);

    for (std::size_t index = 0;
         index < parsed_coding_count;
         ++index) {
        XllMsbCoding& coding = parameters.coding[index];
        coding.rice_coding = source.extract_unsigned(1U) != 0U;
        coding.escape_width = 0U;
        if (!parameters.shared && coding.rice_coding
            && source.extract_unsigned(1U) != 0U) {
            coding.escape_width = static_cast<std::uint8_t>(
                source.extract_unsigned(parameter_bits) + 1U);
        }
    }

    const std::uint8_t maximum_order = *std::max_element(
        adaptive_prediction_orders.begin(),
        adaptive_prediction_orders.end());
    for (std::size_t index = 0;
         index < parsed_coding_count;
         ++index) {
        XllMsbCoding& coding = parameters.coding[index];
        if (segment_index == 0U) {
            coding.initial_parameter = static_cast<std::uint8_t>(
                source.extract_unsigned(parameter_bits));
            if (!coding.rice_coding
                && coding.initial_parameter != 0U) {
                ++coding.initial_parameter;
            }
            coding.initial_sample_count = parameters.shared
                ? maximum_order
                : adaptive_prediction_orders[index];
        } else {
            coding.initial_parameter = 0U;
            coding.initial_sample_count = 0U;
        }

        coding.rice_parameter = static_cast<std::uint8_t>(
            source.extract_unsigned(parameter_bits));
        if (!coding.rice_coding && coding.rice_parameter != 0U) {
            ++coding.rice_parameter;
        }
    }
    return true;
}

} // namespace dtsx
