#pragma once

#include "bitstream/dtsx_bitstream.hpp"
#include "dtsx/xll_entropy.hpp"

#include <cstdint>
#include <vector>

namespace dtsx {

struct XllChannelParameters final {
    bool shared = false;
    std::vector<XllMsbCoding> coding;
};

[[nodiscard]] bool unpack_xll_channel_parameters(
    bitstream::Cursor& source,
    std::uint32_t segment_index,
    std::uint8_t parameter_bits,
    const std::vector<std::uint8_t>& adaptive_prediction_orders,
    XllChannelParameters& parameters) noexcept;

} // namespace dtsx
