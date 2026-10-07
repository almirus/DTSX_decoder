#include "dtsx/xll_entropy.hpp"

#include <utility>

namespace dtsx {
namespace {

std::int32_t unfold_signed(std::uint32_t value) noexcept {
    const std::uint32_t magnitude = value >> 1U;
    return (value & 1U) != 0U
        ? static_cast<std::int32_t>(~magnitude)
        : static_cast<std::int32_t>(magnitude);
}

bool unpack_unary(bitstream::Cursor& source,
                  std::uint32_t& leading_zero_count) noexcept {
    leading_zero_count = 0U;
    while (source.remaining_bits() != 0U) {
        if (source.extract_unsigned(1U) != 0U) {
            return true;
        }
        ++leading_zero_count;
    }
    return false;
}

} // namespace

bool unpack_xll_msb_binary(
    bitstream::Cursor& source,
    std::uint32_t sample_count,
    std::uint8_t bit_width,
    std::vector<std::int32_t>& values) {
    values.clear();
    values.reserve(sample_count);
    if (bit_width > 32U) {
        return false;
    }
    for (std::uint32_t index = 0; index < sample_count; ++index) {
        if (source.remaining_bits() < bit_width) {
            values.clear();
            return false;
        }
        values.push_back(unfold_signed(
            source.extract_unsigned(bit_width)));
    }
    return true;
}

bool unpack_xll_msb_rice(
    bitstream::Cursor& source,
    const std::vector<std::uint8_t>& escape_flags,
    std::uint8_t escape_width,
    std::vector<std::int32_t>& values) {
    values.clear();
    values.reserve(escape_flags.size());
    if (escape_width > 32U) {
        return false;
    }
    for (const std::uint8_t escape : escape_flags) {
        if (escape != 0U) {
            if (source.remaining_bits() < escape_width) {
                return false;
            }
            values.push_back(unfold_signed(
                source.extract_unsigned(escape_width)));
            continue;
        }
        std::uint32_t quotient = 0U;
        if (!unpack_unary(source, quotient)) {
            return false;
        }
        values.push_back(unfold_signed(quotient));
    }
    return true;
}

bool unpack_xll_msb_rice_binary(
    bitstream::Cursor& source,
    const std::vector<std::uint8_t>& escape_flags,
    std::uint8_t rice_bits,
    std::uint8_t escape_width,
    std::vector<std::int32_t>& values) {
    values.clear();
    values.reserve(escape_flags.size());
    if (rice_bits >= 32U || escape_width > 32U) {
        return false;
    }
    for (const std::uint8_t escape : escape_flags) {
        if (escape != 0U) {
            if (source.remaining_bits() < escape_width) {
                return false;
            }
            values.push_back(unfold_signed(
                source.extract_unsigned(escape_width)));
            continue;
        }
        std::uint32_t quotient = 0U;
        if (!unpack_unary(source, quotient)
            || source.remaining_bits() < rice_bits) {
            return false;
        }
        const std::uint32_t remainder =
            source.extract_unsigned(rice_bits);
        const std::uint32_t code =
            (quotient << rice_bits) | remainder;
        values.push_back(unfold_signed(code));
    }
    return true;
}

bool unpack_xll_lsb_core(
    bitstream::Cursor& source,
    std::uint32_t sample_count,
    std::uint8_t bit_width,
    std::vector<std::uint32_t>& values) {
    values.clear();
    values.reserve(sample_count);
    if (bit_width > 32U) {
        return false;
    }
    for (std::uint32_t index = 0; index < sample_count; ++index) {
        if (source.remaining_bits() < bit_width) {
            return false;
        }
        values.push_back(source.extract_unsigned(bit_width));
    }
    return true;
}

bool unpack_xll_msb(
    bitstream::Cursor& source,
    std::uint32_t sample_count,
    const XllMsbCoding& coding,
    std::vector<std::int32_t>& values) {
    if (coding.initial_sample_count > sample_count) {
        return false;
    }

    std::vector<std::int32_t> initial;
    if (coding.rice_coding) {
        const std::vector<std::uint8_t> initial_flags(
            coding.initial_sample_count, 0U);
        if (!unpack_xll_msb_rice_binary(
                source,
                initial_flags,
                coding.initial_parameter,
                0U,
                initial)) {
            return false;
        }
    } else if (coding.initial_parameter != 0U) {
        if (!unpack_xll_msb_binary(
                source,
                coding.initial_sample_count,
                coding.initial_parameter,
                initial)) {
            return false;
        }
    } else {
        initial.assign(coding.initial_sample_count, 0);
    }

    const std::uint32_t remaining_count =
        sample_count - coding.initial_sample_count;
    std::vector<std::int32_t> remaining;
    if (!coding.rice_coding) {
        if (coding.rice_parameter != 0U) {
            if (!unpack_xll_msb_binary(
                    source,
                    remaining_count,
                    coding.rice_parameter,
                    remaining)) {
                return false;
            }
        } else {
            remaining.assign(remaining_count, 0);
        }
    } else {
        std::vector<std::uint8_t> escape_flags(remaining_count, 0U);
        if (coding.escape_width != 0U && sample_count > 1U) {
            std::uint8_t index_bits = 0U;
            for (std::uint32_t value = 1U;
                 value < sample_count;
                 value <<= 1U) {
                ++index_bits;
            }
            const std::uint32_t escape_count =
                source.extract_unsigned(index_bits);
            for (std::uint32_t index = 0;
                 index < escape_count;
                 ++index) {
                const std::uint32_t position =
                    source.extract_unsigned(index_bits);
                if (position >= remaining_count) {
                    return false;
                }
                escape_flags[position] = 1U;
            }
        }
        const bool unpacked = coding.rice_parameter != 0U
            ? unpack_xll_msb_rice_binary(
                source,
                escape_flags,
                coding.rice_parameter,
                coding.escape_width,
                remaining)
            : unpack_xll_msb_rice(
                source,
                escape_flags,
                coding.escape_width,
                remaining);
        if (!unpacked) {
            return false;
        }
    }

    values = std::move(initial);
    values.insert(values.end(), remaining.begin(), remaining.end());
    return true;
}

bool combine_xll_msb_lsb(
    const std::vector<std::int32_t>& msb,
    const std::vector<std::uint32_t>& lsb,
    std::uint8_t msb_shift,
    std::uint8_t lsb_shift,
    std::vector<std::int32_t>& samples) {
    if (msb.size() != lsb.size()
        || msb_shift >= 32U
        || lsb_shift >= 32U) {
        return false;
    }
    samples.resize(msb.size());
    for (std::size_t index = 0; index < msb.size(); ++index) {
        const std::uint32_t high =
            static_cast<std::uint32_t>(msb[index]) << msb_shift;
        const std::uint32_t low = lsb[index] << lsb_shift;
        samples[index] = static_cast<std::int32_t>(high + low);
    }
    return true;
}

bool unpack_xll_decimator_history(
    bitstream::Cursor& source,
    std::uint32_t segment_index,
    std::uint8_t mode,
    std::uint8_t channel_count,
    std::uint8_t first_channel,
    XllDecimatorHistory& history) {
    history = {};
    if (first_channel > channel_count) {
        return false;
    }
    if (segment_index != 0U || (mode != 1U && mode != 3U)) {
        return true;
    }
    if (source.remaining_bits() < 5U) {
        return false;
    }
    history.bit_width = static_cast<std::uint8_t>(
        source.extract_unsigned(5U) + 1U);
    history.channels.resize(channel_count - first_channel);
    for (auto& channel : history.channels) {
        for (std::int32_t& value : channel) {
            if (source.remaining_bits() < history.bit_width) {
                history = {};
                return false;
            }
            value = static_cast<std::int32_t>(
                source.extract_unsigned(history.bit_width));
        }
    }
    return true;
}

} // namespace dtsx
