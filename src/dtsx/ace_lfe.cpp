#include "dtsx/ace_lfe.hpp"

#include "dtsx/ace_bit_reader.hpp"
#include "dtsx/ace_coarse_residual.hpp"
#include "dtsx/ace_lfe_predictor_table.hpp"
#include "dtsx/ace_lfe_tables.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <climits>
#include <limits>
#include <utility>

namespace dtsx {
namespace {

std::int32_t negative_rice_map(std::uint32_t value) noexcept {
    return ace_integer_map_invmap_neg(value);
}

bool read_decimated_values(
    AceBitReader& source,
    std::uint32_t resolution,
    std::uint32_t count,
    AceLfeChannelData& channel) noexcept {
    if (resolution == 0U || resolution > 30U
        || !read_ace_uniform(
            source,
            resolution,
            channel.golomb_parameter)) {
        return false;
    }
    const std::uint32_t alphabet = std::uint32_t{1U} << resolution;
    channel.decimated_values.resize(count);
    for (std::int32_t& sample : channel.decimated_values) {
        std::uint32_t encoded = 0U;
        if (!read_ace_golomb(
                source,
                alphabet,
                channel.golomb_parameter,
                encoded)) {
            return false;
        }
        sample = negative_rice_map(encoded);
    }
    return true;
}

std::int32_t saturate_i32(std::int64_t value) noexcept {
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return static_cast<std::int32_t>(value);
}

unsigned leading_zero_count(std::uint32_t value) noexcept {
    if (value == 0U) {
        return 32U;
    }
    unsigned count = 0U;
    for (std::uint32_t mask = 0x80000000U;
         (value & mask) == 0U;
         mask >>= 1U) {
        ++count;
    }
    return count;
}

std::int32_t ace_fixed_divide_native_impl(
    int numerator_binary_point,
    std::int32_t numerator,
    int denominator_binary_point,
    std::int32_t denominator,
    int output_binary_point) noexcept {
    if (denominator == 0) {
        return INT32_MAX;
    }
    std::uint32_t normalized = denominator < 0
        ? static_cast<std::uint32_t>(-static_cast<std::int64_t>(denominator))
        : static_cast<std::uint32_t>(denominator);
    const int zeros = static_cast<int>(leading_zero_count(normalized));
    int normalization_bias = 2;
    if (zeros > 0) {
        normalized <<= static_cast<unsigned>(zeros - 1);
        normalization_bias = 1;
    }
    const int exponent = normalization_bias + zeros;
    const std::uint32_t estimate0 =
        1993294322U - 2U * normalized;
    const std::uint32_t error0 = 0U - 2U * static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(estimate0) * normalized) >> 32U);
    const std::uint32_t estimate1 = 2U * static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(estimate0) * error0) >> 32U);
    const std::uint32_t error1 = 0U - 2U * static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(normalized) * estimate1) >> 32U);
    const std::uint32_t estimate2 = 2U * static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(estimate1) * error1) >> 32U);
    const std::uint32_t error2 = 0U - 2U * static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(normalized) * estimate2) >> 32U);
    std::int32_t reciprocal = static_cast<std::int32_t>(
        (static_cast<std::uint64_t>(estimate2) * error2) >> 32U);
    if (denominator < 0) {
        reciprocal = static_cast<std::int32_t>(
            0U - static_cast<std::uint32_t>(reciprocal));
    }

    const int shift = exponent + denominator_binary_point
        - numerator_binary_point - output_binary_point + 1;
    const std::int64_t product = static_cast<std::int64_t>(numerator)
        * reciprocal;
    if (shift >= 0 && shift < 32) {
        return saturate_i32(
            (product + (std::int64_t{1} << (31 - shift)))
            >> (32 - shift));
    }
    if (shift >= 32) {
        return saturate_i32(product << (shift - 32));
    }
    const unsigned right = static_cast<unsigned>(32 - shift);
    return right < 63U
        ? saturate_i32(product >> right)
        : (product < 0 ? -1 : 0);
}

std::int32_t db_to_linear_fixed(std::uint32_t dbnorm) noexcept {
    const std::uint32_t remainder = dbnorm % 3U;
    const std::uint32_t exponent = dbnorm / 3U;
    const std::int64_t power = std::int64_t{1} << exponent;
    return saturate_i32(
        (power * kAceLfeDbFactorQ30[remainder] + 0x100000LL) >> 21U);
}

std::int32_t predictor(std::int32_t index) noexcept {
    const std::uint32_t magnitude = static_cast<std::uint32_t>(
        index < 0 ? -static_cast<std::int64_t>(index) : index);
    if (magnitude >= kAceLfePredictorQ30.size()) {
        return 0;
    }
    const std::int32_t value = kAceLfePredictorQ30[magnitude];
    return index < 0 ? -value : value;
}

std::int32_t random_in_lfe_range(std::uint32_t& state) noexcept {
    state = 894955033U * state + 1831589974U;
    constexpr std::int32_t kMinimum = -134217728;
    constexpr std::uint32_t kWidth = 268435456U;
    const std::uint32_t offset = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(state) * kWidth + 0x80000000ULL)
        >> 32U);
    return static_cast<std::int32_t>(
        static_cast<std::uint32_t>(kMinimum) + offset);
}

std::int32_t multiply_q29(
    std::int32_t left,
    std::int32_t right) noexcept {
    return static_cast<std::int32_t>(
        (static_cast<std::int64_t>(left) * right + 0x10000000LL)
        >> 29U);
}

std::int32_t to_q24(std::int32_t sample) noexcept {
    if (sample >= 2147483520) {
        return 0x7FFFFF;
    }
    return ((sample >> 7) + 1) >> 1;
}

} // namespace

std::int32_t ace_fixed_divide_native(
    const int numerator_binary_point,
    const std::int32_t numerator,
    const int denominator_binary_point,
    const std::int32_t denominator,
    const int output_binary_point) noexcept {
    return ace_fixed_divide_native_impl(
        numerator_binary_point, numerator,
        denominator_binary_point, denominator, output_binary_point);
}

AceLfeParseResult parse_ace_lfe_stream(
    const std::uint8_t* bytes,
    std::size_t size,
    std::uint32_t channel_count,
    std::uint32_t frame_duration,
    bool predictive_stream_set,
    AceLfeStreamData& stream) noexcept {
    stream = {};
    if (bytes == nullptr || size == 0U || channel_count == 0U
        || channel_count > 32U || frame_duration == 0U
        || (frame_duration & 63U) != 0U
        || size > std::numeric_limits<std::uint32_t>::max()) {
        return AceLfeParseResult::Invalid;
    }
    AceBitReader source(bytes, size);
    const std::uint32_t decimated_count = frame_duration / 64U;
    stream.channels.reserve(channel_count);
    static constexpr std::uint32_t kResolutionTable[3] = {8U, 10U, 12U};
    for (std::uint32_t index = 0U; index < channel_count; ++index) {
        AceLfeChannelData channel;
        std::uint32_t code = 0U;
        if (!read_ace_uniform(source, 3U, code) || code >= 3U) {
            return AceLfeParseResult::Invalid;
        }
        channel.resolution = kResolutionTable[code];
        if (!source.read(2U, channel.savings)
            || !source.read(6U, channel.dbnorm)) {
            return AceLfeParseResult::Invalid;
        }
        if (channel.savings != 0U) {
            ++channel.savings;
            channel.mode = AceLfeMode::Predictive;
            if (!predictive_stream_set
                || channel.savings >= channel.resolution
                ) {
                return AceLfeParseResult::Invalid;
            }
            std::uint32_t predictor_code_0 = 0U;
            std::uint32_t predictor_code_1 = 0U;
            if (!read_ace_uniform(source, 255U, predictor_code_0)
                || !read_ace_uniform(source, 255U, predictor_code_1)) {
                return AceLfeParseResult::Invalid;
            }
            channel.predictor_index_0 = negative_rice_map(predictor_code_0);
            channel.predictor_index_1 = negative_rice_map(predictor_code_1);
            channel.resolution -= channel.savings;
            if (!read_decimated_values(
                    source,
                    channel.resolution,
                    decimated_count,
                    channel)) {
                return AceLfeParseResult::Invalid;
            }
        } else if (channel.dbnorm <= 45U) {
            channel.mode = AceLfeMode::Absolute;
            if (!read_decimated_values(
                    source,
                    channel.resolution,
                    decimated_count,
                    channel)) {
                return AceLfeParseResult::Invalid;
            }
        } else if (channel.dbnorm <= 60U) {
            channel.mode = AceLfeMode::Reduced;
            channel.resolution -= 4U;
            if (!read_decimated_values(
                    source,
                    channel.resolution,
                    decimated_count,
                    channel)) {
                return AceLfeParseResult::Invalid;
            }
        } else {
            channel.mode = AceLfeMode::Synthetic;
        }
        stream.channels.push_back(std::move(channel));
    }
    if (!source.valid() || source.bits_left() >= 8U) {
        return AceLfeParseResult::Invalid;
    }
    stream.bits_consumed = source.position();
    return AceLfeParseResult::Complete;
}

bool decode_ace_lfe_stream(
    const AceLfeStreamData& stream,
    std::uint32_t frame_duration,
    AceLfeDecoderState& state,
    std::vector<std::vector<float>>& previous_pcm) noexcept {
    if (stream.channels.empty() || frame_duration == 0U
        || (frame_duration & 63U) != 0U) {
        return false;
    }
    const std::size_t decimated_count = frame_duration / 64U;
    if (decimated_count == 0U) {
        return false;
    }
    if (!state.initialized
        || state.previous_decimated.size() != stream.channels.size()) {
        state.previous_decimated.assign(
            stream.channels.size(),
            std::vector<std::int32_t>(decimated_count * 2U + 15U, 0));
        state.initialized = true;
    }
    for (const auto& channel : state.previous_decimated) {
        if (channel.size() != decimated_count * 2U + 15U) {
            state.previous_decimated.assign(
                stream.channels.size(),
                std::vector<std::int32_t>(
                    decimated_count * 2U + 15U, 0));
            break;
        }
    }

    previous_pcm.assign(
        stream.channels.size(),
        std::vector<float>(frame_duration, 0.0F));
    state.last_steps.assign(stream.channels.size(), 0);
    for (std::size_t channel_index = 0U;
         channel_index < stream.channels.size();
         ++channel_index) {
        const AceLfeChannelData& channel = stream.channels[channel_index];
        std::vector<std::int32_t>& history =
            state.previous_decimated[channel_index];
        const std::size_t current_offset = decimated_count + 7U;
        const std::int32_t db_linear = db_to_linear_fixed(channel.dbnorm);
        if (channel.mode == AceLfeMode::Predictive) {
            if (channel.decimated_values.size() != decimated_count
                || channel.resolution == 0U
                || channel.resolution > 30U) {
                return false;
            }
            const std::int32_t p0 = predictor(channel.predictor_index_0) >> 1;
            const std::int32_t p1 = predictor(channel.predictor_index_1) >> 1;
            const std::int32_t feedback0 = static_cast<std::int32_t>(
                (-static_cast<std::int64_t>(p0)
                    * (static_cast<std::int64_t>(p1) + 0x20000000LL)
                    + 0x10000000LL) >> 29U);
            const std::int32_t feedback1 = -p1;
            const std::int64_t denominator =
                ((std::int64_t{1} << (channel.resolution - 1U))
                    * db_linear + 16LL) >> 5U;
            const std::int32_t step = ace_fixed_divide_native_impl(
                2, 1202590848, 4,
                saturate_i32(denominator), 28);
            state.last_steps[channel_index] = step;
            for (std::size_t index = 0U; index < decimated_count; ++index) {
                const std::size_t position = current_offset + index;
                const std::int32_t prior2 = history[position - 2U];
                const std::int32_t prior1 = history[position - 1U];
                const std::int64_t value = multiply_q29(feedback0, prior1)
                    + ((static_cast<std::int64_t>(step)
                        * channel.decimated_values[index] + 1LL) >> 1U)
                    + multiply_q29(feedback1, prior2);
                history[position] = saturate_i32(value);
            }
        } else if (channel.mode == AceLfeMode::Absolute
                   || channel.mode == AceLfeMode::Reduced) {
            if (channel.decimated_values.size() != decimated_count
                || channel.resolution == 0U
                || channel.resolution > 30U) {
                return false;
            }
            const std::int64_t denominator =
                ((std::int64_t{1} << (channel.resolution - 1U))
                    * db_linear + 8LL) >> 4U;
            const std::int32_t step = ace_fixed_divide_native_impl(
                2, 1202590848, 4,
                saturate_i32(denominator), 27);
            state.last_steps[channel_index] = step;
            for (std::size_t index = 0U; index < decimated_count; ++index) {
                history[current_offset + index] = saturate_i32(
                    (static_cast<std::int64_t>(step)
                        * channel.decimated_values[index] + 1LL) >> 1U);
            }
        } else {
        std::int32_t scale = ace_fixed_divide_native_impl(
                2, 928786688, 3, db_linear, 23);
            state.last_steps[channel_index] = scale;
            if (channel.dbnorm == 63U) {
                scale = 0;
            }
            for (std::size_t index = 0U; index < decimated_count; ++index) {
                const std::int32_t noise = random_in_lfe_range(
                    state.random_state);
                history[current_offset + index] = saturate_i32(
                    (static_cast<std::int64_t>(scale) * noise
                        + 0x8000000LL) >> 28U);
            }
        }

        for (std::size_t block = 0U; block < decimated_count; ++block) {
            for (std::size_t phase = 0U; phase < 64U; ++phase) {
                std::int64_t accumulator = 0;
                for (std::size_t tap = 0U; tap < 16U; ++tap) {
                    const std::size_t sample_index = block + 15U - tap;
                    const std::size_t coefficient_index = phase + tap * 64U;
                    accumulator +=
                        (static_cast<std::int64_t>(history[sample_index])
                            * kAceLfeInterpolationFilterQ30[coefficient_index]
                            + 0x20000000LL) >> 30U;
                }
                const std::int32_t interpolated = saturate_i32(
                    accumulator * 2LL);
                previous_pcm[channel_index][block * 64U + phase] =
                    static_cast<float>(to_q24(interpolated))
                    / 8388608.0F;
            }
        }
        std::copy_n(
            history.begin() + static_cast<std::ptrdiff_t>(decimated_count),
            decimated_count + 15U,
            history.begin());
    }
    return true;
}

} // namespace dtsx
