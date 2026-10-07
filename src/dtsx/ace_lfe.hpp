#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx {

enum class AceLfeMode {
    Absolute,
    Predictive,
    Reduced,
    Synthetic,
};

struct AceLfeChannelData final {
    AceLfeMode mode = AceLfeMode::Absolute;
    std::uint32_t resolution = 0U;
    std::uint32_t savings = 0U;
    std::uint32_t dbnorm = 0U;
    std::int32_t predictor_index_0 = 0;
    std::int32_t predictor_index_1 = 0;
    std::uint32_t golomb_parameter = 0U;
    std::vector<std::int32_t> decimated_values;
};

// Native dts_flib_div fixed-point divider used by ACE LFE/THF paths.
[[nodiscard]] std::int32_t ace_fixed_divide_native(
    int numerator_binary_point,
    std::int32_t numerator,
    int denominator_binary_point,
    std::int32_t denominator,
    int output_binary_point) noexcept;

struct AceLfeStreamData final {
    std::vector<AceLfeChannelData> channels;
    std::size_t bits_consumed = 0U;
};

struct AceLfeDecoderState final {
    bool initialized = false;
    std::uint32_t random_state = 0U;
    std::vector<std::vector<std::int32_t>> previous_decimated;
    std::vector<std::int32_t> last_steps;
};

enum class AceLfeParseResult {
    Complete,
    Invalid,
    Unsupported,
};

[[nodiscard]] AceLfeParseResult parse_ace_lfe_stream(
    const std::uint8_t* bytes,
    std::size_t size,
    std::uint32_t channel_count,
    std::uint32_t frame_duration,
    bool predictive_stream_set,
    AceLfeStreamData& stream) noexcept;

[[nodiscard]] bool decode_ace_lfe_stream(
    const AceLfeStreamData& stream,
    std::uint32_t frame_duration,
    AceLfeDecoderState& state,
    std::vector<std::vector<float>>& previous_pcm) noexcept;

} // namespace dtsx
