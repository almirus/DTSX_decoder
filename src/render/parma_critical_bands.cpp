#include "render/parma_critical_bands.hpp"

#include <cmath>

namespace dtsx_decode {

bool parma_initialize_critical_band_partitions(
    std::uint32_t sample_rate,
    std::size_t frequency_bin_count,
    std::size_t partition_count,
    std::vector<std::uint32_t>& widths,
    std::vector<float>& scale) noexcept {
    // libdtsx.so: DTS_CritBandInitPartitions, 0x76764.
    widths.clear();
    scale.clear();
    if (sample_rate == 0U
        || frequency_bin_count == 0U
        || partition_count == 0U) {
        return false;
    }

    try {
        scale.resize(frequency_bin_count);
        widths.assign(partition_count, 0U);
    } catch (...) {
        widths.clear();
        scale.clear();
        return false;
    }

    const float rate = static_cast<float>(sample_rate);
    const float bin_count =
        static_cast<float>(frequency_bin_count);
    for (std::size_t bin = 0U;
         bin < frequency_bin_count;
         ++bin) {
        const float frequency =
            ((static_cast<float>(bin) + 0.5F) * 0.5F)
            * rate / bin_count;
        const float low =
            std::atan(frequency * 0.00076F);
        const float high = frequency / 7500.0F;
        scale[bin] =
            std::atan(high * high) * 3.5F
            + low * 13.0F;
    }

    const float partition_step =
        scale.back()
        / static_cast<float>(partition_count);
    std::size_t consumed = 0U;
    for (std::size_t partition = 0U;
         partition < partition_count;
         ++partition) {
        if (partition == 0U) {
            widths[partition] = 1U;
            consumed = 1U;
        } else {
            widths[partition] =
                widths[partition - 1U];
            consumed += widths[partition];
        }

        const float boundary =
            static_cast<float>(partition + 1U)
            * partition_step;
        while (consumed < frequency_bin_count
               && boundary > scale[consumed]) {
            ++widths[partition];
            ++consumed;
        }
    }
    if (consumed < frequency_bin_count) {
        widths.back() += static_cast<std::uint32_t>(
            frequency_bin_count - consumed);
        consumed = frequency_bin_count;
    }
    return consumed == frequency_bin_count;
}

void parma_group_critical_bands(
    const float* input,
    float* output,
    const std::uint32_t* widths,
    std::size_t partition_count) noexcept {
    // libdtsx.so: DTS_CritBandGroup, 0x76948.
    std::size_t input_offset = 0U;
    for (std::size_t partition = 0U;
         partition < partition_count;
         ++partition) {
        float sum = 0.0F;
        for (std::uint32_t bin = 0U;
             bin < widths[partition];
             ++bin) {
            sum += input[input_offset++];
        }
        output[partition] = sum;
    }
}

void parma_ungroup_critical_bands(
    const float* input,
    float* output,
    const std::uint32_t* widths,
    std::size_t partition_count) noexcept {
    // libdtsx.so: DTS_CritBandUnGroup, 0x769b4.
    std::size_t output_offset = 0U;
    for (std::size_t partition = 0U;
         partition < partition_count;
         ++partition) {
        for (std::uint32_t bin = 0U;
             bin < widths[partition];
             ++bin) {
            output[output_offset++] = input[partition];
        }
    }
}

} // namespace dtsx_decode
