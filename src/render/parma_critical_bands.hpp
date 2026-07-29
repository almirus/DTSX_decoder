#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx_decode {

[[nodiscard]] bool parma_initialize_critical_band_partitions(
    std::uint32_t sample_rate,
    std::size_t frequency_bin_count,
    std::size_t partition_count,
    std::vector<std::uint32_t>& widths,
    std::vector<float>& scale) noexcept;

void parma_group_critical_bands(
    const float* input,
    float* output,
    const std::uint32_t* widths,
    std::size_t partition_count) noexcept;

void parma_ungroup_critical_bands(
    const float* input,
    float* output,
    const std::uint32_t* widths,
    std::size_t partition_count) noexcept;

} // namespace dtsx_decode
