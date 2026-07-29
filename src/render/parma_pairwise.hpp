#pragma once

#include <cstddef>

namespace dtsx_decode {

void parma_filterbank_phase_shift(
    float* real,
    float* imaginary,
    float radians,
    std::size_t count) noexcept;

void parma_compute_downmatrix_coefficients_pairwise(
    const float* position,
    const float* balance,
    const float* diffuseness,
    float* first,
    float* second,
    float minimum_angle,
    float first_decode_angle,
    float second_decode_angle,
    float maximum_angle,
    std::size_t count) noexcept;

void parma_compute_repan_coefficients_pairwise(
    const float* position,
    const float* balance,
    const float* diffuseness,
    float* first,
    float* second,
    float minimum_angle,
    float maximum_angle,
    std::size_t count) noexcept;

void parma_extract_matrixed_pairwise_channel(
    const float* position,
    const float* balance,
    const float* diffuseness,
    const float* first_source_real,
    const float* first_source_imaginary,
    const float* second_source_real,
    const float* second_source_imaginary,
    float* target_real,
    float* target_imaginary,
    float minimum_angle,
    float decode_angle,
    float maximum_angle,
    std::size_t count) noexcept;

void parma_repan_pairwise_channel(
    const float* position,
    const float* balance,
    const float* diffuseness,
    const float* first_source_real,
    const float* first_source_imaginary,
    const float* second_source_real,
    const float* second_source_imaginary,
    float* first_target_real,
    float* first_target_imaginary,
    float* second_target_real,
    float* second_target_imaginary,
    float minimum_angle,
    float maximum_angle,
    std::size_t count) noexcept;

} // namespace dtsx_decode
