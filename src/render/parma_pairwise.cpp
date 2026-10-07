#include "render/parma_pairwise.hpp"

#include <array>
#include <cmath>

namespace dtsx_decode {

void parma_filterbank_phase_shift(
    float* real,
    float* imaginary,
    float radians,
    std::size_t count) noexcept {
    const float cosine = std::cos(radians);
    const float sine = std::sin(radians);
    for (std::size_t index = 0U; index < count; ++index) {
        const float shifted_imaginary =
            cosine * imaginary[index]
            + sine * real[index];
        real[index] =
            real[index] * cosine
            - imaginary[index] * sine;
        imaginary[index] = shifted_imaginary;
    }
}

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
    std::size_t count) noexcept {
    constexpr float kHalfPi = 1.5708F;
    const float decode_midpoint =
        (first_decode_angle + second_decode_angle) * 0.5F;
    for (std::size_t index = 0U; index < count; ++index) {
        const float diffuse = diffuseness[index];
        const float direct = 1.0F - diffuse;
        const float source_angle =
            (1.0F - decode_midpoint)
            + direct
                * (position[index]
                    - (1.0F - decode_midpoint));
        const float blend =
            direct * (balance[index] - 0.5F) + 0.5F;

        float mirror_angle = -source_angle;
        if (minimum_angle < source_angle) {
            if (first_decode_angle >= source_angle) {
                mirror_angle =
                    (source_angle - minimum_angle)
                        * (1.0F
                           / (first_decode_angle
                              - minimum_angle))
                    - source_angle;
            } else if (second_decode_angle < source_angle) {
                if (maximum_angle < source_angle) {
                    mirror_angle = 2.0F - source_angle;
                } else {
                    mirror_angle =
                        source_angle - second_decode_angle;
                }
                if (maximum_angle >= source_angle) {
                    mirror_angle =
                        mirror_angle
                            * (1.0F
                               / (maximum_angle
                                  - second_decode_angle))
                        - source_angle + 1.0F;
                }
            } else {
                mirror_angle = 1.0F - source_angle;
            }
        }

        const float angle =
            ((1.0F - blend) * source_angle
             + blend * mirror_angle)
            * kHalfPi;
        first[index] = std::sin(angle);
        second[index] = std::cos(angle);
    }
}

void parma_compute_repan_coefficients_pairwise(
    const float* position,
    const float* balance,
    const float* diffuseness,
    float* first,
    float* second,
    float minimum_angle,
    float maximum_angle,
    std::size_t count) noexcept {
    constexpr float kHalfPi = 1.5708F;
    for (std::size_t index = 0U; index < count; ++index) {
        const float direct = 1.0F - diffuseness[index];
        const float source_angle =
            direct * position[index];
        const float blend =
            direct * (balance[index] - 0.5F) + 0.5F;
        const float inverse_blend = 1.0F - blend;

        float first_position = source_angle;
        if (minimum_angle >= source_angle) {
            float extension = 1.0F;
            extension -=
                source_angle * (1.0F / minimum_angle);
            first_position = source_angle + extension;
        }
        const float first_angle =
            (inverse_blend + blend * first_position)
            * kHalfPi;
        first[index] = std::sin(first_angle);
        second[index] = std::cos(first_angle);

        float inverse_position = 1.0F - source_angle;
        if (maximum_angle >= inverse_position) {
            const float extension =
                1.0F
                - inverse_position
                    * (1.0F / maximum_angle);
            inverse_position += extension;
        }
        const float second_angle =
            (inverse_blend + blend * inverse_position)
            * kHalfPi;
        first[count + index] = std::sin(second_angle);
        second[count + index] = std::cos(second_angle);
    }
}

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
    std::size_t count) noexcept {
    if (count == 0U || count > 64U) {
        return;
    }
    std::array<float, 64U> first{};
    std::array<float, 64U> second{};
    parma_compute_downmatrix_coefficients_pairwise(
        position,
        balance,
        diffuseness,
        first.data(),
        second.data(),
        minimum_angle,
        decode_angle,
        decode_angle,
        maximum_angle,
        count);
    for (std::size_t bin = 0U; bin < count; ++bin) {
        target_real[bin] =
            second_source_real[bin] * second[bin]
            + first_source_real[bin] * first[bin];
        target_imaginary[bin] =
            second_source_imaginary[bin] * second[bin]
            + first_source_imaginary[bin] * first[bin];
    }
}

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
    std::size_t count) noexcept {
    if (count == 0U || count > 64U) {
        return;
    }
    std::array<float, 128U> sine{};
    std::array<float, 128U> cosine{};
    parma_compute_repan_coefficients_pairwise(
        position,
        balance,
        diffuseness,
        sine.data(),
        cosine.data(),
        minimum_angle,
        maximum_angle,
        count);
    for (std::size_t bin = 0U; bin < count; ++bin) {
        first_target_real[bin] =
            sine[bin] * first_target_real[bin]
            - second_source_real[bin] * cosine[bin];
        first_target_imaginary[bin] =
            sine[bin] * first_target_imaginary[bin]
            - second_source_imaginary[bin] * cosine[bin];
        second_target_real[bin] =
            sine[count + bin] * second_target_real[bin]
            - first_source_real[bin] * cosine[count + bin];
        second_target_imaginary[bin] =
            sine[count + bin] * second_target_imaginary[bin]
            - first_source_imaginary[bin] * cosine[count + bin];
    }
}

} // namespace dtsx_decode
