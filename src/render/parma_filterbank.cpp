#include "render/parma_filterbank.hpp"

#include "render/parma_filterbank_coefficients.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstring>

namespace dtsx_decode {
namespace {

constexpr std::size_t kBandCount = 64U;
constexpr std::size_t kTransformSize = 128U;
constexpr std::size_t kPrototypeSize = 1024U;
constexpr float kPi = 3.14159265F;

float coefficient(
    const std::array<std::uint32_t, kPrototypeSize>& words,
    std::size_t index) noexcept {
    float value = 0.0F;
    const std::uint32_t word = words[index & (kPrototypeSize - 1U)];
    std::memcpy(&value, &word, sizeof(value));
    return value;
}

std::complex<float> first_modulation(std::size_t index) noexcept {
    // dts_flib_osfb_f32_t_initialize / DTS_ParmaDec_OSFilter_Init.
    const float angle =
        -static_cast<float>(index)
        * (static_cast<float>(kBandCount) - 0.5F)
        * 2.0F * kPi
        / static_cast<float>(kTransformSize);
    return {std::cos(angle), std::sin(angle)};
}

std::complex<float> second_modulation(std::size_t index) noexcept {
    const float angle =
        (static_cast<float>(index)
         - (static_cast<float>(kBandCount) - 0.5F))
        * kPi / static_cast<float>(kTransformSize);
    return {std::cos(angle), std::sin(angle)};
}

void forward_fft(
    std::array<std::complex<float>, kTransformSize>& values) noexcept {
    // dts_flib_fft_c_i_f32_t_forward uses the unscaled forward transform.
    for (std::size_t index = 1U, reverse = 0U;
         index < kTransformSize;
         ++index) {
        std::size_t bit = kTransformSize >> 1U;
        while ((reverse & bit) != 0U) {
            reverse ^= bit;
            bit >>= 1U;
        }
        reverse ^= bit;
        if (index < reverse) {
            std::swap(values[index], values[reverse]);
        }
    }
    for (std::size_t length = 2U;
         length <= kTransformSize;
         length <<= 1U) {
        const float angle =
            -2.0F * kPi / static_cast<float>(length);
        const std::complex<float> step{
            std::cos(angle), std::sin(angle)};
        for (std::size_t base = 0U;
             base < kTransformSize;
             base += length) {
            std::complex<float> phase{1.0F, 0.0F};
            const std::size_t half = length >> 1U;
            for (std::size_t offset = 0U;
                 offset < half;
                 ++offset) {
                const std::complex<float> even =
                    values[base + offset];
                const std::complex<float> odd =
                    values[base + offset + half] * phase;
                values[base + offset] = even + odd;
                values[base + offset + half] = even - odd;
                phase *= step;
            }
        }
    }
}

} // namespace

ParmaAnalysisFilterBank::ParmaAnalysisFilterBank() noexcept {
    reset();
}

void ParmaAnalysisFilterBank::reset() noexcept {
    delay_.fill(0.0F);
    position_ = 0U;
}

void ParmaAnalysisFilterBank::process(
    const float* input,
    float* real,
    float* imaginary) noexcept {
    // libdtsx.so: dts_flib_osfb_f32_t_analysis, 0x11d588.
    if (input == nullptr || real == nullptr || imaginary == nullptr) {
        return;
    }
    const std::size_t old_position = position_;
    position_ = (position_ + kBandCount) & (kPrototypeSize - 1U);
    for (std::size_t sample = 0U; sample < kBandCount; ++sample) {
        delay_[(old_position + kBandCount - 1U - sample)
               & (kPrototypeSize - 1U)] = input[sample];
    }

    const std::size_t half_position = old_position & kBandCount;
    const std::size_t coefficient_base =
        (half_position
         + ((kPrototypeSize - old_position) & 3968U))
        & (kPrototypeSize - 1U);
    std::array<std::complex<float>, kTransformSize> transform{};
    for (std::size_t bin = 0U; bin < kBandCount; ++bin) {
        float sum = 0.0F;
        for (std::size_t tap = 0U; tap < 8U; ++tap) {
            const std::size_t offset =
                half_position + bin + tap * kTransformSize;
            sum += delay_[offset & (kPrototypeSize - 1U)]
                * coefficient(
                    detail::kParmaAnalysisCoefficients,
                    coefficient_base + offset);
        }
        const std::complex<float> modulation =
            second_modulation(bin);
        // dts_flib_osfb_f32_t_analysis writes the sine component first
        // and the cosine component second into the interleaved FFT input.
        transform[bin] = {
            modulation.imag() * sum,
            modulation.real() * sum};
    }
    for (std::size_t bin = 0U; bin < kBandCount; ++bin) {
        float sum = 0.0F;
        const std::size_t start =
            kBandCount - half_position + bin;
        for (std::size_t tap = 0U; tap < 8U; ++tap) {
            const std::size_t offset =
                start + tap * kTransformSize;
            sum += delay_[offset & (kPrototypeSize - 1U)]
                * coefficient(
                    detail::kParmaAnalysisCoefficients,
                    coefficient_base + offset);
        }
        const std::complex<float> modulation =
            second_modulation(kBandCount + bin);
        transform[kBandCount + bin] = {
            modulation.imag() * sum,
            modulation.real() * sum};
    }

    forward_fft(transform);
    for (std::size_t bin = 0U; bin < kBandCount; ++bin) {
        const std::complex<float> modulation =
            first_modulation(bin);
        const float fft_real = transform[bin].real();
        const float fft_imaginary = transform[bin].imag();
        real[bin] =
            2.0F
            * (fft_imaginary * modulation.real()
               - modulation.imag() * fft_real);
        imaginary[bin] =
            2.0F
            * (modulation.real() * fft_real
               + modulation.imag() * fft_imaginary);
    }
}

ParmaSynthesisFilterBank::ParmaSynthesisFilterBank() noexcept {
    reset();
}

void ParmaSynthesisFilterBank::reset() noexcept {
    first_delay_.fill(0.0F);
    second_delay_.fill(0.0F);
    history_.fill(0.0F);
    position_ = 0U;
}

void ParmaSynthesisFilterBank::process(
    const float* real,
    const float* imaginary,
    float* output) noexcept {
    // libdtsx.so: DTS_ParmaDec_OSFilter_Synthesis, 0x76048.
    if (real == nullptr || imaginary == nullptr || output == nullptr) {
        return;
    }
    std::array<std::complex<float>, kTransformSize> transform{};
    for (std::size_t bin = 0U; bin < kBandCount; ++bin) {
        const std::complex<float> modulation =
            first_modulation(bin);
        transform[bin] = {
            real[bin] * modulation.real()
                + imaginary[bin] * modulation.imag(),
            imaginary[bin] * modulation.real()
                - real[bin] * modulation.imag()};
    }
    forward_fft(transform);

    const std::size_t old_position = position_;
    const std::size_t half_position = old_position & kBandCount;
    const std::size_t coefficient_base =
        (half_position
         + ((kPrototypeSize - old_position) & 3968U)
         - kTransformSize)
        & (kPrototypeSize - 1U);
    position_ = (position_ + kBandCount) & (kPrototypeSize - 1U);

    for (std::size_t bin = 0U; bin < kBandCount; ++bin) {
        const std::complex<float> modulation =
            second_modulation(bin);
        first_delay_[(old_position + bin)
                     & (kPrototypeSize - 1U)] =
            transform[bin].real() * modulation.real()
            + transform[bin].imag() * modulation.imag();
    }
    for (std::size_t bin = 0U; bin < kBandCount; ++bin) {
        const std::size_t transform_bin = kBandCount + bin;
        const std::complex<float> modulation =
            second_modulation(transform_bin);
        second_delay_[(old_position + bin)
                      & (kPrototypeSize - 1U)] =
            transform[transform_bin].real()
                * modulation.real()
            + transform[transform_bin].imag()
                * modulation.imag();
    }

    const std::size_t convolution_position =
        half_position + kBandCount;
    for (std::size_t sample = 0U;
         sample < kBandCount;
         ++sample) {
        float sum = history_[sample];
        for (std::size_t tap = 0U; tap < 8U; ++tap) {
            sum += second_delay_[
                       (half_position + sample
                        + tap * kTransformSize)
                       & (kPrototypeSize - 1U)]
                * coefficient(
                    detail::kParmaSynthesisCoefficients,
                    coefficient_base + convolution_position
                        + sample + tap * kTransformSize);
        }
        output[kBandCount - 1U - sample] =
            sum * static_cast<float>(kBandCount);
    }

    for (std::size_t sample = 0U;
         sample < kBandCount;
         ++sample) {
        float sum = 0.0F;
        for (std::size_t tap = 0U; tap < 8U; ++tap) {
            sum += first_delay_[
                       (half_position + sample
                        + tap * kTransformSize)
                       & (kPrototypeSize - 1U)]
                * coefficient(
                    detail::kParmaSynthesisCoefficients,
                    coefficient_base + half_position + sample
                        + tap * kTransformSize);
        }
        history_[sample] = sum;
    }
}

void ParmaSynthesisFilterBank::process_x2(
    const float* first_real,
    const float* first_imaginary,
    ParmaSynthesisFilterBank& second_filter,
    const float* second_real,
    const float* second_imaginary,
    float* first_output,
    float* second_output) noexcept {
    // visio-libdtsx.so: DTS_ParmaDec_OSFilter_Synthesis_x2, 0xf9ba8.
    // The native routine packs two complex subband vectors into one FFT;
    // calling the scalar synthesis twice changes cancellation and rounding.
    if (first_real == nullptr || first_imaginary == nullptr
        || second_real == nullptr || second_imaginary == nullptr
        || first_output == nullptr || second_output == nullptr
        || position_ != second_filter.position_) {
        return;
    }

    std::array<std::complex<float>, kTransformSize> transform{};
    for (std::size_t bin = 0U; bin < kBandCount; ++bin) {
        const std::complex<float> current = first_modulation(bin);
        const std::complex<float> reverse =
            first_modulation(kTransformSize - 1U - bin);
        const float ar = first_real[bin];
        const float ai = first_imaginary[bin];
        const float br = second_real[bin];
        const float bi = second_imaginary[bin];
        const float sum_imaginary_real = ai + br;
        const float difference_real_imaginary = ar - bi;
        transform[bin] = {
            current.real() * difference_real_imaginary
                + current.imag() * sum_imaginary_real,
            current.real() * sum_imaginary_real
                - current.imag() * difference_real_imaginary};
        const float difference_imaginary_real = ai - br;
        const float negative_real_imaginary = -ar - bi;
        transform[kTransformSize - 1U - bin] = {
            reverse.real() * negative_real_imaginary
                + reverse.imag() * difference_imaginary_real,
            reverse.real() * difference_imaginary_real
                - reverse.imag() * negative_real_imaginary};
    }
    forward_fft(transform);

    const std::size_t old_position = position_;
    const std::size_t half_position = old_position & kBandCount;
    const std::size_t coefficient_base =
        (half_position
         + ((kPrototypeSize - old_position) & 3968U)
         - kTransformSize)
        & (kPrototypeSize - 1U);
    position_ = (position_ + kBandCount) & (kPrototypeSize - 1U);
    second_filter.position_ = position_;

    for (std::size_t bin = 0U; bin < kTransformSize; ++bin) {
        const std::complex<float> modulation = second_modulation(bin);
        const float packed_real = transform[bin].real();
        const float packed_imaginary = transform[bin].imag();
        const float first_value =
            packed_real * modulation.real()
            + packed_imaginary * modulation.imag();
        const float second_value =
            packed_imaginary * modulation.real()
            - packed_real * modulation.imag();
        if (bin < kBandCount) {
            first_delay_[(old_position + bin)
                         & (kPrototypeSize - 1U)] = first_value;
            second_filter.first_delay_[
                (old_position + bin) & (kPrototypeSize - 1U)] =
                second_value;
        } else {
            const std::size_t delay_bin = bin - kBandCount;
            second_delay_[(old_position + delay_bin)
                          & (kPrototypeSize - 1U)] = first_value;
            second_filter.second_delay_[
                (old_position + delay_bin) & (kPrototypeSize - 1U)] =
                second_value;
        }
    }

    const std::size_t convolution_position =
        half_position + kBandCount;
    for (std::size_t sample = 0U; sample < kBandCount; ++sample) {
        float first_sum = history_[sample];
        float second_sum = second_filter.history_[sample];
        for (std::size_t tap = 0U; tap < 8U; ++tap) {
            const float prototype = coefficient(
                detail::kParmaSynthesisCoefficients,
                coefficient_base + convolution_position
                    + sample + tap * kTransformSize);
            first_sum += second_delay_[
                (half_position + sample + tap * kTransformSize)
                    & (kPrototypeSize - 1U)] * prototype;
            second_sum += second_filter.second_delay_[
                (half_position + sample + tap * kTransformSize)
                    & (kPrototypeSize - 1U)] * prototype;
        }
        first_output[kBandCount - 1U - sample] =
            first_sum * static_cast<float>(kBandCount) * 0.5F;
        second_output[kBandCount - 1U - sample] =
            second_sum * static_cast<float>(kBandCount) * 0.5F;
    }

    for (std::size_t sample = 0U; sample < kBandCount; ++sample) {
        float first_sum = 0.0F;
        float second_sum = 0.0F;
        for (std::size_t tap = 0U; tap < 8U; ++tap) {
            const float prototype = coefficient(
                detail::kParmaSynthesisCoefficients,
                coefficient_base + half_position + sample
                    + tap * kTransformSize);
            first_sum += first_delay_[
                (half_position + sample + tap * kTransformSize)
                    & (kPrototypeSize - 1U)] * prototype;
            second_sum += second_filter.first_delay_[
                (half_position + sample + tap * kTransformSize)
                    & (kPrototypeSize - 1U)] * prototype;
        }
        history_[sample] = first_sum;
        second_filter.history_[sample] = second_sum;
    }
}

} // namespace dtsx_decode
