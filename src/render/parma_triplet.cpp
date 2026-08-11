#include "render/parma_triplet.hpp"

#include "render/parma_critical_bands.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace dtsx_decode {
namespace {
constexpr float kEpsilon = 1.0e-12F;
constexpr float kDenormal = 1.0e-18F;
constexpr float kHalfPi = 1.5708F;
constexpr float kTwoOverPi = 0.63662F;
constexpr float kSqrtThreeOverTwo = 0.86603F;

float smooth(float previous, float value, float coefficient) noexcept {
    return ((1.0F - coefficient) * previous + coefficient * value
            + kDenormal) - kDenormal;
}

float coefficient(std::uint32_t sample_rate, float seconds) noexcept {
    return 1.0F - std::exp(-64.0F /
        (static_cast<float>(sample_rate) * seconds));
}

std::uint32_t energy_order(const std::array<float, 3U>& energy) noexcept {
    float angle = std::atan2(
        kSqrtThreeOverTwo * (energy[1U] - energy[2U]),
        energy[0U] - 0.5F * (energy[1U] + energy[2U]));
    if (angle < 0.0F) angle += 6.2832F;
    if (angle <= 2.0944F) return 0U;
    return angle <= 4.1888F ? 1U : 2U;
}

void ordered_energy(
    const std::array<float, 3U>& energy,
    std::uint32_t order,
    float& maximum,
    float& middle,
    float& minimum) noexcept {
    if (order == 0U) {
        maximum = energy[1U]; middle = energy[0U]; minimum = energy[2U];
    } else if (order == 1U) {
        maximum = energy[2U]; middle = energy[1U]; minimum = energy[0U];
    } else {
        maximum = energy[0U]; middle = energy[2U]; minimum = energy[1U];
    }
}

void triplet_coordinates(
    const std::array<float, 3U>& energy,
    std::uint32_t order,
    float& position,
    float& diffuseness) noexcept {
    float maximum = 0.0F, middle = 0.0F, minimum = 0.0F;
    ordered_energy(energy, order, maximum, middle, minimum);
    const float inverse_total = 1.0F /
        (maximum + middle + minimum + kEpsilon);
    float radial_squared = 1.0F - 3.0F * minimum * inverse_total;
    radial_squared = std::clamp(radial_squared, 0.0F, 1.0F);
    diffuseness = std::asin(std::sqrt(radial_squared)) * kTwoOverPi;
    if (radial_squared <= 1.0e-10F) {
        position = 0.5F;
        return;
    }
    const float encoded_radius = std::sin(diffuseness * kHalfPi);
    const float encoded_radius_squared =
        encoded_radius * encoded_radius + kEpsilon;
    const float floor =
        (1.0F - encoded_radius * encoded_radius) * 0.33333F;
    float angular = (middle * inverse_total - floor) /
        encoded_radius_squared;
    angular = std::clamp(angular, 0.0F, 1.0F);
    position = std::acos(std::sqrt(angular)) * kTwoOverPi;
}

void triplet_magnitudes(
    float position,
    float diffuseness,
    std::array<float, 3U>& value) noexcept {
    const float radius = std::sin(diffuseness * kHalfPi);
    const float radial_squared = std::clamp(radius * radius, 0.0F, 1.0F);
    const float floor = (1.0F - radial_squared) * (1.0F / 3.0F);
    const float residual = std::sqrt(std::max(0.0F,
        1.0F - radial_squared));
    const float angular = position * kHalfPi;
    const float cosine = std::cos(angular);
    const float middle = floor + radial_squared * cosine * cosine;
    value[0U] = std::sqrt(std::max(0.0F,
        floor + radial_squared - radial_squared * cosine * cosine)) * residual;
    value[1U] = std::sqrt(std::max(0.0F, middle)) * residual;
    value[2U] = std::sqrt(std::max(0.0F, floor)) * residual;
}

void permute(std::uint32_t order, std::array<float, 3U>& value) noexcept {
    const auto source = value;
    if (order == 0U) {
        value = {source[1U], source[0U], source[2U]};
    } else if (order == 1U) {
        value = {source[2U], source[1U], source[0U]};
    } else {
        value = {source[0U], source[2U], source[1U]};
    }
}
} // namespace

bool ParmaTripletAnalysis::initialize(std::uint32_t sample_rate) noexcept {
    if (sample_rate != 32000U && sample_rate != 44100U
        && sample_rate != 48000U) return false;
    std::vector<std::uint32_t> widths;
    std::vector<float> bark;
    if (!parma_initialize_critical_band_partitions(
            sample_rate, 64U, kBandCount, widths, bark)
        || widths.size() != kBandCount) return false;
    std::copy(widths.begin(), widths.end(), widths_.begin());
    energy_attack_ = coefficient(sample_rate, 0.0063275F);
    energy_release_ = coefficient(sample_rate, 0.012655F);
    // DTS_ParmaDec_SetSampleRate stores these at decoder + 78920.
    control_coefficients_[0U] = energy_attack_;
    control_coefficients_[1U] = coefficient(sample_rate, 0.026332F);
    control_coefficients_[2U] = coefficient(sample_rate, 0.083F);
    control_coefficients_[3U] = coefficient(sample_rate, 0.333F);
    initialized_ = true;
    reset();
    return true;
}

void ParmaTripletAnalysis::reset() noexcept {
    for (auto& values : energy_) values.fill(0.0F);
    for (auto& values : position_energy_) values.fill(0.0F);
    for (auto& values : sum_energy_) values.fill(0.0F);
    for (auto& values : difference_energy_) values.fill(0.0F);
    smoothed_coherence_.fill(0.0F);
}

void ParmaTripletAnalysis::process(
    const std::array<const float*, 3U>& source_real,
    const std::array<const float*, 3U>& source_imaginary,
    std::uint32_t* order, float* position, float* diffuseness,
    float* coherence) noexcept {
    if (!initialized_ || order == nullptr || position == nullptr
        || diffuseness == nullptr || coherence == nullptr) return;
    std::array<std::array<float, 64U>, 3U> raw_energy{};
    std::array<std::array<float, 64U>, 3U> raw_sum{};
    std::array<std::array<float, 64U>, 3U> raw_difference{};
    constexpr std::array<std::array<std::size_t, 2U>, 3U> pairs =
        {{{0U, 1U}, {0U, 2U}, {1U, 2U}}};
    for (std::size_t bin = 0U; bin < 64U; ++bin) {
        std::array<float, 3U> normal_real{}, normal_imaginary{};
        for (std::size_t channel = 0U; channel < 3U; ++channel) {
            const float re = source_real[channel][bin];
            const float im = source_imaginary[channel][bin];
            raw_energy[channel][bin] = re * re + im * im;
            const float normalization =
                1.0F / std::sqrt(raw_energy[channel][bin] + kEpsilon);
            normal_real[channel] = re * normalization;
            normal_imaginary[channel] = im * normalization;
        }
        for (std::size_t pair = 0U; pair < pairs.size(); ++pair) {
            const auto a = pairs[pair][0U], b = pairs[pair][1U];
            const float combined = raw_energy[a][bin] + raw_energy[b][bin];
            const float sum_re = normal_real[a] + normal_real[b];
            const float sum_im = normal_imaginary[a] + normal_imaginary[b];
            const float diff_re = normal_real[a] - normal_real[b];
            const float diff_im = normal_imaginary[a] - normal_imaginary[b];
            raw_sum[pair][bin] = combined *
                (sum_re * sum_re + sum_im * sum_im);
            raw_difference[pair][bin] = combined *
                (diff_re * diff_re + diff_im * diff_im);
        }
    }
    std::array<std::array<float, kBandCount>, 3U> grouped_energy{};
    std::array<std::array<float, kBandCount>, 3U> grouped_sum{};
    std::array<std::array<float, kBandCount>, 3U> grouped_difference{};
    for (std::size_t channel = 0U; channel < 3U; ++channel) {
        parma_group_critical_bands(raw_energy[channel].data(),
            grouped_energy[channel].data(), widths_.data(), kBandCount);
        parma_group_critical_bands(raw_sum[channel].data(),
            grouped_sum[channel].data(), widths_.data(), kBandCount);
        parma_group_critical_bands(raw_difference[channel].data(),
            grouped_difference[channel].data(), widths_.data(), kBandCount);
    }
    std::array<std::uint32_t, kBandCount> raw_order{};
    std::array<float, kBandCount> raw_position{}, raw_diffuseness{};
    std::array<float, kBandCount> raw_coherence{};
    for (std::size_t band = 0U; band < kBandCount; ++band) {
        bool attack = false;
        for (std::size_t channel = 0U; channel < 3U; ++channel)
            attack = attack || grouped_energy[channel][band] > energy_[channel][band];
        for (std::size_t pair = 0U; pair < 3U; ++pair)
            attack = attack || grouped_sum[pair][band] > sum_energy_[pair][band]
                || grouped_difference[pair][band] > difference_energy_[pair][band];
        const float alpha = attack ? energy_attack_ : energy_release_;
        for (std::size_t channel = 0U; channel < 3U; ++channel)
            energy_[channel][band] = smooth(energy_[channel][band],
                grouped_energy[channel][band], alpha);
        for (std::size_t pair = 0U; pair < 3U; ++pair) {
            sum_energy_[pair][band] = smooth(sum_energy_[pair][band],
                grouped_sum[pair][band], alpha);
            difference_energy_[pair][band] = smooth(
                difference_energy_[pair][band],
                grouped_difference[pair][band], alpha);
        }
        std::array<float, 3U> first_pass = {
            energy_[0U][band], energy_[1U][band], energy_[2U][band]};
        raw_order[band] = energy_order(first_pass);
        triplet_coordinates(first_pass, raw_order[band],
            raw_position[band], raw_diffuseness[band]);
        float weighted_coherence = 0.0F;
        float product_sum = 0.0F;
        for (std::size_t pair = 0U; pair < pairs.size(); ++pair) {
            const float product = energy_[pairs[pair][0U]][band]
                * energy_[pairs[pair][1U]][band];
            product_sum += product;
            weighted_coherence += product * sum_energy_[pair][band]
                / (sum_energy_[pair][band]
                    + difference_energy_[pair][band] + kEpsilon);
        }
        raw_coherence[band] = std::clamp(
            1.2F * weighted_coherence / (product_sum + kEpsilon) - 0.1F,
            0.0F, 1.0F);
        const float metric = raw_diffuseness[band]
            + std::max(0.0F, 2.0F * raw_coherence[band] - 1.0F);
        const float control = metric >= 0.9F ? control_coefficients_[0U]
            : metric >= 0.7F ? control_coefficients_[1U]
            : metric >= 0.25F ? control_coefficients_[2U]
            : control_coefficients_[3U];
        for (std::size_t channel = 0U; channel < 3U; ++channel)
            position_energy_[channel][band] = smooth(
                position_energy_[channel][band], energy_[channel][band], control);
        smoothed_coherence_[band] = smooth(smoothed_coherence_[band],
            raw_coherence[band], control);
        const std::array<float, 3U> final_energy = {
            position_energy_[0U][band], position_energy_[1U][band],
            position_energy_[2U][band]};
        order[band] = energy_order(final_energy);
        triplet_coordinates(final_energy, order[band],
            position[band], diffuseness[band]);
        coherence[band] = 1.0F -
            std::max(0.0F, 2.0F * smoothed_coherence_[band] - 1.0F);
    }
    std::array<float, 64U> expanded_position{}, expanded_diffuseness{},
        expanded_coherence{};
    parma_ungroup_critical_bands(position, expanded_position.data(),
        widths_.data(), kBandCount);
    parma_ungroup_critical_bands(diffuseness, expanded_diffuseness.data(),
        widths_.data(), kBandCount);
    parma_ungroup_critical_bands(coherence, expanded_coherence.data(),
        widths_.data(), kBandCount);
    std::copy(expanded_position.begin(), expanded_position.end(), position);
    std::copy(expanded_diffuseness.begin(), expanded_diffuseness.end(), diffuseness);
    std::copy(expanded_coherence.begin(), expanded_coherence.end(), coherence);
    std::array<std::uint32_t, 64U> expanded_order{};
    std::size_t offset = 0U;
    for (std::size_t band = 0U; band < kBandCount; ++band)
        for (std::size_t i = 0U; i < widths_[band]; ++i)
            expanded_order[offset++] = order[band];
    std::copy(expanded_order.begin(), expanded_order.end(), order);
}

void parma_extract_matrixed_triplet_channel(
    const std::uint32_t* order, const float* position,
    const float* diffuseness,
    const std::array<const float*, 3U>& source_real,
    const std::array<const float*, 3U>& source_imaginary,
    float* target_real, float* target_imaginary,
    std::size_t count) noexcept {
    for (std::size_t bin = 0U; bin < count; ++bin) {
        std::array<float, 3U> gain{};
        triplet_magnitudes(position[bin], diffuseness[bin], gain);
        permute(order[bin], gain);
        target_real[bin] = gain[0U] * source_real[0U][bin]
            + gain[1U] * source_real[1U][bin]
            + gain[2U] * source_real[2U][bin];
        target_imaginary[bin] = gain[0U] * source_imaginary[0U][bin]
            + gain[1U] * source_imaginary[1U][bin]
            + gain[2U] * source_imaginary[2U][bin];
    }
}

void parma_repan_triplet_channels(
    const std::uint32_t* order, const float* position,
    const float* diffuseness, const float* coherence,
    const std::array<const float*, 3U>& source_real,
    const std::array<const float*, 3U>& source_imaginary,
    const std::array<float*, 3U>& target_real,
    const std::array<float*, 3U>& target_imaginary,
    std::size_t count) noexcept {
    for (std::size_t bin = 0U; bin < count; ++bin) {
        const float radius = std::sin(diffuseness[bin] * kHalfPi);
        const float radial_squared = std::clamp(
            radius * radius, 0.0F, 1.0F);
        const float residual_floor = 1.0F - radial_squared;
        const float angle = position[bin] * kHalfPi;
        const float angular = std::cos(angle);
        const float angular_squared = angular * angular;
        const float complementary_squared = 1.0F - angular_squared;
        const float common = std::sqrt(residual_floor / 3.0F);
        const float cross_floor = std::sqrt(residual_floor / 6.0F);
        std::array<float, 3U> magnitude = {
            std::sqrt(radial_squared + residual_floor * (2.0F / 3.0F)),
            std::sqrt(radial_squared * angular_squared
                + residual_floor / 3.0F),
            std::sqrt(radial_squared * complementary_squared
                + residual_floor / 3.0F)};
        std::array<float, 3U> numerator = {
            magnitude[0U] * magnitude[1U]
                - std::sqrt(radial_squared * angular_squared),
            magnitude[0U] * magnitude[2U]
                - std::sqrt(radial_squared * complementary_squared),
            magnitude[0U] * common};
        const std::array<float, 3U> denominator = {
            (magnitude[2U] + common) * cross_floor + kEpsilon,
            (magnitude[1U] + common) * cross_floor + kEpsilon,
            (magnitude[1U] + magnitude[2U]) * cross_floor + kEpsilon};
        std::array<float, 3U> retain{}, cancellation{};
        const float residual = std::clamp(coherence[bin], 0.0F, 1.0F);
        for (std::size_t channel = 0U; channel < 3U; ++channel) {
            retain[channel] = std::sqrt(residual
                + (1.0F - residual) * magnitude[0U] * magnitude[0U]);
            cancellation[channel] = std::sqrt(1.0F - residual)
                * (numerator[channel] / denominator[channel]) * cross_floor;
        }
        permute(order[bin], retain);
        permute(order[bin], cancellation);
        for (std::size_t channel = 0U; channel < 3U; ++channel) {
            const std::size_t a = (channel + 1U) % 3U;
            const std::size_t b = (channel + 2U) % 3U;
            target_real[channel][bin] = retain[channel]
                    * target_real[channel][bin]
                - cancellation[channel] * source_real[a][bin]
                - cancellation[channel] * source_real[b][bin];
            target_imaginary[channel][bin] = retain[channel]
                    * target_imaginary[channel][bin]
                - cancellation[channel] * source_imaginary[a][bin]
                - cancellation[channel] * source_imaginary[b][bin];
        }
    }
}

} // namespace dtsx_decode
