#include "render/parma_quadruplet.hpp"

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

float smooth(float previous, float value, float coefficient) noexcept {
    return ((1.0F - coefficient) * previous + coefficient * value
            + kDenormal) - kDenormal;
}

float smoothing_coefficient(std::uint32_t sample_rate, float seconds) noexcept {
    return 1.0F - std::exp(-64.0F /
        (static_cast<float>(sample_rate) * seconds));
}

std::uint32_t energy_order(const std::array<float, 4U>& energy) noexcept {
    float angle = std::atan2(energy[1U] - energy[3U],
        energy[0U] - energy[2U]);
    if (angle < 0.0F) angle += 6.2832F;
    if (angle <= 1.5708F) return 0U;
    if (angle <= 3.1416F) return 1U;
    if (angle <= 4.7124F) return 2U;
    return 3U;
}

void rotate_to_canonical(std::uint32_t order,
                         const std::array<float, 4U>& source,
                         std::array<float, 4U>& value) noexcept {
    for (std::size_t channel = 0U; channel < 4U; ++channel)
        value[channel] = source[(channel + order) & 3U];
}

void rotate_from_canonical(std::uint32_t order,
                           std::array<float, 4U>& value) noexcept {
    const auto source = value;
    for (std::size_t channel = 0U; channel < 4U; ++channel)
        value[(channel + order) & 3U] = source[channel];
}

void quadruplet_coordinates(const std::array<float, 4U>& energy,
                            std::uint32_t order, float& position,
                            float& diffuseness) noexcept {
    std::array<float, 4U> canonical{};
    rotate_to_canonical(order, energy, canonical);
    const float total = canonical[0U] + canonical[1U]
        + canonical[2U] + canonical[3U];
    const float inverse_total = 1.0F / (total + kEpsilon);
    const float minimum = std::min(std::min(canonical[0U], canonical[1U]),
        std::min(canonical[2U], canonical[3U]));
    float radial_squared = 1.0F - 4.0F * minimum * inverse_total;
    radial_squared = std::clamp(radial_squared, 0.0F, 1.0F);
    diffuseness = std::asin(std::sqrt(radial_squared)) * kTwoOverPi;
    if (radial_squared <= 1.0e-10F) {
        position = 0.5F;
        return;
    }
    const float radius = std::sin(diffuseness * kHalfPi);
    const float radius_squared = radius * radius + kEpsilon;
    const float floor = (1.0F - radius * radius) * 0.25F;
    float angular = (canonical[0U] * inverse_total - floor)
        / radius_squared;
    angular = std::clamp(angular, 0.0F, 1.0F);
    position = std::acos(std::sqrt(angular)) * kTwoOverPi;
}

void quadruplet_magnitudes(float position, float diffuseness,
                           std::array<float, 4U>& value) noexcept {
    const float radius = std::sin(diffuseness * kHalfPi);
    const float radial_squared = std::clamp(radius * radius, 0.0F, 1.0F);
    const float residual = std::sqrt(std::max(0.0F, 1.0F - radial_squared));
    const float floor = (1.0F - radial_squared) * 0.25F;
    const float cosine = std::cos(position * kHalfPi);
    const float cosine_squared = cosine * cosine;
    value[0U] = std::sqrt(std::max(0.0F,
        floor + radial_squared * cosine_squared)) * residual;
    value[1U] = std::sqrt(std::max(0.0F,
        floor + radial_squared * (1.0F - cosine_squared))) * residual;
    value[2U] = std::sqrt(std::max(0.0F, floor)) * residual;
    value[3U] = value[2U];
}
} // namespace

bool ParmaQuadrupletAnalysis::initialize(std::uint32_t sample_rate) noexcept {
    if (sample_rate != 32000U && sample_rate != 44100U
        && sample_rate != 48000U) return false;
    std::vector<std::uint32_t> widths;
    std::vector<float> bark;
    if (!parma_initialize_critical_band_partitions(sample_rate, 64U,
            kBandCount, widths, bark) || widths.size() != kBandCount)
        return false;
    std::copy(widths.begin(), widths.end(), widths_.begin());
    energy_attack_ = smoothing_coefficient(sample_rate, 0.0063275F);
    energy_release_ = smoothing_coefficient(sample_rate, 0.012655F);
    control_coefficients_[0U] = energy_attack_;
    control_coefficients_[1U] = smoothing_coefficient(sample_rate, 0.026332F);
    control_coefficients_[2U] = smoothing_coefficient(sample_rate, 0.083F);
    control_coefficients_[3U] = smoothing_coefficient(sample_rate, 0.333F);
    initialized_ = true;
    reset();
    return true;
}

void ParmaQuadrupletAnalysis::reset() noexcept {
    for (auto& values : energy_) values.fill(0.0F);
    for (auto& values : position_energy_) values.fill(0.0F);
    for (auto& values : sum_energy_) values.fill(0.0F);
    for (auto& values : difference_energy_) values.fill(0.0F);
    smoothed_coherence_.fill(0.0F);
}

void ParmaQuadrupletAnalysis::process(
    const std::array<const float*, 4U>& source_real,
    const std::array<const float*, 4U>& source_imaginary,
    std::uint32_t* order, float* position, float* diffuseness,
    float* coherence) noexcept {
    if (!initialized_ || order == nullptr || position == nullptr
        || diffuseness == nullptr || coherence == nullptr) return;
    constexpr std::array<std::array<std::size_t, 2U>, 6U> pairs = {{
        {{0U, 1U}}, {{0U, 2U}}, {{0U, 3U}},
        {{1U, 2U}}, {{1U, 3U}}, {{2U, 3U}}}};
    std::array<std::array<float, 64U>, 4U> raw_energy{};
    std::array<std::array<float, 64U>, 6U> raw_sum{}, raw_difference{};
    for (std::size_t bin = 0U; bin < 64U; ++bin) {
        std::array<float, 4U> normal_real{}, normal_imaginary{};
        for (std::size_t channel = 0U; channel < 4U; ++channel) {
            const float re = source_real[channel][bin];
            const float im = source_imaginary[channel][bin];
            raw_energy[channel][bin] = re * re + im * im;
            const float normalization = 1.0F /
                std::sqrt(raw_energy[channel][bin] + kEpsilon);
            normal_real[channel] = re * normalization;
            normal_imaginary[channel] = im * normalization;
        }
        for (std::size_t pair = 0U; pair < pairs.size(); ++pair) {
            const auto a = pairs[pair][0U], b = pairs[pair][1U];
            const float combined = raw_energy[a][bin] + raw_energy[b][bin];
            const float sr = normal_real[a] + normal_real[b];
            const float si = normal_imaginary[a] + normal_imaginary[b];
            const float dr = normal_real[a] - normal_real[b];
            const float di = normal_imaginary[a] - normal_imaginary[b];
            raw_sum[pair][bin] = combined * (sr * sr + si * si);
            raw_difference[pair][bin] = combined * (dr * dr + di * di);
        }
    }
    std::array<std::array<float, kBandCount>, 4U> grouped_energy{};
    std::array<std::array<float, kBandCount>, 6U> grouped_sum{}, grouped_difference{};
    for (std::size_t channel = 0U; channel < 4U; ++channel)
        parma_group_critical_bands(raw_energy[channel].data(),
            grouped_energy[channel].data(), widths_.data(), kBandCount);
    for (std::size_t pair = 0U; pair < pairs.size(); ++pair) {
        parma_group_critical_bands(raw_sum[pair].data(),
            grouped_sum[pair].data(), widths_.data(), kBandCount);
        parma_group_critical_bands(raw_difference[pair].data(),
            grouped_difference[pair].data(), widths_.data(), kBandCount);
    }
    for (std::size_t band = 0U; band < kBandCount; ++band) {
        bool attack = false;
        for (std::size_t channel = 0U; channel < 4U; ++channel)
            attack = attack || grouped_energy[channel][band] > energy_[channel][band];
        for (std::size_t pair = 0U; pair < pairs.size(); ++pair)
            attack = attack || grouped_sum[pair][band] > sum_energy_[pair][band]
                || grouped_difference[pair][band] > difference_energy_[pair][band];
        const float alpha = attack ? energy_attack_ : energy_release_;
        for (std::size_t channel = 0U; channel < 4U; ++channel)
            energy_[channel][band] = smooth(energy_[channel][band],
                grouped_energy[channel][band], alpha);
        for (std::size_t pair = 0U; pair < pairs.size(); ++pair) {
            sum_energy_[pair][band] = smooth(sum_energy_[pair][band],
                grouped_sum[pair][band], alpha);
            difference_energy_[pair][band] = smooth(difference_energy_[pair][band],
                grouped_difference[pair][band], alpha);
        }
        const std::array<float, 4U> first = {energy_[0U][band],
            energy_[1U][band], energy_[2U][band], energy_[3U][band]};
        float raw_position = 0.0F, raw_diffuseness = 0.0F;
        const std::uint32_t raw_order = energy_order(first);
        quadruplet_coordinates(first, raw_order, raw_position, raw_diffuseness);
        float weighted_coherence = 0.0F, product_sum = 0.0F;
        for (std::size_t pair = 0U; pair < pairs.size(); ++pair) {
            const float product = energy_[pairs[pair][0U]][band]
                * energy_[pairs[pair][1U]][band];
            product_sum += product;
            weighted_coherence += product * sum_energy_[pair][band]
                / (sum_energy_[pair][band] + difference_energy_[pair][band]
                    + kEpsilon);
        }
        const float raw_coherence = std::clamp(
            1.2F * weighted_coherence / (product_sum + kEpsilon) - 0.1F,
            0.0F, 1.0F);
        const float metric = raw_diffuseness
            + std::max(0.0F, 2.0F * raw_coherence - 1.0F);
        const float control = metric >= 0.9F ? control_coefficients_[0U]
            : metric >= 0.7F ? control_coefficients_[1U]
            : metric >= 0.25F ? control_coefficients_[2U]
            : control_coefficients_[3U];
        for (std::size_t channel = 0U; channel < 4U; ++channel)
            position_energy_[channel][band] = smooth(
                position_energy_[channel][band], energy_[channel][band], control);
        smoothed_coherence_[band] = smooth(smoothed_coherence_[band],
            raw_coherence, control);
        const std::array<float, 4U> final = {position_energy_[0U][band],
            position_energy_[1U][band], position_energy_[2U][band],
            position_energy_[3U][band]};
        order[band] = energy_order(final);
        quadruplet_coordinates(final, order[band], position[band],
            diffuseness[band]);
        coherence[band] = 1.0F
            - std::max(0.0F, 2.0F * smoothed_coherence_[band] - 1.0F);
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

void parma_extract_matrixed_quadruplet_channel(
    const std::uint32_t* order, const float* position,
    const float* diffuseness,
    const std::array<const float*, 4U>& source_real,
    const std::array<const float*, 4U>& source_imaginary,
    float* target_real, float* target_imaginary,
    std::size_t count) noexcept {
    for (std::size_t bin = 0U; bin < count; ++bin) {
        std::array<float, 4U> gain{};
        quadruplet_magnitudes(position[bin], diffuseness[bin], gain);
        rotate_from_canonical(order[bin], gain);
        target_real[bin] = 0.0F;
        target_imaginary[bin] = 0.0F;
        for (std::size_t channel = 0U; channel < 4U; ++channel) {
            target_real[bin] += gain[channel] * source_real[channel][bin];
            target_imaginary[bin] += gain[channel] * source_imaginary[channel][bin];
        }
    }
}

void parma_repan_quadruplet_channels(
    const std::uint32_t* order, const float* position,
    const float* diffuseness, const float* coherence,
    const std::array<const float*, 4U>& source_real,
    const std::array<const float*, 4U>& source_imaginary,
    const std::array<float*, 4U>& target_real,
    const std::array<float*, 4U>& target_imaginary,
    std::size_t count) noexcept {
    for (std::size_t bin = 0U; bin < count; ++bin) {
        const float radius = std::sin(diffuseness[bin] * kHalfPi);
        const float r2 = std::clamp(radius * radius, 0.0F, 1.0F);
        const float residual_floor = 1.0F - r2;
        const float cosine = std::cos(position[bin] * kHalfPi);
        const float p2 = cosine * cosine;
        const float a2 = r2 * p2;
        const float b2 = r2 * (1.0F - p2);
        const float dominant = std::sqrt(r2 + residual_floor * 0.75F);
        const float adjacent_a = std::sqrt(residual_floor * 0.25F + a2);
        const float adjacent_b = std::sqrt(residual_floor * 0.25F + b2);
        const float quiet = std::sqrt(residual_floor * 0.25F);
        const float cross = std::sqrt(residual_floor / 12.0F);
        const float residual = std::sqrt(residual_floor);
        std::array<float, 4U> numerator = {
            dominant * adjacent_a - std::sqrt(a2),
            dominant * adjacent_b - std::sqrt(b2),
            dominant * quiet,
            dominant * quiet};
        std::array<float, 4U> denominator = {
            (residual + adjacent_b) * cross + kEpsilon,
            (residual + adjacent_a) * cross + kEpsilon,
            (adjacent_a + adjacent_b + quiet) * cross + kEpsilon,
            (adjacent_a + adjacent_b + quiet) * cross + kEpsilon};
        std::array<float, 4U> retain{}, cancellation{};
        const float coherent_residual = std::clamp(coherence[bin], 0.0F, 1.0F);
        for (std::size_t channel = 0U; channel < 4U; ++channel) {
            retain[channel] = std::sqrt(coherent_residual
                + (1.0F - coherent_residual) * dominant * dominant);
            cancellation[channel] = std::sqrt(1.0F - coherent_residual)
                * (numerator[channel] / denominator[channel]) * cross;
        }
        rotate_from_canonical(order[bin], retain);
        rotate_from_canonical(order[bin], cancellation);
        for (std::size_t channel = 0U; channel < 4U; ++channel) {
            float real = retain[channel] * target_real[channel][bin];
            float imaginary = retain[channel] * target_imaginary[channel][bin];
            for (std::size_t source = 0U; source < 4U; ++source) {
                if (source == channel) continue;
                real -= cancellation[channel] * source_real[source][bin];
                imaginary -= cancellation[channel] * source_imaginary[source][bin];
            }
            target_real[channel][bin] = real;
            target_imaginary[channel][bin] = imaginary;
        }
    }
}

} // namespace dtsx_decode
