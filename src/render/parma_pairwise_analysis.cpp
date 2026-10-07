#include "render/parma_pairwise_analysis.hpp"

#include "render/parma_critical_bands.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace dtsx_decode {
namespace {

constexpr float kEpsilon = 1.0e-12F;
constexpr float kDenormalOffset = 1.0e-18F;

float smooth(float previous, float value, float coefficient) noexcept {
    return ((1.0F - coefficient) * previous
            + coefficient * value
            + kDenormalOffset)
        - kDenormalOffset;
}

float time_coefficient(
    std::uint32_t sample_rate,
    float seconds,
    bool half_rate = false) noexcept {
    const float rate =
        static_cast<float>(sample_rate)
        * (half_rate ? 0.5F : 1.0F);
    return 1.0F - std::exp(-64.0F / (rate * seconds));
}

} // namespace

bool ParmaPairwiseAnalysis::initialize(
    std::uint32_t sample_rate) noexcept {
    if (sample_rate != 32000U
        && sample_rate != 44100U
        && sample_rate != 48000U) {
        return false;
    }
    std::vector<std::uint32_t> widths;
    std::vector<float> bark_scale;
    if (!parma_initialize_critical_band_partitions(
            sample_rate, 64U, kBandCount, widths, bark_scale)
        || widths.size() != kBandCount) {
        return false;
    }
    std::copy(widths.begin(), widths.end(), widths_.begin());

    energy_attack_ =
        time_coefficient(sample_rate, 0.0063275F);
    energy_release_ =
        time_coefficient(sample_rate, 0.012655F);
    metric_attack_ = energy_release_;
    metric_fast_release_ =
        time_coefficient(sample_rate, 0.052664F);
    metric_release_ =
        time_coefficient(sample_rate, 0.166F);
    metric_slow_release_ =
        time_coefficient(sample_rate, 0.666F);

    diffuseness_attack_ = energy_release_;
    diffuseness_release_ = metric_fast_release_;
    diffuseness_slow_release_ = metric_release_;
    initialized_ = true;
    reset();
    return true;
}

void ParmaPairwiseAnalysis::reset() noexcept {
    first_energy_.fill(0.0F);
    second_energy_.fill(0.0F);
    sum_energy_.fill(0.0F);
    difference_energy_.fill(0.0F);
    energy_ratio_state_.fill(0.5F);
    position_state_.fill(0.5F);
    balance_state_.fill(1.0F);
    energy_metric_state_.fill(1.0F);
    diffuseness_state_.fill(0.0F);
}

void ParmaPairwiseAnalysis::process(
    const float* first_real,
    const float* first_imaginary,
    const float* second_real,
    const float* second_imaginary,
    float* position,
    float* balance,
    float* diffuseness) noexcept {
    if (!initialized_ || first_real == nullptr
        || first_imaginary == nullptr || second_real == nullptr
        || second_imaginary == nullptr || position == nullptr
        || balance == nullptr || diffuseness == nullptr) {
        return;
    }

    std::array<float, 64U> first_bin_energy{};
    std::array<float, 64U> second_bin_energy{};
    std::array<float, 64U> sum_bin_energy{};
    std::array<float, 64U> difference_bin_energy{};
    for (std::size_t bin = 0U; bin < 64U; ++bin) {
        const float first_power =
            first_real[bin] * first_real[bin]
            + first_imaginary[bin] * first_imaginary[bin];
        const float second_power =
            second_real[bin] * second_real[bin]
            + second_imaginary[bin] * second_imaginary[bin];
        const float first_normalization =
            1.0F / std::sqrt(first_power + kEpsilon);
        const float second_normalization =
            1.0F / std::sqrt(second_power + kEpsilon);
        const float first_normal_real =
            first_real[bin] * first_normalization;
        const float first_normal_imaginary =
            first_imaginary[bin] * first_normalization;
        const float second_normal_real =
            second_real[bin] * second_normalization;
        const float second_normal_imaginary =
            second_imaginary[bin] * second_normalization;
        const float combined_power = first_power + second_power;

        first_bin_energy[bin] = first_power;
        second_bin_energy[bin] = second_power;
        const float sum_real =
            first_normal_real + second_normal_real;
        const float sum_imaginary =
            first_normal_imaginary + second_normal_imaginary;
        const float difference_real =
            first_normal_real - second_normal_real;
        const float difference_imaginary =
            first_normal_imaginary - second_normal_imaginary;
        sum_bin_energy[bin] =
            combined_power
            * (sum_real * sum_real
               + sum_imaginary * sum_imaginary);
        difference_bin_energy[bin] =
            combined_power
            * (difference_real * difference_real
               + difference_imaginary * difference_imaginary);
    }

    std::array<float, kBandCount> grouped_first{};
    std::array<float, kBandCount> grouped_second{};
    std::array<float, kBandCount> grouped_sum{};
    std::array<float, kBandCount> grouped_difference{};
    parma_group_critical_bands(
        first_bin_energy.data(), grouped_first.data(),
        widths_.data(), kBandCount);
    parma_group_critical_bands(
        second_bin_energy.data(), grouped_second.data(),
        widths_.data(), kBandCount);
    parma_group_critical_bands(
        sum_bin_energy.data(), grouped_sum.data(),
        widths_.data(), kBandCount);
    parma_group_critical_bands(
        difference_bin_energy.data(), grouped_difference.data(),
        widths_.data(), kBandCount);

    std::array<float, kBandCount> raw_energy_ratio{};
    std::array<float, kBandCount> raw_position{};
    std::array<float, kBandCount> raw_balance{};
    for (std::size_t band = 0U; band < kBandCount; ++band) {
        const bool attack =
            grouped_first[band] > first_energy_[band]
            || grouped_second[band] > second_energy_[band]
            || grouped_sum[band] > sum_energy_[band]
            || grouped_difference[band] > difference_energy_[band];
        const float coefficient =
            attack ? energy_attack_ : energy_release_;
        first_energy_[band] = smooth(
            first_energy_[band], grouped_first[band], coefficient);
        second_energy_[band] = smooth(
            second_energy_[band], grouped_second[band], coefficient);
        sum_energy_[band] = smooth(
            sum_energy_[band], grouped_sum[band], coefficient);
        difference_energy_[band] = smooth(
            difference_energy_[band],
            grouped_difference[band], coefficient);

        raw_energy_ratio[band] =
            first_energy_[band]
            / (first_energy_[band]
               + second_energy_[band] + kEpsilon);
        raw_position[band] =
            2.0F
            * std::acos(std::sqrt(raw_energy_ratio[band]))
            / 3.1416F;
        raw_balance[band] =
            (sum_energy_[band]
             / (sum_energy_[band]
                + difference_energy_[band] + kEpsilon))
                * 1.2F
            - 0.1F;
        raw_balance[band] =
            std::max(0.0F, std::min(1.0F, raw_balance[band]));
    }

    std::array<float, kBandCount> smoothed_position{};
    std::array<float, kBandCount> smoothed_balance{};
    std::array<float, kBandCount> smoothed_energy_ratio{};
    for (std::size_t band = 0U; band < kBandCount; ++band) {
        const float balance_axis =
            2.0F * raw_balance[band] - 1.0F;
        const float energy_axis =
            2.0F * raw_energy_ratio[band] - 1.0F;
        float metric =
            balance_axis * balance_axis
            + energy_axis * energy_axis;
        metric = std::min(1.0F, metric);

        float coefficient = 0.0F;
        if (metric >= 0.9F) {
            coefficient =
                energy_metric_state_[band] >= metric
                ? metric_fast_release_
                : metric_attack_;
        } else if (metric < 0.7F) {
            if (metric < 0.25F) {
                coefficient = metric_slow_release_;
            } else {
                coefficient =
                    metric > energy_metric_state_[band]
                    ? metric_release_
                    : metric_slow_release_;
            }
        } else {
            // In the 0.7..0.9 transition region the controller uses
            // the fast coefficient while the metric rises, and the regular
            // release coefficient while it falls.
            coefficient =
                metric <= energy_metric_state_[band]
                ? metric_release_
                : metric_fast_release_;
        }
        energy_metric_state_[band] = smooth(
            energy_metric_state_[band], metric, coefficient);
        energy_ratio_state_[band] = smooth(
            energy_ratio_state_[band],
            raw_energy_ratio[band], coefficient);
        position_state_[band] = smooth(
            position_state_[band], raw_position[band], coefficient);
        balance_state_[band] = smooth(
            balance_state_[band], raw_balance[band], coefficient);
        smoothed_position[band] = position_state_[band];
        smoothed_balance[band] = balance_state_[band];
        smoothed_energy_ratio[band] =
            energy_ratio_state_[band];
    }

    std::array<float, kBandCount> smoothed_diffuseness{};
    for (std::size_t band = 0U; band < kBandCount; ++band) {
        const float energy_axis =
            2.0F
                * (1.2F * smoothed_energy_ratio[band]
                   - 0.1F)
            - 1.0F;
        const float bounded_energy_axis =
            energy_axis < -1.0F || energy_axis > 1.0F
            ? 1.0F
            : energy_axis * energy_axis;
        const float balance_axis =
            2.0F * smoothed_balance[band] - 1.0F;
        const float radius =
            bounded_energy_axis
            + balance_axis * balance_axis;
        const float target =
            radius > 1.0F ? 0.0F : 1.0F - radius;
        float coefficient = 0.0F;
        if (target < 0.1F) {
            coefficient =
                diffuseness_state_[band] > target
                ? diffuseness_attack_
                : diffuseness_release_;
        } else if (target >= 0.5F) {
            coefficient = diffuseness_slow_release_;
        } else {
            coefficient =
                target >= diffuseness_state_[band]
                ? diffuseness_slow_release_
                : diffuseness_release_;
        }
        diffuseness_state_[band] = smooth(
            diffuseness_state_[band], target, coefficient);
        smoothed_diffuseness[band] =
            diffuseness_state_[band];
    }

    parma_ungroup_critical_bands(
        smoothed_position.data(), position,
        widths_.data(), kBandCount);
    parma_ungroup_critical_bands(
        smoothed_balance.data(), balance,
        widths_.data(), kBandCount);
    parma_ungroup_critical_bands(
        smoothed_diffuseness.data(), diffuseness,
        widths_.data(), kBandCount);
}

} // namespace dtsx_decode
