#include "dtsx/ace_stream.hpp"

#include "dtsx/ace_bit_reader.hpp"
#include "dtsx/ace_coarse_residual.hpp"
#include "dtsx/ace_lfe.hpp"
#include "dtsx/ace_scalar_dequant.hpp"
#include "dtsx/speaker_mask.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace dtsx {

namespace {

constexpr std::array<float, 100> kTemporalHoleFillBitAllocThreshold = {
    1.0F, .87055F, .75786F, .65975F, .57435F, .5F, .43528F, .37893F,
    .32988F, .28717F, .25F, .21764F, .18946F, .16494F, .14359F, .125F,
    .10882F, .094732F, .082469F, .071794F, .0625F, .054409F, .047366F,
    .041235F, .035897F, .03125F, .027205F, .023683F, .020617F, .017948F,
    .015625F, .013602F, .011842F, .010309F, .0089742F, .0078125F,
    .0068012F, .0059208F, .0051543F, .0044871F, .0039062F, .0034006F,
    .0029604F, .0025772F, .0022436F, .0019531F, .0017003F, .0014802F,
    .0012886F, .0011218F, .00097656F, .00085015F, .0007401F, .00064429F,
    .00056089F, .00048828F, .00042507F, .00037005F, .00032215F, .00028044F,
    .00024414F, .00021254F, .00018502F, .00016107F, .00014022F, .00012207F,
    .00010627F, .000092512F, .000080536F, .000070111F, .000061035F,
    .000053134F, .000046256F, .000040268F, .000035055F, .000030518F,
    .000026567F, .000023128F, .000020134F, .000017528F, .000015259F,
    .000013284F, .000011564F, .000010067F, .0000087639F, .0000076294F,
    .0000066418F, .000005782F, .0000050335F, .0000043819F, .0000038147F,
    .0000033209F, .000002891F, .0000025168F, .000002191F, .0000019073F,
    .0000016604F, .0000014455F, .0000012584F, .0000010955F};

constexpr float kThfNormEpsilon = 5.421e-20F;

float ace_thf_sony_sentinel() noexcept {
    constexpr std::uint32_t kBits = 0x5F7FFFD8U;
    float value = 0.0F;
    std::memcpy(&value, &kBits, sizeof(value));
    return value;
}

float ace_thf_unnormalize_gain(const std::int32_t lognorm_q10) noexcept {
    // Sony x64 UnNormalize converts Q10 to float as *1/1024 then stores
    // exp2f(lognorm) into the same slot THF reads. The ARM pow2_i32 path is
    // a different build.
    if (lognorm_q10 <= -32768) {
        return 0.0F;
    }
    return std::exp2(static_cast<float>(lognorm_q10) * 0.0009765625F);
}

// dtsAce_ComputeNormsFromBeta writes *a3 (left). FillHoles only uses that
// word for v80 = (a3*a3 + 0x10000000) >> 29 against (PRNGRand >> 3).
std::int32_t ace_thf_compute_norms_left_i32(
    const std::int32_t beta,
    const std::uint32_t band_size) noexcept {
    if (beta == -32767) {
        return 0;
    }
    if (beta == 32767) {
        return 0x20000000;
    }
    (void)band_size;
    const float scaled = static_cast<float>(beta) * 0.00048828F;
    const float pow2 = std::exp2(scaled + scaled);
    const float denom = pow2 + 1.0F;
    const float left_norm = std::sqrt(pow2 / denom);
    if (!(left_norm > 0.0F)) {
        return 0;
    }
    // General *a3 is Q30 (1.0 -> 0x40000000). Specials above are native
    // literals and must not use this scale (32767 stores 0x20000000).
    const double scaled_q30 = static_cast<double>(left_norm) * 1073741824.0;
    if (scaled_q30 >= 2147483647.0) {
        return 2147483647;
    }
    return static_cast<std::int32_t>(std::lrint(scaled_q30));
}

std::int32_t ace_thf_coherent_sign_threshold(const std::int32_t left_i32) noexcept {
    return static_cast<std::int32_t>(
        (static_cast<std::int64_t>(left_i32) * left_i32 + 0x10000000LL) >> 29);
}

} // namespace

float ace_temporal_hole_fill_bit_alloc_threshold(
    const std::uint32_t index) noexcept {
    return kTemporalHoleFillBitAllocThreshold[
        (std::min)(index, static_cast<std::uint32_t>(99U))];
}

void ace_temporal_hole_fill_calc_norms_thresh_offset(
    const std::array<std::array<std::uint8_t, 2>, 22>& active_bins,
    const std::array<std::array<float, 2>, 22>& norms,
    std::array<std::array<float, 2>, 22>& thresholds,
    const std::uint32_t channel_count,
    const std::uint32_t first_band,
    const std::uint32_t last_band) noexcept {
    if (channel_count == 0U || channel_count > 2U
        || first_band >= 22U || last_band >= 22U || first_band > last_band) {
        return;
    }
    for (std::uint32_t band = first_band; band <= last_band; ++band) {
        for (std::uint32_t channel = 0U; channel < channel_count; ++channel) {
            if (active_bins[band][channel] == 1U) {
                thresholds[band][channel] = std::sqrt(norms[band][channel]);
            }
        }
    }
}

void ace_temporal_hole_fill_calc_norms_thresh_offset_flat(
    const std::uint8_t* active_bins,
    const float* norms,
    float* thresholds,
    const std::uint32_t channel_count,
    const std::uint32_t first_band,
    const std::uint32_t last_band) noexcept {
    if (active_bins == nullptr || norms == nullptr || thresholds == nullptr
        || channel_count == 0U || first_band >= 22U || last_band >= 22U
        || first_band > last_band) {
        return;
    }
    for (std::uint32_t band = first_band; band <= last_band; ++band) {
        const std::size_t offset = static_cast<std::size_t>(band)
            * channel_count;
        for (std::uint32_t channel = 0U; channel < channel_count; ++channel) {
            if (active_bins[offset + channel] == 1U) {
                thresholds[offset + channel] = std::sqrt(
                    norms[offset + channel]);
            }
        }
    }
}

void ace_temporal_hole_fill_calc_norm_thresh_bit_alloc(
    const float reciprocal_band_bins,
    const std::int32_t bit_allocation,
    const std::array<float, 22>& norms,
    std::array<float, 22>& thresholds,
    const std::uint32_t first_band,
    const std::uint32_t last_band) noexcept {
    if (!std::isfinite(reciprocal_band_bins)
        || first_band >= 22U || last_band >= 22U || first_band > last_band) {
        return;
    }
    const float raw_index = reciprocal_band_bins
        * static_cast<float>(bit_allocation) * 10.0F;
    const auto rounded_index = static_cast<std::int64_t>(
        std::nearbyint(raw_index));
    const auto clamped_index = static_cast<std::uint32_t>(
        (std::max)(std::int64_t{0}, (std::min)(std::int64_t{99}, rounded_index)));
    const float scale =
        ace_temporal_hole_fill_bit_alloc_threshold(clamped_index);
    for (std::uint32_t band = first_band; band <= last_band; ++band) {
        thresholds[band] = norms[band] * scale;
    }
}

void ace_temporal_hole_fill_calc_norm_noise_predicted(
    const std::array<float, 22>& previous,
    const std::array<float, 22>& history,
    const std::array<float, 22>& predicted,
    std::array<float, 22>& output,
    const std::uint32_t first_band,
    const std::uint32_t last_band) noexcept {
    if (first_band >= 22U || last_band >= 22U || first_band > last_band) {
        return;
    }
    for (std::uint32_t band = first_band; band <= last_band; ++band) {
        output[band] = (std::min)(
            previous[band], (std::min)(history[band], predicted[band]));
    }
}

void ace_temporal_hole_fill_calc_norm_thresh_low_confidence(
    const std::uint32_t confidence_level,
    const std::array<float, 22>& channel_norms,
    std::array<float, 22>& thresholds,
    const std::uint32_t first_band,
    const std::uint32_t last_band) noexcept {
    if (first_band >= 22U || last_band >= 22U || first_band > last_band) {
        return;
    }
    if (confidence_level <= 2U) {
        constexpr std::uint32_t kNativeSentinelBits = 0x5F7FFFD8U;
        float sentinel = 0.0F;
        std::memcpy(&sentinel, &kNativeSentinelBits, sizeof(sentinel));
        for (std::uint32_t band = first_band; band <= last_band; ++band) {
            thresholds[band] = sentinel;
        }
        return;
    }
    for (std::uint32_t band = first_band; band <= last_band; ++band) {
        thresholds[band] = std::sqrt(channel_norms[band]);
    }
}

void ace_temporal_hole_fill_calc_norm_thresh_low_confidence_homatic(
    const std::uint32_t confidence_level,
    const std::array<float, 22>& channel_norms,
    std::array<float, 22>& thresholds,
    const std::uint32_t first_band,
    const std::uint32_t last_band) noexcept {
    if (first_band >= 22U || last_band >= 22U || first_band > last_band) {
        return;
    }
    if (confidence_level <= 2U) {
        // This profile writes the integer sentinel directly (0x7fffffff),
        // unlike the float-domain implementation above.
        constexpr float sentinel = 2147483647.0F;
        for (std::uint32_t band = first_band; band <= last_band; ++band) {
            thresholds[band] = sentinel;
        }
        return;
    }
    for (std::uint32_t band = first_band; band <= last_band; ++band) {
        thresholds[band] = std::sqrt(channel_norms[band]);
    }
}

float ace_temporal_hole_fill_calc_offset_index(
    const float current_norm,
    const float previous_norm,
    const float history_norm) noexcept {
    // DTSAceTemporalHoleFill_CalcOffsetIndex first limits the reference norm
    // to min(previous, history), then applies the native epsilon and affine
    // ramp.  Keep the constants as float literals: they are the constants
    // emitted by the native ARM implementation, not reconstructed values.
    const float reference = (std::min)(previous_norm, history_norm);
    const float denominator = reference + 5.421e-20F;
    if (!(current_norm <= denominator)) {
        return 0.0F;
    }
    float ramp = (current_norm / denominator - 0.70795F) * 3.424F;
    if (ramp < 0.0F) {
        ramp = 0.0F;
    }
    return 1.0F - (std::min)(ramp, 1.0F);
}

float ace_temporal_hole_fill_calc_bin_noise_level(
    const float current_sq_norm,
    const float previous_norm,
    const float history_norm,
    const float threshold,
    const float bin_norm,
    const float output_scale) noexcept {
    if (!std::isfinite(current_sq_norm) || !std::isfinite(previous_norm)
        || !std::isfinite(history_norm) || !std::isfinite(threshold)
        || !std::isfinite(bin_norm) || !std::isfinite(output_scale)) {
        return 0.0F;
    }
    const float floor = current_sq_norm * 0.0001F;
    if (threshold < floor) {
        return 0.0F;
    }
    const float offset = ace_temporal_hole_fill_calc_offset_index(
        current_sq_norm, previous_norm, history_norm);
    const float bounded_bin = (std::min)(bin_norm, threshold);
    const float mixed = threshold * (1.0F - offset) + offset * bounded_bin;
    return mixed >= floor ? mixed * output_scale : 0.0F;
}

std::int32_t ace_temporal_hole_fill_calc_bin_noise_level_q30(
    const std::int32_t bin_norm,
    const std::int32_t predicted_threshold,
    const std::int32_t bit_alloc_threshold,
    const std::int32_t low_confidence_threshold,
    const std::int32_t previous_norm,
    const std::int32_t history_norm,
    const std::int32_t reciprocal_sqrt_q30,
    const std::int32_t output_scale_q30,
    const std::int32_t bin_floor,
    const std::int32_t offset_q30) noexcept {
    // Direct scalar translation of DTSAceTemporalHoleFill_CalcBinNoiseLevels
    // (homatic/tcl ARM path).  The three threshold inputs correspond to the
    // native predicted, bit-allocation and low-confidence arrays.
    // Native compares min(predicted, bit-alloc, low-confidence) to
    // a7 * 0.0001 (Q31), then scales the mix by recip-sqrt only for the
    // output word.  Folding recip-sqrt into the floor made the compare
    // live in the wrong domain.
    const std::int32_t floor_q31 = bin_floor;
    std::int32_t threshold = predicted_threshold;
    if (bit_alloc_threshold < threshold) {
        threshold = bit_alloc_threshold;
    }
    if (low_confidence_threshold < threshold) {
        threshold = low_confidence_threshold;
    }
    if (threshold < floor_q31) {
        return 0;
    }

    (void)reciprocal_sqrt_q30;
    (void)previous_norm;
    (void)history_norm;
    if (bin_norm < 0) {
        return 0;
    }
    const std::int32_t bounded_bin = (std::min)(bin_norm, threshold);
    const std::int32_t offset = (std::max)(
        0, (std::min)(offset_q30, static_cast<std::int32_t>(0x40000000)));
    const std::int64_t weighted_bin =
        (static_cast<std::int64_t>(bounded_bin) * offset + 0x20000000LL)
        >> 30U;
    const std::int64_t weighted_threshold =
        (static_cast<std::int64_t>(0x40000000 - offset) * threshold
            + 0x20000000LL) >> 30U;
    const std::int32_t mixed = static_cast<std::int32_t>(
        weighted_bin + weighted_threshold);
    if (mixed < floor_q31) {
        return 0;
    }
    const std::int64_t output =
        (static_cast<std::int64_t>(output_scale_q30) * mixed
            + 0x40000000LL) >> 31U;
    return static_cast<std::int32_t>((std::max)(
        static_cast<std::int64_t>((std::numeric_limits<std::int32_t>::min)()),
        (std::min)(static_cast<std::int64_t>(
                       (std::numeric_limits<std::int32_t>::max)()), output)));
}

float ace_temporal_hole_fill_calc_block_sq_norm(
    const float* coefficients,
    const std::size_t coefficient_count,
    const std::size_t band_start,
    const std::size_t band_size,
    const std::uint32_t sample_stride) noexcept {
    if (coefficients == nullptr || sample_stride == 0U) {
        return 0.0F;
    }
    float sum = 0.0F;
    for (std::size_t bin = 0U; bin < band_size; ++bin) {
        const std::size_t index = band_start + bin * sample_stride;
        if (index >= coefficient_count) {
            break;
        }
        const float sample = coefficients[index];
        sum += sample * sample;
    }
    return sum * static_cast<float>(sample_stride);
}

std::uint64_t ace_temporal_hole_fill_calc_block_sq_norm_i32(
    const std::int32_t* coefficients,
    const std::size_t coefficient_count,
    const std::size_t band_start,
    const std::size_t band_size,
    const std::uint32_t sample_stride) noexcept {
    if (coefficients == nullptr || sample_stride == 0U) {
        return 0U;
    }
    std::uint64_t sum = 0U;
    for (std::size_t bin = 0U; bin < band_size; ++bin) {
        const std::size_t index = band_start + bin * sample_stride;
        if (index >= coefficient_count) {
            break;
        }
        const std::int64_t sample = coefficients[index];
        sum += static_cast<std::uint64_t>(sample * sample);
    }
    return sum * sample_stride;
}

std::uint32_t ace_sqrt_i64_to_i32_native(
    const std::uint32_t low,
    const std::int32_t high) noexcept {
    // dts_flib_sqrt_i64_to_i32 (Sony/Homatic ARM). Third argument is clz
    // scratch, not a band index.
    if (high < 0) {
        return 0U;
    }
    if (high == 0 && low == 0U) {
        return 0U;
    }
    const auto clz32 = [](const std::uint32_t value) noexcept {
        if (value == 0U) {
            return 32U;
        }
        std::uint32_t bits = value;
        std::uint32_t count = 0U;
        if (bits <= 0x0000FFFFU) {
            count += 16U;
            bits <<= 16U;
        }
        if (bits <= 0x00FFFFFFU) {
            count += 8U;
            bits <<= 8U;
        }
        if (bits <= 0x0FFFFFFFU) {
            count += 4U;
            bits <<= 4U;
        }
        if (bits <= 0x3FFFFFFFU) {
            count += 2U;
            bits <<= 2U;
        }
        if (bits <= 0x7FFFFFFFU) {
            count += 1U;
        }
        return count;
    };
    const auto smmulr = [](const std::int32_t left,
                           const std::int32_t right) noexcept {
        const std::int64_t product =
            static_cast<std::int64_t>(left) * right;
        return static_cast<std::int32_t>(
            (product + 0x80000000LL) >> 32);
    };
    const std::uint32_t leading = high != 0
        ? clz32(static_cast<std::uint32_t>(high))
        : clz32(low) + 32U;
    const std::uint32_t shift = leading - 1U;
    std::int32_t mantissa = 0;
    if (static_cast<std::int32_t>(shift) < 32) {
        mantissa = static_cast<std::int32_t>(
            (static_cast<std::uint32_t>(high) << shift)
            | (shift == 0U ? 0U : (low >> (32U - shift))));
    } else {
        mantissa = static_cast<std::int32_t>(low << (shift - 32U));
    }
    const std::int32_t square = smmulr(mantissa, mantissa);
    const std::int32_t cube = smmulr(square, mantissa);
    const std::int32_t fourth = smmulr(square, square);
    const std::uint32_t half = shift >> 1U;
    const std::int64_t polynomial =
        735160951LL * cube + -675411656LL * square
        + 489953829LL * mantissa
        + ((static_cast<std::int64_t>(43836469) << 32) | 0x90000000LL);
    std::int64_t rounded = -352725462LL * fourth + polynomial;
    const int original_right = static_cast<int>(half + 27U);
    int add_shift = original_right;
    if (original_right >= 32) {
        add_shift = static_cast<int>(half) - 5;
    }
    const std::uint32_t addend = add_shift >= 0 && add_shift < 32
        ? (1U << add_shift) : 0U;
    if (static_cast<int>(half) - 5 < 0) {
        rounded += addend;
    } else {
        rounded += static_cast<std::int64_t>(addend) << 32;
    }
    const int extract = static_cast<int>(half + 28U);
    std::uint32_t result = 0U;
    if (extract < 32) {
        const int high_shift = 4 - static_cast<int>(half);
        result = static_cast<std::uint32_t>(rounded >> extract);
        if (high_shift > 0) {
            result |= static_cast<std::uint32_t>(
                static_cast<std::uint64_t>(rounded) >> 32)
                << high_shift;
        }
    } else {
        const int high_shift = static_cast<int>(half) - 4;
        result = static_cast<std::uint32_t>(
            static_cast<std::int64_t>(rounded >> 32) >> high_shift);
    }
    if ((shift & 1U) != 0U) {
        return static_cast<std::uint32_t>(
            (1518500250LL * static_cast<std::int32_t>(result)
                + 0x40000000LL) >> 31);
    }
    return result;
}

bool ace_temporal_hole_fill_calc_block_sq_norms(
    const std::vector<std::vector<float>>& coefficients,
    const std::uint32_t sample_rate,
    const std::uint32_t bandwidth_mode,
    const std::uint32_t band_count,
    const std::uint32_t channel_count,
    const std::uint32_t sample_stride,
    const std::uint32_t energy_shift,
    std::vector<std::vector<float>>& norms,
    std::vector<std::vector<std::uint8_t>>& active) noexcept {
    norms.clear();
    active.clear();
    if ((sample_rate != 44100U && sample_rate != 48000U)
        || bandwidth_mode > 3U || band_count == 0U || band_count > 22U
        || channel_count == 0U || channel_count > coefficients.size()
        || sample_stride == 0U || energy_shift >= 31U) {
        return false;
    }
    norms.assign(band_count,
        std::vector<float>(channel_count, 0.0F));
    active.assign(band_count,
        std::vector<std::uint8_t>(channel_count, 0U));
    std::size_t band_start = 0U;
    for (std::uint32_t band = 0U; band < band_count; ++band) {
        const std::uint32_t band_size =
            ace_stream_band_size(sample_rate, band);
        if (band_size == 0U) {
            norms.clear();
            active.clear();
            return false;
        }
        const std::size_t bins = band_size >> energy_shift;
        for (std::uint32_t channel = 0U; channel < channel_count;
             ++channel) {
            const auto& plane = coefficients[channel];
            if (band_start > plane.size()
                || band_size > plane.size() - band_start) {
                norms.clear();
                active.clear();
                return false;
            }
            if (bins == 0U) {
                active[band][channel] = 1U;
                continue;
            }
            bool any_zero = false;
            bool have_min = false;
            float min_energy = 0.0F;
            for (std::uint32_t subblock = 0U; subblock < sample_stride;
                 ++subblock) {
                const std::size_t start = band_start + subblock;
                const std::size_t last =
                    start + (bins - 1U) * sample_stride;
                if (last >= plane.size()) {
                    norms.clear();
                    active.clear();
                    return false;
                }
                const float energy =
                    ace_temporal_hole_fill_calc_block_sq_norm(
                        plane.data(), plane.size(), start, bins,
                        sample_stride);
                if (!std::isfinite(energy)) {
                    norms.clear();
                    active.clear();
                    return false;
                }
                if (energy == 0.0F) {
                    any_zero = true;
                    continue;
                }
                if (!(energy > kThfNormEpsilon)) {
                    continue;
                }
                if (!have_min || energy < min_energy) {
                    min_energy = energy;
                    have_min = true;
                }
            }
            // Native a4/a1[7] keep the minimum positive sub-block energy;
            // a zero lattice still sets the hole-fill active flag.
            norms[band][channel] = have_min ? min_energy : 0.0F;
            active[band][channel] = any_zero ? 1U : 0U;
        }
        band_start += band_size;
    }
    return true;
}

void ace_temporal_hole_fill_update_history_planes(
    const std::uint32_t plane_count,
    const std::uint32_t algorithm,
    const std::uint32_t active_bands,
    const float* current,
    float* previous,
    float* history) noexcept {
    if (current == nullptr || previous == nullptr || history == nullptr
        || plane_count == 0U || plane_count > 10U
        || (algorithm != 7U && algorithm != 10U)) {
        return;
    }
    const std::uint32_t bands = (std::min)(active_bands, 22U);
    if (algorithm == 10U) {
        for (std::uint32_t channel = 0U; channel < plane_count; ++channel) {
            float* const previous_plane = previous + channel * 22U;
            float* const history_plane = history + channel * 22U;
            const float* const current_plane = current + channel * 22U;
            // Homatic shifts the complete planes first, then clears bands
            // above the current active-band count in both destinations.
            // Keeping stale high-band history changes the next frame's THF
            // state even though the 10-channel branch does not noise-fill.
            for (std::uint32_t band = 0U; band < 22U; ++band) {
                history_plane[band] = previous_plane[band];
                previous_plane[band] = current_plane[band];
            }
            for (std::uint32_t band = bands; band < 22U; ++band) {
                previous_plane[band] = 0.0F;
                history_plane[band] = 0.0F;
            }
        }
        return;
    }
    for (std::uint32_t channel = 0U; channel < plane_count; ++channel) {
        float* const previous_plane = previous + channel * 22U;
        float* const history_plane = history + channel * 22U;
        const float* const current_plane = current + channel * 22U;
        for (std::uint32_t band = 0U; band < bands; ++band) {
            // Homatic 7-channel branch folds current norms into a1[5]
            // (the previous/current plane); a1[6] is not min-reduced here.
            previous_plane[band] = (std::min)(
                previous_plane[band], current_plane[band]);
        }
        for (std::uint32_t band = bands; band < 22U; ++band) {
            previous_plane[band] = 0.0F;
            history_plane[band] = 0.0F;
        }
    }
}

void ace_temporal_hole_fill_update_history(
    const std::uint32_t channel_count,
    const std::uint32_t active_bands,
    const float* current,
    float* previous,
    float* history) noexcept {
    if (channel_count != 7U && channel_count != 10U) {
        return;
    }
    ace_temporal_hole_fill_update_history_planes(
        channel_count, channel_count, active_bands, current, previous,
        history);
}

void AceTemporalHoleFillHistory::reset(
    const std::uint32_t channel_count) noexcept {
    if (channel_count == 7U || channel_count == 10U) {
        reset(channel_count, channel_count);
        return;
    }
    channel_count_ = 0U;
    algorithm_ = 0U;
    short_process_count_ = 0U;
    previous_.clear();
    history_.clear();
    min_plane_.clear();
    previous_a7_.clear();
    history_a7_.clear();
}

void AceTemporalHoleFillHistory::reset(
    const std::uint32_t plane_count,
    const std::uint32_t algorithm) noexcept {
    if (plane_count == 0U || plane_count > 10U
        || (algorithm != 7U && algorithm != 10U)) {
        channel_count_ = 0U;
        algorithm_ = 0U;
        short_process_count_ = 0U;
        previous_.clear();
        history_.clear();
        min_plane_.clear();
        previous_a7_.clear();
        history_a7_.clear();
        return;
    }
    channel_count_ = plane_count;
    algorithm_ = algorithm;
    short_process_count_ = 0U;
    previous_.assign(static_cast<std::size_t>(channel_count_) * 22U, 0.0F);
    history_.assign(static_cast<std::size_t>(channel_count_) * 22U, 0.0F);
    min_plane_.assign(static_cast<std::size_t>(channel_count_) * 22U, 0.0F);
    previous_a7_.assign(static_cast<std::size_t>(channel_count_) * 22U, 0);
    history_a7_.assign(static_cast<std::size_t>(channel_count_) * 22U, 0);
    fill_min_plane_sentinel();
}

void AceTemporalHoleFillHistory::fill_min_plane_sentinel() noexcept {
    const float sentinel = ace_thf_sony_sentinel();
    std::fill(min_plane_.begin(), min_plane_.end(), sentinel);
}

void AceTemporalHoleFillHistory::set_algorithm(
    const std::uint32_t algorithm) noexcept {
    if (channel_count_ == 0U || (algorithm != 7U && algorithm != 10U)) {
        return;
    }
    algorithm_ = algorithm;
}

void AceTemporalHoleFillHistory::note_process(
    const std::uint32_t algorithm) noexcept {
    if (algorithm == 10U) {
        short_process_count_ = 0U;
        fill_min_plane_sentinel();
        return;
    }
    if (algorithm == 7U) {
        ++short_process_count_;
    }
}

std::uint32_t AceTemporalHoleFillHistory::short_process_count() const noexcept {
    return short_process_count_;
}

std::uint32_t AceTemporalHoleFillHistory::channel_count() const noexcept {
    return channel_count_;
}

std::uint32_t AceTemporalHoleFillHistory::algorithm() const noexcept {
    return algorithm_;
}

std::vector<float>& AceTemporalHoleFillHistory::previous() noexcept {
    return previous_;
}

std::vector<float>& AceTemporalHoleFillHistory::history() noexcept {
    return history_;
}

std::vector<float>& AceTemporalHoleFillHistory::min_plane() noexcept {
    return min_plane_;
}

const std::vector<float>& AceTemporalHoleFillHistory::previous() const noexcept {
    return previous_;
}

const std::vector<float>& AceTemporalHoleFillHistory::history() const noexcept {
    return history_;
}

const std::vector<float>& AceTemporalHoleFillHistory::min_plane() const noexcept {
    return min_plane_;
}

void AceTemporalHoleFillHistory::update(
    const std::vector<float>& current,
    const std::uint32_t active_bands) noexcept {
    if (channel_count_ == 0U
        || current.size() != previous_.size()
        || current.size() != history_.size()) {
        return;
    }
    const std::uint32_t algorithm =
        algorithm_ != 0U ? algorithm_ : channel_count_;
    ace_temporal_hole_fill_update_history_planes(
        channel_count_, algorithm, active_bands, current.data(),
        previous_.data(), history_.data());
}

const std::vector<std::int32_t>&
AceTemporalHoleFillHistory::previous_a7() const noexcept {
    return previous_a7_;
}

const std::vector<std::int32_t>&
AceTemporalHoleFillHistory::history_a7() const noexcept {
    return history_a7_;
}

void AceTemporalHoleFillHistory::update_a7(
    const std::vector<std::int32_t>& current,
    const std::uint32_t active_bands) noexcept {
    if (channel_count_ == 0U
        || current.size() != previous_a7_.size()
        || current.size() != history_a7_.size()) {
        return;
    }
    const std::uint32_t algorithm =
        algorithm_ != 0U ? algorithm_ : channel_count_;
    const std::uint32_t bands = (std::min)(active_bands, 22U);
    if (algorithm != 7U && algorithm != 10U) {
        return;
    }
    for (std::uint32_t channel = 0U; channel < channel_count_; ++channel) {
        std::int32_t* const previous_plane =
            previous_a7_.data() + channel * 22U;
        std::int32_t* const history_plane =
            history_a7_.data() + channel * 22U;
        const std::int32_t* const current_plane =
            current.data() + channel * 22U;
        if (algorithm == 10U) {
            for (std::uint32_t band = 0U; band < 22U; ++band) {
                history_plane[band] = previous_plane[band];
                previous_plane[band] = current_plane[band];
            }
        } else {
            for (std::uint32_t band = 0U; band < 22U; ++band) {
                if (current_plane[band] < previous_plane[band]) {
                    previous_plane[band] = current_plane[band];
                }
            }
        }
        for (std::uint32_t band = bands; band < 22U; ++band) {
            previous_plane[band] = 0;
            history_plane[band] = 0;
        }
    }
}

bool ace_temporal_hole_fill_fill_holes_with_noise(
    std::vector<std::vector<float>>& coefficients,
    const std::vector<std::vector<std::uint8_t>>& active_bins,
    const std::vector<std::vector<float>>& noise_levels,
    const std::vector<std::vector<float>>& target_norms,
    const std::uint32_t sample_rate,
    const std::uint32_t bandwidth_mode,
    const std::uint32_t band,
    const std::uint32_t first_channel,
    const std::uint32_t last_channel,
    const std::uint32_t sample_stride,
    const std::uint32_t block_shift,
    const std::int32_t mode,
    std::uint32_t& random_state,
    const std::uint32_t subblock_index,
    const bool renormalize,
    const std::uint32_t stereo_coding,
    const bool all_subblocks,
    const std::int32_t band_beta) noexcept {
    (void)bandwidth_mode;
    if (coefficients.empty() || band >= active_bins.size()
        || band >= noise_levels.size() || band >= target_norms.size()
        || sample_stride == 0U || subblock_index >= sample_stride
        || first_channel > last_channel
        || last_channel >= coefficients.size()
        || last_channel >= active_bins[band].size()
        || last_channel >= target_norms[band].size()) {
        return false;
    }
    const bool packed_noise = noise_levels[band].size()
        >= static_cast<std::size_t>(last_channel + 1U) * 8U;
    if (!packed_noise && last_channel >= noise_levels[band].size()) {
        return false;
    }
    const bool packed_active = active_bins[band].size()
        >= static_cast<std::size_t>(last_channel + 1U) * 8U;
    const std::uint32_t band_size = ace_stream_band_size(sample_rate, band);
    if (band_size == 0U || block_shift >= 31U) {
        return false;
    }
    std::size_t region_start = 0U;
    for (std::uint32_t index = 0U; index < band; ++index) {
        region_start += ace_stream_band_size(sample_rate, index);
    }
    const std::size_t block_count =
        static_cast<std::size_t>(band_size >> block_shift);
    if (block_count == 0U) {
        return false;
    }
    const std::size_t band_end = region_start + band_size;
    const std::uint32_t sub_begin = all_subblocks ? 0U : subblock_index;
    const std::uint32_t sub_end = all_subblocks
        ? sample_stride : subblock_index + 1U;

    const auto active = [&](const std::uint32_t channel,
                            const std::uint32_t sub) noexcept {
        if (packed_active) {
            const std::size_t index =
                static_cast<std::size_t>(channel) * 8U + sub;
            return index < active_bins[band].size()
                && active_bins[band][index] != 0U;
        }
        return channel < active_bins[band].size()
            && active_bins[band][channel] != 0U;
    };
    const auto level = [&](const std::uint32_t channel,
                           const std::uint32_t sub) noexcept {
        if (packed_noise) {
            return noise_levels[band][
                static_cast<std::size_t>(channel) * 8U + sub];
        }
        return noise_levels[band][channel];
    };
    const auto fill_channel = [&](const std::uint32_t channel,
                                  const float amplitude,
                                  const std::size_t band_start) noexcept {
        if (channel >= coefficients.size() || amplitude <= 0.0F
            || !std::isfinite(amplitude)) {
            return false;
        }
        std::vector<float>& plane = coefficients[channel];
        for (std::size_t index = 0U; index < block_count; ++index) {
            const std::size_t coefficient =
                band_start + index * sample_stride;
            if (coefficient >= plane.size()) {
                return false;
            }
            (void)ace_prng_rand_uint(random_state);
            const float value = (random_state & 0x80000000U) != 0U
                ? amplitude : -amplitude;
            plane[coefficient] = value;
        }
        return true;
    };

    const auto level_q31 = [&](const std::uint32_t channel,
                               const std::uint32_t sub) noexcept {
        const float amplitude = level(channel, sub);
        if (!std::isfinite(amplitude) || !(amplitude > 0.0F)) {
            return 0;
        }
        const double scaled = static_cast<double>(amplitude) * 2147483648.0;
        if (scaled >= 2147483647.0) {
            return 2147483647;
        }
        return static_cast<std::int32_t>(scaled);
    };

    std::vector<std::uint8_t> filled_channel(
        static_cast<std::size_t>(last_channel) + 1U, 0U);
    const bool stereo_coherent_mode = mode == 2
        && (stereo_coding | 2U) == 3U
        && first_channel < last_channel;

    for (std::uint32_t sub = sub_begin; sub < sub_end; ++sub) {
        const std::size_t band_start = region_start + sub;
        bool all_holes = stereo_coherent_mode;
        if (all_holes) {
            for (std::uint32_t channel = first_channel;
                 channel <= last_channel; ++channel) {
                if (!active(channel, sub)) {
                    all_holes = false;
                    break;
                }
            }
        }
        if (stereo_coherent_mode && all_holes) {
            // Native LABEL_10: after every lattice in [first, last] is set,
            // FillHoles never falls through to the independent path.  Skip
            // the sub when left<=0 and right<1 (integer noise levels).
            const std::int32_t left_q = level_q31(first_channel, sub);
            const std::int32_t right_q = level_q31(last_channel, sub);
            if (left_q <= 0 && right_q < 1) {
                continue;
            }
            const float first_level = level(first_channel, sub);
            const float last_level = level(last_channel, sub);
            const std::int32_t sign_threshold = ace_thf_coherent_sign_threshold(
                ace_thf_compute_norms_left_i32(band_beta, band_size));
            for (std::size_t index = 0U; index < block_count; ++index) {
                const std::size_t coefficient =
                    band_start + index * sample_stride;
                if (coefficient >= coefficients[first_channel].size()
                    || coefficient >= coefficients[last_channel].size()) {
                    return false;
                }
                const std::uint32_t uint_state =
                    ace_prng_rand_uint(random_state);
                const float sign =
                    static_cast<std::int32_t>(uint_state) < 0 ? 1.0F : -1.0F;
                random_state = 894955033U * uint_state + 1831589974U;
                float second_sign = sign;
                // TCL/Sony FillHoles LABEL_10: v80 <= (int)(PRNGRand >> 3).
                if (sign_threshold
                    <= (static_cast<std::int32_t>(random_state) >> 3)) {
                    second_sign = -second_sign;
                }
                coefficients[first_channel][coefficient] = sign * first_level;
                coefficients[last_channel][coefficient] =
                    second_sign * last_level;
            }
            filled_channel[first_channel] = 1U;
            filled_channel[last_channel] = 1U;
        } else {
            for (std::uint32_t channel = first_channel;
                 channel <= last_channel; ++channel) {
                if (!active(channel, sub)
                    || level_q31(channel, sub) < 1) {
                    continue;
                }
                if (!fill_channel(channel, level(channel, sub),
                        band_start)) {
                    return false;
                }
                filled_channel[channel] = 1U;
            }
        }
    }

    if (renormalize) {
        for (std::uint32_t channel = first_channel;
             channel <= last_channel; ++channel) {
            if (channel >= filled_channel.size()
                || filled_channel[channel] == 0U
                || channel >= target_norms[band].size()) {
                continue;
            }
            const float target = target_norms[band][channel];
            if (!(target > 0.0F) || !std::isfinite(target)) {
                continue;
            }
            std::vector<float>& plane = coefficients[channel];
            // Sony x64 FillHolesWithNoise: gain = a7 / (sqrt(sum(x*x))+eps)
            // over the packed band, then x *= gain.  The ARM Q31/div path
            // crushed float [-1,1] bins to 0 and undid the hole write.
            float energy = 0.0F;
            for (std::size_t index = region_start; index < band_end
                 && index < plane.size(); ++index) {
                const float sample = plane[index];
                if (std::isfinite(sample)) {
                    energy += sample * sample;
                }
            }
            const float scale = target
                / (std::sqrt(energy) + 5.421e-20F);
            if (!std::isfinite(scale) || scale <= 0.0F) {
                continue;
            }
            for (std::size_t index = region_start; index < band_end
                 && index < plane.size(); ++index) {
                plane[index] *= scale;
            }
        }
    }
    return true;
}

bool ace_temporal_hole_fill_process(
    std::vector<std::vector<float>>& coefficients,
    const std::array<std::uint32_t, 22>& bit_allocation,
    const std::uint32_t sample_rate,
    const std::uint32_t bandwidth_mode,
    const std::uint32_t band_count,
    const std::uint32_t block_shift,
    const bool short_transform,
    const bool temporal_hole_fill,
    AceTemporalHoleFillHistory& history,
    std::uint32_t& random_state,
    const std::array<std::uint32_t, 22>* stereo_coding,
    const std::array<std::array<std::int32_t, 22>, 2>* lognorm_q10,
    const AceCodingMode coding_mode,
    const std::array<std::int32_t, 22>* band_beta) noexcept {
    const std::uint32_t plane_count =
        static_cast<std::uint32_t>(coefficients.size());
    if (plane_count == 0U || plane_count > 4U || band_count == 0U
        || band_count > 22U) {
        return false;
    }
    const std::uint32_t algorithm = short_transform ? 7U : 10U;
    if (history.channel_count() != plane_count) {
        history.reset(plane_count, algorithm);
    } else {
        history.set_algorithm(algorithm);
    }
    history.note_process(algorithm);
    std::vector<std::vector<float>> norms;
    std::vector<std::vector<std::uint8_t>> active;
    (void)block_shift;
    // Sony DTSAceStreamDecoder_Process stores HIDWORD(v63) = 0 when the
    // coded length is 1024. THF a10 is therefore always 8; a11 is still 3
    // for algorithm 7. Using log2(MDCT size) here made 128-point short
    // frames scan consecutive bins instead of the native stride-8 lattice.
    const std::uint32_t sample_stride = ace_thf_block_sample_stride(0U);
    const std::uint32_t energy_shift =
        ace_thf_energy_bin_shift(short_transform);
    if (!ace_temporal_hole_fill_calc_block_sq_norms(
            coefficients, sample_rate, bandwidth_mode, band_count,
            plane_count, sample_stride, energy_shift, norms, active)) {
        return false;
    }

    std::uint32_t first_channel = 0U;
    std::uint32_t last_channel = plane_count - 1U;
    if (coding_mode == AceCodingMode::Left) {
        first_channel = 0U;
        last_channel = 0U;
    } else if (coding_mode == AceCodingMode::Right && plane_count > 1U) {
        first_channel = 1U;
        last_channel = 1U;
    }
    const std::int32_t fill_mode = static_cast<std::int32_t>(
        last_channel + 1U - first_channel);

    std::vector<float> lognorm_gains(
        static_cast<std::size_t>(plane_count) * 22U, 0.0F);
    std::vector<std::int32_t> lognorm_a7(
        static_cast<std::size_t>(plane_count) * 22U, 0);
    for (std::uint32_t channel = 0U; channel < plane_count; ++channel) {
        for (std::uint32_t band = 0U; band < band_count; ++band) {
            const std::size_t index =
                static_cast<std::size_t>(channel) * 22U + band;
            if (lognorm_q10 != nullptr && channel < 2U) {
                const std::int32_t q10 = ace_unnormalize_lognorm_q10(
                    (*lognorm_q10)[channel][band], sample_rate, band);
                lognorm_gains[index] = ace_thf_unnormalize_gain(q10);
                lognorm_a7[index] = q10 <= -32768
                    ? 0 : ace_pow2_i32_native(q10, 10U, 10);
            } else {
                lognorm_gains[index] = norms[band][channel];
            }
        }
    }

    std::vector<std::uint8_t> lattice(
        static_cast<std::size_t>(plane_count) * 176U, 0U);
    std::vector<std::uint64_t> lattice_energy_q64(
        static_cast<std::size_t>(plane_count) * 176U, 0U);
    for (std::uint32_t channel = 0U; channel < plane_count; ++channel) {
        std::size_t band_start = 0U;
        for (std::uint32_t band = 0U; band < band_count; ++band) {
            const std::uint32_t band_size =
                ace_stream_band_size(sample_rate, band);
            const std::size_t bins = band_size >> energy_shift;
            const auto& plane = coefficients[channel];
            const std::size_t min_index =
                static_cast<std::size_t>(channel) * 22U + band;
            if (min_index < history.min_plane().size()
                && band < norms.size()
                && channel < norms[band].size()) {
                float reduced = norms[band][channel];
                if (reduced > history.min_plane()[min_index]) {
                    reduced = history.min_plane()[min_index];
                }
                history.min_plane()[min_index] = reduced;
            }
            for (std::uint32_t sub = 0U; sub < sample_stride && bins != 0U;
                 ++sub) {
                // Native CalcBlockSqNorms zeros the lattice from the integer
                // vmlal_s32 sum before multiplying by a10.  Convert the same
                // Q31 samples the MDCT overlap path uses so tiny float bins
                // that saturate to 0 match the ARM hole flags.
                std::uint64_t sum_sq = 0U;
                bool in_range = true;
                for (std::size_t bin = 0U; bin < bins; ++bin) {
                    const std::size_t index = band_start + sub
                        + bin * sample_stride;
                    if (index >= plane.size()) {
                        in_range = false;
                        break;
                    }
                    const float clamped = (std::max)(-1.0F,
                        (std::min)(1.0F, plane[index]));
                    const double scaled =
                        static_cast<double>(clamped) * 2147483648.0;
                    const std::int32_t q31 = scaled >= 2147483647.0
                        ? 2147483647
                        : scaled <= -2147483648.0
                            ? static_cast<std::int32_t>(-2147483647 - 1)
                            : static_cast<std::int32_t>(std::lrint(scaled));
                    sum_sq += static_cast<std::uint64_t>(
                        static_cast<std::int64_t>(q31) * q31);
                }
                const std::size_t lattice_index =
                    static_cast<std::size_t>(channel) * 176U
                    + static_cast<std::size_t>(band) * 8U + sub;
                if (in_range) {
                    lattice_energy_q64[lattice_index] =
                        sum_sq * sample_stride;
                }
                if (in_range && sum_sq == 0U) {
                    lattice[lattice_index] = 1U;
                }
            }
            band_start += band_size;
        }
    }

    const bool fill = algorithm == 7U && temporal_hole_fill;
    std::vector<std::vector<float>> filled;
    if (fill) {
        filled = coefficients;
    }
    std::vector<std::vector<float>>& planes = fill ? filled : coefficients;
    if (fill) {
        std::vector<std::uint8_t> band_active(band_count, 0U);
        for (std::uint32_t band = 0U; band < band_count; ++band) {
            for (std::uint32_t channel = 0U; channel < plane_count;
                 ++channel) {
                for (std::uint32_t sub = 0U; sub < sample_stride; ++sub) {
                    if (lattice[static_cast<std::size_t>(channel) * 176U
                            + static_cast<std::size_t>(band) * 8U + sub]
                        != 0U) {
                        band_active[band] = 1U;
                        break;
                    }
                }
                if (band_active[band] != 0U) {
                    break;
                }
            }
        }
        std::vector<std::vector<float>> target(band_count,
            std::vector<float>(plane_count, 0.0F));
        for (std::uint32_t channel = 0U; channel < plane_count; ++channel) {
            for (std::uint32_t band = 0U; band < band_count; ++band) {
                target[band][channel] = lognorm_gains[
                    static_cast<std::size_t>(channel) * 22U + band];
            }
        }
        for (std::uint32_t band = 0U; band < band_count; ++band) {
            if (band_active[band] == 0U) {
                continue;
            }
            const std::uint32_t reciprocal_q28 =
                ace_temporal_hole_fill_recip_num_total_bins_q28(
                    sample_rate, bandwidth_mode, band);
            const std::uint32_t recip_sqrt_q30 =
                ace_temporal_hole_fill_recip_sqrt_num_bins_q30(
                    sample_rate, bandwidth_mode, band);
            const float reciprocal_bins =
                static_cast<float>(reciprocal_q28) / 268435456.0F;
            const float recip_sqrt =
                static_cast<float>(recip_sqrt_q30) / 1073741824.0F;
            const std::uint32_t coding = stereo_coding != nullptr
                ? (*stereo_coding)[band] : 3U;
            std::vector<std::vector<std::uint8_t>> packed_active(band_count,
                std::vector<std::uint8_t>(
                    static_cast<std::size_t>(plane_count) * 8U, 0U));
            std::vector<std::vector<float>> packed_noise(band_count,
                std::vector<float>(
                    static_cast<std::size_t>(plane_count) * 8U, 0.0F));
            for (std::uint32_t channel = first_channel;
                 channel <= last_channel; ++channel) {
                std::array<float, 22> channel_norms{};
                std::array<float, 22> bit_alloc_thr{};
                std::array<float, 22> low_conf{};
                std::array<float, 22> min_norms{};
                const std::size_t plane_index =
                    static_cast<std::size_t>(channel) * 22U + band;
                channel_norms[band] = lognorm_gains[plane_index];
                if (plane_index < history.min_plane().size()) {
                    min_norms[band] = history.min_plane()[plane_index];
                }
                ace_temporal_hole_fill_calc_norm_thresh_bit_alloc(
                    reciprocal_bins,
                    static_cast<std::int32_t>(bit_allocation[band]),
                    channel_norms, bit_alloc_thr, band, band);
                ace_temporal_hole_fill_calc_norm_thresh_low_confidence(
                    history.short_process_count(), min_norms, low_conf,
                    band, band);
                const float current_f = lognorm_gains[plane_index];
                float previous_f = 0.0F;
                float history_f = 0.0F;
                if (plane_index < history.previous().size()) {
                    previous_f = history.previous()[plane_index];
                    history_f = history.history()[plane_index];
                }
                const float predicted_f = (std::min)(current_f,
                    (std::min)(previous_f, history_f));
                const float threshold = (std::min)(predicted_f,
                    (std::min)(bit_alloc_thr[band], low_conf[band]));
                for (std::uint32_t sub = 0U; sub < sample_stride; ++sub) {
                    const std::size_t packed =
                        static_cast<std::size_t>(channel) * 8U + sub;
                    const std::size_t lattice_index =
                        static_cast<std::size_t>(channel) * 176U
                        + static_cast<std::size_t>(band) * 8U + sub;
                    packed_active[band][packed] = lattice[lattice_index];
                    float bin_f = 0.0F;
                    if (lattice[lattice_index] != 0U) {
                        const std::uint64_t energy =
                            lattice_energy_q64[lattice_index];
                        bin_f = static_cast<float>(
                            ace_sqrt_i64_to_i32_native(
                                static_cast<std::uint32_t>(energy),
                                static_cast<std::int32_t>(energy >> 32)));
                    }
                    packed_noise[band][packed] =
                        ace_temporal_hole_fill_calc_bin_noise_level(
                            current_f, previous_f, history_f, threshold,
                            bin_f, recip_sqrt);
                }
            }
            const std::int32_t beta = band_beta != nullptr
                ? (*band_beta)[band] : 0;
            (void)ace_temporal_hole_fill_fill_holes_with_noise(
                planes, packed_active, packed_noise, target, sample_rate,
                bandwidth_mode, band, first_channel, last_channel,
                sample_stride, energy_shift, fill_mode, random_state,
                0U, true, coding, true, beta);
        }
        coefficients = std::move(filled);
    }
    history.update(lognorm_gains, band_count);
    history.update_a7(lognorm_a7, band_count);
    return true;
}

std::uint32_t ace_prng_rand_uint(std::uint32_t& state) noexcept {
    state = 1640531513U - 1403630843U * state;
    return state;
}

float ace_prng_rand(std::uint32_t& state) noexcept {
    state = 894955033U * state + 1831589974U;
    return static_cast<float>(state) * 2.3283e-10F;
}

float ace_prng_rand_in_range(
    std::uint32_t& state,
    const float minimum,
    const float maximum) noexcept {
    return minimum + (maximum - minimum) * ace_prng_rand(state);
}

std::int32_t ace_prng_rand_in_range_i32(
    std::uint32_t& state,
    const std::int32_t minimum,
    const std::int32_t maximum) noexcept {
    state = 894955033U * state + 1831589974U;
    const std::uint32_t range = static_cast<std::uint32_t>(
        static_cast<std::int64_t>(maximum) - minimum);
    const std::uint64_t product = static_cast<std::uint64_t>(state) * range;
    return static_cast<std::int32_t>(
        static_cast<std::int64_t>(minimum)
        + static_cast<std::uint32_t>(product >> 32U));
}

std::vector<std::uint32_t> ace_waveform_channel_masks(
    std::uint32_t speaker_activity_mask,
    std::uint32_t stream_index,
    bool stereo) {
    // DTSX2_ACEW_RegisterObjs stores one 12-byte descriptor and expands its
    // activity mask through DTSX2_SPKRACTNUMCH_TABLE.  Streamset output is
    // consequently indexed in the expanded physical-channel order, not by
    // the ordinal activity bit.  Keep this mapping shared by the diagnostic
    // and future native-free object bus paths.
    // The homatic and TCL ARM builds use the same 20-entry table and the same
    // RegisterStreamsets contract; only the temporary GetStreamsetOutputBuffer
    // stack scratch size differs, so no platform-specific remap is required.
    const std::vector<std::uint32_t> speakers =
        dtsx::expand_speaker_activity_mask(speaker_activity_mask);
    const std::size_t width = stereo ? 2U : 1U;
    const std::size_t first = static_cast<std::size_t>(stream_index) * width;
    std::vector<std::uint32_t> result(width, 0U);
    for (std::size_t channel = 0U; channel < width; ++channel) {
        const std::size_t index = first + channel;
        if (index < speakers.size()) {
            result[channel] = speakers[index];
        }
    }
    return result;
}

AceWaveformRegistration ace_make_waveform_registration(
    std::uint32_t stream_set_id,
    std::uint32_t stream_index,
    std::uint32_t speaker_activity_mask,
    bool stereo) {
    const std::vector<std::uint32_t> channels =
        ace_waveform_channel_masks(
            speaker_activity_mask, stream_index, stereo);
    AceWaveformRegistration result;
    result.stream_set_id = stream_set_id;
    result.stream_index = stream_index;
    result.speaker_activity_mask = speaker_activity_mask;
    result.first_channel = stream_index * (stereo ? 2U : 1U);
    result.channel_count = static_cast<std::uint32_t>(channels.size());
    // Homatic ACEW marks activity 3/12 at the current expanded physical
    // ordinal (v16), not on every channel in the descriptor.  Reconstruct
    // those per-channel markers before selecting this stream's ordinal.
    std::vector<std::uint8_t> all_relabel;
    all_relabel.reserve(dtsx::speaker_count_from_activity_mask(
        speaker_activity_mask));
    for (std::uint32_t activity = 0U; activity < 20U; ++activity) {
        if ((speaker_activity_mask & (1U << activity)) == 0U) {
            continue;
        }
        const std::uint8_t count =
            activity == 1U || activity == 2U || activity == 5U
                || activity == 6U || activity == 9U || activity == 10U
                || activity == 11U || activity == 13U || activity == 15U
                || activity == 17U || activity == 18U || activity == 19U
                ? 2U : 1U;
        const std::uint8_t marker =
            activity == 3U ? 1U : activity == 12U ? 2U : 0U;
        for (std::uint8_t channel = 0U; channel < count; ++channel) {
            all_relabel.push_back(marker);
        }
    }
    const std::size_t first = result.first_channel;
    result.relabel_flags.assign(result.channel_count, 0U);
    for (std::size_t channel = 0U; channel < result.channel_count; ++channel) {
        const std::size_t ordinal = first + channel;
        if (ordinal < all_relabel.size()) {
            result.relabel_flags[channel] = all_relabel[ordinal];
        }
        result.left_relabel = result.left_relabel
            || (result.relabel_flags[channel] & 1U) != 0U;
        result.right_relabel = result.right_relabel
            || (result.relabel_flags[channel] & 2U) != 0U;
    }
    return result;
}

namespace {

std::int32_t negative_rice_map(std::uint32_t value) noexcept {
    return (value & 1U) != 0U
        ? -static_cast<std::int32_t>((value + 1U) >> 1U)
        : static_cast<std::int32_t>(value >> 1U);
}

std::int32_t positive_rice_map(std::uint32_t value) noexcept {
    return (value & 1U) != 0U
        ? static_cast<std::int32_t>((value + 1U) >> 1U)
        : -static_cast<std::int32_t>(value >> 1U);
}

std::uint32_t modular(
    std::int64_t value,
    std::uint32_t low,
    std::uint32_t high) noexcept {
    const std::int64_t alphabet =
        static_cast<std::int64_t>(high) - low + 1;
    while (value > high) {
        value -= alphabet;
    }
    while (value < low) {
        value += alphabet;
    }
    return static_cast<std::uint32_t>(value);
}

bool read_primary_lognorms(
    AceBitReader& source,
    AceStreamPrefix& prefix,
    const AceStreamPrefixState* previous) noexcept {
    // DTSAceBitStreamUnpacker_UnpackCoarseResiduals chooses its low-band
    // Golomb table by (predictive, short_transform).  The four native rows
    // are: B/A/A, B/A/A, C/B/A and D/A/A.
    const AceCoarseResidualControl residual_control{
        prefix.first_effective_channel,
        prefix.effective_channel_count,
        prefix.effective_bands,
        prefix.predictive ? 0U : 1U,
        prefix.predictive || !prefix.short_transform ? 0U : 1U,
    };
    std::size_t residual_bits = 0U;
    if (!unpack_ace_coarse_residuals(
            source, residual_control, prefix.primary_lognorm_residual,
            prefix.lognorm_k_boost, residual_bits)) {
        return false;
    }
    const AceScalarDequantControl control{
        prefix.predictive,
        prefix.first_effective_channel,
        prefix.effective_channel_count,
        prefix.effective_bands,
    };
    const AceScalarMatrix* previous_q10 = previous != nullptr
            && previous->initialized
        ? &previous->primary_lognorm_q10 : nullptr;
    return ace_scalar_dequant_coarse(
        prefix.primary_lognorm_residual,
        previous_q10,
        control,
        prefix.primary_lognorm_q10);
}

void enforce_block_change_range(
    std::int32_t& value,
    bool short_transform,
    bool narrow) noexcept {
    const std::int32_t alphabet = narrow ? 4 : 5;
    const std::int32_t minimum = short_transform ? -3 : 0;
    while (value >= minimum + alphabet) {
        value -= alphabet;
    }
    while (value < minimum) {
        value += alphabet;
    }
}

bool read_band_block_counts(
    AceBitReader& source,
    std::uint32_t sample_rate,
    AceStreamPrefix& prefix) noexcept {
    std::array<std::int32_t, 22> changes{};
    std::uint32_t value = 0U;
    if (!source.read(1U, value)) {
        return false;
    }
    if (value != 0U) {
        static constexpr std::array<std::uint32_t, 4> kIndexToDk{
            2U, 3U, 0U, 1U};
        std::uint32_t index = 0U;
        if (!read_ace_unary(source, 4U, index) || index >= kIndexToDk.size()) {
            return false;
        }
        const std::uint32_t dk = kIndexToDk[index];
        const bool differential = (dk >> 1U) != 0U;
        const bool uniform = (dk & 1U) != 0U;
        std::uint32_t head_minus_one = 0U;
        if (!read_ace_uniform(
                source, prefix.effective_bands, head_minus_one)) {
            return false;
        }
        const std::uint32_t head_length = head_minus_one + 1U;
        const std::uint32_t narrow_bands = sample_rate == 44100U ? 6U : 8U;
        static constexpr std::array<std::int32_t, 4> kShortNarrow{
            0, -1, -2, -3};
        static constexpr std::array<std::int32_t, 5> kShortWide{
            0, 1, -1, -2, -3};
        for (std::uint32_t band = 0U; band < head_length; ++band) {
            const bool narrow = band < narrow_bands;
            const std::uint32_t alphabet = narrow ? 4U : 5U;
            std::uint32_t mapped = 0U;
            if (!(uniform
                    ? read_ace_uniform(source, alphabet, mapped)
                    : read_ace_unary(source, alphabet, mapped))) {
                return false;
            }
            if (differential) {
                changes[band] = positive_rice_map(mapped);
            } else if (prefix.short_transform) {
                changes[band] = narrow
                    ? kShortNarrow[mapped]
                    : kShortWide[mapped];
            } else {
                changes[band] = static_cast<std::int32_t>(mapped);
            }
        }
        if (differential) {
            for (std::uint32_t band = 0U;
                 band < prefix.effective_bands;
                 ++band) {
                if (band != 0U) {
                    changes[band] += changes[band - 1U];
                }
                enforce_block_change_range(
                    changes[band],
                    prefix.short_transform,
                    band < narrow_bands);
            }
        }
    }
    const std::int32_t base_log = prefix.short_transform ? 3 : 0;
    for (std::uint32_t band = 0U;
         band < prefix.effective_bands;
         ++band) {
        const std::int32_t block_log = base_log + changes[band];
        if (block_log < 0 || block_log > 4) {
            return false;
        }
        prefix.band_block_count[band] = 1U << block_log;
    }
    return true;
}

std::uint32_t center_map(
    std::uint32_t alphabet,
    std::uint32_t center,
    std::uint32_t value) noexcept {
    if (center > (alphabet - 1U) / 2U) {
        return alphabet - center_map(
            alphabet, alphabet - center - 1U, value) - 1U;
    }
    if (value > 2U * center) {
        return value;
    }
    return (value & 1U) != 0U
        ? center + ((value + 1U) >> 1U)
        : center - (value >> 1U);
}

bool read_predictive_band_count(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t previous,
    std::uint32_t& value) noexcept {
    std::uint32_t encoded = 0U;
    if (!read_ace_golomb_limited(
            source, alphabet, 0U, 7U, encoded)) {
        return false;
    }
    value = modular(
        static_cast<std::int64_t>(previous) + negative_rice_map(encoded),
        0U,
        alphabet - 1U);
    return true;
}

bool read_stereo_allocation_header(
    AceBitReader& source,
    const AceStreamPrefixState* previous,
    AceStreamPrefix& prefix) noexcept {
    prefix.lr_allocation_mode.fill(0U);
    if (!prefix.stereo) {
        return true;
    }
    if (prefix.coding_mode != AceCodingMode::Full) {
        std::fill_n(
            prefix.stereo_coding.begin(), prefix.effective_bands, 2U);
        return true;
    }

    const std::uint32_t previous_ms = previous != nullptr
            && previous->initialized
        ? std::min(prefix.effective_bands, previous->num_ms_mono_bands)
        : 0U;
    std::uint32_t ms_mono_bands = 0U;
    if (prefix.predictive) {
        if (!read_predictive_band_count(
                source,
                prefix.effective_bands + 1U,
                previous_ms,
                ms_mono_bands)) {
            return false;
        }
    } else if (!read_ace_uniform(
                   source, prefix.effective_bands + 1U, ms_mono_bands)) {
        return false;
    }
    if (ms_mono_bands > prefix.effective_bands) {
        return false;
    }
    prefix.num_ms_mono_bands = ms_mono_bands;
    const std::uint32_t non_ms_bands =
        prefix.effective_bands - ms_mono_bands;
    for (std::uint32_t band = non_ms_bands;
         band < prefix.effective_bands;
         ++band) {
        prefix.stereo_coding[band] = 1U;
    }

    std::uint32_t full_frame = 0U;
    if (!source.read(1U, full_frame)) {
        return false;
    }
    bool has_left_right = false;
    if (full_frame != 0U) {
        std::uint32_t mid_side = 0U;
        if (!source.read(1U, mid_side)) {
            return false;
        }
        std::fill_n(
            prefix.stereo_coding.begin(),
            non_ms_bands,
            mid_side != 0U ? 3U : 2U);
        has_left_right = mid_side == 0U && non_ms_bands != 0U;
    } else {
        for (std::uint32_t band = 0U; band < non_ms_bands; ++band) {
            std::uint32_t mid_side = 0U;
            if (!source.read(1U, mid_side)) {
                return false;
            }
            prefix.stereo_coding[band] = mid_side != 0U ? 3U : 2U;
            has_left_right = has_left_right || mid_side == 0U;
        }
    }
    if (!has_left_right) {
        return true;
    }

    const std::uint32_t region_count = non_ms_bands == 0U
        ? 0U : non_ms_bands <= 4U ? 1U : non_ms_bands <= 12U ? 2U : 3U;
    std::uint32_t all_equal = 0U;
    if (!source.read(1U, all_equal)) {
        return false;
    }
    if (all_equal != 0U) {
        return true;
    }
    static constexpr std::array<std::uint32_t, 3> kRegionEnds{
        4U, 12U, 22U};
    std::uint32_t begin = 0U;
    for (std::uint32_t region = 0U; region < region_count; ++region) {
        const std::uint32_t end = std::min(
            non_ms_bands, kRegionEnds[region]);
        bool region_has_left_right = false;
        for (std::uint32_t band = begin; band < end; ++band) {
            region_has_left_right = region_has_left_right
                || prefix.stereo_coding[band] == 2U;
        }
        begin = end;
        if (!region_has_left_right) {
            continue;
        }
        std::uint32_t equal = 0U;
        if (!source.read(1U, equal)) {
            return false;
        }
        if (equal == 0U) {
            std::uint32_t mode = 0U;
            if (!read_ace_uniform(source, 4U, mode)) {
                return false;
            }
            prefix.lr_allocation_mode[region] = mode + 1U;
        }
    }
    return true;
}

bool read_allocation_header(
    AceBitReader& source,
    const AceStreamPrefixState* previous,
    AceStreamPrefix& prefix) noexcept {
    const std::uint32_t previous_coded = previous != nullptr
            && previous->initialized
        ? std::min(prefix.effective_bands, previous->num_coded_bands)
        : 0U;
    std::uint32_t noncoded = 0U;
    if (prefix.predictive) {
        if (!read_predictive_band_count(
                source,
                prefix.effective_bands + 1U,
                prefix.effective_bands - previous_coded,
                noncoded)) {
            return false;
        }
    } else if (!read_ace_uniform(
                   source, prefix.effective_bands + 1U, noncoded)) {
        return false;
    }
    if (noncoded > prefix.effective_bands) {
        return false;
    }
    prefix.num_coded_bands = prefix.effective_bands - noncoded;
    if (prefix.num_coded_bands == 0U) {
        prefix.allocation_model_parameter = 0;
        return true;
    }
    if (!read_stereo_allocation_header(source, previous, prefix)
        || !read_ace_uniform(source, 6U, prefix.allocation_model)) {
        return false;
    }
    if (prefix.allocation_model <= 3U) {
        std::uint32_t encoded = 0U;
        if (prefix.predictive) {
            if (!read_ace_unary(source, 21U, encoded)) {
                return false;
            }
            const std::int32_t previous_parameter = previous != nullptr
                    && previous->initialized
                ? previous->allocation_model_parameter : 0;
            const std::uint32_t mapped = center_map(
                21U,
                static_cast<std::uint32_t>(
                    std::clamp(previous_parameter, -10, 10) + 10),
                encoded);
            prefix.allocation_model_parameter =
                static_cast<std::int32_t>(mapped) - 10;
        } else {
            if (!read_ace_uniform(source, 21U, encoded)) {
                return false;
            }
            prefix.allocation_model_parameter =
                static_cast<std::int32_t>(encoded) - 10;
        }
    }
    return true;
}

bool read_allocation_delta(
    AceBitReader& source,
    std::uint32_t sample_rate,
    AceStreamPrefix& prefix) noexcept {
    if (prefix.num_coded_bands == 0U || prefix.allocation_model > 2U) {
        return true;
    }
    static constexpr std::array<std::array<std::uint16_t, 22>, 2>
        kMaxSteps48{{
            {11, 11, 11, 11, 11, 11, 11, 11, 23, 23, 23,
             45, 45, 45, 45, 64, 64, 59, 51, 30, 29, 29},
            {24, 24, 24, 24, 24, 24, 24, 24, 48, 48, 48,
             72, 72, 72, 72, 68, 68, 63, 55, 34, 33, 33},
        }};
    static constexpr std::array<std::array<std::uint16_t, 22>, 2>
        kBitSteps48{{
            {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
             6, 6, 6, 6, 6, 6, 10, 12, 20, 22, 22},
            {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
             8, 8, 8, 8, 12, 12, 20, 24, 40, 44, 44},
        }};
    static constexpr std::array<std::array<std::uint16_t, 22>, 2>
        kMaxSteps441{{
            {11, 11, 11, 11, 11, 11, 23, 23, 23, 23, 23,
             45, 45, 45, 68, 64, 64, 59, 51, 30, 29, 29},
            {24, 24, 24, 24, 24, 24, 48, 48, 48, 48, 48,
             72, 72, 72, 72, 68, 68, 63, 55, 34, 33, 33},
        }};
    static constexpr std::array<std::array<std::uint16_t, 22>, 2>
        kBitSteps441{{
            {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
             6, 6, 6, 6, 6, 6, 10, 14, 20, 26, 12},
            {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
             8, 8, 8, 12, 12, 12, 20, 28, 40, 52, 24},
        }};
    const std::size_t channel_index =
        prefix.effective_channel_count == 2U ? 1U : 0U;
    const auto& max_steps = sample_rate == 44100U
        ? kMaxSteps441[channel_index] : kMaxSteps48[channel_index];
    const auto& bit_steps = sample_rate == 44100U
        ? kBitSteps441[channel_index] : kBitSteps48[channel_index];

    std::uint32_t head_length = 0U;
    if (prefix.allocation_model == 0U) {
        std::uint32_t all_zero = 0U;
        if (!source.read(1U, all_zero)) {
            return false;
        }
        if (all_zero == 0U) {
            std::uint32_t head_minus_one = 0U;
            if (!read_ace_uniform(
                    source, prefix.effective_bands, head_minus_one)) {
                return false;
            }
            head_length = head_minus_one + 1U;
        }
    } else if (!read_ace_uniform(
                   source, prefix.effective_bands + 1U, head_length)) {
        return false;
    }
    for (std::uint32_t band = 0U; band < head_length; ++band) {
        std::uint32_t steps = 0U;
        if (!read_ace_unary(source, max_steps[band] + 1U, steps)) {
            return false;
        }
        prefix.allocation_delta[band] = static_cast<std::int32_t>(
            steps * bit_steps[band]);
    }
    return true;
}

struct AllocationTables final {
    const std::array<std::uint16_t, 22>& maximum;
    const std::array<std::uint16_t, 22>& minimum;
    const std::array<std::uint16_t, 22>& band_size;
    const std::array<std::int32_t, 22>& log_band_size_q10;
    const std::array<std::int32_t, 22>& lognorm_offset_q10;
};

constexpr std::array<std::uint32_t, 100> kTemporalHoleFillBitAllocThresholdQ30 = {{
    1073741824U, 934746560U, 813744128U, 708405440U, 616702720U,
    536870912U, 467373280U, 406872064U, 354202720U, 308351360U,
    268435456U, 233686640U, 203436032U, 177101360U, 154175680U,
    134217728U, 116843320U, 101718016U, 88550680U, 77087840U,
    67108864U, 58421660U, 50859008U, 44275340U, 38543920U,
    33554432U, 29210830U, 25429504U, 22137670U, 19271960U,
    16777216U, 14605415U, 12714752U, 11068835U, 9635980U,
    8388608U, 7302707U, 6357376U, 5534417U, 4817990U,
    4194304U, 3651353U, 3178688U, 2767208U, 2408995U,
    2097152U, 1825676U, 1589344U, 1383604U, 1204497U,
    1048576U, 912838U, 794672U, 691802U, 602248U,
    524288U, 456419U, 397336U, 345901U, 301124U,
    262144U, 228209U, 198668U, 172950U, 150562U,
    131072U, 114104U, 99334U, 86475U, 75281U,
    65536U, 57052U, 49667U, 43237U, 37640U,
    32768U, 28526U, 24833U, 21618U, 18820U,
    16384U, 14263U, 12416U, 10809U, 9410U,
    8192U, 7131U, 6208U, 5404U, 4705U,
    4095U, 3565U, 3104U, 2702U, 2352U,
    2048U, 1782U, 1552U, 1351U, 1176U}};

// Extracted from homatic/libHwAudio_dtsx.so weak symbols:
// DTS_ACE_THF_RECIP_NUM_TOTAL_BINS_PER_BAND_* (Q28), two 22-band rows.
constexpr std::array<std::uint32_t, 44> kThfRecipTotalBins48Q28 = {{
    0x10000000U,0x10000000U,0x10000000U,0x10000000U,0x10000000U,0x10000000U,
    0x10000000U,0x10000000U,0x08000000U,0x08000000U,0x08000000U,0x04000000U,
    0x04000000U,0x04000000U,0x04000000U,0x02AAAAACU,0x02AAAAACU,0x0199999AU,
    0x01555556U,0x00CCCCCDU,0x00BA2E8CU,0x00BA2E8CU,0x08000000U,0x08000000U,
    0x08000000U,0x08000000U,0x08000000U,0x08000000U,0x08000000U,0x08000000U,
    0x04000000U,0x04000000U,0x04000000U,0x02000000U,0x02000000U,0x02000000U,
    0x02000000U,0x01555556U,0x01555556U,0x00CCCCCDU,0x00AAAAABU,0x00666666U,
    0x005D1746U,0x005D1746U}};
constexpr std::array<std::uint32_t, 44> kThfRecipTotalBins44Q28 = {{
    0x10000000U,0x10000000U,0x10000000U,0x10000000U,0x10000000U,0x10000000U,
    0x08000000U,0x08000000U,0x08000000U,0x08000000U,0x08000000U,0x04000000U,
    0x04000000U,0x04000000U,0x02AAAAACU,0x02AAAAACU,0x02AAAAACU,0x0199999AU,
    0x0124924AU,0x00CCCCCDU,0x009D89D9U,0x01555556U,0x08000000U,0x08000000U,
    0x08000000U,0x08000000U,0x08000000U,0x08000000U,0x04000000U,0x04000000U,
    0x04000000U,0x04000000U,0x04000000U,0x02000000U,0x02000000U,0x02000000U,
    0x01555556U,0x01555556U,0x01555556U,0x00CCCCCDU,0x00924925U,0x00666666U,
    0x004EC4ECU,0x00AAAAABU}};
constexpr std::array<std::uint32_t, 22> kThfRecipSqrtBins48Q30 = {{
    0x2D413CC0U,0x2D413CC0U,0x2D413CC0U,0x2D413CC0U,0x2D413CC0U,0x2D413CC0U,
    0x2D413CC0U,0x2D413CC0U,0x20000000U,0x20000000U,0x20000000U,0x16A09E60U,
    0x16A09E60U,0x16A09E60U,0x16A09E60U,0x1279A740U,0x1279A740U,0x0E4F92E0U,
    0x0D105EB0U,0x0A1E89B0U,0x09A5FB20U,0x09A5FB20U}};
constexpr std::array<std::uint32_t, 22> kThfRecipSqrtBins44Q30 = {{
    0x2D413CC0U,0x2D413CC0U,0x2D413CC0U,0x2D413CC0U,0x2D413CC0U,0x2D413CC0U,
    0x20000000U,0x20000000U,0x20000000U,0x20000000U,0x20000000U,0x16A09E60U,
    0x16A09E60U,0x16A09E60U,0x1279A740U,0x1279A740U,0x1279A740U,0x0E4F92E0U,
    0x0C184900U,0x0A1E89B0U,0x08E00D50U,0x0D105EB0U}};

std::uint32_t temporal_hole_fill_bit_alloc_threshold_q30_impl(
    const std::uint32_t index) noexcept {
    return kTemporalHoleFillBitAllocThresholdQ30[
        (std::min)(index, static_cast<std::uint32_t>(99U))];
}

AllocationTables allocation_tables(
    std::uint32_t sample_rate,
    std::uint32_t channels) noexcept {
    static constexpr std::array<std::uint16_t, 22> kMaximum48Mono{
        68, 68, 68, 68, 68, 68, 68, 68, 136, 136, 136,
        272, 272, 272, 272, 384, 384, 592, 608, 608, 640, 640};
    static constexpr std::array<std::uint16_t, 22> kMaximum48Stereo{
        144, 144, 144, 144, 144, 144, 144, 144, 288, 288, 288,
        576, 576, 576, 576, 816, 816, 1264, 1312, 1376, 1456, 1456};
    static constexpr std::array<std::uint16_t, 22> kMinimum48{
        2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3,
        6, 6, 6, 6, 8, 8, 14, 17, 28, 30, 30};
    static constexpr std::array<std::uint16_t, 22> kBandSize48{
        8, 8, 8, 8, 8, 8, 8, 8, 16, 16, 16,
        32, 32, 32, 32, 48, 48, 80, 96, 160, 176, 176};
    static constexpr std::array<std::int32_t, 22> kLogBandSize48Q10{
        -1152, -1152, -1152, -1152, -1152, -1152, -1152, -1152,
        -640, -640, -640, -128, -128, -128, -128, 128, 128, 512,
        640, 1024, 1088, 1088};
    static constexpr std::array<std::int32_t, 22> kLognormOffsetQ10{
        9068, 8977, 8600, 8214, 7992, 7744, 7602, 7574,
        8070, 7937, 7824, 8322, 8109, 7907, 7700, 7889,
        7716, 7909, 7723, 7515, 6622, 5758};
    static constexpr std::array<std::uint16_t, 22> kMaximum441Mono{
        68, 68, 68, 68, 68, 68, 136, 136, 136, 136, 136,
        272, 272, 272, 408, 384, 384, 592, 708, 608, 756, 348};
    static constexpr std::array<std::uint16_t, 22> kMaximum441Stereo{
        144, 144, 144, 144, 144, 144, 288, 288, 288, 288, 288,
        576, 576, 576, 864, 816, 816, 1264, 1528, 1376, 1720, 792};
    static constexpr std::array<std::uint16_t, 22> kMinimum441{
        2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3,
        6, 6, 6, 8, 8, 8, 14, 19, 28, 36, 17};
    // The 44.1-kHz sizes follow the native band boundaries. They are also
    // reflected by its max-allocation and quantization-step tables.
    static constexpr std::array<std::uint16_t, 22> kBandSize441{
        8, 8, 8, 8, 8, 8, 16, 16, 16, 16, 16,
        32, 32, 32, 48, 48, 48, 80, 112, 160, 208, 96};
    static constexpr std::array<std::int32_t, 22> kLogBandSize441Q10{
        -1152, -1152, -1152, -1152, -1152, -1152,
        -640, -640, -640, -640, -640, -128, -128, -128,
        128, 128, 128, 512, 768, 1024, 1216, 640};
    const bool stereo = channels == 2U;
    if (sample_rate == 44100U) {
        return {
            stereo ? kMaximum441Stereo : kMaximum441Mono,
            kMinimum441,
            kBandSize441,
            kLogBandSize441Q10,
            kLognormOffsetQ10};
    }
    return {
        stereo ? kMaximum48Stereo : kMaximum48Mono,
        kMinimum48,
        kBandSize48,
        kLogBandSize48Q10,
        kLognormOffsetQ10};
}

std::uint32_t allocation_bit_step(
    const AllocationTables& tables,
    std::uint32_t channels,
    std::uint32_t band) noexcept {
    const std::uint32_t bins = channels * tables.band_size[band];
    return bins < 6U ? bins : bins < 48U ? 6U : bins / 8U;
}

bool read_allocation_difference(
    AceBitReader& source,
    const AceStreamPrefix& prefix,
    const AllocationTables& tables,
    const std::array<std::uint32_t, 22>& reference,
    std::uint32_t reference_bands,
    std::uint32_t reference_guaranteed,
    std::array<std::uint32_t, 22>& output) noexcept {
    const std::uint32_t guaranteed = prefix.effective_channel_count;
    std::array<std::uint32_t, 22> reduced_reference{};
    std::array<std::uint32_t, 22> reduced_maximum{};
    std::array<std::int32_t, 22> minimum_delta{};
    std::array<std::int32_t, 22> maximum_delta{};
    std::uint32_t maximum_alphabet = 0U;
    for (std::uint32_t band = 0U;
         band < prefix.effective_bands;
         ++band) {
        reduced_maximum[band] = tables.maximum[band] - guaranteed;
        reduced_reference[band] = band < reference_bands
            && reference[band] >= reference_guaranteed
            ? reference[band] - reference_guaranteed : 0U;
    }
    for (std::uint32_t band = 0U;
         band < prefix.num_coded_bands;
         ++band) {
        std::uint32_t& ref = reduced_reference[band];
        ref = std::min(ref, reduced_maximum[band]);
        const std::uint32_t step = allocation_bit_step(
            tables, prefix.effective_channel_count, band);
        maximum_delta[band] = static_cast<std::int32_t>(
            (reduced_maximum[band] - ref + step - 1U) / step);
        minimum_delta[band] = -static_cast<std::int32_t>(
            (ref + step - 1U) / step);
        maximum_alphabet = std::max(
            maximum_alphabet,
            static_cast<std::uint32_t>(
                maximum_delta[band] - minimum_delta[band] + 1));
    }
    std::uint32_t parameter = 0U;
    const std::uint32_t parameter_alphabet =
        ace_num_bits(maximum_alphabet - 1U);
    if (!read_ace_unary(source, parameter_alphabet, parameter)) {
        return false;
    }
    output.fill(guaranteed);
    for (std::uint32_t band = 0U;
         band < prefix.num_coded_bands;
         ++band) {
        const std::uint32_t alphabet = static_cast<std::uint32_t>(
            maximum_delta[band] - minimum_delta[band] + 1);
        std::uint32_t encoded = 0U;
        if (!read_ace_golomb(source, alphabet, parameter, encoded)) {
            return false;
        }
        const std::uint32_t mapped = center_map(
            alphabet,
            static_cast<std::uint32_t>(-minimum_delta[band]),
            encoded);
        const std::int32_t delta = static_cast<std::int32_t>(mapped)
            + minimum_delta[band];
        const std::int32_t value = static_cast<std::int32_t>(
            reduced_reference[band])
            + delta * static_cast<std::int32_t>(allocation_bit_step(
                tables, prefix.effective_channel_count, band));
        output[band] = guaranteed + static_cast<std::uint32_t>(
            std::clamp<std::int32_t>(
                value, 0, static_cast<std::int32_t>(
                    reduced_maximum[band])));
    }
    return true;
}

void split_refinement_allocation(
    const AceStreamPrefix& prefix,
    const AllocationTables& tables,
    std::array<std::uint32_t, 22>& allocation,
    std::array<std::uint32_t, 22>& refinement) noexcept {
    static constexpr std::array<std::int16_t, 22> kRefinementDeltaQ10{
        384, 384, 384, 384, 384, 384, 384, 384,
        512, 512, 512, 640, 640, 640, 640, 696, 752, 808,
        864, 920, 960, 960};
    refinement.fill(0U);
    std::int32_t carry = 0;
    for (std::uint32_t band = 0U;
         band < prefix.num_coded_bands;
         ++band) {
        const std::int32_t combined =
            carry + static_cast<std::int32_t>(allocation[band]);
        allocation[band] = static_cast<std::uint32_t>(
            std::min<std::int32_t>(combined, tables.maximum[band]));
        carry = combined - static_cast<std::int32_t>(allocation[band]);

        const std::int32_t band_size = static_cast<std::int32_t>(
            tables.band_size[band]);
        const std::int32_t channels = static_cast<std::int32_t>(
            prefix.effective_channel_count);
        const std::int32_t bin_bits_q10 =
            (band_size * channels) << 10;
        std::int32_t estimate_q10 =
            static_cast<std::int32_t>(allocation[band] << 10U)
            + static_cast<std::int32_t>(
                (static_cast<std::int64_t>(tables.log_band_size_q10[band])
                    * bin_bits_q10) >> 10U);
        if (estimate_q10 < 3072 * band_size * channels) {
            const std::int32_t multiplier =
                estimate_q10 < (bin_bits_q10 << 1U) ? 2 : 1;
            estimate_q10 += static_cast<std::int32_t>(
                (static_cast<std::int64_t>(bin_bits_q10)
                    * kRefinementDeltaQ10[band] * multiplier) >> 10U);
        }
        const std::int64_t numerator = static_cast<std::int64_t>(
            bin_bits_q10 + 2 * estimate_q10) << 9U;
        std::int32_t refinement_bits = static_cast<std::int32_t>(
            (numerator / bin_bits_q10) >> 10U);
        refinement_bits = std::clamp(refinement_bits, 0, 8);
        if (channels * refinement_bits
            > static_cast<std::int32_t>(allocation[band])) {
            refinement_bits = static_cast<std::int32_t>(
                allocation[band]) / channels;
        }
        refinement[band] = static_cast<std::uint32_t>(refinement_bits);
        allocation[band] -= static_cast<std::uint32_t>(
            channels * refinement_bits);
        if (carry > 0) {
            const std::int32_t extra = std::min(
                8 - refinement_bits, carry / channels);
            refinement[band] += static_cast<std::uint32_t>(extra);
            carry -= channels * extra;
        }
    }
    for (std::uint32_t band = prefix.num_coded_bands;
         band < prefix.effective_bands;
         ++band) {
        refinement[band] =
            allocation[band] / prefix.effective_channel_count;
        allocation[band] = 0U;
    }
}

std::uint32_t allocation_model_step(
    const AceStreamPrefix& prefix,
    const AllocationTables& tables,
    std::int32_t first_bin_q10,
    const std::array<std::int32_t, 22>& adjustments,
    std::array<std::uint32_t, 22>& output) noexcept {
    const std::uint32_t guaranteed = prefix.effective_channel_count;
    output.fill(guaranteed);
    std::int32_t band_bin_q10 = first_bin_q10;
    for (std::uint32_t band = 0U;
         band < prefix.num_coded_bands;
         ++band) {
        std::int32_t effective_bin_q10 = band_bin_q10;
        if (band > 19U) {
            effective_bin_q10 -= 1024;
        }
        const std::uint32_t band_size = tables.band_size[band];
        std::uint32_t value = static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(prefix.effective_channel_count)
                * static_cast<std::uint32_t>(
                    std::max(effective_bin_q10, 0))
                * band_size) >> 10U);
        if (first_bin_q10 > 0 && adjustments[band] > 0) {
            value += static_cast<std::uint32_t>(adjustments[band]);
        }
        output[band] = std::clamp<std::uint32_t>(
            value, guaranteed, tables.maximum[band]);
        band_bin_q10 += -162 - 8 * prefix.allocation_model_parameter;
    }

    std::int32_t last_nonzero =
        static_cast<std::int32_t>(prefix.num_coded_bands) - 1;
    while (last_nonzero >= 0
           && output[static_cast<std::size_t>(last_nonzero)]
               < tables.minimum[static_cast<std::size_t>(last_nonzero)]) {
        --last_nonzero;
    }
    for (std::int32_t band = last_nonzero; band >= 0; --band) {
        output[static_cast<std::size_t>(band)] =
            std::max<std::uint32_t>(
                output[static_cast<std::size_t>(band)],
                tables.minimum[static_cast<std::size_t>(band)]);
    }
    for (std::uint32_t band =
             static_cast<std::uint32_t>(last_nonzero + 1);
         band < prefix.num_coded_bands;
         ++band) {
        output[band] = guaranteed;
    }
    std::uint32_t total = 0U;
    for (std::uint32_t band = 0U;
         band < prefix.effective_bands;
         ++band) {
        total += output[band];
    }
    return total;
}

std::array<std::int32_t, 22> compute_lognorm_adjustments(
    std::uint32_t sample_rate,
    std::uint32_t total_stream_bits,
    AceStreamPrefix& prefix) noexcept {
    static constexpr std::array<std::int16_t, 21> kForwardMaskQ10{
        1280, 1216, 1216, 1216, 1280, 1600, 1408,
        2112, 1280, 1408, 2048, 1216, 1216, 1216,
        1792, 1216, 1600, 1280, 1216, 704, 1536};
    static constexpr std::array<std::int16_t, 21> kBackwardMaskQ10{
        1792, 1536, 1728, 1792, 1920, 2112, 1920,
        2624, 1920, 1792, 2560, 1792, 1792, 1792,
        2304, 1792, 2112, 2048, 1920, 1088, 2048};
    const AllocationTables tables = allocation_tables(
        sample_rate, prefix.effective_channel_count);
    std::array<std::array<std::int32_t, 22>, 2> lognorm{};
    for (std::uint32_t channel = 0U; channel < 2U; ++channel) {
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands;
             ++band) {
            const std::int32_t value =
                prefix.primary_lognorm_q10[channel][band];
            lognorm[channel][band] = value == -32768
                ? -32768 : value + tables.lognorm_offset_q10[band];
        }
    }
    std::array<std::array<std::int32_t, 22>, 2> masked{};
    const std::uint32_t stream_channels = prefix.stereo ? 2U : 1U;
    for (std::uint32_t channel = 0U; channel < stream_channels; ++channel) {
        masked[channel][0] = lognorm[channel][0];
        std::uint32_t last_rise = 0U;
        for (std::uint32_t band = 1U;
             band < prefix.effective_bands;
             ++band) {
            if (lognorm[channel][band]
                > lognorm[channel][band - 1U] + 512) {
                last_rise = band;
            }
            masked[channel][band] = std::min(
                lognorm[channel][band],
                masked[channel][band - 1U] + kForwardMaskQ10[band - 1U]);
        }
        for (std::int32_t band = static_cast<std::int32_t>(last_rise) - 1;
             band >= 0;
             --band) {
            masked[channel][static_cast<std::size_t>(band)] = std::min(
                masked[channel][static_cast<std::size_t>(band)],
                masked[channel][static_cast<std::size_t>(band + 1)]
                    + kBackwardMaskQ10[static_cast<std::size_t>(band)]);
        }
    }

    std::array<std::int32_t, 22> levels{};
    if (prefix.effective_channel_count == 2U) {
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands;
             ++band) {
            masked[1][band] = std::max(
                masked[1][band], masked[0][band] - 4096);
            masked[0][band] = std::max(
                masked[0][band], masked[1][band] - 4096);
            const std::int32_t left = std::max(
                lognorm[0][band] - masked[0][band], 0);
            const std::int32_t right = std::max(
                lognorm[1][band] - masked[1][band], 0);
            levels[band] = (left + right) >> 1U;
        }
    } else {
        const std::uint32_t channel = prefix.first_effective_channel;
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands;
             ++band) {
            levels[band] = std::max(
                lognorm[channel][band]
                    - masked[channel][band],
                0);
        }
    }
    prefix.allocation_lognorm_level = levels;
    if (!prefix.short_transform) {
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands;
             ++band) {
            levels[band] >>= 1U;
        }
    }
    for (std::uint32_t band = 0U;
         band < prefix.effective_bands;
         ++band) {
        std::int32_t value = levels[band];
        if (band < 8U) {
            value *= 2;
        }
        if (band >= 12U - (prefix.effective_bands < 20U ? 1U : 0U)) {
            value >>= 1U;
        }
        if (band >= 17U) {
            value = (3 * value) >> 2U;
        }
        levels[band] = std::min(value, 4096);
    }

    std::int32_t total = 0;
    for (std::uint32_t band = 0U;
         band < prefix.effective_bands;
         ++band) {
        levels[band] = std::min<std::int32_t>(
            static_cast<std::int32_t>(
                (static_cast<std::int64_t>(levels[band])
                    * prefix.effective_channel_count
                    * tables.band_size[band]) >> 10U),
            tables.maximum[band]);
        total += levels[band];
    }
    const std::int32_t cap = std::max(
        (static_cast<std::int32_t>(total_stream_bits) - 60) >> 1U, 0);
    if (total > cap && prefix.effective_bands != 0U) {
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands;
             ++band) {
            if (levels[band] != 0) {
                levels[band] = static_cast<std::int32_t>(
                    static_cast<std::int64_t>(levels[band]) * cap / total);
            }
        }
    }
    return levels;
}

bool derive_allocation(
    AceBitReader& source,
    std::uint32_t sample_rate,
    std::uint32_t total_stream_bits,
    const AceStreamPrefixState* previous,
    AceStreamPrefix& prefix) noexcept {
    if (prefix.allocation_model > 4U) {
        return true;
    }
    const AllocationTables tables = allocation_tables(
        sample_rate, prefix.effective_channel_count);
    std::array<std::int32_t, 22> adjustments = prefix.allocation_delta;
    if (prefix.allocation_model == 0U
        || prefix.allocation_model == 2U
        || prefix.allocation_model == 3U) {
        const auto lognorm = compute_lognorm_adjustments(
            sample_rate, total_stream_bits, prefix);
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands;
             ++band) {
            std::int32_t value = lognorm[band];
            if (prefix.allocation_model == 2U) {
                const std::int32_t step = static_cast<std::int32_t>(
                    allocation_bit_step(
                        tables, prefix.effective_channel_count, band));
                if (value > step) {
                    value -= step;
                }
            }
            adjustments[band] += value;
        }
    }
    prefix.allocation_adjustment = adjustments;
    if (prefix.allocation_model == 4U) {
        if (previous == nullptr || !previous->initialized
            || !read_allocation_difference(
                source,
                prefix,
                tables,
                previous->raw_allocation,
                previous->num_coded_bands,
                previous->guaranteed_allocation,
                prefix.allocation)) {
            return false;
        }
        prefix.allocation_total_bits = 0U;
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands;
             ++band) {
            prefix.allocation_total_bits += prefix.allocation[band];
        }
        prefix.raw_allocation = prefix.allocation;
        split_refinement_allocation(
            prefix, tables, prefix.allocation, prefix.refinement_allocation);
        return true;
    }
    std::int32_t low = 0;
    std::int32_t high = 8192;
    for (std::uint32_t iteration = 0U; iteration < 13U; ++iteration) {
        const std::int32_t middle = (low + high) >> 1U;
        std::uint32_t budget = prefix.allocation_available_bits;
        if (prefix.allocation_model == 3U) {
            std::uint32_t signalling_cost = 0U;
            for (std::uint32_t band = 0U;
                 band < prefix.num_coded_bands;
                 ++band) {
                ++signalling_cost;
                if (adjustments[band] > 0) {
                    const std::uint32_t step = allocation_bit_step(
                        tables, prefix.effective_channel_count, band);
                    signalling_cost += static_cast<std::uint32_t>(
                        adjustments[band] + static_cast<std::int32_t>(step / 2U))
                        / step;
                }
            }
            budget = signalling_cost <= budget
                ? budget - signalling_cost : 0U;
        }
        const std::uint32_t total = allocation_model_step(
            prefix, tables, middle, adjustments,
            prefix.allocation);
        if (total > budget) {
            high = middle;
        } else {
            low = middle;
        }
    }
    std::uint32_t final_budget = prefix.allocation_available_bits;
    if (prefix.allocation_model == 3U) {
        std::uint32_t signalling_cost = 0U;
        for (std::uint32_t band = 0U;
             band < prefix.num_coded_bands;
             ++band) {
            ++signalling_cost;
            if (adjustments[band] > 0) {
                const std::uint32_t step = allocation_bit_step(
                    tables, prefix.effective_channel_count, band);
                signalling_cost += static_cast<std::uint32_t>(
                    adjustments[band] + static_cast<std::int32_t>(step / 2U))
                    / step;
            }
        }
        final_budget = signalling_cost <= final_budget
            ? final_budget - signalling_cost : 0U;
    }
    (void)final_budget;
    prefix.allocation_total_bits = allocation_model_step(
        prefix, tables, low, adjustments, prefix.allocation);
    if (prefix.allocation_model == 3U) {
        const auto reference = prefix.allocation;
        if (!read_allocation_difference(
                source,
                prefix,
                tables,
                reference,
                prefix.num_coded_bands,
                prefix.effective_channel_count,
                prefix.allocation)) {
            return false;
        }
        prefix.allocation_total_bits = 0U;
        for (std::uint32_t band = 0U;
             band < prefix.effective_bands;
             ++band) {
            prefix.allocation_total_bits += prefix.allocation[band];
        }
    }
    prefix.raw_allocation = prefix.allocation;
    split_refinement_allocation(
        prefix, tables, prefix.allocation, prefix.refinement_allocation);
    return true;
}

bool read_lognorm_refinement(
    AceBitReader& source,
    AceStreamPrefix& prefix) noexcept {
    prefix.refined_lognorm_q10 = prefix.primary_lognorm_q10;
    for (std::uint32_t band = 0U;
         band < prefix.effective_bands;
         ++band) {
        const std::uint32_t bits = prefix.refinement_allocation[band];
        if (bits == 0U) {
            continue;
        }
        if (bits > 16U) {
            return false;
        }
        for (std::uint32_t channel = prefix.first_effective_channel;
             channel < prefix.first_effective_channel
                     + prefix.effective_channel_count;
             ++channel) {
            std::uint32_t code = 0U;
            if (!source.read(bits, code)) {
                return false;
            }
            prefix.refinement_code[channel][band] =
                static_cast<std::uint16_t>(code);
        }
    }
    const AceScalarDequantControl control{
        prefix.predictive,
        prefix.first_effective_channel,
        prefix.effective_channel_count,
        prefix.effective_bands,
    };
    if (!ace_scalar_dequant_fine(
            prefix.refined_lognorm_q10,
            prefix.refinement_code,
            prefix.refinement_allocation,
            control)) {
        return false;
    }
    prefix.refinement_bits_consumed = source.position();
    return true;
}

} // namespace

std::uint32_t ace_temporal_hole_fill_bit_alloc_threshold_q30(
    const std::uint32_t index) noexcept {
    return temporal_hole_fill_bit_alloc_threshold_q30_impl(index);
}

std::uint32_t ace_temporal_hole_fill_bit_alloc_index_q30(
    const std::int32_t scaled_allocation_q4) noexcept {
    const std::int64_t scaled = static_cast<std::int64_t>(
        scaled_allocation_q4);
    const std::int64_t value = ((10LL * scaled + 8LL) >> 4) + 0x7FFFFFLL;
    if (value < 0LL || value >= 0x32000000LL) {
        return 99U;
    }
    return (std::min)(static_cast<std::uint32_t>(value >> 23), 99U);
}

std::uint32_t ace_temporal_hole_fill_recip_num_total_bins_q28(
    const std::uint32_t sample_rate,
    const std::uint32_t bandwidth_mode,
    const std::uint32_t band) noexcept {
    if (band >= 22U || bandwidth_mode >= 4U) {
        return 0U;
    }
    const std::size_t index = static_cast<std::size_t>(bandwidth_mode) * 22U
        + band;
    if (bandwidth_mode >= 2U) {
        // Homatic exports only two rows for this weak symbol; modes 2/3 are
        // accepted by SetCommonParam but their reciprocal rows are not
        // present in the artifact. Preserve the mode and report unavailable
        // data instead of aliasing a row heuristically.
        return 0U;
    }
    return sample_rate == 44100U
        ? kThfRecipTotalBins44Q28[index]
        : sample_rate == 48000U ? kThfRecipTotalBins48Q28[index] : 0U;
}

std::uint32_t ace_temporal_hole_fill_recip_sqrt_num_bins_q30(
    const std::uint32_t sample_rate,
    const std::uint32_t bandwidth_mode,
    const std::uint32_t band) noexcept {
    if (band >= 22U || bandwidth_mode >= 4U) {
        return 0U;
    }
    return sample_rate == 44100U
        ? kThfRecipSqrtBins44Q30[band]
        : sample_rate == 48000U ? kThfRecipSqrtBins48Q30[band] : 0U;
}

std::uint32_t ace_temporal_hole_fill_bit_alloc_index_from_recip_q28(
    const std::uint32_t reciprocal_q28,
    const std::int32_t bit_allocation) noexcept {
    if (reciprocal_q28 == 0U || bit_allocation < 0) {
        return 99U;
    }
    const std::int64_t product = static_cast<std::int64_t>(
        reciprocal_q28) * bit_allocation;
    const std::int64_t result = (product + 8LL) >> 4U;
    const std::int64_t value = ((10LL * result + 8LL) >> 4U)
        + 0x7FFFFFLL;
    if (value < 0LL || value >= 0x32000000LL) {
        return 99U;
    }
    return (std::min)(static_cast<std::uint32_t>(value >> 23U), 99U);
}

std::int32_t ace_fixed_mul_q31_round(
    const std::int32_t left,
    const std::int32_t right) noexcept {
    // Native fixed-point branches use (a*b + 0x40000000) >> 31.
    const std::int64_t product = static_cast<std::int64_t>(left) * right;
    const std::int64_t rounded = (product + 0x40000000LL) >> 31;
    return static_cast<std::int32_t>((std::max)(
        static_cast<std::int64_t>((std::numeric_limits<std::int32_t>::min)()),
        (std::min)(static_cast<std::int64_t>(
                       (std::numeric_limits<std::int32_t>::max)()), rounded)));
}

std::int32_t ace_temporal_hole_fill_norm_floor_q31(
    const std::int32_t norm_q31) noexcept {
    constexpr std::int32_t kNormFloorQ31 = 214906;
    return ace_fixed_mul_q31_round(norm_q31, kNormFloorQ31);
}

std::int32_t ace_temporal_hole_fill_calc_offset_index_q30(
    const std::int32_t ratio_q2) noexcept {
    // DTSAceTemporalHoleFill_CalcOffsetIndex, homatic/tcl ARM fixed-point path.
    const std::int64_t v11 = static_cast<std::int64_t>(ratio_q2)
        - 760150976LL;
    const std::int64_t v12 = 1838257536LL * v11;
    std::int64_t v13 = (v12 + 0x10000000LL) >> 29;
    if (v12 < static_cast<std::int64_t>(-0x100000010000000LL)) {
        v13 = static_cast<std::int64_t>(
            (std::numeric_limits<std::int32_t>::min)());
    } else if (v12 > 0x0FFFFFFEFFFFFFFLL) {
        v13 = static_cast<std::int64_t>(
            (std::numeric_limits<std::int32_t>::max)());
    }
    const std::int64_t non_negative = v13 & ~(v13 >> 31);
    if (non_negative < 0x40000000LL) {
        return static_cast<std::int32_t>(0x40000000LL - non_negative);
    }
    return 0;
}

std::int32_t ace_temporal_hole_fill_calc_offset_index_from_norms_q30(
    const std::int32_t current_norm_q22,
    const std::int32_t previous_norm_q22,
    const std::int32_t history_norm_q22) noexcept {
    const std::int32_t reference = (std::min)(
        previous_norm_q22, history_norm_q22);
    if (reference < 0 || current_norm_q22 < 0) {
        return 0;
    }
    // homatic CalcBinNoiseLevels: offset stays 0 unless
    // current <= min(previous, history) + 1.
    const std::int32_t denominator = reference ==
        (std::numeric_limits<std::int32_t>::max)()
        ? reference
        : static_cast<std::int32_t>(reference + 1);
    if (current_norm_q22 > denominator) {
        return 0;
    }
    const std::int32_t ratio_q2 = ace_fixed_divide_native(
        2, current_norm_q22, 22, denominator, 22);
    return ace_temporal_hole_fill_calc_offset_index_q30(ratio_q2);
}

void ace_temporal_hole_fill_apply_bit_alloc_threshold_q30(
    const std::int32_t* norms_q30,
    std::int32_t* thresholds_q30,
    const std::uint32_t first_band,
    const std::uint32_t last_band,
    const std::uint32_t band_stride,
    const std::uint32_t band_index,
    const std::int32_t scaled_allocation_q4) noexcept {
    if (norms_q30 == nullptr || thresholds_q30 == nullptr
        || first_band > last_band || band_stride == 0U
        || band_index >= band_stride) {
        return;
    }
    const std::uint32_t index =
        ace_temporal_hole_fill_bit_alloc_index_q30(scaled_allocation_q4);
    const std::int64_t threshold =
        ace_temporal_hole_fill_bit_alloc_threshold_q30(index);
    for (std::uint32_t band = first_band; band <= last_band; ++band) {
        const std::int64_t product = static_cast<std::int64_t>(
            norms_q30[static_cast<std::size_t>(band) * band_stride
                      + band_index]) * threshold;
        const std::int64_t rounded = (product + 0x20000000LL) >> 30;
        thresholds_q30[band] = static_cast<std::int32_t>(
            (std::max)(static_cast<std::int64_t>(
                           (std::numeric_limits<std::int32_t>::min)()),
                (std::min)(static_cast<std::int64_t>(
                               (std::numeric_limits<std::int32_t>::max)()),
                    rounded)));
    }
}

AceStreamParseResult parse_ace_stream_prefix(
    const std::uint8_t* bytes,
    std::size_t size,
    bool stereo,
    bool predictive,
    std::uint32_t sample_rate,
    std::uint32_t bandwidth_mode,
    const AceStreamPrefixState* previous,
    AceStreamPrefixState& state,
    AceStreamPrefix& prefix) noexcept {
    prefix = {};
    if (bytes == nullptr || size == 0U || bandwidth_mode > 3U
        || (sample_rate != 44100U && sample_rate != 48000U)
        || size > std::numeric_limits<std::uint32_t>::max()) {
        return AceStreamParseResult::Invalid;
    }
    static constexpr std::uint32_t kEffectiveBands[4] = {
        17U, 19U, 21U, 22U};
    AceBitReader source(bytes, size);
    prefix.stereo = stereo;
    prefix.predictive = predictive;
    prefix.bandwidth_mode = bandwidth_mode;
    prefix.effective_bands = kEffectiveBands[bandwidth_mode];
    if (previous != nullptr && previous->initialized) {
        prefix.previous_lognorm_q10 = previous->primary_lognorm_q10;
    }
    std::uint32_t value = 0U;
    if (!read_ace_unary(source, 4U, value) || value > 3U) {
        return AceStreamParseResult::Invalid;
    }
    prefix.coding_mode = static_cast<AceCodingMode>(value);
    if (!stereo || prefix.coding_mode == AceCodingMode::Full
        || prefix.coding_mode == AceCodingMode::Off) {
        prefix.first_effective_channel = 0U;
        prefix.effective_channel_count = stereo ? 2U : 1U;
    } else {
        prefix.first_effective_channel =
            prefix.coding_mode == AceCodingMode::Left ? 0U : 1U;
        prefix.effective_channel_count = 1U;
    }
    if (!source.read(1U, value)) {
        return AceStreamParseResult::Invalid;
    }
    prefix.lts_enabled = value != 0U;
    if (prefix.lts_enabled) {
        const bool differential = predictive && previous != nullptr
            && previous->initialized && previous->lts_lag != 0U;
        if (differential) {
            std::uint32_t encoded = 0U;
            if (!read_ace_golomb_limited(
                    source, 1020U, 4U, 127U, encoded)) {
                return AceStreamParseResult::Invalid;
            }
            prefix.lts_lag = modular(
                static_cast<std::int64_t>(previous->lts_lag)
                    + negative_rice_map(encoded),
                3U,
                1022U);
            prefix.lts_lag_bits_consumed = source.position();
            if (!source.read(1U, value)) {
                return AceStreamParseResult::Invalid;
            }
            prefix.lts_filter_reused = value != 0U;
            if (value == 0U) {
                if (!read_ace_uniform(source, 29U, prefix.lts_filter_index)) {
                    return AceStreamParseResult::Invalid;
                }
            } else {
                prefix.lts_filter_index = previous->lts_filter_index;
            }
        } else {
            if (!read_ace_uniform(source, 1020U, prefix.lts_lag)
                || !read_ace_uniform(
                    source, 29U, prefix.lts_filter_index)) {
                return AceStreamParseResult::Invalid;
            }
            prefix.lts_lag += 3U;
        }
    }
    prefix.lts_bits_consumed = source.position();
    if (prefix.coding_mode != AceCodingMode::Off) {
        const std::uint64_t threshold = 1360ULL
            * prefix.effective_channel_count;
        if (size * 8ULL >= threshold) {
            if (!source.read(1U, value)) {
                return AceStreamParseResult::Invalid;
            }
            prefix.high_resolution_vq = value != 0U;
        }
        if (!source.read(1U, value)) {
            return AceStreamParseResult::Invalid;
        }
        prefix.short_transform = value != 0U;
        if (!source.read(1U, value)) {
            return AceStreamParseResult::Invalid;
        }
        prefix.spectral_hole_fill = value != 0U;
        if (prefix.short_transform) {
            if (!source.read(1U, value)) {
                return AceStreamParseResult::Invalid;
            }
            prefix.temporal_hole_fill = value != 0U;
        }
        prefix.parsed_stage = 1U;
        prefix.stream_flags_bits_consumed = source.position();
        if (!read_primary_lognorms(source, prefix, previous)) {
            prefix.bits_consumed = source.position();
            return AceStreamParseResult::Invalid;
        }
        prefix.primary_lognorm_bits_consumed = source.position();
        prefix.parsed_stage = 2U;
        if (!read_band_block_counts(source, sample_rate, prefix)) {
            prefix.bits_consumed = source.position();
            return AceStreamParseResult::Invalid;
        }
        prefix.band_block_bits_consumed = source.position();
        prefix.parsed_stage = 3U;
        if (!read_ace_uniform(source, 4U, prefix.conditioning_level)) {
            prefix.bits_consumed = source.position();
            return AceStreamParseResult::Invalid;
        }
        prefix.conditioning_bits_consumed = source.position();
        prefix.parsed_stage = 4U;
        prefix.bits_consumed = source.position();
        if (!read_allocation_header(source, previous, prefix)) {
            prefix.allocation_header_bits_consumed = source.position();
            return AceStreamParseResult::Invalid;
        }
        prefix.parsed_stage = 5U;
        prefix.allocation_header_bits_consumed = source.position();
        if (!read_allocation_delta(source, sample_rate, prefix)) {
            prefix.allocation_bits_consumed = source.position();
            return AceStreamParseResult::Invalid;
        }
        prefix.parsed_stage = 6U;
        prefix.allocation_bits_consumed = source.position();
        prefix.allocation_available_bits = static_cast<std::uint32_t>(
            size * 8U - source.position());
        if (!derive_allocation(
                source,
                sample_rate,
                static_cast<std::uint32_t>(size * 8U),
                previous,
                prefix)) {
            return AceStreamParseResult::Invalid;
        }
        prefix.parsed_stage = 7U;
        prefix.allocation_bits_consumed = source.position();
        if (!read_lognorm_refinement(source, prefix)) {
            prefix.refinement_bits_consumed = source.position();
            return AceStreamParseResult::Invalid;
        }
        prefix.parsed_stage = 8U;
    } else if (source.bits_left() >= 8U) {
        return AceStreamParseResult::Invalid;
    }
    if (prefix.bits_consumed == 0U) {
        prefix.bits_consumed = source.position();
    }
    prefix.spectral_payload_bit_offset = source.position();
    prefix.spectral_payload_bits_remaining = source.bits_left();
    state.initialized = true;
    state.lts_lag = prefix.lts_lag;
    state.lts_filter_index = prefix.lts_filter_index;
    if (prefix.coding_mode != AceCodingMode::Off) {
        state.primary_lognorm_q10 = prefix.refined_lognorm_q10;
        state.num_coded_bands = prefix.num_coded_bands;
        state.num_ms_mono_bands = prefix.num_ms_mono_bands;
        state.allocation_model_parameter = prefix.allocation_model_parameter;
        state.raw_allocation = prefix.raw_allocation;
        state.guaranteed_allocation = prefix.effective_channel_count;
        if (stereo && prefix.effective_channel_count == 1U) {
            const std::uint32_t inactive = 1U - prefix.first_effective_channel;
            state.primary_lognorm_q10[inactive].fill(-32768);
        }
    } else {
        // DTSAceStreamDecoder_FillSilence writes the -32768 lognorm
        // sentinel for every channel on every disabled-coding frame.  Do
        // not retain the previous predictive lognorm across this boundary:
        // the next coded frame must start from the native silence state.
        state.primary_lognorm_q10[0].fill(-32768);
        state.primary_lognorm_q10[1].fill(-32768);
        if (!predictive) {
            state.num_coded_bands = prefix.effective_bands;
            state.num_ms_mono_bands = 0U;
            state.allocation_model_parameter = 0;
            state.raw_allocation.fill(prefix.effective_channel_count);
            state.guaranteed_allocation = prefix.effective_channel_count;
        }
    }
    return AceStreamParseResult::Complete;
}

std::uint32_t ace_stream_band_size(
    std::uint32_t sample_rate,
    std::uint32_t band) noexcept {
    if (band >= 22U || (sample_rate != 44100U && sample_rate != 48000U)) {
        return 0U;
    }
    return allocation_tables(sample_rate, 1U).band_size[band];
}

std::int32_t ace_log_band_size_q4(
    const std::uint32_t sample_rate,
    std::uint32_t band) noexcept {
    // DTS_ACE_LOG_BAND_SIZE_48KHZ / 44P1KHZ. GetLogBandSize clamps a2>=0x15
    // to index 21; the 22nd 48 kHz entry is 119 like band 20.
    static constexpr std::array<std::int16_t, 22> k48{{
        48, 48, 48, 48, 48, 48, 48, 48,
        64, 64, 64,
        80, 80, 80, 80,
        89, 89, 101, 105, 117, 119, 119,
    }};
    static constexpr std::array<std::int16_t, 22> k441{{
        48, 48, 48, 48, 48, 48,
        64, 64, 64, 64, 64,
        80, 80, 80,
        89, 89, 89, 101, 109, 117, 123, 105,
    }};
    if (sample_rate != 44100U && sample_rate != 48000U) {
        return -1;
    }
    if (band >= 0x15U) {
        band = 21U;
    }
    return sample_rate == 44100U ? k441[band] : k48[band];
}

std::int32_t ace_band_mean_lognorm_q10(
    std::uint32_t sample_rate,
    std::uint32_t band) noexcept {
    if (band >= 22U || (sample_rate != 44100U && sample_rate != 48000U)) {
        return 0;
    }
    return allocation_tables(sample_rate, 1U).lognorm_offset_q10[band];
}

std::int32_t ace_unnormalize_lognorm_q10(
    const std::int32_t decoded_q10,
    const std::uint32_t sample_rate,
    const std::uint32_t band) noexcept {
    if (decoded_q10 == -32768) {
        return -32768;
    }
    return decoded_q10 + ace_band_mean_lognorm_q10(sample_rate, band);
}

} // namespace dtsx
