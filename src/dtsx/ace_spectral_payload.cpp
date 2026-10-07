#include "dtsx/ace_spectral_payload.hpp"

#include "dtsx/ace_beta_table.generated.hpp"
#include "dtsx/ace_bit_reader.hpp"
#include "dtsx/ace_final_refinement.hpp"
#include "dtsx/ace_scalar_dequant.hpp"
#include "dtsx/ace_vq.hpp"
#include "dtsx/ace_stream.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <cmath>
#include <utility>

namespace dtsx {
namespace {

std::int32_t tf_reshape_mode(
    const std::uint32_t block_count,
    const bool short_transform) noexcept {
    std::uint32_t count = block_count == 0U ? 1U : block_count;
    if ((count & (count - 1U)) != 0U) {
        return 0;
    }
    std::int32_t log = 0;
    while (count > 1U) {
        count >>= 1U;
        ++log;
    }
    return log - (short_transform ? 3 : 0);
}

std::uint32_t shifted_time_slots(
    const std::uint32_t common_slots,
    const std::int32_t mode) noexcept {
    if (mode > 0 && mode < 31) {
        return common_slots << mode;
    }
    if (mode < 0 && mode > -31) {
        return common_slots >> static_cast<unsigned>(-mode);
    }
    return common_slots;
}

bool haar_forward(
    const float* const input,
    float* const output,
    const std::uint32_t start_div,
    const std::uint32_t nfreq,
    const std::uint32_t ntime,
    std::uint32_t& out_nfreq,
    std::uint32_t& out_ntime) noexcept {
    if ((ntime & 1U) != 0U) {
        return true;
    }
    const std::uint32_t half = ntime >> 1U;
    out_nfreq = nfreq * 2U;
    out_ntime = half;
    if (ntime < 2U) {
        return false;
    }
    const std::uint32_t two_time = ntime * 2U;
    if ((start_div & 1U) != 0U) {
        if (nfreq < 2U) {
            if (nfreq != 0U) {
                std::uint32_t column = 0U;
                for (std::uint32_t time = 1U; time < ntime; time += 2U) {
                    std::uint32_t counted = 0U;
                    std::uint32_t in_at = time;
                    std::uint32_t out_at = column;
                    do {
                        const float odd = input[in_at];
                        const float even = input[in_at - 1U];
                        counted += 2U;
                        output[out_at] = even - odd;
                        output[half + out_at] = even + odd;
                        out_at += two_time;
                        in_at += two_time;
                    } while (counted < nfreq);
                    ++column;
                }
            }
        } else {
            std::uint32_t low = ntime;
            std::uint32_t column = 0U;
            std::uint32_t odd_in = ntime + 1U;
            std::uint32_t high = 3U * half;
            std::uint32_t pair = 1U;
            do {
                std::int32_t dest_base = 0;
                std::uint32_t freq = 1U;
                std::uint32_t in_at = odd_in;
                do {
                    const float odd = input[in_at];
                    const float even = input[in_at - 1U];
                    freq += 2U;
                    output[static_cast<std::uint32_t>(low) +
                        static_cast<std::uint32_t>(dest_base)] = even + odd;
                    output[high + static_cast<std::uint32_t>(dest_base)] =
                        even - odd;
                    dest_base += static_cast<std::int32_t>(two_time);
                    in_at += two_time;
                } while (freq < nfreq);
                std::uint32_t counted = 0U;
                std::uint32_t in_pair = pair;
                std::uint32_t out_at = column;
                do {
                    const float odd = input[in_pair];
                    const float even = input[in_pair - 1U];
                    counted += 2U;
                    output[out_at] = even - odd;
                    output[half + out_at] = even + odd;
                    out_at += two_time;
                    in_pair += two_time;
                } while (counted < nfreq);
                pair += 2U;
                ++column;
                ++low;
                ++high;
                odd_in += 2U;
            } while (pair < ntime);
        }
        return false;
    }
    if (nfreq == 0U) {
        return false;
    }
    std::uint32_t low = ntime;
    std::uint32_t column = 0U;
    std::uint32_t odd_in = ntime + 1U;
    std::uint32_t high = 3U * half;
    std::uint32_t pair = 1U;
    do {
        std::uint32_t counted = 0U;
        std::uint32_t in_at = pair;
        std::uint32_t out_at = column;
        do {
            const float odd = input[in_at];
            const float even = input[in_at - 1U];
            counted += 2U;
            output[out_at] = even + odd;
            output[half + out_at] = even - odd;
            out_at += two_time;
            in_at += two_time;
        } while (counted < nfreq);
        if (nfreq >= 2U) {
            std::int32_t dest_base = 0;
            std::uint32_t freq = 1U;
            std::uint32_t in_odd = odd_in;
            do {
                const float odd = input[in_odd];
                const float even = input[in_odd - 1U];
                freq += 2U;
                output[static_cast<std::uint32_t>(low) +
                    static_cast<std::uint32_t>(dest_base)] = even - odd;
                output[high + static_cast<std::uint32_t>(dest_base)] =
                    even + odd;
                dest_base += static_cast<std::int32_t>(two_time);
                in_odd += two_time;
            } while (freq < nfreq);
        }
        pair += 2U;
        ++column;
        ++low;
        ++high;
        odd_in += 2U;
    } while (pair < ntime);
    return false;
}

// sony_new DTSAceBandDequant_Haar_Reverse (ARM32), float domain.
bool haar_reverse(
    const float* const input,
    float* const output,
    const std::uint32_t start_div,
    const std::uint32_t nfreq,
    const std::uint32_t ntime,
    std::uint32_t& out_nfreq,
    std::uint32_t& out_ntime) noexcept {
    if (((nfreq | start_div) & 1U) != 0U) {
        return true;
    }
    const std::uint32_t half_freq = nfreq >> 1U;
    out_nfreq = half_freq;
    out_ntime = ntime * 2U;
    const std::uint32_t doubled = ntime * 2U;
    if (doubled == 0U) {
        return false;
    }
    const std::uint32_t stride = ntime * 4U;
    const std::uint32_t passes = (doubled - 2U) >> 1U;
    if ((start_div & 2U) != 0U) {
        for (std::uint32_t pass = 0U; pass <= passes; ++pass) {
            if (nfreq >= 4U) {
                std::uint32_t freq = 1U;
                std::uint32_t offset = 0U;
                std::uint32_t out_at = ntime * 2U + 1U + pass * 2U;
                do {
                    const float high = input[ntime * 2U + ntime + pass + offset];
                    const float low = input[ntime * 2U + pass + offset];
                    freq += 2U;
                    output[out_at - 1U] = high + low;
                    output[out_at] = low - high;
                    out_at += stride;
                    offset += stride;
                } while (freq < half_freq);
            }
            if (nfreq >= 2U) {
                std::uint32_t counted = 0U;
                std::uint32_t in_at = pass;
                std::uint32_t out_at = 1U + pass * 2U;
                do {
                    const float high = input[in_at + ntime];
                    const float low = input[in_at];
                    counted += 2U;
                    output[out_at - 1U] = low + high;
                    output[out_at] = high - low;
                    in_at += stride;
                    out_at += stride;
                } while (counted < half_freq);
            }
        }
        return false;
    }
    for (std::uint32_t pass = 0U; pass <= passes; ++pass) {
        if (nfreq >= 2U) {
            std::uint32_t counted = 0U;
            std::uint32_t in_at = pass;
            std::uint32_t out_at = 1U + pass * 2U;
            do {
                const float high = input[in_at + ntime];
                const float low = input[in_at];
                counted += 2U;
                output[out_at - 1U] = high + low;
                output[out_at] = low - high;
                in_at += stride;
                out_at += stride;
            } while (counted < half_freq);
            if (nfreq >= 4U) {
                std::uint32_t freq = 1U;
                std::uint32_t offset = 0U;
                std::uint32_t second_out_at =
                    ntime * 2U + 1U + pass * 2U;
                do {
                    const float high = input[ntime * 2U + ntime + pass + offset];
                    const float low = input[ntime * 2U + pass + offset];
                    freq += 2U;
                    output[second_out_at - 1U] = low + high;
                    output[second_out_at] = high - low;
                    second_out_at += stride;
                    offset += stride;
                } while (freq < half_freq);
            }
        }
    }
    return false;
}

bool hadamard_reshape(
    std::vector<float>& dest,
    std::vector<float>& scratch,
    const std::int32_t mode,
    const std::uint32_t start_div,
    const std::uint32_t nfreq,
    const std::uint32_t ntime,
    std::uint32_t* out_nfreq = nullptr,
    std::uint32_t* out_ntime = nullptr) noexcept {
    if (dest.size() != scratch.size() || dest.empty() || mode == 0) {
        return false;
    }
    constexpr float kStage = 1.4142F;
    float scale = 1.0F;
    const float* live = scratch.data();
    std::uint32_t dim_freq = nfreq;
    std::uint32_t dim_time = ntime;
    std::uint32_t haar_div = start_div;
    const auto forward_stage = [&](const float* const in, float* const out) {
        std::uint32_t out_freq = dim_freq;
        std::uint32_t out_time = dim_time;
        if (haar_forward(
                in, out, haar_div, dim_freq, dim_time, out_freq, out_time)) {
            return;
        }
        dim_freq = out_freq;
        dim_time = out_time;
        haar_div <<= 1U;
        live = out;
        scale *= kStage;
    };
    const auto reverse_stage = [&](const float* const in, float* const out) {
        std::uint32_t out_freq = dim_freq;
        std::uint32_t out_time = dim_time;
        if (haar_reverse(
                in, out, haar_div, dim_freq, dim_time, out_freq, out_time)) {
            return;
        }
        dim_freq = out_freq;
        dim_time = out_time;
        haar_div >>= 1U;
        live = out;
        scale *= kStage;
    };
    if (mode >= 1) {
        forward_stage(scratch.data(), dest.data());
        for (std::int32_t stage = 1; stage < mode; ++stage) {
            if ((stage & 1) != 0) {
                forward_stage(dest.data(), scratch.data());
            } else {
                forward_stage(scratch.data(), dest.data());
            }
        }
    }
    if (mode <= -1) {
        reverse_stage(live, dest.data());
        const std::int32_t limit = -mode < 2 ? 2 : -mode;
        for (std::int32_t stage = 1; stage < limit; ++stage) {
            if ((stage & 1) != 0) {
                reverse_stage(dest.data(), scratch.data());
            } else {
                reverse_stage(scratch.data(), dest.data());
            }
        }
    }
    if (!(scale > 0.0F)) {
        return false;
    }
    const float gain = 1.0F / scale;
    for (std::size_t index = 0U; index < dest.size(); ++index) {
        dest[index] = live[index] * gain;
        if (!std::isfinite(dest[index])) {
            return false;
        }
    }
    if (out_nfreq != nullptr) {
        *out_nfreq = dim_freq;
    }
    if (out_ntime != nullptr) {
        *out_ntime = dim_time;
    }
    return true;
}

void inverse_tf_transpose(
    std::vector<float>& band,
    const std::uint32_t ntime,
    const std::uint32_t nfreq) noexcept {
    if (ntime == 0U || nfreq == 0U
        || band.size() != static_cast<std::size_t>(ntime) * nfreq) {
        return;
    }
    std::vector<float> source = band;
    for (std::uint32_t time = 0U; time < ntime; ++time) {
        for (std::uint32_t freq = 0U; freq < nfreq; ++freq) {
            band[static_cast<std::size_t>(time) * nfreq + freq] =
                source[static_cast<std::size_t>(freq) * ntime + time];
        }
    }
}

} // namespace

bool ace_pack_spectral_frame(
    const std::uint32_t sample_rate,
    const std::vector<std::vector<float>>& bands,
    std::vector<float>& coefficients) noexcept {
    coefficients.assign(1024U, 0.0F);
    if (bands.empty() || bands.size() > 22U
        || (sample_rate != 44100U && sample_rate != 48000U)) {
        coefficients.clear();
        return false;
    }
    std::size_t offset = 0U;
    for (std::uint32_t band = 0U;
         band < static_cast<std::uint32_t>(bands.size()); ++band) {
        const std::uint32_t expected = ace_stream_band_size(sample_rate, band);
        if (expected == 0U || bands[band].size() != expected
            || offset > coefficients.size() - expected) {
            coefficients.clear();
            return false;
        }
        std::copy(bands[band].begin(), bands[band].end(),
            coefficients.begin() + static_cast<std::ptrdiff_t>(offset));
        offset += expected;
    }
    return true;
}

bool ace_pack_spectral_frame_with_blocks(
    const std::uint32_t sample_rate,
    const std::vector<std::vector<float>>& bands,
    const std::array<std::uint32_t, 22>& block_counts,
    std::vector<float>& coefficients,
    const bool short_transform) noexcept {
    if (bands.empty() || bands.size() > block_counts.size()) {
        coefficients.clear();
        return false;
    }
    const std::uint32_t common_slots = short_transform ? 8U : 1U;
    std::vector<std::vector<float>> reshaped = bands;
    std::uint32_t band_start = 0U;
    for (std::size_t band = 0U; band < reshaped.size(); ++band) {
        const std::int32_t mode = tf_reshape_mode(
            block_counts[band], short_transform);
        if (!ace_time_frequency_reshape_native(
                reshaped[band], common_slots, mode, band_start)) {
            coefficients.clear();
            return false;
        }
        band_start += static_cast<std::uint32_t>(reshaped[band].size());
    }
    return ace_pack_spectral_frame(sample_rate, reshaped, coefficients);
}

bool ace_time_frequency_reshape(
    std::vector<float>& band,
    const std::uint32_t time_factor) noexcept {
    if (band.empty() || time_factor == 0U
        || time_factor == 1U
        || band.size() % time_factor != 0U
        || time_factor > band.size()) {
        return time_factor == 1U && !band.empty();
    }
    const std::size_t frequency = band.size() / time_factor;
    std::vector<float> source = band;
    // Native loop writes destination at r + c*time_factor while advancing
    // the source row-major block at r*frequency + c.
    for (std::uint32_t row = 0U; row < time_factor; ++row) {
        for (std::size_t column = 0U; column < frequency; ++column) {
            band[column * time_factor + row] =
                source[static_cast<std::size_t>(row) * frequency + column];
        }
    }
    return true;
}

bool ace_time_frequency_reshape_native(
    std::vector<float>& band,
    const std::uint32_t transform_size,
    const std::int32_t reshape_mode,
    const std::uint32_t band_start) noexcept {
    if (band.empty() || transform_size == 0U) {
        return false;
    }
    if (reshape_mode == 0) {
        return ace_time_frequency_reshape(band, transform_size);
    }
    const std::uint32_t time_slots = shifted_time_slots(
        transform_size, reshape_mode);
    if (time_slots == 0U) {
        return false;
    }
    std::vector<float> scratch = band;
    if (time_slots > 1U && time_slots - 1U < band.size()
        && band.size() % time_slots == 0U) {
        if (!ace_time_frequency_reshape(scratch, time_slots)) {
            return false;
        }
    }
    const std::uint32_t start_div = band_start / time_slots;
    const std::uint32_t freq_count =
        static_cast<std::uint32_t>(scratch.size()) / time_slots;
    if (freq_count == 0U) {
        return true;
    }
    return hadamard_reshape(
        band, scratch, reshape_mode, start_div, freq_count, time_slots);
}

bool ace_time_frequency_reshape_buffer(
    std::vector<float>& band,
    const std::uint32_t transform_size,
    const std::int32_t reshape_mode,
    const std::uint32_t buffer_start_div) noexcept {
    if (band.empty() || transform_size == 0U) {
        return false;
    }
    if (reshape_mode == 0) {
        if (transform_size == 1U || transform_size == band.size()) {
            return true;
        }
        if (band.size() % transform_size != 0U) {
            return false;
        }
        inverse_tf_transpose(
            band, transform_size,
            static_cast<std::uint32_t>(band.size() / transform_size));
        return true;
    }
    if (band.size() % transform_size != 0U) {
        return false;
    }
    const std::uint32_t nfreq =
        static_cast<std::uint32_t>(band.size() / transform_size);
    std::vector<float> scratch = band;
    std::uint32_t out_nfreq = nfreq;
    std::uint32_t out_ntime = transform_size;
    if (!hadamard_reshape(
            band, scratch, -reshape_mode, buffer_start_div, nfreq,
            transform_size, &out_nfreq, &out_ntime)) {
        return false;
    }
    inverse_tf_transpose(band, out_ntime, out_nfreq);
    return true;
}

void ace_shf_lr_to_mid_side(
    std::vector<float>& left,
    std::vector<float>& right,
    const std::int32_t left_lognorm_q10,
    const std::int32_t right_lognorm_q10,
    const bool collapse) noexcept {
    if (left.size() != right.size() || left.empty()) {
        return;
    }
    const std::size_t count = left.size();
    const float left_gain = std::exp2(
        static_cast<float>(left_lognorm_q10) * 0.00097656F);
    const float right_gain = std::exp2(
        static_cast<float>(right_lognorm_q10) * 0.00097656F);
    const float target = std::sqrt(static_cast<float>(count));
    std::vector<float> mid(count);
    std::vector<float> side(count);
    float mid_energy = 0.0F;
    float side_energy = 0.0F;
    for (std::size_t index = 0U; index < count; ++index) {
        const float left_scaled = left_gain * left[index];
        const float right_scaled = right_gain * right[index];
        mid[index] = left_scaled + right_scaled;
        side[index] = left_scaled - right_scaled;
        mid_energy += mid[index] * mid[index];
        side_energy += side[index] * side[index];
    }
    auto normalize = [target](std::vector<float>& vector) {
        float energy = 0.0F;
        for (const float value : vector) {
            energy += value * value;
        }
        const float inv = target / (std::sqrt(energy) + 5.421e-20F);
        for (float& value : vector) {
            value *= inv;
        }
    };
    normalize(mid);
    normalize(side);
    if (collapse) {
        if (mid_energy >= side_energy) {
            left = mid;
            right = mid;
        } else {
            left = side;
            right = side;
        }
        return;
    }
    left = std::move(mid);
    right = std::move(side);
}

void ace_shf_stereo_to_ms_mono_transition(
    std::vector<float>& left,
    std::vector<float>& right,
    const std::int32_t left_lognorm_q10,
    const std::int32_t right_lognorm_q10) noexcept {
    ace_shf_lr_to_mid_side(
        left, right, left_lognorm_q10, right_lognorm_q10, true);
}

bool ace_shf_prepare_src_slice(
    const std::vector<std::vector<float>>* previous,
    const std::vector<std::vector<float>>& this_au,
    const std::uint32_t band,
    const std::size_t count,
    std::vector<float>& out) noexcept {
    out.clear();
    if (count == 0U) {
        return false;
    }
    const auto plane_at =
        [&](const std::uint32_t index) -> const std::vector<float>* {
        if (index < band && index < this_au.size()
            && !this_au[index].empty()) {
            return &this_au[index];
        }
        if (previous != nullptr && index < previous->size()
            && !(*previous)[index].empty()) {
            return &(*previous)[index];
        }
        return nullptr;
    };
    std::size_t start = 0U;
    for (std::uint32_t index = 0U; index < band; ++index) {
        const std::vector<float>* plane = plane_at(index);
        if (plane == nullptr) {
            return false;
        }
        start += plane->size();
    }
    const std::size_t offset = start >= count ? start - count : 0U;
    const std::size_t need = offset + count;
    std::vector<float> concat;
    concat.reserve(need);
    const std::uint32_t previous_bands = previous != nullptr
        ? static_cast<std::uint32_t>(previous->size()) : 0U;
    const std::uint32_t this_bands =
        static_cast<std::uint32_t>(this_au.size());
    const std::uint32_t total = std::max(previous_bands, this_bands);
    for (std::uint32_t index = 0U;
         index < total && concat.size() < need; ++index) {
        const std::vector<float>* plane = plane_at(index);
        if (plane == nullptr) {
            break;
        }
        concat.insert(concat.end(), plane->begin(), plane->end());
    }
    if (concat.size() < need) {
        return false;
    }
    out.assign(concat.begin() + static_cast<std::ptrdiff_t>(offset),
        concat.begin() + static_cast<std::ptrdiff_t>(need));
    return true;
}

bool ace_hadamard_butterfly_reference(
    std::vector<float>& row) noexcept {
    const std::size_t length = row.size();
    if (length == 0U || (length & (length - 1U)) != 0U) {
        return false;
    }
    // DTSAceBandDequant_Hadamard uses the same unnormalised pair operation:
    // (x,y) -> (x+y,x-y).  Keep each stage in scalar order so the eventual
    // native SIMD port can be compared without changing rounding order.
    for (std::size_t span = 1U; span < length; span <<= 1U) {
        const std::size_t step = span << 1U;
        for (std::size_t base = 0U; base < length; base += step) {
            for (std::size_t offset = 0U; offset < span; ++offset) {
                const float first = row[base + offset];
                const float second = row[base + span + offset];
                row[base + offset] = first + second;
                row[base + span + offset] = first - second;
            }
        }
    }
    return true;
}

namespace {

AceBandScheduleState ace_payload_schedule_state(
    const AceStreamPrefix& prefix,
    const std::size_t size,
    const std::size_t position) noexcept {
    return AceBandScheduleState{
        static_cast<std::uint32_t>(size * 8U),
        static_cast<std::uint32_t>(position),
        prefix.num_coded_bands, 0U, 0U, 0,
        prefix.effective_channel_count,
        prefix.high_resolution_vq};
}

std::array<std::uint32_t, 22> stereo_tail_allocation(
    const AceStereoSpectralPayload& payload) noexcept {
    std::vector<AceBandScheduleEntry> schedule;
    schedule.reserve(payload.bands.size());
    for (const auto& band : payload.bands) {
        schedule.push_back(band.schedule);
    }
    return ace_band_dequant_tail_allocation(schedule);
}

std::uint32_t clz32(std::uint32_t value) noexcept {
    if (value == 0U) {
        return 32U;
    }
    std::uint32_t bits = 0U;
    if ((value & 0xFFFF0000U) == 0U) {
        bits += 16U;
        value <<= 16U;
    }
    if ((value & 0xFF000000U) == 0U) {
        bits += 8U;
        value <<= 8U;
    }
    if ((value & 0xF0000000U) == 0U) {
        bits += 4U;
        value <<= 4U;
    }
    if ((value & 0xC0000000U) == 0U) {
        bits += 2U;
        value <<= 2U;
    }
    if ((value & 0x80000000U) == 0U) {
        bits += 1U;
    }
    return bits;
}

std::uint32_t tf_buffer_start_div(
    const std::uint32_t band_start,
    const bool short_transform) noexcept {
    const std::uint32_t ntime = short_transform ? 8U : 1U;
    const std::uint32_t half = ntime >> 1U;
    const std::uint32_t v12 = half == 0U ? 0U : 32U - clz32(half);
    if (v12 >= 32U) {
        return 0U;
    }
    return band_start >> v12;
}

bool previous_band_for_fill(
    std::vector<float>& previous,
    const bool short_transform,
    const std::uint32_t block_count,
    const std::uint32_t band_start) noexcept {
    const std::int32_t mode = tf_reshape_mode(block_count, short_transform);
    const std::uint32_t ntime = short_transform ? 8U : 1U;
    return ace_time_frequency_reshape_buffer(
        previous, ntime, mode,
        tf_buffer_start_div(band_start, short_transform));
}

bool store_processed_band(
    std::vector<float>& band,
    const bool short_transform,
    const std::uint32_t block_count,
    const std::uint32_t band_start) noexcept {
    const std::int32_t mode = tf_reshape_mode(block_count, short_transform);
    const std::uint32_t ntime = short_transform ? 8U : 1U;
    return ace_time_frequency_reshape_native(
        band, ntime, mode, band_start);
}

void flip_sequence(float* data, const std::uint32_t count) noexcept {
    if (data == nullptr || count < 2U) {
        return;
    }
    std::vector<float> scratch(data, data + count);
    for (std::uint32_t index = 0U; index < count; ++index) {
        data[index] = scratch[count - 1U - index];
    }
}

void interleave_sequence(
    float* data,
    const std::uint32_t rows,
    const std::uint32_t columns) noexcept {
    if (data == nullptr || rows == 0U || columns == 0U) {
        return;
    }
    std::vector<float> scratch(static_cast<std::size_t>(rows) * columns);
    for (std::uint32_t row = 0U; row < rows; ++row) {
        for (std::uint32_t column = 0U; column < columns; ++column) {
            scratch[column * rows + row] = data[row * columns + column];
        }
    }
    std::copy(scratch.begin(), scratch.end(), data);
}

void mix4(float* const x, const float matrix[4][4]) noexcept {
    float y[4]{};
    for (std::uint32_t row = 0U; row < 4U; ++row) {
        y[row] = matrix[row][0] * x[0]
            + matrix[row][1] * x[1]
            + matrix[row][2] * x[2]
            + matrix[row][3] * x[3];
    }
    x[0] = y[0];
    x[1] = y[1];
    x[2] = y[2];
    x[3] = y[3];
}

void williamson_hadamard_4(float matrix[4][4], const float theta) noexcept {
    constexpr float kPi4 = 0.7854F;
    const float c = std::cos(theta * kPi4);
    const float s = std::sin(theta * kPi4);
    const float sn = std::sin(theta * -kPi4);
    const float s2 = std::sin((theta + theta) * kPi4);
    matrix[0][0] = c;
    matrix[0][1] = -s;
    matrix[0][2] = -sn;
    matrix[0][3] = -s2;
    matrix[1][0] = s;
    matrix[1][1] = c;
    matrix[1][2] = s2;
    matrix[1][3] = -sn;
    matrix[2][0] = sn;
    matrix[2][1] = -s2;
    matrix[2][2] = c;
    matrix[2][3] = s;
    matrix[3][0] = s2;
    matrix[3][1] = sn;
    matrix[3][2] = -s;
    matrix[3][3] = c;
    const float inv = 1.0F / std::sqrt(c * c + s * s + sn * sn + s2 * s2);
    for (std::uint32_t row = 0U; row < 4U; ++row) {
        for (std::uint32_t column = 0U; column < 4U; ++column) {
            matrix[row][column] *= inv;
        }
    }
}

void apply_in_band_smoothing_vector(
    std::vector<float>& spectrum,
    const AceVqPayloadResult& vq,
    const std::uint32_t groups,
    const std::uint32_t conditioning) noexcept {
    for (const AceVqLeaf& leaf : vq.leaves) {
        if (leaf.component_offset >= spectrum.size()
            || leaf.components > spectrum.size() - leaf.component_offset) {
            continue;
        }
        ace_band_dequant_in_band_energy_smoothing(
            spectrum.data() + leaf.component_offset,
            leaf.components,
            groups,
            leaf.pulses,
            conditioning);
    }
}

void apply_hole_fill_vector(
    std::vector<float>& spectrum,
    const AceVqPayloadResult& vq,
    const std::vector<float>* previous,
    const bool fill_enabled,
    std::uint32_t& seed) noexcept {
    const float* previous_data = previous != nullptr
        && previous->size() == spectrum.size()
        ? previous->data() : nullptr;
    for (const AceVqLeaf& leaf : vq.leaves) {
        if (leaf.component_offset >= spectrum.size()
            || leaf.components > spectrum.size() - leaf.component_offset) {
            continue;
        }
        ace_spectral_hole_fill_partition(
            spectrum.data() + leaf.component_offset,
            previous_data != nullptr
                ? previous_data + leaf.component_offset : nullptr,
            leaf.components,
            leaf.pulses,
            fill_enabled,
            leaf.gain,
            seed);
    }
}

} // namespace

void ace_spectral_hole_fill_partition(
    float* const dest,
    const float* const previous,
    const std::uint32_t count,
    const std::uint32_t pulses,
    const bool fill_enabled,
    const float scale,
    std::uint32_t& seed) noexcept {
    if (dest == nullptr || count == 0U) {
        return;
    }
    if (pulses != 0U) {
        return;
    }
    if (!fill_enabled) {
        std::fill(dest, dest + count, 0.0F);
        return;
    }
    bool copy_previous = false;
    if (previous != nullptr) {
        for (std::uint32_t index = 0U; index < count; ++index) {
            if (previous[index] != 0.0F) {
                copy_previous = true;
                break;
            }
        }
    }
    if (copy_previous) {
        std::copy(previous, previous + count, dest);
    } else {
        for (std::uint32_t index = 0U; index < count; ++index) {
            dest[index] = ace_prng_rand_in_range(seed, -1.0F, 1.0F);
        }
    }
    float energy = 0.0F;
    for (std::uint32_t index = 0U; index < count; ++index) {
        energy += dest[index] * dest[index];
    }
    const float inv = scale / (std::sqrt(energy) + 5.421e-20F);
    for (std::uint32_t index = 0U; index < count; ++index) {
        dest[index] *= inv;
    }
}

std::uint32_t ace_band_dequant_in_band_smoothing_groups(
    const bool short_transform,
    const std::int32_t reshape_mode) noexcept {
    const std::uint32_t ntime = short_transform ? 8U : 1U;
    const std::uint32_t half = ntime >> 1U;
    const std::uint32_t v12 = half == 0U ? 0U : 32U - clz32(half);
    const int shift = static_cast<int>(v12) + reshape_mode;
    if (shift <= 0) {
        return 1U;
    }
    if (shift >= 32) {
        return 0U;
    }
    return 1U << shift;
}

void ace_band_dequant_in_band_energy_smoothing(
    float* const dest,
    const std::uint32_t count,
    const std::uint32_t groups,
    const std::uint32_t pulses,
    const std::uint32_t conditioning) noexcept {
    if (dest == nullptr
        || pulses == 0U
        || groups == 0U
        || 2U * pulses >= count
        || conditioning < 1U
        || conditioning > 3U) {
        return;
    }
    static constexpr float kConditioningScale[3]{0.36F, 0.47F, 0.75F};
    const float ratio = static_cast<float>(pulses) / static_cast<float>(count);
    const float shaped = std::pow(ratio, 0.73F);
    const float theta = std::exp(
        (shaped * -3.0F) / kConditioningScale[conditioning - 1U]);
    float matrix[4][4]{
        {1.0F, 0.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, 0.0F, 1.0F},
    };
    if (theta > 0.0F) {
        williamson_hadamard_4(matrix, theta);
    }
    const std::uint32_t width = count / groups;
    const std::uint32_t quartets = width >> 2U;
    if (quartets == 0U) {
        return;
    }
    const bool odd_width = (width & 3U) != 0U;
    for (std::uint32_t group = 0U; group < groups; ++group) {
        float* const row = dest + group * width;
        for (std::uint32_t quartet = 0U; quartet < quartets; ++quartet) {
            mix4(row + 4U * quartet, matrix);
        }
        if (odd_width) {
            flip_sequence(row, width);
            for (std::uint32_t quartet = 0U; quartet < quartets; ++quartet) {
                mix4(row + 4U * quartet, matrix);
            }
        }
        interleave_sequence(row, quartets, 4U);
    }
}

bool consume_ace_mono_spectral_payload(
    const std::uint8_t* bytes,
    std::size_t size,
    std::uint32_t sample_rate,
    const AceStreamPrefix& prefix,
    AceSpectralPayload& payload,
    std::uint32_t* spectral_hole_fill_seed,
    const std::vector<std::vector<float>>* previous_hole_fill) noexcept {
    payload = {};
    if (bytes == nullptr || size == 0U || prefix.stereo
        || prefix.coding_mode == AceCodingMode::Off
        || prefix.num_coded_bands == 0U
        || prefix.num_coded_bands > prefix.effective_bands
        || prefix.effective_bands > prefix.allocation.size()
        || prefix.spectral_payload_bit_offset > size * 8U) {
        return false;
    }
    payload.short_transform = prefix.short_transform;
    payload.band_block_count = prefix.band_block_count;
    AceBitReader source(bytes, size);
    if (!source.seek(prefix.spectral_payload_bit_offset)) {
        return false;
    }
    AceBandScheduleState schedule_state = ace_payload_schedule_state(
        prefix, size, source.position());
    payload.bands.reserve(prefix.num_coded_bands);
    payload.vq_bands.reserve(prefix.num_coded_bands);
    payload.spectra.reserve(prefix.num_coded_bands);
    const std::size_t begin = source.position();

    for (std::uint32_t band = 0U; band < prefix.num_coded_bands; ++band) {
        const std::uint32_t components = ace_stream_band_size(sample_rate, band);
        const std::int32_t log_n = ace_log_band_size_q4(sample_rate, band);
        if (components == 0U || log_n < 0 || prefix.band_block_count[band] == 0U) {
            payload.failed_band = band;
            payload.failed_bit_offset = source.position();
            return false;
        }
        // Calculate the exact native allocation for this band without moving
        // state; its consumed amount is known only after the recursive VQ
        // codeword has been read.
        const std::uint32_t remaining_bands = schedule_state.coded_bands
            - schedule_state.current_band;
        if (schedule_state.used_bits > schedule_state.total_bits
            || schedule_state.deferred_reserve_bits
                > schedule_state.total_bits - schedule_state.used_bits) {
            payload.failed_band = band;
            payload.failed_bit_offset = source.position();
            return false;
        }
        const std::uint32_t remaining = schedule_state.total_bits
            - schedule_state.used_bits - schedule_state.deferred_reserve_bits;
        const std::int64_t desired = static_cast<std::int64_t>(
            prefix.allocation[band]) + ace_distribute_running_balance(
                schedule_state.running_balance, remaining_bands);
        const std::uint32_t allocation = desired <= 0 ? 0U
            : static_cast<std::uint32_t>(
                desired > static_cast<std::int64_t>(remaining)
                    ? remaining : desired);

        AceVqPayloadResult result{};
        const std::size_t before = source.position();
        if (!ace_vq_consume_split_vector(
                source, allocation, remaining, components,
                // Native ProcessOneBand is invoked with a8=1; the parsed
                // band-block count controls the surrounding scheduler, not
                // the VQ leaf recursion depth for this call.
                1U, log_n, 0U,
                prefix.high_resolution_vq, result)) {
            payload.failed_band = band;
            payload.failed_bit_offset = source.position();
            return false;
        }
        const std::size_t consumed_size = source.position() - before;
        if (consumed_size > std::numeric_limits<std::uint32_t>::max()
            || result.bits_consumed != consumed_size) {
            payload.failed_band = band;
            payload.failed_bit_offset = source.position();
            return false;
        }
        AceBandScheduleEntry entry{};
        if (!ace_schedule_next_band(
                schedule_state, prefix.allocation[band],
                static_cast<std::uint32_t>(consumed_size), entry)
            || entry.allocation != allocation) {
            payload.failed_band = band;
            payload.failed_bit_offset = source.position();
            return false;
        }
        payload.bands.push_back(entry);
        std::vector<float> spectrum;
        if (!ace_vq_assemble_vector(result, components, spectrum)) {
            payload.failed_band = band;
            payload.failed_bit_offset = source.position();
            return false;
        }
        payload.vq_bands.push_back(std::move(result));
        payload.spectra.push_back(std::move(spectrum));
    }
    for (std::size_t band = 0U; band < payload.spectra.size(); ++band) {
        apply_in_band_smoothing_vector(
            payload.spectra[band],
            payload.vq_bands[band],
            ace_band_dequant_in_band_smoothing_groups(
                prefix.short_transform,
                tf_reshape_mode(
                    prefix.band_block_count[band], prefix.short_transform)),
            prefix.conditioning_level);
    }
    std::uint32_t fill_band_start = 0U;
    if (prefix.spectral_hole_fill && spectral_hole_fill_seed != nullptr) {
        payload.hole_fill_state.assign(
            payload.spectra.size(), std::vector<float>{});
        for (std::size_t band = 0U; band < payload.spectra.size(); ++band) {
            const std::vector<float>* previous = nullptr;
            std::vector<float> previous_vq;
            if (ace_shf_prepare_src_slice(
                    previous_hole_fill, payload.hole_fill_state,
                    static_cast<std::uint32_t>(band),
                    payload.spectra[band].size(), previous_vq)
                && previous_band_for_fill(
                    previous_vq, prefix.short_transform,
                    prefix.band_block_count[band], fill_band_start)) {
                previous = &previous_vq;
            }
            apply_hole_fill_vector(
                payload.spectra[band], payload.vq_bands[band], previous,
                true, *spectral_hole_fill_seed);
            payload.hole_fill_state[band] = payload.spectra[band];
            if (!store_processed_band(
                    payload.hole_fill_state[band], prefix.short_transform,
                    prefix.band_block_count[band], fill_band_start)) {
                return false;
            }
            fill_band_start += static_cast<std::uint32_t>(
                payload.spectra[band].size());
        }
    } else {
        payload.hole_fill_state = payload.spectra;
        fill_band_start = 0U;
        for (std::size_t band = 0U; band < payload.hole_fill_state.size();
             ++band) {
            if (!store_processed_band(
                    payload.hole_fill_state[band], prefix.short_transform,
                    prefix.band_block_count[band], fill_band_start)) {
                return false;
            }
            fill_band_start += static_cast<std::uint32_t>(
                payload.hole_fill_state[band].size());
        }
    }
    payload.bits_consumed = source.position() - begin;
    payload.final_bit_offset = source.position();
    AceFinalRefinement final_refinement;
    // Native UnpackFinalRefinement reads BandDequant a4 (v19+176 / v10[44]),
    // the 0/1 leftover flag from DTSAceBandDequant_Process, not VQ allocation.
    const std::array<std::uint32_t, 22> initial_final =
        ace_band_dequant_tail_allocation(payload.bands);
    if (!unpack_ace_final_refinement(
            bytes, size, prefix, payload.final_bit_offset,
            initial_final, final_refinement)) {
        payload.failed_band = prefix.num_coded_bands;
        payload.failed_bit_offset = source.position();
        return false;
    }
    AceScalarMatrix final_lognorm = prefix.refined_lognorm_q10;
    const AceScalarDequantControl control{
        prefix.predictive,
        prefix.first_effective_channel,
        prefix.effective_channel_count,
        prefix.effective_bands,
    };
    if (!ace_scalar_dequant_finalize(
            final_lognorm, final_refinement.codes,
            prefix.refinement_allocation, final_refinement.allocation,
            control)) {
        return false;
    }
    payload.finalized_lognorm_q10 = final_lognorm;
    for (std::uint32_t band = 0U;
         band < prefix.num_coded_bands; ++band) {
        if (!ace_apply_lognorm_gain(
                payload.spectra[band],
                ace_unnormalize_lognorm_q10(
                    final_lognorm[prefix.first_effective_channel][band],
                    sample_rate, band))) {
            return false;
        }
    }
    payload.bits_consumed += final_refinement.bits_consumed;
    payload.final_bit_offset = final_refinement.final_bit_offset;
    return true;
}

bool consume_ace_stereo_spectral_payload(
    const std::uint8_t* bytes,
    const std::size_t size,
    const std::uint32_t sample_rate,
    const AceStreamPrefix& prefix,
    AceStereoSpectralPayload& payload,
    std::uint32_t* spectral_hole_fill_seed,
    const std::vector<std::vector<float>>* previous_hole_fill_left,
    const std::vector<std::vector<float>>* previous_hole_fill_right,
    [[maybe_unused]] const std::vector<std::uint32_t>* previous_stereo_coding,
    const std::vector<std::vector<float>>* previous_work_left,
    const std::vector<std::vector<float>>* previous_work_right) noexcept {
    payload = {};
    if (bytes == nullptr || size == 0U || !prefix.stereo
        || prefix.coding_mode == AceCodingMode::Off
        || prefix.num_coded_bands == 0U
        || prefix.num_coded_bands > prefix.effective_bands
        || prefix.effective_bands > prefix.allocation.size()
        || prefix.spectral_payload_bit_offset > size * 8U) {
        return false;
    }
    payload.short_transform = prefix.short_transform;
    payload.band_block_count = prefix.band_block_count;
    AceBitReader source(bytes, size);
    if (!source.seek(prefix.spectral_payload_bit_offset)) {
        return false;
    }
    AceBandScheduleState schedule_state = ace_payload_schedule_state(
        prefix, size, source.position());
    payload.bands.reserve(prefix.num_coded_bands);
    const std::size_t begin = source.position();
    std::int32_t persistent_beta_q15 = 0;
    std::uint32_t fill_band_start = 0U;

    for (std::uint32_t band = 0U; band < prefix.num_coded_bands; ++band) {
        payload.failed_band = band;
        payload.failed_bit_offset = source.position();
        const std::uint32_t components = ace_stream_band_size(sample_rate, band);
        const std::int32_t log_n = ace_log_band_size_q4(sample_rate, band);
        if (components == 0U || log_n < 0 || prefix.band_block_count[band] == 0U
            || schedule_state.used_bits > schedule_state.total_bits) {
            return false;
        }
        const std::uint32_t remaining_bands = schedule_state.coded_bands
            - schedule_state.current_band;
        if (schedule_state.deferred_reserve_bits
                > schedule_state.total_bits - schedule_state.used_bits) {
            return false;
        }
        const std::uint32_t physical_remaining = schedule_state.total_bits
            - schedule_state.used_bits - schedule_state.deferred_reserve_bits;
        const std::int64_t desired = static_cast<std::int64_t>(
            prefix.allocation[band]) + ace_distribute_running_balance(
                schedule_state.running_balance, remaining_bands);
        const std::uint32_t allocation = desired <= 0 ? 0U
            : static_cast<std::uint32_t>((std::min)(
                desired, static_cast<std::int64_t>(physical_remaining)));
        AceStereoSpectralBand result{};
        result.coding = prefix.stereo_coding[band];
        const std::size_t band_begin = source.position();

        if (result.coding == 2U) {
            std::uint32_t left_bits = 0U;
            std::uint32_t right_bits = 0U;
            if (!ace_lr_stereo_bit_allocations(
                    static_cast<std::uint32_t>(prefix.coding_mode),
                    band, allocation,
                    prefix.refined_lognorm_q10[0][band],
                    prefix.refined_lognorm_q10[1][band],
                    prefix.lr_allocation_mode, left_bits, right_bits)
                || !ace_vq_consume_split_vector(
                    source, left_bits, physical_remaining, components,
                    1U, log_n, 0U,
                    prefix.high_resolution_vq, result.left_or_mid)) {
                return false;
            }
            if (result.left_or_mid.bits_consumed > physical_remaining) {
                return false;
            }
            const std::uint32_t remaining_after_left = physical_remaining
                - result.left_or_mid.bits_consumed;
            if (!ace_vq_consume_split_vector(
                    source, right_bits, remaining_after_left, components,
                    1U, log_n, 0U,
                    prefix.high_resolution_vq, result.right_or_side)) {
                return false;
            }
        } else if (result.coding == 1U || result.coding == 3U) {
            std::uint32_t ratio_bits = 0U;
            if (!ace_decode_stereo_mid_side_params(
                    source, components, log_n, result.coding == 1U,
                    allocation, persistent_beta_q15, result.mid_side_ratio,
                    ratio_bits)) {
                return false;
            }
            const std::uint32_t payload_bits = allocation - ratio_bits;
            std::int64_t left = (static_cast<std::int64_t>(payload_bits)
                - result.mid_side_ratio.mid_side_angle_q15 + 1LL) >> 1U;
            left = (std::max)(left, std::int64_t{0});
            left = (std::min)(left, static_cast<std::int64_t>(payload_bits));
            std::uint32_t left_bits = static_cast<std::uint32_t>(left);
            std::uint32_t right_bits = payload_bits - left_bits;
            const bool left_first = left_bits >= right_bits;
            result.left_partition_first = left_first;
            const std::uint32_t first_bits = left_first ? left_bits : right_bits;
            AceVqPayloadResult& first = left_first
                ? result.left_or_mid : result.right_or_side;
            if (ratio_bits > physical_remaining) {
                return false;
            }
            const std::uint32_t remaining_after_ratio = physical_remaining
                - ratio_bits;
            if (!ace_vq_consume_split_vector(
                    source, first_bits, remaining_after_ratio, components,
                    1U, log_n, 0U,
                    prefix.high_resolution_vq, first)) {
                return false;
            }
            const std::uint32_t first_unused = first_bits > first.bits_consumed
                ? first_bits - first.bits_consumed : 0U;
            if (first_unused > 3U
                && result.mid_side_ratio.beta_q15 != -32767
                && result.mid_side_ratio.beta_q15 != 32767) {
                if (left_first) {
                    right_bits += first_unused - 3U;
                } else {
                    left_bits += first_unused - 3U;
                }
            }
            AceVqPayloadResult& second = left_first
                ? result.right_or_side : result.left_or_mid;
            const std::uint32_t second_bits = left_first ? right_bits : left_bits;
            if (first.bits_consumed > remaining_after_ratio) {
                return false;
            }
            const std::uint32_t remaining_after_first = remaining_after_ratio
                - first.bits_consumed;
            if (!ace_vq_consume_split_vector(
                    source, second_bits, remaining_after_first, components,
                    1U, log_n, 0U,
                    prefix.high_resolution_vq, second)) {
                return false;
            }
        } else {
            return false;
        }
        if (!ace_vq_assemble_vector(
                result.left_or_mid, components, result.left_spectrum)
            || !ace_vq_assemble_vector(
                result.right_or_side, components, result.right_spectrum)) {
            return false;
        }
        apply_in_band_smoothing_vector(
            result.left_spectrum,
            result.left_or_mid,
            ace_band_dequant_in_band_smoothing_groups(
                prefix.short_transform,
                tf_reshape_mode(
                    prefix.band_block_count[band], prefix.short_transform)),
            prefix.conditioning_level);
        apply_in_band_smoothing_vector(
            result.right_spectrum,
            result.right_or_side,
            ace_band_dequant_in_band_smoothing_groups(
                prefix.short_transform,
                tf_reshape_mode(
                    prefix.band_block_count[band], prefix.short_transform)),
            prefix.conditioning_level);
        if (prefix.spectral_hole_fill && spectral_hole_fill_seed != nullptr) {
            const bool fill_enabled = result.coding != 3U;
            std::vector<std::vector<float>> this_history_left;
            std::vector<std::vector<float>> this_history_right;
            std::vector<std::vector<float>> this_work_left;
            std::vector<std::vector<float>> this_work_right;
            this_history_left.reserve(payload.bands.size());
            this_history_right.reserve(payload.bands.size());
            this_work_left.reserve(payload.bands.size());
            this_work_right.reserve(payload.bands.size());
            for (const auto& previous_band : payload.bands) {
                this_history_left.push_back(previous_band.hole_fill_left);
                this_history_right.push_back(previous_band.hole_fill_right);
                this_work_left.push_back(previous_band.hole_fill_work_left);
                this_work_right.push_back(previous_band.hole_fill_work_right);
            }
            const std::uint32_t previous_band_coding =
                band > 0U && band - 1U < prefix.stereo_coding.size()
                ? prefix.stereo_coding[band - 1U] : 0U;
            const bool handle_stereo = result.coding == 1U
                && (previous_band_coding == 2U || previous_band_coding == 3U);
            const bool use_working = !handle_stereo
                && (result.coding == 1U || result.coding == 3U);
            std::vector<float> prev_left_vq;
            std::vector<float> prev_right_vq;
            const std::vector<float>* prev_left = nullptr;
            const std::vector<float>* prev_right = nullptr;
            const auto* source_left = use_working
                ? previous_work_left : previous_hole_fill_left;
            const auto* source_right = use_working
                ? previous_work_right : previous_hole_fill_right;
            const auto& this_left = use_working
                ? this_work_left : this_history_left;
            const auto& this_right = use_working
                ? this_work_right : this_history_right;
            if (ace_shf_prepare_src_slice(
                    source_left, this_left, band, components, prev_left_vq)
                && ace_shf_prepare_src_slice(
                    source_right, this_right, band, components,
                    prev_right_vq)) {
                if (handle_stereo && band > 0U
                    && band - 1U < prefix.refined_lognorm_q10[0].size()
                    && band - 1U < prefix.refined_lognorm_q10[1].size()) {
                    ace_shf_stereo_to_ms_mono_transition(
                        prev_left_vq, prev_right_vq,
                        prefix.refined_lognorm_q10[0][band - 1U],
                        prefix.refined_lognorm_q10[1][band - 1U]);
                }
                if (previous_band_for_fill(
                        prev_left_vq, prefix.short_transform,
                        prefix.band_block_count[band], fill_band_start)) {
                    prev_left = &prev_left_vq;
                }
                if (previous_band_for_fill(
                        prev_right_vq, prefix.short_transform,
                        prefix.band_block_count[band], fill_band_start)) {
                    prev_right = &prev_right_vq;
                }
            }
            const bool left_first = result.coding == 2U
                || result.left_partition_first;
            if (left_first) {
                apply_hole_fill_vector(
                    result.left_spectrum, result.left_or_mid, prev_left,
                    fill_enabled, *spectral_hole_fill_seed);
                apply_hole_fill_vector(
                    result.right_spectrum, result.right_or_side, prev_right,
                    fill_enabled, *spectral_hole_fill_seed);
            } else {
                apply_hole_fill_vector(
                    result.right_spectrum, result.right_or_side, prev_right,
                    fill_enabled, *spectral_hole_fill_seed);
                apply_hole_fill_vector(
                    result.left_spectrum, result.left_or_mid, prev_left,
                    fill_enabled, *spectral_hole_fill_seed);
            }
        }
        if ((result.coding == 1U || result.coding == 3U)
            && !ace_stereo_mid_side_revert(
                result.left_spectrum, result.right_spectrum,
                result.mid_side_ratio.left_norm,
                result.mid_side_ratio.right_norm)) {
            return false;
        }
        result.hole_fill_left = result.left_spectrum;
        result.hole_fill_right = result.right_spectrum;
        if (!store_processed_band(
                result.hole_fill_left, prefix.short_transform,
                prefix.band_block_count[band], fill_band_start)
            || !store_processed_band(
                result.hole_fill_right, prefix.short_transform,
                prefix.band_block_count[band], fill_band_start)) {
            return false;
        }
        result.hole_fill_work_left = result.hole_fill_left;
        result.hole_fill_work_right = result.hole_fill_right;
        if (result.coding == 1U
            && band < prefix.refined_lognorm_q10[0].size()
            && band < prefix.refined_lognorm_q10[1].size()) {
            ace_shf_lr_to_mid_side(
                result.hole_fill_work_left, result.hole_fill_work_right,
                prefix.refined_lognorm_q10[0][band],
                prefix.refined_lognorm_q10[1][band], true);
        } else {
            ace_shf_lr_to_mid_side(
                result.hole_fill_work_left, result.hole_fill_work_right,
                0, 0, false);
        }
        fill_band_start += components;
        const std::size_t consumed_size = source.position() - band_begin;
        if (consumed_size > std::numeric_limits<std::uint32_t>::max()) {
            return false;
        }
        if (!ace_schedule_next_band(
                schedule_state, prefix.allocation[band],
                static_cast<std::uint32_t>(consumed_size), result.schedule)
            || result.schedule.allocation != allocation) {
            return false;
        }
        payload.bands.push_back(std::move(result));
    }
    payload.failed_band = std::numeric_limits<std::uint32_t>::max();
    payload.failed_bit_offset = source.position();
    payload.bits_consumed = source.position() - begin;
    payload.final_bit_offset = source.position();
    AceFinalRefinement final_refinement;
    const std::array<std::uint32_t, 22> initial_final =
        stereo_tail_allocation(payload);
    if (!unpack_ace_final_refinement(
            bytes, size, prefix, payload.final_bit_offset,
            initial_final, final_refinement)) {
        return false;
    }
    AceScalarMatrix final_lognorm = prefix.refined_lognorm_q10;
    const AceScalarDequantControl control{
        prefix.predictive,
        prefix.first_effective_channel,
        prefix.effective_channel_count,
        prefix.effective_bands,
    };
    if (!ace_scalar_dequant_finalize(
            final_lognorm, final_refinement.codes,
            prefix.refinement_allocation, final_refinement.allocation,
            control)) {
        return false;
    }
    payload.finalized_lognorm_q10 = final_lognorm;
    for (std::uint32_t band = 0U;
         band < prefix.num_coded_bands; ++band) {
        if (!ace_apply_lognorm_gain(
                payload.bands[band].left_spectrum,
                ace_unnormalize_lognorm_q10(
                    final_lognorm[0][band], sample_rate, band))
            || !ace_apply_lognorm_gain(
                payload.bands[band].right_spectrum,
                ace_unnormalize_lognorm_q10(
                    final_lognorm[1][band], sample_rate, band))) {
            return false;
        }
    }
    payload.bits_consumed += final_refinement.bits_consumed;
    payload.final_bit_offset = final_refinement.final_bit_offset;
    return true;
}

bool ace_lr_stereo_bit_allocations(
    std::uint32_t allocation_select,
    std::uint32_t band,
    std::uint32_t total_bits,
    std::int32_t left_lognorm_q10,
    std::int32_t right_lognorm_q10,
    const std::array<std::uint32_t, 3>& region_modes,
    std::uint32_t& left_bits,
    std::uint32_t& right_bits) noexcept {
    left_bits = 0U;
    right_bits = 0U;
    if (allocation_select == 2U) {
        left_bits = total_bits;
        return true;
    }
    if (allocation_select == 3U) {
        right_bits = total_bits;
        return true;
    }
    if (allocation_select > 3U) {
        return false;
    }
    const std::uint32_t region = band < 4U ? 0U : band < 12U ? 1U : 2U;
    const std::uint32_t mode = region_modes[region];
    if (mode > 4U) {
        return false;
    }
    std::uint32_t share_q5 = 0U;
    if (mode == 1U || mode == 2U) {
        share_q5 = 19U;
    } else if (mode == 3U || mode == 4U) {
        share_q5 = 24U;
    } else {
        share_q5 = 16U;
    }
    const std::uint32_t first =
        (share_q5 * total_bits + 16U) >> 5U;
    const std::uint32_t second = total_bits - first;

    // Native LABEL_16 assigns a7=first/a6=second.  With mode zero the
    // lognorm comparison selects it; for modes 2 and 4 the bit mask
    // `(mode - 1) & ~2` selects it.
    const bool swap = mode == 0U
        ? left_lognorm_q10 < right_lognorm_q10
        : mode == 2U || mode == 4U;
    left_bits = swap ? second : first;
    right_bits = swap ? first : second;
    return true;
}

std::uint32_t ace_stereo_ms_ratio_quantization_level(
    const std::uint32_t available_bits,
    const std::uint32_t band_size,
    const std::int32_t log_band_q4) noexcept {
    if (available_bits == 0U || band_size == 0U || log_band_q4 < -16) {
        return 1U;
    }
    const std::int64_t two_band_minus_one =
        static_cast<std::int64_t>(band_size) * 2LL - 1LL;
    const std::int64_t scaled_bits =
        static_cast<std::int64_t>(available_bits) * 16LL;
    const std::int64_t first = scaled_bits - 32LL;
    const std::int64_t second = (scaled_bits + two_band_minus_one
        * (((static_cast<std::int64_t>(log_band_q4) + 16LL) >> 1U) - 16LL))
        / two_band_minus_one;
    const std::int64_t exponent = (std::min)(first, second);
    if (exponent < 8LL) {
        return 1U;
    }
    if (exponent > 128LL) {
        return 256U;
    }
    const std::uint32_t value = static_cast<std::uint32_t>(exponent);
    const std::uint32_t high = value >> 4U;
    const std::uint32_t base = detail::kAceExponentialTable[
        8U * (value & 0xFU)];
    const std::uint32_t expanded = high <= 5U
        ? base >> (5U - high)
        : base << (high - 5U);
    const std::uint32_t alphabet = ((expanded >> 4U) + 1U) & ~1U;
    return alphabet == 0U ? 1U : alphabet;
}

bool ace_decode_stereo_mid_side_params(
    AceBitReader& source,
    const std::uint32_t band_size,
    const std::int32_t log_band_q4,
    const bool coding_mode_1,
    const std::uint32_t available_bits,
    std::int32_t& persistent_beta_q15,
    AceBandRatio& ratio,
    std::uint32_t& bits_consumed) noexcept {
    bits_consumed = 0U;
    if (band_size == 0U) {
        return false;
    }
    if (available_bits == 0U) {
        return ace_band_ratio_from_beta(
            persistent_beta_q15, band_size, ratio);
    }
    const std::uint32_t alphabet = coding_mode_1 ? 1U
        : ace_stereo_ms_ratio_quantization_level(
            available_bits, band_size, log_band_q4);
    const std::size_t begin = source.position();
    if (!ace_decode_band_ratio(source, alphabet, band_size, true, ratio)) {
        return false;
    }
    bits_consumed = static_cast<std::uint32_t>(source.position() - begin);
    if (bits_consumed > available_bits) {
        return false;
    }
    // ace_decode_band_ratio serves the recursive split decoder and therefore
    // derives its angle from the right partition.  This M/S codec instead
    // passes the complete band size to dtsAce_ComputeNormsFromBeta().
    if (!ace_band_ratio_from_beta(ratio.beta_q15, band_size, ratio)) {
        return false;
    }
    persistent_beta_q15 = ratio.beta_q15;
    return true;
}

bool ace_apply_lognorm_gain(
    std::vector<float>& spectrum,
    const std::int32_t lognorm_q10) noexcept {
    if (spectrum.empty()) {
        return false;
    }
    // Sony x64 DTSAceStreamDecoder_UnNormalize converts Q10 lognorm to
    // float with * 1/1024, then for each band: if lognorm <= -32.0 the
    // bins and the stored norm are zeroed, else bins *= exp2f(lognorm).
    // The ARM32 Q31 (spec * pow2_i32 + 0x10000000) >> 29 path is a
    // different build; folding it here clamped VQ/Hadamard values to
    // +/-1 before the gain and does not match the float decoder.
    if (lognorm_q10 <= -32768) {
        std::fill(spectrum.begin(), spectrum.end(), 0.0F);
        return true;
    }
    const float gain = std::exp2(
        static_cast<float>(lognorm_q10) * 0.0009765625F);
    for (float& value : spectrum) {
        if (!std::isfinite(value)) {
            return false;
        }
        value *= gain;
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

namespace {

void advance_seed_for_empty_vq_leaves(
    std::uint32_t& seed,
    const AceVqPayloadResult& payload) noexcept {
    for (const AceVqLeaf& leaf : payload.leaves) {
        if (leaf.pulses != 0U) {
            continue;
        }
        for (std::uint32_t index = 0U; index < leaf.components; ++index) {
            (void)ace_prng_rand_in_range_i32(
                seed, -134217728, 134217728);
        }
    }
}

} // namespace

void ace_band_dequant_advance_random_seed_after_vq(
    std::uint32_t& seed,
    const bool spectral_hole_fill,
    const AceSpectralPayload& payload) noexcept {
    if (!spectral_hole_fill) {
        return;
    }
    for (const AceVqPayloadResult& band : payload.vq_bands) {
        advance_seed_for_empty_vq_leaves(seed, band);
    }
}

void ace_band_dequant_advance_random_seed_after_vq(
    std::uint32_t& seed,
    const bool spectral_hole_fill,
    const AceStereoSpectralPayload& payload) noexcept {
    if (!spectral_hole_fill) {
        return;
    }
    for (const AceStereoSpectralBand& band : payload.bands) {
        // ProcessOneBandStereo a8 = (coding == 1); coding 3 skips FillPartition.
        if (band.coding == 3U) {
            continue;
        }
        if (band.coding == 2U || band.left_partition_first) {
            advance_seed_for_empty_vq_leaves(seed, band.left_or_mid);
            advance_seed_for_empty_vq_leaves(seed, band.right_or_side);
        } else {
            advance_seed_for_empty_vq_leaves(seed, band.right_or_side);
            advance_seed_for_empty_vq_leaves(seed, band.left_or_mid);
        }
    }
}

} // namespace dtsx
