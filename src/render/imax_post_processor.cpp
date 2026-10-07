#include "render/imax_post_processor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace dtsx_decode {
namespace {

struct Lp4Record final {
    std::uint32_t sample_rate;
    double gain;
    double a1;
    double a2;
};

struct Hp4Record final {
    std::uint32_t sample_rate;
    double gain;
    double a1;
    double a2;
};

constexpr std::array<Lp4Record, 9U> kImaxLp4Records{{
    {32000U, 4.688294939e-05, -1.980539918, 0.9807274342},
    {44100U, 2.475088331e-05, -1.985879064, 0.9859780669},
    {48000U, 2.090419366e-05, -1.987026453, 0.9871100187},
    {88200U, 6.209526418e-06, -1.992939472, 0.9929642677},
    {96000U, 5.242971383e-06, -1.993513107, 0.9935340881},
    {192000U, 1.312866743e-06, -1.996756554, 0.9967617989},
    {64000U, 1.177762169e-05, -1.990269661, 0.9903168082},
    {128000U, 2.951559281e-06, -1.995134830, 0.9951466322},
    {176400U, 1.555119411e-06, -1.996469736, 0.9964759350},
}};

constexpr std::array<Hp4Record, 9U> kImaxHp4Records{{
    {32000U, 0.9903396964, -1.980586052, 0.9807726741},
    {44100U, 0.9929808974, -1.985912561, 0.9860110879},
    {48000U, 0.9935494065, -1.987057090, 0.9871403575},
    {64000U, 0.9951581359, -1.990292788, 0.9903396964},
    {88200U, 0.9964842796, -1.992956161, 0.9929808974},
    {96000U, 0.9967694879, -1.993528485, 0.9935494065},
    {128000U, 0.9975761175, -1.995146394, 0.9951581359},
    {176400U, 0.9982405901, -1.996478081, 0.9964842796},
    {192000U, 0.9983834028, -1.996764183, 0.9967694879},
}};

// The MCU changes Beta 0x40 +0x13 from D0 to E4 for IMAX. Both are signed
// half-dB PA/F volume codes, so the IMAX profile raises the LFE input by
// exactly 10 dB relative to normal decoding. The decoder PCM is already at
// its normal reference level; applying E4 as an absolute -14 dB gain would
// count the PA/F baseline twice.
constexpr double kImaxLfeGain = 3.1622776601683795;

const Lp4Record* find_record(std::uint32_t sample_rate) noexcept {
    const auto record = std::find_if(
        kImaxLp4Records.begin(),
        kImaxLp4Records.end(),
        [sample_rate](const Lp4Record& candidate) {
            return candidate.sample_rate == sample_rate;
        });
    return record == kImaxLp4Records.end() ? nullptr : &*record;
}

const Hp4Record* find_hp_record(std::uint32_t sample_rate) noexcept {
    const auto record = std::find_if(
        kImaxHp4Records.begin(),
        kImaxHp4Records.end(),
        [sample_rate](const Hp4Record& candidate) {
            return candidate.sample_rate == sample_rate;
        });
    return record == kImaxHp4Records.end() ? nullptr : &*record;
}

std::int32_t pcm24(double sample) noexcept {
    const double rounded = std::round(sample);
    return static_cast<std::int32_t>(
        std::max(-8388608.0, std::min(8388607.0, rounded)));
}

bool make_small_channel_mask(
    const ChannelLayout& layout,
    const std::string& specification,
    std::vector<bool>& mask) {
    mask.assign(layout.channels.size(), false);
    if (specification == "none") {
        return true;
    }
    if (specification == "all") {
        for (std::size_t index = 0U; index < layout.channels.size(); ++index) {
            mask[index] = layout.channels[index] != "LFE";
        }
        return true;
    }

    std::unordered_set<std::string> requested;
    std::istringstream input(specification);
    std::string channel;
    while (std::getline(input, channel, ',')) {
        if (channel.empty() || channel == "LFE") {
            return false;
        }
        requested.insert(channel);
    }
    if (requested.empty()) {
        return false;
    }
    for (std::size_t index = 0U; index < layout.channels.size(); ++index) {
        if (requested.erase(layout.channels[index]) != 0U) {
            mask[index] = true;
        }
    }
    return requested.empty();
}

} // namespace

double ImaxPostProcessor::Biquad::process(double sample) noexcept {
    const double output = b0 * sample + z1;
    z1 = b1 * sample - a1 * output + z2;
    z2 = b2 * sample - a2 * output;
    return output;
}

bool ImaxPostProcessor::process(
    std::vector<std::vector<std::int32_t>>& channels,
    const ChannelLayout& layout,
    std::uint32_t sample_rate,
    bool imax_enabled,
    const std::string& small_speakers) {
    if (!imax_enabled) {
        return true;
    }
    if (channels.size() != layout.channels.size()) {
        return false;
    }

    if (!configured_
        || sample_rate_ != sample_rate
        || channel_names_ != layout.channels
        || small_speakers_ != small_speakers) {
        const Lp4Record* const record = find_record(sample_rate);
        const Hp4Record* const hp_record = find_hp_record(sample_rate);
        const auto lfe = std::find(
            layout.channels.begin(), layout.channels.end(), "LFE");
        if (record == nullptr || hp_record == nullptr
            || lfe == layout.channels.end()) {
            return false;
        }

        Biquad section;
        section.b0 = record->gain;
        section.b1 = 2.0 * record->gain;
        section.b2 = record->gain;
        section.a1 = record->a1;
        section.a2 = record->a2;
        lfe_lp_sections_ = {section, section};
        Biquad hp_section;
        hp_section.b0 = hp_record->gain;
        hp_section.b1 = -2.0 * hp_record->gain;
        hp_section.b2 = hp_record->gain;
        hp_section.a1 = hp_record->a1;
        hp_section.a2 = hp_record->a2;
        main_hp_sections_.assign(
            channels.size(), {hp_section, hp_section});
        main_lp_sections_.assign(channels.size(), {section, section});
        if (!make_small_channel_mask(
                layout, small_speakers, small_channels_)) {
            return false;
        }
        sample_rate_ = sample_rate;
        channel_names_ = layout.channels;
        small_speakers_ = small_speakers;
        lfe_channel_ = static_cast<std::size_t>(
            std::distance(layout.channels.begin(), lfe));
        configured_ = true;
    }
    const std::size_t frame_count = channels[lfe_channel_].size();
    std::vector<double> collected_bass(frame_count, 0.0);
    for (std::size_t channel = 0U; channel < channels.size(); ++channel) {
        if (channel == lfe_channel_) {
            continue;
        }
        if (channels[channel].size() != frame_count) {
            return false;
        }
        if (!small_channels_[channel]) {
            continue;
        }
        for (std::size_t frame = 0U; frame < frame_count; ++frame) {
            const double input = static_cast<double>(channels[channel][frame]);
            double high = input;
            for (Biquad& section : main_hp_sections_[channel]) {
                high = section.process(high);
            }
            double low = input;
            for (Biquad& section : main_lp_sections_[channel]) {
                low = section.process(low);
            }
            channels[channel][frame] = pcm24(high);
            collected_bass[frame] += low;
        }
    }

    for (std::size_t frame = 0U; frame < frame_count; ++frame) {
        double lfe = static_cast<double>(channels[lfe_channel_][frame]);
        for (Biquad& section : lfe_lp_sections_) {
            lfe = section.process(lfe);
        }
        channels[lfe_channel_][frame] =
            pcm24(lfe * kImaxLfeGain + collected_bass[frame]);
    }
    return true;
}

void ImaxPostProcessor::reset() noexcept {
    lfe_lp_sections_ = {};
    main_hp_sections_.clear();
    main_lp_sections_.clear();
    small_channels_.clear();
    sample_rate_ = 0U;
    channel_names_.clear();
    small_speakers_.clear();
    lfe_channel_ = 0U;
    configured_ = false;
}

} // namespace dtsx_decode
