#include "audio/dca_bed_decoder.hpp"

extern "C" {
#include "dca_context.h"
}

#include <array>
#include <limits>

namespace dtsx_decode {
namespace {

struct WaveSpeaker final {
    std::uint32_t wave_bit;
    std::uint32_t dtsx_physical_bit;
};

constexpr std::array<WaveSpeaker, 8U> kWaveToDtsx71 = {{
    {2U, 0U},  // FC
    {0U, 1U},  // FL
    {1U, 2U},  // FR
    {3U, 5U},  // LFE
    {4U, 7U},  // BL
    {5U, 8U},  // BR
    {9U, 9U},  // SL -> LSS
    {10U, 10U}, // SR -> RSS
}};

std::uint32_t activity_for_physical_bit(
    std::uint32_t physical_bit) noexcept;

bool filter_context(
    dcadec_context* context,
    DcaDecodedBed& decoded,
    bool require_dtsx_71,
    std::string& error) {
    decoded = {};
    int** samples = nullptr;
    int sample_count = 0;
    int wave_mask = 0;
    int sample_rate = 0;
    int bits_per_sample = 0;
    int profile = 0;
    const int filter_result = dcadec_context_filter(
        context,
        &samples,
        &sample_count,
        &wave_mask,
        &sample_rate,
        &bits_per_sample,
        &profile);
    if (filter_result < 0 || samples == nullptr
        || sample_count <= 0 || sample_rate <= 0) {
        error = dcadec_strerror(filter_result);
        return false;
    }

    std::array<int, 32U> plane_by_wave_bit{};
    plane_by_wave_bit.fill(-1);
    int plane = 0;
    for (std::uint32_t bit = 0U; bit < 32U; ++bit) {
        if ((static_cast<std::uint32_t>(wave_mask)
             & (1U << bit))
            != 0U) {
            plane_by_wave_bit[bit] = plane++;
        }
    }

    decoded.channels.reserve(kWaveToDtsx71.size());
    decoded.channel_speaker_masks.reserve(
        kWaveToDtsx71.size());
    for (const WaveSpeaker& mapping : kWaveToDtsx71) {
        const int source_plane =
            plane_by_wave_bit[mapping.wave_bit];
        if (source_plane < 0) {
            continue;
        }
        std::vector<std::int32_t> channel(
            static_cast<std::size_t>(sample_count));
        for (int sample = 0; sample < sample_count; ++sample) {
            const std::int64_t value =
                samples[source_plane][sample];
            channel[static_cast<std::size_t>(sample)] =
                static_cast<std::int32_t>(
                    value < -0x800000LL
                        ? -0x800000LL
                        : (value > 0x7FFFFFLL
                               ? 0x7FFFFFLL
                               : value));
        }
        decoded.channels.push_back(std::move(channel));
        decoded.channel_speaker_masks.push_back(
            1U << mapping.dtsx_physical_bit);
        decoded.speaker_activity_mask |=
            activity_for_physical_bit(
                mapping.dtsx_physical_bit);
    }
    if (decoded.channels.empty()
        || (require_dtsx_71
            && (decoded.channels.size()
                    != kWaveToDtsx71.size()
                || decoded.speaker_activity_mask != 0x84BU))) {
        decoded = {};
        error = require_dtsx_71
            ? "decoded bed is not DTS:X 7.1"
            : "decoded DTS Core has no mapped channels";
        return false;
    }
    decoded.sample_rate =
        static_cast<std::uint32_t>(sample_rate);
    decoded.samples_per_channel =
        static_cast<std::uint32_t>(sample_count);
    return true;
}

std::uint32_t activity_for_physical_bit(
    std::uint32_t physical_bit) noexcept {
    switch (physical_bit) {
    case 0U:
        return 1U << 0U;
    case 1U:
    case 2U:
        return 1U << 1U;
    case 5U:
        return 1U << 3U;
    case 7U:
    case 8U:
        return 1U << 6U;
    case 9U:
    case 10U:
        return 1U << 11U;
    default:
        return 0U;
    }
}

} // namespace

void DcaBedDecoder::ContextDeleter::operator()(
    dcadec_context* context) const noexcept {
    dcadec_context_destroy(context);
}

DcaBedDecoder::DcaBedDecoder()
    : context_(dcadec_context_create(0))
    , core_context_(
          dcadec_context_create(
              DCADEC_FLAG_CORE_ONLY
              | DCADEC_FLAG_CORE_BIT_EXACT)) {
    if (!context_ || !core_context_) {
        last_error_ = "libdcadec context allocation failed";
    }
}

DcaBedDecoder::~DcaBedDecoder() = default;

void DcaBedDecoder::remember_core(
    const dtsx::ElementaryFrame& frame) {
    pending_core_ = frame.bytes;
    decoded_core_ = {};
    if (!core_context_ || frame.bytes.empty()) {
        return;
    }
    std::vector<std::uint8_t> packet = frame.bytes;
    const std::size_t packet_size = packet.size();
    packet.resize(
        packet_size + DCADEC_BUFFER_PADDING, 0U);
    const int parse_result = dcadec_context_parse(
        core_context_.get(), packet.data(), packet_size);
    if (parse_result < 0) {
        last_error_ = dcadec_strerror(parse_result);
        return;
    }
    (void)filter_context(
        core_context_.get(),
        decoded_core_,
        false,
        last_error_);
}

bool DcaBedDecoder::decode_extension(
    const dtsx::ElementaryFrame& frame,
    DcaDecodedBed& decoded) {
    decoded = {};
    last_error_.clear();
    if (!context_) {
        last_error_ = "libdcadec context is unavailable";
        return false;
    }

    std::vector<std::uint8_t> packet;
    packet.reserve(
        pending_core_.size() + frame.bytes.size()
        + DCADEC_BUFFER_PADDING);
    packet.insert(
        packet.end(), pending_core_.begin(), pending_core_.end());
    packet.insert(
        packet.end(), frame.bytes.begin(), frame.bytes.end());
    const std::size_t packet_size = packet.size();
    packet.resize(packet_size + DCADEC_BUFFER_PADDING, 0U);
    pending_core_.clear();

    const int parse_result = dcadec_context_parse(
        context_.get(), packet.data(), packet_size);
    if (parse_result < 0) {
        last_error_ = dcadec_strerror(parse_result);
        return false;
    }

    return filter_context(
        context_.get(), decoded, true, last_error_);
}

} // namespace dtsx_decode
