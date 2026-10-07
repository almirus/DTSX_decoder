#include "audio/dca_bed_decoder.hpp"

#include "bitstream/dtsx_word_buffer.hpp"
#include "dtsx/exss_asset.hpp"
#include "dtsx/exss_header.hpp"

extern "C" {
#include "dca_context.h"
}

#include <algorithm>
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

bool filter_object_context(
    dcadec_context* context,
    DcaDecodedObjectAsset& decoded,
    std::string& error) {
    int** samples = nullptr;
    int sample_count = 0;
    int channel_mask = 0;
    int sample_rate = 0;
    int bits_per_sample = 0;
    int profile = 0;
    const int filter_result = dcadec_context_filter(
        context,
        &samples,
        &sample_count,
        &channel_mask,
        &sample_rate,
        &bits_per_sample,
        &profile);
    if (filter_result < 0 || samples == nullptr
        || sample_count <= 0 || sample_rate <= 0) {
        error = dcadec_strerror(filter_result);
        return false;
    }
    const std::uint32_t mask =
        static_cast<std::uint32_t>(channel_mask);
    std::uint32_t channel_count = 0U;
    for (std::uint32_t bit = 0U; bit < 32U; ++bit) {
        channel_count += (mask >> bit) & 1U;
    }
    if (channel_count == 0U) {
        error = "decoded object asset has no channels";
        return false;
    }
    decoded.channels.resize(channel_count);
    for (std::uint32_t channel = 0U;
         channel < channel_count;
         ++channel) {
        decoded.channels[channel].assign(
            samples[channel], samples[channel] + sample_count);
    }
    decoded.sample_rate = static_cast<std::uint32_t>(sample_rate);
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
              | DCADEC_FLAG_CORE_BIT_EXACT))
    , core_probe_context_(
          dcadec_context_create(DCADEC_FLAG_CORE_BIT_EXACT)) {
    if (!context_ || !core_context_ || !core_probe_context_) {
        last_error_ = "libdcadec context allocation failed";
    }
}

DcaBedDecoder::~DcaBedDecoder() = default;

void DcaBedDecoder::remember_core(
    const dtsx::ElementaryFrame& frame) {
    pending_core_ = frame.bytes;
    decoded_core_ = {};
    core_stream_info_ = {};
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

    if (!core_probe_context_) {
        return;
    }
    std::vector<std::uint8_t> probe_packet = frame.bytes;
    probe_packet.resize(
        packet_size + DCADEC_BUFFER_PADDING, 0U);
    const int probe_parse_result = dcadec_context_parse(
        core_probe_context_.get(),
        probe_packet.data(),
        packet_size);
    if (probe_parse_result < 0) {
        return;
    }

    dcadec_core_info* core_info =
        dcadec_context_get_core_info(core_probe_context_.get());
    dcadec_exss_info* stream_info =
        dcadec_context_get_exss_info(core_probe_context_.get());
    if (core_info != nullptr) {
        core_stream_info_.sample_rate =
            static_cast<std::uint32_t>(
                (std::max)(core_info->sample_rate, 0));
        core_stream_info_.source_pcm_bits =
            static_cast<std::uint32_t>(
                (std::max)(core_info->source_pcm_res, 0));
        core_stream_info_.samples_per_frame =
            static_cast<std::uint32_t>(
                (std::max)(core_info->npcmblocks, 0)) * 32U;
        core_stream_info_.bit_rate = core_info->bit_rate;
        core_stream_info_.es_matrix_surround =
            core_info->es_format;
    }
    if (stream_info != nullptr) {
        core_stream_info_.channels =
            static_cast<std::uint32_t>(
                (std::max)(stream_info->nchannels, 0));
        core_stream_info_.sample_rate =
            static_cast<std::uint32_t>(
                (std::max)(stream_info->sample_rate, 0));
        core_stream_info_.source_pcm_bits =
            static_cast<std::uint32_t>(
                (std::max)(stream_info->bits_per_sample, 0));
        core_stream_info_.speaker_activity_mask =
            static_cast<std::uint32_t>(stream_info->spkr_mask);
        core_stream_info_.profile = stream_info->profile;
        core_stream_info_.matrix_encoding =
            stream_info->matrix_encoding;
        core_stream_info_.embedded_6ch =
            stream_info->embedded_6ch;
    }
    core_stream_info_.valid =
        core_info != nullptr || stream_info != nullptr;
    dcadec_context_free_core_info(core_info);
    dcadec_context_free_exss_info(stream_info);
    (void)dcadec_context_filter(
        core_probe_context_.get(),
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr);
}

bool DcaBedDecoder::decode_extension(
    const dtsx::ElementaryFrame& frame,
    DcaDecodedBed& decoded,
    bool require_dtsx_71,
    std::vector<DcaDecodedObjectAsset>* object_assets) {
    decoded = {};
    extension_stream_info_ = {};
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

    const int parse_result = dcadec_context_parse(
        context_.get(), packet.data(), packet_size);
    if (parse_result < 0) {
        last_error_ = dcadec_strerror(parse_result);
        pending_core_.clear();
        return false;
    }

    dcadec_exss_info* stream_info =
        dcadec_context_get_exss_info(context_.get());
    if (stream_info != nullptr) {
        extension_stream_info_.channels =
            static_cast<std::uint32_t>(
                (std::max)(stream_info->nchannels, 0));
        extension_stream_info_.sample_rate =
            static_cast<std::uint32_t>(
                (std::max)(stream_info->sample_rate, 0));
        extension_stream_info_.source_pcm_bits =
            static_cast<std::uint32_t>(
                (std::max)(stream_info->bits_per_sample, 0));
        extension_stream_info_.speaker_activity_mask =
            static_cast<std::uint32_t>(stream_info->spkr_mask);
        extension_stream_info_.profile = stream_info->profile;
        extension_stream_info_.matrix_encoding =
            stream_info->matrix_encoding;
        extension_stream_info_.embedded_stereo =
            stream_info->embedded_stereo;
        extension_stream_info_.embedded_6ch =
            stream_info->embedded_6ch;
        extension_stream_info_.valid = true;
    }
    dcadec_context_free_exss_info(stream_info);

    const bool bed_decoded = filter_context(
        context_.get(), decoded, require_dtsx_71, last_error_);

    if (object_assets != nullptr) {
        object_assets->clear();
        const bool swap =
            frame.packing == dtsx::StreamPacking::ExtensionBigEndian;
        dtsx::bitstream::WordBuffer words(frame.bytes, swap);
        dtsx::bitstream::Cursor header_source = words.cursor();
        dtsx::ExssHeader header;
        if (dtsx::unpack_exss_header(header_source, header)) {
            if (object_asset_contexts_.size() < header.asset_count) {
                object_asset_contexts_.resize(header.asset_count);
            }
            for (std::uint32_t ordinal = 0U;
                 ordinal < header.asset_count
                     && ordinal < header.asset_header_bit_offsets.size();
                 ++ordinal) {
                dtsx::bitstream::Cursor asset_source = words.cursor();
                asset_source.fast_forward(static_cast<std::int32_t>(
                    header.asset_header_bit_offsets[ordinal]));
                dtsx::ExssAssetSummary asset;
                if (!dtsx::unpack_exss_asset_summary(
                        asset_source, header, asset, ordinal)
                    || asset.object_audio_type != 1U
                    || (asset.coding_components & (1U << 6U)) == 0U
                    || (asset.coding_components & (1U << 9U)) != 0U) {
                    continue;
                }
                auto& asset_context = object_asset_contexts_[ordinal];
                if (!asset_context) {
                    asset_context.reset(dcadec_context_create(
                        DCADEC_FLAG_NATIVE_LAYOUT));
                    if (!asset_context
                        || dcadec_context_set_exss_asset(
                               asset_context.get(), ordinal) < 0) {
                        last_error_ =
                            "XXCH object asset decoder allocation failed";
                        pending_core_.clear();
                        return false;
                    }
                }
                const int object_parse = dcadec_context_parse(
                    asset_context.get(), packet.data(), packet_size);
                if (object_parse < 0) {
                    last_error_ = dcadec_strerror(object_parse);
                    pending_core_.clear();
                    return false;
                }
                DcaDecodedObjectAsset decoded_asset;
                decoded_asset.asset_ordinal =
                    static_cast<std::uint8_t>(ordinal);
                decoded_asset.asset_index = asset.asset_index;
                decoded_asset.coding_components =
                    asset.coding_components;
                if (!filter_object_context(
                        asset_context.get(), decoded_asset, last_error_)) {
                    pending_core_.clear();
                    return false;
                }
                object_assets->push_back(std::move(decoded_asset));
            }
        }
    }
    pending_core_.clear();
    return bed_decoded;
}

} // namespace dtsx_decode
