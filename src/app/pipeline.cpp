#include "pipeline.hpp"

#include "../audio/layout.hpp"
#include "../audio/dca_bed_decoder.hpp"
#include "../app/channel_capacity.hpp"
#include "../app/object_frame_decoder.hpp"
#include "../app/progress.hpp"
#include "../io/ffmpeg.hpp"
#include "../io/wav_writer.hpp"
#include "../bitstream/dtsx_word_buffer.hpp"
#include "../dtsx/exss_header.hpp"
#include "../dtsx/exss_asset.hpp"
#include "../dtsx/frame_header.hpp"
#include "../dtsx/core_substream.hpp"
#include "../dtsx/metadata_chunk.hpp"
#include "../dtsx/object_waveform_map.hpp"
#include "../dtsx/preliminary_metadata.hpp"
#include "../dtsx/speaker_mask.hpp"
#include "../dtsx/uhd_frame.hpp"
#include "../dtsx/xll_common_header.hpp"
#include "../dtsx/xll_channel_set.hpp"
#include "../dtsx/xll_frame_decoder.hpp"
#include "../dtsx/xll_navigation.hpp"
#include "../io/dts_frame_reader.hpp"
#include "../io/object_sidecar_writer.hpp"
#include "../io/object_stem_writer.hpp"
#include "../io/p2_decoder.hpp"
#include "../render/object_audio_renderer.hpp"
#include "../render/parma_layout.hpp"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cwctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace dtsx_decode {
namespace {

std::wstring lower_extension(std::filesystem::path path) {
    std::wstring extension = path.extension().wstring();
    for (wchar_t& c : extension) {
        c = std::towlower(c);
    }
    return extension;
}

bool supported_input_extension(const std::filesystem::path& path) {
    const std::wstring extension = lower_extension(path);
    return extension == L".mkv" || extension == L".mp4"
        || extension == L".m2ts" || extension == L".dts"
        || extension == L".dtshd";
}

bool is_elementary_dts(const std::filesystem::path& path) {
    const std::wstring extension = lower_extension(path);
    return extension == L".dts" || extension == L".dtshd";
}

enum class DtsTrackRank : unsigned {
    None = 0U,
    Dts = 1U,
    DtsHdMa = 2U,
    DtsX = 3U,
};

void classify_dts_frame(
    const dtsx::ElementaryFrame& frame,
    DtsTrackRank& rank) {
    if (frame.packing == dtsx::StreamPacking::DtsUhd) {
        rank = DtsTrackRank::DtsX;
        return;
    }
    if (frame.packing != dtsx::StreamPacking::ExtensionBigEndian
        && frame.packing
            != dtsx::StreamPacking::ExtensionLittleEndian) {
        rank = (std::max)(rank, DtsTrackRank::Dts);
        return;
    }
    const bool swap =
        frame.packing == dtsx::StreamPacking::ExtensionBigEndian;
    dtsx::bitstream::WordBuffer words(frame.bytes, swap);
    dtsx::ExssHeader header;
    dtsx::bitstream::Cursor header_source = words.cursor();
    if (!dtsx::unpack_exss_header(header_source, header)) {
        rank = (std::max)(rank, DtsTrackRank::Dts);
        return;
    }
    for (std::uint32_t asset_index = 0U;
         asset_index < header.asset_count
         && asset_index < header.asset_header_bit_offsets.size();
         ++asset_index) {
        dtsx::bitstream::Cursor asset_source = words.cursor();
        asset_source.fast_forward(static_cast<std::int32_t>(
            header.asset_header_bit_offsets[asset_index]));
        dtsx::ExssAssetSummary asset;
        if (!dtsx::unpack_exss_asset_summary(
                asset_source, header, asset, asset_index)) {
            continue;
        }
        if ((asset.coding_components & (1U << 9U)) != 0U) {
            rank = (std::max)(rank, DtsTrackRank::DtsHdMa);
        }
        if (asset.object_audio_type != 0U
            || asset.xll_object_metadata_present
            || (asset.xll_metadata_present
                && !asset.xll_metadata_chunk_sizes.empty())) {
            rank = DtsTrackRank::DtsX;
            return;
        }
    }
}

struct DtsTrackCandidate final {
    bool exists = false;
    DtsTrackRank rank = DtsTrackRank::None;
};

DtsTrackCandidate inspect_audio_track(
    const Options& options,
    unsigned audio_track) {
    Options candidate_options = options;
    candidate_options.audio_track = audio_track;
    candidate_options.audio_track_optional = true;
    candidate_options.probe = true;
    candidate_options.full_probe = false;
    FfmpegDtsReader demuxer(candidate_options);
    dtsx::FrameAssembler assembler(dtsx::SyncAlignment::AnyByte);
    std::array<std::uint8_t, 256U * 1024U> bytes{};
    DtsTrackCandidate candidate;
    while (true) {
        const std::size_t size =
            demuxer.read(bytes.data(), bytes.size());
        if (size == 0U) {
            break;
        }
        candidate.exists = true;
        for (std::size_t index = 0U; index < size; ++index) {
            std::optional<dtsx::ElementaryFrame> frame =
                assembler.push(bytes[index]);
            if (frame) {
                classify_dts_frame(*frame, candidate.rank);
            }
        }
    }
    try {
        demuxer.finish();
    } catch (const std::runtime_error&) {
        if (candidate.exists) {
            throw;
        }
    }
    return candidate;
}

unsigned select_default_dts_track(const Options& options) {
    constexpr unsigned kMaximumAudioTracks = 32U;
    unsigned selected = 0U;
    DtsTrackRank selected_rank = DtsTrackRank::None;
    for (unsigned track = 0U; track < kMaximumAudioTracks; ++track) {
        const DtsTrackCandidate candidate =
            inspect_audio_track(options, track);
        if (!candidate.exists) {
            break;
        }
        if (candidate.rank > selected_rank) {
            selected = track;
            selected_rank = candidate.rank;
        }
        if (selected_rank == DtsTrackRank::DtsX) {
            // Keep scanning: the first DTS:X track wins, while a missing
            // audio ordinal still terminates container enumeration.
        }
    }
    if (selected_rank == DtsTrackRank::None) {
        throw std::runtime_error(
            "container has no DTS audio track");
    }
    return selected;
}

class ScopedTemporaryFile final {
public:
    ScopedTemporaryFile() = default;

    ~ScopedTemporaryFile() {
        if (!path_.empty()) {
            std::error_code ignored;
            std::filesystem::remove(path_, ignored);
        }
    }

    ScopedTemporaryFile(const ScopedTemporaryFile&) = delete;
    ScopedTemporaryFile& operator=(const ScopedTemporaryFile&) = delete;

    void set(std::filesystem::path path) {
        path_ = std::move(path);
    }

    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    std::filesystem::path path_;
};

std::filesystem::path temporary_elementary_path() {
    static std::atomic<std::uint64_t> sequence{0U};
    const std::uint64_t stamp = static_cast<std::uint64_t>(
        std::chrono::high_resolution_clock::now()
            .time_since_epoch()
            .count());
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path();
    for (std::uint64_t attempt = 0U; attempt < 1024U; ++attempt) {
        const std::filesystem::path candidate =
            directory
            / (L"dtsx-decode-"
               + std::to_wstring(stamp)
               + L"-"
               + std::to_wstring(
                   sequence.fetch_add(1U, std::memory_order_relaxed))
               + L".dtshd");
        if (!std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    throw std::runtime_error(
        "cannot allocate a temporary DTS elementary stream");
}

void demux_container(
    const Options& options,
    const std::filesystem::path& destination) {
    ProgressReporter progress;
    progress.update("demux", -1);
    FfmpegDtsReader demuxer(options);
    std::ofstream output(destination, std::ios::binary);
    if (!output) {
        throw std::runtime_error(
            "cannot create temporary DTS elementary stream");
    }
    std::array<std::uint8_t, 256U * 1024U> buffer{};
    while (true) {
        const std::size_t size =
            demuxer.read(buffer.data(), buffer.size());
        if (size == 0U) {
            break;
        }
        output.write(
            reinterpret_cast<const char*>(buffer.data()),
            static_cast<std::streamsize>(size));
        if (!output) {
            throw std::runtime_error(
                "cannot write temporary DTS elementary stream");
        }
    }
    demuxer.finish();
    output.close();
    if (!output) {
        throw std::runtime_error(
            "cannot finalize temporary DTS elementary stream");
    }
    progress.done("demux");
}

int decode_percent(
    std::uint64_t consumed,
    std::uint64_t total) noexcept {
    if (total == 0U) {
        return -1;
    }
    return static_cast<int>(
        std::min<std::uint64_t>(99U, consumed * 100U / total));
}

std::vector<dtsx::XllLossyBaseChannel> make_xll_lossy_base(
    const DcaDecodedBed& core);

struct ChannelCapacity final {
    std::uint32_t physical_speakers = 0U;
    bool dynamic_objects = false;
};

ChannelCapacity inspect_channel_capacity(const Options& options) {
    constexpr std::uint32_t kExtensionFrames = 16U;
    constexpr std::uint32_t kMaximumFrames = 256U;
    DtsFrameReader reader(options);
    ObjectFrameDecoder object_decoder;
    DcaBedDecoder bed_decoder;
    dtsx::UhdFrameParserState uhd_state;
    ChannelCapacity capacity;
    dtsx::ElementaryFrame frame;
    std::uint32_t frames = 0U;
    std::uint32_t extension_frames = 0U;
    while (frames++ < kMaximumFrames && reader.read(frame)) {
        if (frame.packing == dtsx::StreamPacking::DtsUhd) {
            dtsx::UhdFrameHeader header;
            if (dtsx::parse_uhd_frame_header(
                    frame.bytes, uhd_state, header)
                    == dtsx::UhdHeaderParseResult::Complete
                && dtsx::parse_uhd_full_mix_metadata(
                    frame.bytes, header)) {
                add_activity_speakers(
                    header.speaker_activity_mask,
                    capacity.physical_speakers);
                if (capacity.physical_speakers != 0U) {
                    break;
                }
            }
            continue;
        }
        if (frame.packing != dtsx::StreamPacking::ExtensionBigEndian
            && frame.packing
                != dtsx::StreamPacking::ExtensionLittleEndian) {
            bed_decoder.remember_core(frame);
            continue;
        }
        ++extension_frames;
        DcaDecodedBed decoded_bed;
        const bool bed_available =
            bed_decoder.decode_extension(frame, decoded_bed);
        if (bed_available) {
            add_activity_speakers(
                decoded_bed.speaker_activity_mask,
                capacity.physical_speakers);
        }
        DecodedObjectAudioFrame decoded;
        const auto lossy_base =
            make_xll_lossy_base(bed_decoder.decoded_core());
        if (object_decoder.decode(frame, decoded, lossy_base)
            == ObjectFrameDecodeResult::Decoded) {
            add_activity_speakers(
                decoded.bed_speaker_activity_mask,
                capacity.physical_speakers);
            for (const dtsx::XllHierarchicalDownmixOutput& downmix :
                 decoded.bed_downmix_outputs) {
                for (const std::uint32_t speaker :
                     downmix.speaker_masks) {
                    capacity.physical_speakers |= speaker;
                }
            }
            for (std::size_t waveform = 0U;
                 waveform < decoded.waveform_speaker_masks.size()
                 && waveform
                        < decoded.waveform_is_supplemental.size();
                 ++waveform) {
                if (decoded.waveform_is_supplemental[waveform]) {
                    capacity.physical_speakers |=
                        decoded.waveform_speaker_masks[waveform];
                }
            }
            capacity.dynamic_objects =
                capacity.dynamic_objects || !decoded.objects.empty();
        }
        if (extension_frames >= kExtensionFrames) {
            break;
        }
    }
    return capacity;
}

void print_decode_summary(
    const Options& options,
    const AudioProbe& probe,
    const ChannelLayout& layout,
    const std::filesystem::path& output,
    const std::string& warning) {
    std::cerr << "Input:  " << options.input.string() << '\n';
    std::cerr << "Audio:  stream #" << probe.stream_index << ", "
              << (probe.codec_name.empty() ? "unknown codec" : probe.codec_name);
    if (!probe.profile.empty()) {
        std::cerr << ", " << probe.profile;
    }
    if (probe.sample_rate != 0U) {
        std::cerr << ", " << probe.sample_rate << " Hz";
    } else {
        std::cerr << ", internal sample rate";
    }
    std::cerr << ", " << probe.channels << " ch\n";
    std::cerr << "Output: " << output.string() << '\n';
    std::cerr << "Layout: " << layout.name << " ["
              << join_channel_names(layout) << "]\n";
    if (!warning.empty()) {
        const bool color = console_style::color_enabled(stderr);
        console_style::paint(
            std::cerr, color, console_style::bold);
        console_style::paint(
            std::cerr, color, console_style::bright_yellow);
        std::cerr << warning;
        console_style::reset(std::cerr, color);
        std::cerr << '\n';
    }
}

std::string format_probe_duration(
    std::uint64_t samples,
    std::uint32_t sample_rate) {
    if (sample_rate == 0U) {
        return "unknown";
    }
    const std::uint64_t total_seconds = samples / sample_rate;
    const std::uint64_t hours = total_seconds / 3600U;
    const std::uint64_t minutes =
        (total_seconds / 60U) % 60U;
    const std::uint64_t seconds = total_seconds % 60U;
    const std::uint64_t milliseconds =
        ((samples % sample_rate) * 1000U) / sample_rate;
    std::ostringstream output;
    if (hours != 0U) {
        output << hours << ':'
               << std::setw(2) << std::setfill('0') << minutes
               << ':';
    } else {
        output << minutes << ':';
    }
    output << std::setw(2) << std::setfill('0') << seconds
           << '.' << std::setw(3) << milliseconds;
    return output.str();
}

std::string probe_hex(std::uint32_t value) {
    std::ostringstream output;
    output << "0x" << std::hex << value;
    return output.str();
}

std::string format_probe_speaker_layout(std::uint32_t activity_mask) {
    if (activity_mask == 0U) {
        return "not declared";
    }
    const std::vector<std::uint32_t> speakers =
        dtsx::expand_speaker_activity_mask(activity_mask);
    std::ostringstream output;
    output << speakers.size() << " ch (";
    for (std::size_t index = 0U; index < speakers.size(); ++index) {
        if (index != 0U) {
            output << ", ";
        }
        if (speakers[index] == (1U << 5U)) {
            output << "LFE";
            continue;
        }
        float azimuth = 0.0F;
        float elevation = 0.0F;
        std::string_view name;
        if (dtsx::standard_speaker_coordinates(
                speakers[index], azimuth, elevation, name)) {
            output << name;
        } else {
            std::uint32_t bit = 0U;
            std::uint32_t speaker = speakers[index];
            while ((speaker >>= 1U) != 0U) {
                ++bit;
            }
            output << "speaker-bit-" << bit;
        }
    }
    output << ')';
    return output.str();
}

struct ProbeObjectInfo final {
    std::uint32_t id = 0U;
    bool id_available = false;
    std::uint8_t metadata_mode = 0U;
    std::size_t waveform_count = 0U;
    std::set<std::uint8_t> waveform_types;
    bool group_present = false;
    std::uint8_t group = 0U;
    bool spatial_group_present = false;
    std::uint8_t spatial_group = 0U;
    std::size_t maximum_point_sources = 0U;
    bool position_available = false;
    float initial_azimuth = 0.0F;
    float initial_elevation = 0.0F;
    float initial_distance = 0.0F;
    float latest_azimuth = 0.0F;
    float latest_elevation = 0.0F;
    float latest_distance = 0.0F;
    bool position_changed = false;
};

void update_probe_object_info(
    ProbeObjectInfo& info,
    const dtsx::ObjectMetadataBlock& object,
    std::uint32_t fallback_id) {
    info.id_available = object.object_id_available;
    info.id = object.object_id_available
        ? object.object_id
        : fallback_id;
    info.metadata_mode = object.preamble.metadata_mode;
    info.waveform_count = (std::max)(
        info.waveform_count,
        static_cast<std::size_t>(object.preamble.waveform_count));
    info.waveform_types.insert(
        object.preamble.waveform_types.begin(),
        object.preamble.waveform_types.end());
    if (object.group_assignment_present) {
        info.group_present = true;
        info.group = object.group_assignment;
    }
    if (object.spatial_group_present) {
        info.spatial_group_present = true;
        info.spatial_group = object.spatial_group;
    }
    info.maximum_point_sources = (std::max)(
        info.maximum_point_sources, object.points.size());
    if (object.points.empty()) {
        return;
    }
    const dtsx::RendererCoordinates& coordinates =
        object.points.front().coordinates;
    if (!info.position_available) {
        info.position_available = true;
        info.initial_azimuth = coordinates.azimuth_degrees;
        info.initial_elevation = coordinates.elevation_degrees;
        info.initial_distance = coordinates.distance;
    } else if (std::fabs(
                   coordinates.azimuth_degrees
                       - info.latest_azimuth)
                   > 0.01F
               || std::fabs(
                      coordinates.elevation_degrees
                          - info.latest_elevation)
                      > 0.01F
               || std::fabs(
                      coordinates.distance
                          - info.latest_distance)
                      > 0.001F) {
        info.position_changed = true;
    }
    info.latest_azimuth = coordinates.azimuth_degrees;
    info.latest_elevation = coordinates.elevation_degrees;
    info.latest_distance = coordinates.distance;
}

std::string format_probe_waveform_types(
    const std::set<std::uint8_t>& types) {
    if (types.empty()) {
        return "none";
    }
    std::ostringstream output;
    bool first = true;
    for (const std::uint8_t type : types) {
        if (!first) {
            output << ", ";
        }
        output << static_cast<unsigned>(type);
        first = false;
    }
    return output.str();
}

std::string format_probe_position(const ProbeObjectInfo& object) {
    if (!object.position_available) {
        return "not present in probe window";
    }
    std::ostringstream output;
    output << std::fixed << std::setprecision(1)
           << "azimuth " << object.latest_azimuth
           << " deg, elevation " << object.latest_elevation
           << " deg, distance " << object.latest_distance;
    return output.str();
}

template <typename Value>
void print_probe_field(
    std::string_view name,
    const Value& value) {
    std::cout << "  " << std::left << std::setw(30)
              << std::setfill(' ') << name << ": "
              << value << '\n';
}

void print_probe_section(std::string_view name, bool color) {
    std::cout << '\n';
    console_style::paint(
        std::cout, color, console_style::bold);
    console_style::paint(
        std::cout, color, console_style::bright_cyan);
    std::cout << name;
    console_style::reset(std::cout, color);
    std::cout << '\n';
}

template <typename Value>
void print_probe_highlighted_field(
    std::string_view name,
    const Value& value,
    bool color) {
    std::cout << "  " << std::left << std::setw(30)
              << std::setfill(' ') << name << ": ";
    console_style::paint(
        std::cout, color, console_style::bold);
    console_style::paint(
        std::cout, color, console_style::bright_magenta);
    std::cout << value;
    console_style::reset(std::cout, color);
    std::cout << '\n';
}

std::uint32_t layout_speaker_activity_mask(
    const ChannelLayout& layout) noexcept {
    std::uint32_t physical_mask = 0U;
    for (const std::string& channel : layout.channels) {
        std::uint32_t speaker_mask = 0U;
        if (!dtsx::standard_speaker_mask(
                channel, speaker_mask)) {
            return 0U;
        }
        physical_mask |= speaker_mask;
    }
    return dtsx::speaker_mask_to_activity_mask(
        physical_mask);
}

std::vector<dtsx::XllLossyBaseChannel> make_xll_lossy_base(
    const DcaDecodedBed& core) {
    std::vector<dtsx::XllLossyBaseChannel> result;
    if (core.channels.size()
        != core.channel_speaker_masks.size()) {
        return result;
    }
    result.reserve(core.channels.size());
    for (std::size_t channel = 0U;
         channel < core.channels.size();
         ++channel) {
        result.push_back({
            core.channel_speaker_masks[channel],
            &core.channels[channel],
        });
    }
    return result;
}

std::filesystem::path generated_output_path(
    const std::filesystem::path& input, const ChannelLayout& layout) {
    std::string safe_layout = layout.name;
    for (char& c : safe_layout) {
        if (c == '(' || c == ')') {
            c = '_';
        }
    }
    return input.parent_path() / (input.stem().wstring() + L"_"
        + std::wstring(safe_layout.begin(), safe_layout.end()) + L".wav");
}

void run_internal_probe(
    const Options& options,
    const AudioProbe& audio_probe,
    const ChannelLayout* layout) {
    constexpr std::uint64_t kQuickProbeExtensionFrames = 64U;
    constexpr std::uint64_t kQuickProbeElementaryFrames = 256U;

    ProgressReporter progress;
    progress.update("probe", -1);
    DtsFrameReader reader(options);
    const bool probe_is_demuxer_bounded = reader.container_input();
    ObjectFrameDecoder object_decoder;
    DcaBedDecoder dca_bed_decoder;
    dtsx::ElementaryFrame frame;
    std::uint64_t frame_count = 0U;
    std::uint64_t core_frame_count = 0U;
    std::uint64_t extension_frame_count = 0U;
    std::uint64_t uhd_frame_count = 0U;
    std::uint64_t xll_asset_count = 0U;
    std::uint64_t xll_hierarchical_channel_set_count = 0U;
    std::uint64_t xll_downmix_channel_set_count = 0U;
    std::uint32_t maximum_xll_channel_sets_per_asset = 0U;
    std::uint64_t object_audio_asset_count = 0U;
    std::uint64_t malformed_object_frame_count = 0U;
    std::uint64_t xll_pbr_fallback_frame_count = 0U;
    std::uint32_t maximum_supplemental_xll_channels = 0U;
    std::uint32_t maximum_object_waveform_channels = 0U;
    std::uint64_t metadata_chunk_count = 0U;
    std::uint64_t metadata_body_parse_failure_count = 0U;
    std::uint64_t raw_metadata_envelope_count = 0U;
    std::uint64_t raw_metadata_crc_failure_count = 0U;
    std::size_t maximum_object_count = 0U;
    std::set<std::uint32_t> object_ids;
    std::uint32_t detected_frame_duration = 0U;
    std::uint32_t detected_object_sample_rate = 0U;
    std::uint32_t detected_stream_sample_rate = 0U;
    std::uint32_t detected_bed_activity_mask = 0U;
    std::uint32_t detected_metadata_activity_mask = 0U;
    std::uint32_t detected_supplemental_activity_mask = 0U;
    bool detected_imax_enhanced = false;
    std::uint32_t detected_dtsx_extension_sync_word = 0U;
    std::uint32_t ignored_unmapped_objects = 0U;
    std::uint32_t detected_uhd_audio_chunk_id = 0U;
    std::map<std::uint32_t, ProbeObjectInfo> probe_objects;
    std::vector<std::vector<dtsx::XllChannelSetHeader>>
        probe_previous_xll_headers;
    dtsx::UhdFrameParserState probe_uhd_state;

    while (reader.read(frame)) {
        ++frame_count;
        if (frame.packing == dtsx::StreamPacking::DtsUhd) {
            dtsx::UhdFrameHeader uhd;
            const dtsx::UhdHeaderParseResult result =
                dtsx::parse_uhd_frame_header(
                    frame.bytes, probe_uhd_state, uhd);
            if (result != dtsx::UhdHeaderParseResult::Complete) {
                ++malformed_object_frame_count;
                continue;
            }
            ++uhd_frame_count;
            detected_frame_duration = uhd.samples_per_channel;
            detected_stream_sample_rate = uhd.sample_rate;
            metadata_chunk_count += uhd.metadata_chunks.size();
            if (!uhd.audio_chunks.empty()) {
                detected_uhd_audio_chunk_id =
                    uhd.audio_chunks.front().id;
            }
            if (!uhd.metadata_chunks.empty()
                && dtsx::parse_uhd_full_mix_metadata(
                    frame.bytes, uhd)) {
                detected_bed_activity_mask =
                    uhd.speaker_activity_mask;
            }
        } else if (frame.packing == dtsx::StreamPacking::ExtensionBigEndian
            || frame.packing == dtsx::StreamPacking::ExtensionLittleEndian) {
            ++extension_frame_count;
            DcaDecodedBed decoded_bed;
            const bool internal_bed_decoded =
                dca_bed_decoder.decode_extension(frame, decoded_bed);
            if (internal_bed_decoded) {
                detected_bed_activity_mask =
                    decoded_bed.speaker_activity_mask;
                if (detected_stream_sample_rate == 0U) {
                    detected_stream_sample_rate =
                        decoded_bed.sample_rate;
                }
            }
            const bool swap =
                frame.packing == dtsx::StreamPacking::ExtensionBigEndian;
            dtsx::bitstream::WordBuffer words(frame.bytes, swap);
            dtsx::ExssHeader header;
            dtsx::bitstream::Cursor header_source = words.cursor();
            if (!dtsx::unpack_exss_header(header_source, header)) {
                ++malformed_object_frame_count;
                continue;
            }
            detected_frame_duration = header.frame_duration != 0U
                ? header.frame_duration
                : detected_frame_duration;
            bool asset_parse_failed = false;
            for (std::uint32_t asset_index = 0U;
                 asset_index < header.asset_count
                 && asset_index < header.asset_header_bit_offsets.size();
                 ++asset_index) {
                dtsx::bitstream::Cursor asset_source = words.cursor();
                asset_source.fast_forward(static_cast<std::int32_t>(
                    header.asset_header_bit_offsets[asset_index]));
                dtsx::ExssAssetSummary asset;
                if (!dtsx::unpack_exss_asset_summary(
                        asset_source, header, asset, asset_index)) {
                    ++malformed_object_frame_count;
                    asset_parse_failed = true;
                    break;
                }
                if ((asset.coding_components & (1U << 9U)) != 0U) {
                    ++xll_asset_count;
                    if (asset.xll_metadata_present
                        && !asset.xll_metadata_chunk_sizes.empty()) {
                        ++metadata_chunk_count;
                    }
                    std::uint32_t asset_payload_offset =
                        header.header_size;
                    for (std::uint32_t previous = 0U;
                         previous < asset_index;
                         ++previous) {
                        asset_payload_offset +=
                            header.asset_sizes[previous];
                    }
                    dtsx::bitstream::Cursor xll_source =
                        words.cursor();
                    xll_source.fast_forward(
                        static_cast<std::int32_t>(
                            8U * (asset_payload_offset
                                + asset.component_byte_offsets[9U])));
                    xll_source = xll_source.limited(
                        8U * asset.component_size_bytes[9U]);
                    dtsx::XllCommonHeader xll;
                    if (dtsx::unpack_xll_common_header(
                            xll_source, xll)) {
                        maximum_xll_channel_sets_per_asset =
                            (std::max)(
                                maximum_xll_channel_sets_per_asset,
                                 static_cast<std::uint32_t>(
                                     xll.channel_set_count));
                        if (asset.asset_index
                            >= probe_previous_xll_headers.size()) {
                            probe_previous_xll_headers.resize(
                                static_cast<std::size_t>(
                                    asset.asset_index)
                                + 1U);
                        }
                        const auto& previous_raw_headers =
                            probe_previous_xll_headers[
                                asset.asset_index];
                        std::vector<dtsx::XllChannelSetHeader>
                            current_raw_headers(
                                xll.channel_set_count);
                        bool all_headers_parsed = true;
                        std::uint32_t preceding_channels = 0U;
                        for (std::uint32_t channel_set = 0U;
                             channel_set < xll.channel_set_count;
                             ++channel_set) {
                            dtsx::XllChannelSetHeader set_header;
                            const dtsx::XllChannelSetHeader*
                                previous_header =
                                    channel_set
                                            < previous_raw_headers.size()
                                        ? &previous_raw_headers[channel_set]
                                        : nullptr;
                            if (!dtsx::unpack_xll_primary_channel_set_header(
                                    xll_source,
                                    xll,
                                    set_header,
                                    asset.one_to_one_mapping,
                                    preceding_channels,
                                    previous_header)) {
                                all_headers_parsed = false;
                                break;
                            }
                            current_raw_headers[channel_set] =
                                set_header;
                            const bool selected_replacement_set =
                                set_header.probe
                                        .replacement_set_index == 0U
                                || set_header.probe
                                       .default_replacement_set;
                            if (!selected_replacement_set) {
                                continue;
                            }
                            if (set_header.hierarchical_channel_set) {
                                ++xll_hierarchical_channel_set_count;
                                preceding_channels +=
                                    set_header.probe.channel_count;
                            }
                            if (set_header
                                    .downmix_coefficients_present) {
                                    ++xll_downmix_channel_set_count;
                            }
                        }
                        if (all_headers_parsed) {
                            probe_previous_xll_headers[
                                asset.asset_index] =
                                    std::move(current_raw_headers);
                        }
                    }
                }
                if (asset.object_audio_type == 0U
                    && asset.speaker_mask_present) {
                    detected_bed_activity_mask =
                        asset.speaker_activity_mask;
                }
                if (asset.sample_rate != 0U
                    && detected_stream_sample_rate == 0U) {
                    detected_stream_sample_rate = asset.sample_rate;
                }
                const bool object_audio_asset =
                    (asset_index < header.asset_object_audio.size()
                        && header.asset_object_audio[asset_index] != 0U)
                    || asset.object_audio_type == 1U;
                if (object_audio_asset) {
                    ++object_audio_asset_count;
                }
            }
            if (asset_parse_failed) {
                continue;
            }

            metadata_chunk_count += dtsx::scan_metadata_chunks(
                words.cursor(),
                static_cast<std::uint32_t>(frame.bytes.size()),
                static_cast<std::uint8_t>(header.asset_count)).size();

            DecodedObjectAudioFrame decoded;
            const auto lossy_base = make_xll_lossy_base(
                dca_bed_decoder.decoded_core());
            const ObjectFrameDecodeResult result =
                object_decoder.decode(
                    frame, decoded, lossy_base);
            if (result == ObjectFrameDecodeResult::Malformed) {
                if (options.verbose
                    && malformed_object_frame_count == 0U
                    && xll_pbr_fallback_frame_count == 0U) {
                    std::cerr << "DTS:X frame "
                              << frame_count
                              << ": "
                              << object_decoder.last_error()
                              << '\n';
                }
                if (internal_bed_decoded) {
                    ++xll_pbr_fallback_frame_count;
                    continue;
                }
                ++malformed_object_frame_count;
                continue;
            }
            if (!decoded.waveform_channels.empty()) {
                const std::uint32_t supplemental_channels =
                    static_cast<std::uint32_t>(std::count(
                        decoded.waveform_is_supplemental.begin(),
                        decoded.waveform_is_supplemental.end(),
                        true));
                maximum_supplemental_xll_channels =
                    (std::max)(
                        maximum_supplemental_xll_channels,
                        supplemental_channels);
                maximum_object_waveform_channels =
                    (std::max)(
                        maximum_object_waveform_channels,
                        static_cast<std::uint32_t>(
                            decoded.waveform_channels.size())
                            - supplemental_channels);
            }
            if (decoded.sample_rate != 0U) {
                detected_object_sample_rate = decoded.sample_rate;
                if (detected_stream_sample_rate == 0U) {
                    detected_stream_sample_rate = decoded.sample_rate;
                }
            }
            detected_imax_enhanced =
                detected_imax_enhanced
                || decoded.imax_enhanced;
            if (decoded.dtsx_extension_sync_word != 0U) {
                detected_dtsx_extension_sync_word =
                    decoded.dtsx_extension_sync_word;
            }
            ignored_unmapped_objects = (std::max)(
                ignored_unmapped_objects,
                decoded.ignored_unmapped_objects);
            if (decoded.metadata_speaker_activity_mask != 0U) {
                detected_metadata_activity_mask =
                    decoded.metadata_speaker_activity_mask;
            }
            for (std::size_t waveform = 0U;
                 waveform < decoded.waveform_is_supplemental.size()
                 && waveform < decoded.waveform_speaker_masks.size();
                 ++waveform) {
                if (decoded.waveform_is_supplemental[waveform]) {
                    detected_supplemental_activity_mask |=
                        dtsx::speaker_mask_to_activity_mask(
                            decoded.waveform_speaker_masks[waveform]);
                }
            }
            metadata_body_parse_failure_count +=
                decoded.metadata_body_parse_failures;
            raw_metadata_envelope_count +=
                decoded.raw_metadata_envelopes;
            raw_metadata_crc_failure_count +=
                decoded.raw_metadata_crc_failures;
            if (!decoded.objects.empty()) {
                maximum_object_count = (std::max)(
                    maximum_object_count, decoded.objects.size());
                for (std::size_t object_index = 0U;
                     object_index < decoded.objects.size();
                     ++object_index) {
                    const dtsx::ObjectMetadataBlock& object =
                        decoded.objects[object_index];
                    const std::uint32_t object_id =
                        object.object_id_available
                            ? object.object_id
                            : static_cast<std::uint32_t>(
                                  object_index);
                    object_ids.insert(object_id);
                    const std::uint32_t key =
                        object.object_id_available
                            ? object.object_id
                            : 0x80000000U
                                | static_cast<std::uint32_t>(
                                      object_index);
                    update_probe_object_info(
                        probe_objects[key],
                        object,
                        static_cast<std::uint32_t>(
                            object_index));
                }
            }
        } else if (frame.packing == dtsx::StreamPacking::Core16BitBigEndian
                   || frame.packing
                       == dtsx::StreamPacking::Core16BitLittleEndian
                   || frame.packing
                       == dtsx::StreamPacking::Core14BitBigEndian
                   || frame.packing
                       == dtsx::StreamPacking::Core14BitLittleEndian) {
            ++core_frame_count;
            dca_bed_decoder.remember_core(frame);
        }
        if (!options.full_probe
            && !probe_is_demuxer_bounded
            && (extension_frame_count >= kQuickProbeExtensionFrames
                || frame_count >= kQuickProbeElementaryFrames)) {
            break;
        }
    }

    progress.done("probe");

    const bool color = console_style::color_enabled(stdout);
    const std::uint32_t sample_rate =
        audio_probe.sample_rate != 0U
            ? audio_probe.sample_rate
            : detected_stream_sample_rate;
    const std::uint64_t scanned_samples =
        (extension_frame_count + uhd_frame_count)
        * detected_frame_duration;

    console_style::paint(
        std::cout, color, console_style::bold);
    console_style::paint(
        std::cout, color, console_style::white);
    std::cout << (options.full_probe
        ? "DTS:X full stream probe"
        : "DTS:X stream probe");
    console_style::reset(std::cout, color);
    std::cout << '\n';

    print_probe_section("Stream", color);
    print_probe_field(
        "Audio track",
        std::to_string(audio_probe.stream_index)
            + (options.audio_track_explicit
                   ? " (explicit)"
                   : " (auto)"));
    std::string coding = uhd_frame_count != 0U
        ? "DTS-UHD/ACE"
        : core_frame_count != 0U
            ? "DTS Core"
            : "extension-only DTS";
    if (xll_asset_count != 0U) {
        coding += " + DTS-HD MA/XLL";
    }
    if (metadata_chunk_count != 0U
        || raw_metadata_envelope_count != 0U) {
        coding += " + DTS:X metadata";
    }
    if (detected_imax_enhanced) {
        coding += " + IMAX Enhanced";
    }
    print_probe_field("Coding", coding);
    print_probe_field(
        "Profile",
        uhd_frame_count != 0U
            ? "DTS:X Profile 2"
            : detected_imax_enhanced
            ? "IMAX Enhanced"
            : metadata_chunk_count != 0U
                  || raw_metadata_envelope_count != 0U
                  || detected_dtsx_extension_sync_word != 0U
            ? "DTS:X"
            : xll_asset_count != 0U
            ? "DTS-HD Master Audio"
            : "DTS");
    if (uhd_frame_count != 0U) {
        print_probe_field(
            "Audio chunk",
            probe_hex(detected_uhd_audio_chunk_id));
        print_probe_field(
            "Full channel-based mix",
            "yes");
    }
    if (detected_dtsx_extension_sync_word != 0U) {
        print_probe_field(
            "DTS:X extension sync",
            probe_hex(detected_dtsx_extension_sync_word));
    }
    if (detected_imax_enhanced) {
        print_probe_field(
            "IMAX metadata version",
            "not exposed by the recognized D0 header");
    }
    print_probe_field("Sample rate", sample_rate);
    print_probe_field(
        options.full_probe
            ? "Scanned duration"
            : "Quick probe window",
        format_probe_duration(scanned_samples, sample_rate));
    print_probe_field("Scanned frames", frame_count);
    if (detected_frame_duration != 0U) {
        print_probe_field(
            "Frame duration",
            detected_frame_duration);
    }

    print_probe_section("Layouts", color);
    print_probe_field(
        "Coded bed",
        format_probe_speaker_layout(detected_bed_activity_mask));
    print_probe_field(
        "Coded bed mask",
        probe_hex(detected_bed_activity_mask));
    if (detected_metadata_activity_mask != 0U) {
        print_probe_field(
            "Metadata reference layout",
            format_probe_speaker_layout(
                detected_metadata_activity_mask));
        print_probe_field(
            "Metadata reference mask",
            probe_hex(detected_metadata_activity_mask));
    }
    if (maximum_supplemental_xll_channels != 0U) {
        print_probe_field(
            "Supplemental XLL channels",
            maximum_supplemental_xll_channels);
        print_probe_field(
            "Supplemental layout",
            detected_supplemental_activity_mask != 0U
                ? format_probe_speaker_layout(
                      detected_supplemental_activity_mask)
                : "private upper layer; positions supplied by DTS:X metadata");
    }
    print_probe_field(
        "Height in coded bed",
        dtsx::has_height_channels(detected_bed_activity_mask)
            ? "yes"
            : "no");
    print_probe_field(
        "XLL channel-set structure",
        xll_hierarchical_channel_set_count != 0U
            ? "hierarchical"
            : "independent");
    print_probe_field(
        "Embedded downmix",
        xll_downmix_channel_set_count != 0U
            ? "present"
            : "not declared");
    print_probe_field(
        "Maximum sets per asset",
        maximum_xll_channel_sets_per_asset);

    print_probe_section("Dynamic objects", color);
    print_probe_highlighted_field(
        "Dynamic objects", object_ids.size(), color);
    if (ignored_unmapped_objects != 0U) {
        print_probe_field(
            "Ignored service/unmapped records",
            ignored_unmapped_objects);
    }
    print_probe_field(
        "Maximum objects per frame",
        maximum_object_count);
    print_probe_field(
        "Object waveform channels",
        maximum_object_waveform_channels);
    print_probe_field(
        "Dedicated object asset",
        object_audio_asset_count != 0U
            ? "declared"
            : maximum_object_waveform_channels != 0U
                ? "not declared; objects use XLL waveform channels"
                : "not declared");
    if (detected_object_sample_rate != 0U) {
        print_probe_field(
            "Object sample rate",
            detected_object_sample_rate);
    }
    for (const auto& entry : probe_objects) {
        const ProbeObjectInfo& object = entry.second;
        std::ostringstream title;
        title << "Object ";
        if (object.id_available) {
            title << "ID " << object.id;
        } else {
            title << "index " << object.id << " (ID unavailable)";
        }
        print_probe_section(title.str(), color);
        print_probe_field(
            "Metadata mode",
            static_cast<unsigned>(object.metadata_mode));
        print_probe_field(
            "Waveforms",
            object.waveform_count);
        print_probe_field(
            "Waveform types",
            format_probe_waveform_types(
                object.waveform_types));
        print_probe_field(
            "Point sources",
            object.maximum_point_sources);
        if (object.group_present) {
            print_probe_field(
                "Group",
                static_cast<unsigned>(object.group));
        }
        if (object.spatial_group_present) {
            print_probe_field(
                "Spatial group",
                static_cast<unsigned>(
                    object.spatial_group));
        }
        print_probe_field(
            "Last position",
            format_probe_position(object));
        print_probe_field(
            "Motion in probe window",
            object.position_changed
                ? "observed"
                : "not observed");
    }

    const std::uint64_t probe_failures =
        malformed_object_frame_count
        + metadata_body_parse_failure_count
        + raw_metadata_crc_failure_count
        + xll_pbr_fallback_frame_count;
    if (probe_failures != 0U) {
        print_probe_section("Probe warnings", color);
        print_probe_field(
            "Malformed object frames",
            malformed_object_frame_count);
        print_probe_field(
            "Metadata body failures",
            metadata_body_parse_failure_count);
        print_probe_field(
            "Raw metadata CRC failures",
            raw_metadata_crc_failure_count);
        print_probe_field(
            "XLL PBR fallback frames",
            xll_pbr_fallback_frame_count);
    }

    if (layout != nullptr) {
        ParmaLayoutControls parma;
        const std::uint32_t output_activity_mask =
            layout_speaker_activity_mask(*layout);
        const bool parma_supported = derive_parma_layout_controls(
            detected_bed_activity_mask,
            output_activity_mask,
            parma);
        print_probe_section("Renderer", color);
        print_probe_field(
            "PARMA input mask",
            probe_hex(parma.input_channel_mask));
        print_probe_field(
            "PARMA output mask",
            probe_hex(parma.output_channel_mask));
        print_probe_field(
            "PARMA blind table row",
            parma.blind_table_row);
        print_probe_field(
            "PARMA blind layers",
            parma_supported
                ? parma_blind_layer_count(parma)
                : 0U);
        print_probe_field(
            "PARMA supported",
            parma_supported ? "yes" : "no");
        print_probe_field(
            "PARMA required",
            parma_supported
                    && !parma.output_is_horizontal
                    && dtsx::has_height_channels(output_activity_mask)
                    && !dtsx::has_height_channels(
                        detected_bed_activity_mask)
                ? "yes"
                : "no");
    }
    std::cout << std::flush;
}

void dump_metadata(const Options& options, const std::filesystem::path& path) {
    if (std::filesystem::exists(path) && !options.overwrite) {
        throw std::runtime_error(
            "metadata output exists; pass --overwrite to replace it");
    }
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("cannot open metadata output");
    }
    DtsFrameReader reader(options);
    std::unique_ptr<ObjectSidecarWriter> coordinate_output;
    if (options.coordinates_output_explicit) {
        coordinate_output = std::make_unique<ObjectSidecarWriter>(
            options.coordinates_output, options.overwrite);
    }
    dtsx::ElementaryFrame frame;
    std::uint64_t frame_index = 0;
    std::uint64_t metadata_sample_position = 0;
    std::uint32_t metadata_frame_duration = 0U;
    std::uint32_t metadata_sample_rate = 0U;
    std::vector<dtsx::XllFrameDecoder> detailed_xll_decoders;
    std::vector<std::vector<dtsx::XllChannelSetHeader>>
        detailed_previous_xll_headers;
    ObjectFrameDecoder coordinate_decoder;
    DcaBedDecoder coordinate_bed_decoder;
    while (reader.read(frame)) {
        output << "{\"frameIndex\":" << frame_index++
               << ",\"streamOffset\":" << frame.stream_offset
               << ",\"packing\":"
               << static_cast<unsigned>(frame.packing);
        if (frame.packing == dtsx::StreamPacking::ExtensionBigEndian
            || frame.packing == dtsx::StreamPacking::ExtensionLittleEndian) {
            const bool swap =
                frame.packing == dtsx::StreamPacking::ExtensionBigEndian;
            dtsx::bitstream::WordBuffer words(frame.bytes, swap);
            dtsx::bitstream::Cursor cursor = words.cursor();
            dtsx::ExssHeader header;
            const bool crc_valid = dtsx::validate_exss_frame(cursor);
            cursor = words.cursor();
            if (dtsx::unpack_exss_header(cursor, header)) {
                if (header.frame_duration != 0U) {
                    metadata_frame_duration =
                        header.frame_duration;
                }
                const std::vector<dtsx::MetadataChunkLocation> chunks =
                    dtsx::scan_metadata_chunks(
                        words.cursor(),
                        static_cast<std::uint32_t>(frame.bytes.size()),
                        static_cast<std::uint8_t>(header.asset_count));
                output << ",\"exss\":{\"streamIndex\":"
                       << static_cast<unsigned>(header.stream_index)
                       << ",\"headerSize\":" << header.header_size
                       << ",\"frameSize\":" << header.frame_size
                       << ",\"assetCount\":" << header.asset_count
                       << ",\"waveformCount\":" << header.waveform_count
                       << ",\"selectedWaveform\":"
                       << static_cast<unsigned>(
                              header.selected_waveform)
                       << ",\"waveformMetadataMode\":"
                       << static_cast<unsigned>(
                              header.waveform_metadata_mode)
                       << ",\"waveformMetadataType\":"
                       << static_cast<unsigned>(
                              header.waveform_metadata_type)
                       << ",\"totalAssetBytes\":"
                       << header.total_asset_bytes
                       << ",\"crcValid\":" << (crc_valid ? "true" : "false")
                       << ",\"speakerMaskPresent\":"
                       << static_cast<unsigned>(header.speaker_metadata_present)
                       << ",\"assets\":[";
                std::vector<std::uint8_t>
                    xll_waveform_decoder_ids;
                for (std::uint32_t asset_index = 0;
                     asset_index < header.asset_count
                     && asset_index < header.asset_header_bit_offsets.size();
                     ++asset_index) {
                    if (asset_index != 0U) {
                        output << ',';
                    }
                    dtsx::bitstream::Cursor asset_source = words.cursor();
                    asset_source.fast_forward(static_cast<std::int32_t>(
                        header.asset_header_bit_offsets[asset_index]));
                    dtsx::ExssAssetSummary asset;
                    const bool parsed = dtsx::unpack_exss_asset_summary(
                        asset_source,
                        header,
                        asset,
                        asset_index);
                    std::uint32_t asset_payload_offset = header.header_size;
                    for (std::uint32_t previous = 0;
                         previous < asset_index;
                         ++previous) {
                        asset_payload_offset += header.asset_sizes[previous];
                    }
                    output << "{\"index\":" << asset_index
                           << ",\"headerBitOffset\":"
                           << header.asset_header_bit_offsets[asset_index]
                           << ",\"parsed\":"
                           << (parsed ? "true" : "false")
                           << ",\"headerBytes\":" << asset.header_size
                           << ",\"assetIndex\":"
                           << static_cast<unsigned>(asset.asset_index)
                           << ",\"objectAudioType\":"
                           << static_cast<unsigned>(
                                  (asset.object_audio_type != 0U
                                       ? asset.object_audio_type
                                       : (asset_index < header.asset_object_audio.size()
                                              ? header.asset_object_audio[asset_index]
                                              : 0U)))
                           << ",\"sampleRate\":" << asset.sample_rate
                           << ",\"channels\":" << asset.channel_count
                           << ",\"oneToOneMapping\":"
                           << (asset.one_to_one_mapping ? "true" : "false")
                           << ",\"speakerMask\":"
                           << asset.speaker_activity_mask
                           << ",\"codingModeAvailable\":"
                           << (asset.coding_mode_available
                                   ? "true" : "false")
                           << ",\"codingMode\":"
                           << static_cast<unsigned>(asset.coding_mode)
                           << ",\"codingComponents\":"
                           << asset.coding_components
                           << ",\"xllSyncPresent\":"
                           << (asset.xll_sync_present ? "true" : "false")
                           << ",\"xllSyncOffset\":"
                           << asset.xll_sync_offset
                           << ",\"xllSmoothingBufferKbytes\":"
                           << static_cast<unsigned>(
                                  asset.xll_smoothing_buffer_kbytes)
                           << ",\"xllInitialDelayBits\":"
                           << static_cast<unsigned>(
                                  asset.xll_initial_delay_bits)
                           << ",\"xllInitialDelayFrames\":"
                           << asset.xll_initial_delay_frames
                           << ",\"dtsHdStreamId\":"
                           << static_cast<unsigned>(
                                  asset.dts_hd_stream_id)
                           << ",\"xllMetadataPresent\":"
                           << (asset.xll_metadata_present
                                   ? "true" : "false")
                           << ",\"xllObjectMetadataPresent\":"
                           << (asset.xll_object_metadata_present
                                   ? "true" : "false")
                           << ",\"xllMetadataOffset\":"
                           << asset.xll_metadata_offset
                           << ",\"xllNavigationBitOffset\":"
                           << asset.xll_navigation_bit_offset
                           << ",\"xllObjectSizesBitOffset\":"
                           << asset.xll_object_sizes_bit_offset
                           << ",\"descriptorBitsUsed\":"
                           << asset.descriptor_bits_used
                           << ",\"xllMetadataChunkSizes\":[";
                    for (std::size_t index = 0U;
                         index < asset.xll_metadata_chunk_sizes.size();
                         ++index) {
                        if (index != 0U) {
                            output << ',';
                        }
                        output << static_cast<unsigned>(
                            asset.xll_metadata_chunk_sizes[index]);
                    }
                    output << "],\"xllAssociatedChunkTypes\":[";
                    for (std::size_t index = 0U;
                         index < asset.xll_associated_chunk_types.size();
                         ++index) {
                        if (index != 0U) {
                            output << ',';
                        }
                        output << static_cast<unsigned>(
                            asset.xll_associated_chunk_types[index]);
                    }
                    output << "],\"xllAssociatedChunkExtents\":[";
                    for (std::size_t index = 0U;
                         index < asset.xll_associated_chunk_extents.size();
                         ++index) {
                        if (index != 0U) {
                            output << ',';
                        }
                        output << asset.xll_associated_chunk_extents[index];
                    }
                    output << ']';
                    if ((asset.coding_components & (1U << 9U)) != 0U) {
                        xll_waveform_decoder_ids.push_back(
                            asset.asset_index);
                        dtsx::bitstream::Cursor xll_source = words.cursor();
                        xll_source.fast_forward(static_cast<std::int32_t>(
                            8U * (asset_payload_offset
                                + asset.component_byte_offsets[9U])));
                        const dtsx::bitstream::Cursor xll_frame_start =
                            xll_source;
                        dtsx::XllCommonHeader xll;
                        const bool xll_parsed =
                            dtsx::unpack_xll_common_header(xll_source, xll);
                        output << ",\"xll\":{\"parsed\":"
                               << (xll_parsed ? "true" : "false")
                               << ",\"headerSize\":" << xll.header_size
                               << ",\"frameSize\":" << xll.frame_size
                               << ",\"channelSets\":"
                               << static_cast<unsigned>(
                                      xll.channel_set_count)
                               << ",\"segments\":"
                               << xll.segments_per_frame
                                   << ",\"samplesPerSegment\":"
                                   << xll.samples_per_segment
                                   << ",\"segmentSizeBits\":"
                                   << static_cast<unsigned>(
                                          xll.segment_size_bits)
                                   << ",\"bandCrcPresent\":"
                                   << static_cast<unsigned>(
                                          xll.band_crc_present)
                                   << ",\"scalableLsb\":"
                                   << (xll.scalable_lsb
                                           ? "true" : "false")
                                   << ",\"scalableResolution\":"
                                   << static_cast<unsigned>(
                                          xll.scalable_resolution)
                                   << ",\"crcValid\":"
                               << (xll.crc_valid ? "true" : "false")
                               << ",\"channelSetHeaders\":[";
                        std::vector<std::uint8_t> xll_band_counts;
                        bool xll_headers_complete = xll_parsed;
                        bool all_full_headers_parsed = xll_parsed;
                        std::vector<dtsx::XllChannelSetHeader>
                            current_raw_headers(xll.channel_set_count);
                        if (asset.asset_index
                            >= detailed_previous_xll_headers.size()) {
                            detailed_previous_xll_headers.resize(
                                static_cast<std::size_t>(
                                    asset.asset_index)
                                + 1U);
                        }
                        const auto& previous_raw_headers =
                            detailed_previous_xll_headers[
                                asset.asset_index];
                        std::uint32_t
                            preceding_xll_hierarchy_channels = 0U;
                        for (std::uint32_t channel_set = 0;
                             xll_parsed
                             && channel_set < xll.channel_set_count;
                             ++channel_set) {
                            if (channel_set != 0U) {
                                output << ',';
                            }
                            const dtsx::bitstream::Cursor
                                channel_set_start = xll_source;
                            dtsx::XllChannelSetHeader
                                channel_set_header;
                            dtsx::bitstream::Cursor full_header_source =
                                channel_set_start;
                            const dtsx::XllChannelSetHeader*
                                previous_header =
                                    channel_set
                                            < previous_raw_headers.size()
                                        ? &previous_raw_headers[channel_set]
                                        : nullptr;
                            const bool full_header_parsed =
                                dtsx::unpack_xll_primary_channel_set_header(
                                    full_header_source,
                                    xll,
                                    channel_set_header,
                                    asset.one_to_one_mapping,
                                    preceding_xll_hierarchy_channels,
                                    previous_header);
                            dtsx::XllChannelSetProbe channel_set_probe;
                            bool channel_set_parsed = false;
                            if (full_header_parsed) {
                                current_raw_headers[channel_set] =
                                    channel_set_header;
                                channel_set_probe =
                                    channel_set_header.probe;
                                const bool selected_replacement_set =
                                    channel_set_probe
                                            .replacement_set_index == 0U
                                    || channel_set_probe
                                           .default_replacement_set;
                                if (selected_replacement_set
                                    && channel_set_header
                                           .hierarchical_channel_set) {
                                    preceding_xll_hierarchy_channels +=
                                        channel_set_probe.channel_count;
                                }
                                if (selected_replacement_set) {
                                    xll_band_counts.push_back(
                                        channel_set_probe
                                            .frequency_band_count);
                                }
                                xll_source = full_header_source;
                                channel_set_parsed = true;
                            } else {
                                all_full_headers_parsed = false;
                                dtsx::bitstream::Cursor probe_source =
                                    channel_set_start;
                                channel_set_parsed =
                                    dtsx::probe_xll_channel_set_header(
                                        probe_source,
                                         false,
                                         channel_set_probe,
                                         xll.channel_set_count,
                                         xll.legacy_sync);
                                xll_source = probe_source;
                            }
                            xll_headers_complete =
                                xll_headers_complete && channel_set_parsed
                                && channel_set_probe
                                       .frequency_band_count_available;
                            metadata_sample_rate = (std::max)(
                                metadata_sample_rate,
                                channel_set_probe.sample_rate);
                            output << "{\"parsed\":"
                                   << (channel_set_parsed
                                           ? "true"
                                           : "false")
                                   << ",\"headerSize\":"
                                   << channel_set_probe.header_size
                                   << ",\"channels\":"
                                   << static_cast<unsigned>(
                                          channel_set_probe.channel_count)
                                   << ",\"channelMask\":"
                                   << channel_set_probe.channel_mask
                                   << ",\"bitDepth\":"
                                   << static_cast<unsigned>(
                                          channel_set_probe.bit_depth)
                                   << ",\"storageBitDepth\":"
                                   << static_cast<unsigned>(
                                          channel_set_probe.storage_bit_depth)
                                   << ",\"sampleRate\":"
                                   << channel_set_probe.sample_rate
                                   << ",\"frequencyRatio\":"
                                          << static_cast<unsigned>(
                                          channel_set_probe
                                              .frequency_ratio)
                                    << ",\"replacementSetIndex\":"
                                   << static_cast<unsigned>(
                                          channel_set_probe
                                               .replacement_set_index)
                                     << ",\"reusePreviousHeader\":"
                                     << (channel_set_probe
                                                 .reuse_previous_header
                                             ? "true"
                                             : "false")
                                    << ",\"defaultReplacementSet\":"
                                    << (channel_set_probe
                                                .default_replacement_set
                                            ? "true"
                                            : "false")
                                    << ",\"selected\":"
                                    << (channel_set_probe
                                                    .replacement_set_index
                                                == 0U
                                            || channel_set_probe
                                                   .default_replacement_set
                                        ? "true"
                                        : "false")
                                   << ",\"bandsAvailable\":"
                                   << (channel_set_probe
                                               .frequency_band_count_available
                                           ? "true"
                                           : "false")
                                   << ",\"bands\":"
                                   << static_cast<unsigned>(
                                          channel_set_probe
                                              .frequency_band_count)
                                   << ",\"crcValid\":"
                                   << (channel_set_probe.crc_valid
                                           ? "true"
                                           : "false")
                                   << ",\"fullHeaderParsed\":"
                                   << (full_header_parsed
                                           ? "true"
                                           : "false")
                                   << ",\"predictionBands\":"
                                    << channel_set_header.bands.size()
                                   << ",\"primary\":"
                                   << (channel_set_header
                                               .primary_channel_set
                                           ? "true"
                                           : "false")
                                   << ",\"embeddedDownmix\":"
                                   << (channel_set_header
                                               .embedded_downmix_present
                                           ? "true"
                                           : "false")
                                   << ",\"speakerChannelMask\":"
                                   << channel_set_header
                                          .speaker_channel_mask
                                   << ",\"channelOrders\":[";
                        for (std::size_t band_index = 0U;
                             band_index
                                 < channel_set_header.bands.size();
                             ++band_index) {
                            if (band_index != 0U) {
                                output << ',';
                            }
                            output << '[';
                            const auto& order =
                                channel_set_header.bands[band_index]
                                    .channel_order;
                            for (std::size_t channel = 0U;
                                 channel < order.size();
                                 ++channel) {
                                if (channel != 0U) {
                                    output << ',';
                                }
                                output << static_cast<unsigned>(
                                    order[channel]);
                            }
                            output << ']';
                        }
                        output << ']'
                               << ",\"bandParameters\":[";
                        for (std::size_t band_index = 0U;
                             band_index
                                 < channel_set_header.bands.size();
                             ++band_index) {
                            if (band_index != 0U) {
                                output << ',';
                            }
                            const auto& band =
                                channel_set_header.bands[band_index];
                            output << "{\"primarySizePresent\":"
                                   << (band.primary_size_present
                                           ? "true"
                                           : "false")
                                   << ",\"primarySize\":"
                                   << band.primary_size
                                   << ",\"primaryWidths\":[";
                            for (std::size_t channel = 0U;
                                 channel < band.primary_widths.size();
                                 ++channel) {
                                if (channel != 0U) {
                                    output << ',';
                                }
                                output << static_cast<unsigned>(
                                    band.primary_widths[channel]);
                            }
                            output << "],\"secondaryWidths\":[";
                            for (std::size_t channel = 0U;
                                 channel < band.secondary_widths.size();
                                 ++channel) {
                                if (channel != 0U) {
                                    output << ',';
                                }
                                output << static_cast<unsigned>(
                                    band.secondary_widths[channel]);
                            }
                            output << "]}";
                        }
                        output << "]}";
                        }
                        if (all_full_headers_parsed) {
                            detailed_previous_xll_headers[
                                asset.asset_index] =
                                    std::move(current_raw_headers);
                        }
                        xll_headers_complete =
                            xll_headers_complete
                            && !xll_band_counts.empty();
                        output << ']';
                        if (xll_headers_complete) {
                            dtsx::XllNavigationTable navigation;
                            const bool navigation_parsed =
                                dtsx::unpack_xll_navigation_table(
                                    xll_source,
                                    xll.segment_size_bits,
                                    xll.segments_per_frame,
                                    xll_band_counts,
                                    navigation);
                            output << ",\"navigation\":{\"parsed\":"
                                   << (navigation_parsed
                                           ? "true"
                                           : "false")
                                   << ",\"bytes\":" << navigation.byte_size
                                   << ",\"bands\":"
                                   << static_cast<unsigned>(
                                          navigation.band_count)
                                   << ",\"segments\":"
                                   << navigation.segment_count
                                   << ",\"crcValid\":"
                                   << (navigation.crc_valid
                                           ? "true"
                                           : "false");
                            std::uint64_t navigation_payload_bytes = 0U;
                            output << ",\"entries\":[";
                            for (std::size_t entry_index = 0U;
                                 entry_index < navigation.entries.size();
                                 ++entry_index) {
                                if (entry_index != 0U) {
                                    output << ',';
                                }
                                const dtsx::XllNavigationEntry& entry =
                                    navigation.entries[entry_index];
                                navigation_payload_bytes =
                                    (std::max)(
                                        navigation_payload_bytes,
                                        static_cast<std::uint64_t>(
                                            entry.byte_offset)
                                            + entry.size_bytes);
                                output << "{\"offset\":"
                                       << entry.byte_offset
                                       << ",\"bytes\":"
                                       << entry.size_bytes << '}';
                            }
                            output << ']'
                                   << ",\"payloadBytes\":"
                                   << navigation_payload_bytes
                                   << ",\"remainingPayloadBytes\":"
                                   << (xll_source.remaining_bits() / 8U)
                                   << '}';
                        }
                        if (asset.asset_index
                            >= detailed_xll_decoders.size()) {
                            detailed_xll_decoders.resize(
                                static_cast<std::size_t>(
                                    asset.asset_index)
                                + 1U);
                        }
                        dtsx::XllDecodedFrame decoded_xll_frame;
                        const bool msb_frame_decoded =
                            detailed_xll_decoders[asset.asset_index]
                                .decode_msb_frame(
                                xll_frame_start,
                                decoded_xll_frame,
                                asset.one_to_one_mapping);
                        output << ",\"msbFrameDecoded\":"
                               << (msb_frame_decoded
                                       ? "true"
                                       : "false")
                               << ",\"msbDecodeError\":\""
                               << detailed_xll_decoders[
                                      asset.asset_index]
                                      .last_error()
                               << "\""
                               << ",\"decodedChannels\":"
                               << decoded_xll_frame
                                      .planar_channels.size()
                               << ",\"decodedSamplesPerChannel\":"
                               << decoded_xll_frame
                                      .samples_per_channel
                               << ",\"extension\":{\"present\":"
                               << (decoded_xll_frame.extension.present
                                       ? "true"
                                       : "false")
                               << ",\"offset\":"
                               << decoded_xll_frame.extension.frame_offset
                               << ",\"sync\":"
                               << decoded_xll_frame.extension.sync_word
                               << ",\"payloadBytes\":"
                               << decoded_xll_frame.extension.payload.size()
                               << ",\"payloadHex\":\"";
                        static constexpr char kHex[] =
                            "0123456789abcdef";
                        for (std::uint8_t byte :
                             decoded_xll_frame.extension.payload) {
                            output << kHex[byte >> 4U]
                                   << kHex[byte & 0x0FU];
                        }
                        output << "\"}";
                        output << '}';
                    }
                    output << '}';
                }
                output << ']'
                       << ",\"metadataChunks\":[";
                for (std::size_t index = 0; index < chunks.size(); ++index) {
                    if (index != 0U) {
                        output << ',';
                    }
                    output << "{\"offset\":" << chunks[index].byte_offset
                           << ",\"elements\":"
                           << chunks[index].envelope.element_sizes.size()
                           << ",\"payloadBytes\":"
                           << chunks[index].envelope.element_payload_size
                           << ",\"crcValid\":"
                           << (chunks[index].envelope.crc_valid
                                   ? "true" : "false")
                           << ",\"chunkIds\":[";
                    const auto& elements = chunks[index].envelope.elements;
                    for (std::size_t element_index = 0;
                         element_index < elements.size();
                         ++element_index) {
                        if (element_index != 0U) {
                            output << ',';
                        }
                        const auto& element = elements[element_index];
                        output << "{\"id\":" << static_cast<unsigned>(
                                      element.chunk_id)
                               << ",\"primary\":"
                               << (element.primary ? "true" : "false")
                               << ",\"association\":"
                               << static_cast<unsigned>(
                                      element.association_index);
                        if (chunks[index].envelope.crc_valid
                            && element.chunk_id == 241U
                            && element.primary) {
                            const std::uint32_t element_byte_offset =
                                chunks[index].byte_offset + 5U
                                + static_cast<std::uint32_t>(
                                    chunks[index].envelope.element_sizes.size())
                                + element.byte_offset + 2U;
                            dtsx::bitstream::Cursor presentation_source =
                                words.cursor();
                            presentation_source.fast_forward(
                                static_cast<std::int32_t>(
                                    8U * element_byte_offset));
                            dtsx::PreliminaryMetadataHeader preliminary;
                            preliminary.chunk_id = element.chunk_id;
                            preliminary.raw_flags = element.flags;
                            preliminary.primary = element.primary;
                            preliminary.short_form =
                                element.short_form;
                            preliminary.alternate_association =
                                element.alternate_association;
                            preliminary.association_type =
                                element.association_type;
                            preliminary.association_index =
                                element.association_index;
                            dtsx::AudioPresentationMetadata presentation;
                                if (dtsx::unpack_audio_presentation_metadata(
                                    presentation_source,
                                    preliminary,
                                    preliminary.short_form,
                                    presentation)) {
                                std::vector<bool>
                                    waveform_decoder_available(
                                        presentation.objects.size(),
                                        false);
                                for (std::size_t object_index = 0U;
                                     object_index
                                         < presentation.objects.size();
                                     ++object_index) {
                                    waveform_decoder_available[
                                        object_index] =
                                        presentation.objects[object_index]
                                            .waveform_id_available;
                                }
                                bool bodies_parsed =
                                    dtsx::unpack_audio_presentation_object_bodies(
                                        presentation_source,
                                        presentation,
                                        &waveform_decoder_available);
                                output << ",\"presentationIndex\":"
                                       << static_cast<unsigned>(
                                              presentation.presentation_index)
                                       << ",\"presentationCount\":"
                                       << static_cast<unsigned>(
                                              presentation.presentation_count)
                                       << ",\"objectCount\":"
                                       << static_cast<unsigned>(
                                              presentation.object_count)
                                       << ",\"speakerMask\":"
                                       << presentation.speaker_activity_mask
                                       << ",\"renderGainCode\":"
                                       << static_cast<unsigned>(
                                              presentation.render_gain_code)
                                       << ",\"objectBodiesParsed\":"
                                       << (bodies_parsed ? "true" : "false")
                                       << ",\"objects\":[";
                                for (std::size_t object_index = 0;
                                     object_index < presentation.objects.size();
                                     ++object_index) {
                                    if (object_index != 0U) {
                                        output << ',';
                                    }
                                    const auto& object =
                                        presentation.objects[object_index];
                                    const std::uint32_t object_id =
                                        object.object_id_available
                                        ? object.object_id
                                        : static_cast<std::uint32_t>(
                                              object_index);
                                    output << "{\"id\":" << object_id
                                           << ",\"metadataPresent\":"
                                           << (object.metadata_present
                                                   ? "true"
                                                   : "false")
                                           << ",\"mode\":"
                                           << static_cast<unsigned>(
                                                  object.preamble.metadata_mode)
                                           << ",\"objectGainPresent\":"
                                           << (object.spatial_header.gain_present
                                                   ? "true"
                                                   : "false")
                                           << ",\"objectGainExponent\":"
                                           << static_cast<unsigned>(
                                                  object.spatial_header
                                                      .gain_exponent)
                                           << ",\"objectGainCode\":"
                                           << static_cast<unsigned>(
                                                  object.spatial_header
                                                      .gain_code)
                                           << ",\"waveformCount\":"
                                           << static_cast<unsigned>(
                                                  object.preamble.waveform_count)
                                           << ",\"waveformIdAvailable\":"
                                           << (object.waveform_id_available
                                                   ? "true"
                                                   : "false")
                                           << ",\"waveformId\":"
                                           << static_cast<unsigned>(
                                                  object.waveform_id)
                                           << ",\"waveformChannelOffsets\":[";
                                    for (std::size_t offset_index = 0;
                                         offset_index
                                             < object
                                                   .waveform_channel_offsets
                                                   .size();
                                         ++offset_index) {
                                        if (offset_index != 0U) {
                                            output << ',';
                                        }
                                        output << static_cast<unsigned>(
                                            object
                                                .waveform_channel_offsets
                                                    [offset_index]);
                                    }
                                    output
                                           << ']'
                                           << ",\"pointSources\":[";
                                    for (std::size_t point_index = 0;
                                         point_index < object.points.size();
                                         ++point_index) {
                                        if (point_index != 0U) {
                                            output << ',';
                                        }
                                        const auto& point =
                                            object.points[point_index];
                                        output << "{\"waveform\":"
                                               << point.waveform_index
                                               << ",\"index\":"
                                               << point.point_source_index
                                               << ",\"azimuth\":"
                                               << point.coordinates.azimuth_degrees
                                               << ",\"elevation\":"
                                               << point.coordinates.elevation_degrees
                                               << ",\"distance\":"
                                               << point.coordinates.distance
                                               << ",\"gainCode\":"
                                               << static_cast<unsigned>(
                                                      point.gain_code)
                                               << ",\"width\":"
                                               << point.width_degrees
                                               << ",\"height\":"
                                               << point.height_degrees
                                               << ",\"rotation\":"
                                               << point.rotation_degrees << '}';
                                    }
                                    output << "]}";
                                }
                                output << ']';
                            }
                        }
                        output << '}';
                    }
                    output << "]}";
                }
                output << "]}";
                if (coordinate_output != nullptr) {
                    DecodedObjectAudioFrame decoded_coordinates;
                    const auto lossy_base = make_xll_lossy_base(
                        coordinate_bed_decoder.decoded_core());
                    if (coordinate_decoder.decode(
                            frame,
                            decoded_coordinates,
                            lossy_base)
                        == ObjectFrameDecodeResult::Decoded) {
                        const std::uint32_t coordinate_duration =
                            decoded_coordinates.samples_per_channel != 0U
                            ? decoded_coordinates.samples_per_channel
                            : metadata_frame_duration;
                        const std::uint32_t coordinate_sample_rate =
                            decoded_coordinates.sample_rate != 0U
                            ? decoded_coordinates.sample_rate
                            : metadata_sample_rate;
                        if (!decoded_coordinates.objects.empty()) {
                            for (std::size_t object_index = 0U;
                                 object_index
                                     < decoded_coordinates.objects.size();
                                 ++object_index) {
                                const dtsx::ObjectMetadataBlock& object =
                                    decoded_coordinates.objects[object_index];
                                const std::uint32_t object_id =
                                    object.object_id_available
                                    ? object.object_id
                                    : static_cast<std::uint32_t>(
                                          object_index);
                                std::vector<std::uint32_t>
                                    object_waveform_channels;
                                bool object_waveform_available =
                                    dtsx::object_waveform_channel_indices(
                                        object,
                                        object_waveform_channels,
                                        &decoded_coordinates
                                             .waveform_base_by_id)
                                    && !object_waveform_channels.empty();
                                for (const std::uint32_t channel :
                                     object_waveform_channels) {
                                    if (channel
                                        >= decoded_coordinates
                                               .waveform_channels.size()) {
                                        object_waveform_available = false;
                                        break;
                                    }
                                }
                                for (const dtsx::PointSourceMetadata& point :
                                     object.points) {
                                    coordinate_output->write(
                                        metadata_sample_position,
                                        coordinate_duration,
                                        coordinate_sample_rate,
                                        object_id,
                                        object.waveform_id_available
                                            ? static_cast<std::uint32_t>(
                                                  object.waveform_id)
                                            : static_cast<std::uint32_t>(
                                                  point.waveform_index),
                                        point,
                                        object.preamble.metadata_mode,
                                        object_waveform_available
                                            && (object.preamble.metadata_mode
                                                    != 0U
                                                || dtsx::
                                                    point_source_is_renderable(
                                                        object.preamble
                                                            .metadata_mode,
                                                        point)),
                                        std::nullopt);
                                }
                            }
                        } else {
                            for (std::size_t waveform = 0U;
                                 waveform
                                     < decoded_coordinates
                                           .waveform_speaker_masks.size();
                                 ++waveform) {
                                const std::uint32_t speaker_mask =
                                    decoded_coordinates
                                        .waveform_speaker_masks[waveform];
                                if (speaker_mask == 0U) {
                                    continue;
                                }
                                coordinate_output->write_destination(
                                    metadata_sample_position,
                                    coordinate_duration,
                                    coordinate_sample_rate,
                                    0U,
                                    static_cast<std::uint32_t>(waveform),
                                    speaker_mask,
                                    61U,
                                    0U);
                            }
                        }
                    }
                }
                metadata_sample_position +=
                    metadata_frame_duration;
            }
        } else if (frame.packing == dtsx::StreamPacking::Core16BitBigEndian
                   || frame.packing == dtsx::StreamPacking::Core16BitLittleEndian) {
            const bool swap =
                frame.packing == dtsx::StreamPacking::Core16BitLittleEndian;
            dtsx::bitstream::WordBuffer words(frame.bytes, swap);
            dtsx::bitstream::Cursor cursor = words.cursor();
            std::uint32_t core_size = 0;
                if (dtsx::validate_core_frame_header(cursor, core_size)) {
                    coordinate_bed_decoder.remember_core(frame);
                    dtsx::CoreSubstreamInfo core;
                cursor = words.cursor();
                if (dtsx::parse_core_substream_header(cursor, core)) {
                    output << ",\"core\":{\"frameSize\":" << core_size
                           << ",\"sampleRate\":" << core.sample_rate
                           << ",\"sampleRateIndex\":"
                           << static_cast<unsigned>(core.sample_rate_index)
                           << ",\"channelSets\":"
                           << static_cast<unsigned>(core.channel_set_count)
                           << ",\"outputBlockSamples\":"
                           << core.output_block_samples << "}";
                }
            }
        }
        output << "}\n";
    }
    if (coordinate_output != nullptr) {
        coordinate_output->close();
    }
}

void export_object_stems(const Options& options) {
    DtsFrameReader reader(options);
    ObjectFrameDecoder decoder;
    DcaBedDecoder dca_bed_decoder;
    std::unique_ptr<ObjectStemWriter> writer;
    dtsx::ElementaryFrame elementary;
    std::uint64_t sample_position = 0U;
    std::uint32_t last_extension_frame_duration = 0U;
    bool wrote_audio = false;

    const auto advance_ignored_extension =
        [&sample_position, &last_extension_frame_duration](
            const dtsx::ElementaryFrame& frame) {
            const bool swap =
                frame.packing == dtsx::StreamPacking::ExtensionBigEndian;
            dtsx::bitstream::WordBuffer words(frame.bytes, swap);
            dtsx::bitstream::Cursor source = words.cursor();
            dtsx::ExssHeader header;
            const bool header_parsed =
                dtsx::unpack_exss_header(source, header);
            const std::uint32_t duration =
                header_parsed && header.frame_duration != 0U
                ? header.frame_duration
                : last_extension_frame_duration;
            if (duration == 0U
                || sample_position
                    > std::numeric_limits<std::uint64_t>::max() - duration) {
                throw std::runtime_error(
                    "invalid DTS:X extension frame timeline while exporting "
                    "object waveforms");
            }
            last_extension_frame_duration = duration;
            sample_position += duration;
        };

    while (reader.read(elementary)) {
        if (elementary.packing
                != dtsx::StreamPacking::ExtensionBigEndian
            && elementary.packing
                != dtsx::StreamPacking::ExtensionLittleEndian) {
            dca_bed_decoder.remember_core(elementary);
            continue;
        }
        DecodedObjectAudioFrame decoded;
        const auto lossy_base = make_xll_lossy_base(
            dca_bed_decoder.decoded_core());
        const ObjectFrameDecodeResult decode_result =
            decoder.decode(
                elementary, decoded, lossy_base);
        if (decode_result != ObjectFrameDecodeResult::Decoded) {
            if (elementary.packing == dtsx::StreamPacking::ExtensionBigEndian
                || elementary.packing
                    == dtsx::StreamPacking::ExtensionLittleEndian) {
                advance_ignored_extension(elementary);
            }
            continue;
        }
        last_extension_frame_duration = decoded.samples_per_channel;
        if (writer == nullptr) {
            writer = std::make_unique<ObjectStemWriter>(
                options.objects_output_directory,
                decoded.sample_rate,
                options.overwrite);
        }
        if (!writer->write(
                decoded,
                sample_position,
                decoded.samples_per_channel)) {
            throw std::runtime_error(
                "decoded object waveform channel is unavailable");
        }
        sample_position += decoded.samples_per_channel;
        wrote_audio = true;
    }
    if (writer != nullptr) {
        writer->close();
    }
    if (!wrote_audio) {
        throw std::runtime_error(
            "no decodable DTS:X object waveforms were found");
    }
}

bool map_xll_bed_to_layout(
    const DecodedObjectAudioFrame& decoded,
    const ChannelLayout& layout,
    std::vector<std::vector<std::int32_t>>& planar) {
    for (const dtsx::XllHierarchicalDownmixOutput& downmix :
         decoded.bed_downmix_outputs) {
        if (downmix.speaker_masks.size() != layout.channels.size()
            || downmix.planar_channels.size()
                   != layout.channels.size()
            || downmix.planar_channels.empty()) {
            continue;
        }
        std::vector<std::size_t> source_by_output(
            layout.channels.size(), downmix.speaker_masks.size());
        bool exact_layout = true;
        for (std::size_t output = 0U;
             output < layout.channels.size();
             ++output) {
            std::uint32_t speaker_mask = 0U;
            if (!dtsx::standard_speaker_mask(
                    layout.channels[output], speaker_mask)) {
                exact_layout = false;
                break;
            }
            auto found = std::find(
                downmix.speaker_masks.begin(),
                downmix.speaker_masks.end(),
                speaker_mask);
            if (found == downmix.speaker_masks.end()
                && layout.channels[output] == "SL") {
                found = std::find(
                    downmix.speaker_masks.begin(),
                    downmix.speaker_masks.end(),
                    1U << 9U);
            } else if (found == downmix.speaker_masks.end()
                       && layout.channels[output] == "SR") {
                found = std::find(
                    downmix.speaker_masks.begin(),
                    downmix.speaker_masks.end(),
                    1U << 10U);
            }
            if (found == downmix.speaker_masks.end()) {
                exact_layout = false;
                break;
            }
            source_by_output[output] =
                static_cast<std::size_t>(std::distance(
                    downmix.speaker_masks.begin(), found));
        }
        if (!exact_layout) {
            continue;
        }
        const std::size_t frame_count =
            downmix.planar_channels.front().size();
        planar.assign(layout.channels.size(), {});
        for (std::size_t output = 0U;
             output < layout.channels.size();
             ++output) {
            const auto& source =
                downmix.planar_channels[source_by_output[output]];
            if (source.size() != frame_count) {
                return false;
            }
            planar[output] = source;
        }
        return true;
    }
    if (decoded.bed_channels.empty()
        || decoded.bed_speaker_activity_mask == 0U) {
        return false;
    }
    const std::vector<std::uint32_t> physical_masks =
        dtsx::expand_speaker_activity_mask(
            decoded.bed_speaker_activity_mask);
    if (physical_masks.size() < decoded.bed_channels.size()) {
        return false;
    }
    const std::size_t frame_count = decoded.bed_channels.front().size();
    planar.assign(
        layout.channels.size(),
        std::vector<std::int32_t>(frame_count, 0));
    for (std::size_t output = 0U;
         output < layout.channels.size();
         ++output) {
        std::uint32_t speaker_mask = 0U;
        if (!dtsx::standard_speaker_mask(
                layout.channels[output], speaker_mask)) {
            return false;
        }
        auto found = std::find(
            physical_masks.begin(), physical_masks.end(), speaker_mask);
        if (found == physical_masks.end()
            && output < layout.channels.size()) {
            if (layout.channels[output] == "SL") {
                speaker_mask = 1U << 9U;
            } else if (layout.channels[output] == "SR") {
                speaker_mask = 1U << 10U;
            }
            found = std::find(
                physical_masks.begin(), physical_masks.end(), speaker_mask);
        }
        if (found == physical_masks.end()) {
            continue;
        }
        const std::size_t source = static_cast<std::size_t>(
            std::distance(physical_masks.begin(), found));
        if (source >= decoded.bed_channels.size()
            || decoded.bed_channels[source].size() != frame_count) {
            return false;
        }
        planar[output] = decoded.bed_channels[source];
    }
    return true;
}

bool map_supplemental_xll_to_height_layout(
    const DecodedObjectAudioFrame& decoded,
    const ChannelLayout& layout,
    std::vector<std::vector<std::int32_t>>& planar) {
    if (decoded.waveform_channels.empty()
        || decoded.waveform_speaker_masks.size()
               != decoded.waveform_channels.size()
        || decoded.waveform_is_supplemental.size()
               != decoded.waveform_channels.size()
        || !map_xll_bed_to_layout(decoded, layout, planar)) {
        return false;
    }
    bool supplemental_available = false;
    std::vector<bool> mapped_outputs(
        layout.channels.size(), false);
    std::vector<bool> mapped_waveforms(
        decoded.waveform_channels.size(), false);
    for (std::size_t waveform = 0U;
         waveform < decoded.waveform_channels.size();
         ++waveform) {
        if (!decoded.waveform_is_supplemental[waveform]) {
            continue;
        }
        supplemental_available = true;
        if (decoded.waveform_channels[waveform].size()
                != decoded.samples_per_channel
            || decoded.waveform_speaker_masks[waveform] == 0U) {
            return false;
        }
        std::size_t output = layout.channels.size();
        for (std::size_t candidate = 0U;
             candidate < layout.channels.size();
             ++candidate) {
            std::uint32_t speaker_mask = 0U;
            if (dtsx::standard_speaker_mask(
                    layout.channels[candidate],
                    speaker_mask)
                && speaker_mask
                    == decoded.waveform_speaker_masks[waveform]) {
                output = candidate;
                break;
            }
        }
        if (output == layout.channels.size()) {
            continue;
        }
        if (mapped_outputs[output]) {
            return false;
        }
        mapped_outputs[output] = true;
        mapped_waveforms[waveform] = true;
        planar[output] = decoded.waveform_channels[waveform];
    }
    for (std::size_t waveform = 0U;
         waveform < decoded.waveform_channels.size();
         ++waveform) {
        if (!decoded.waveform_is_supplemental[waveform]
            || mapped_waveforms[waveform]) {
            continue;
        }
        bool folded = false;
        for (const dtsx::XllEmbeddedDownmixOutput& downmix :
             decoded.supplemental_downmix_outputs) {
            if (!downmix.single_frequency_band
                || downmix.source_speaker_masks.size()
                       != downmix.source_channel_count
                || downmix.reference_speaker_masks.size()
                       != downmix.reference_storage_bit_depths.size()
                || downmix.current_coefficients.size()
                       != downmix.reference_speaker_masks.size()
                              * downmix.source_channel_count
                || downmix.previous_coefficients.size()
                       != downmix.current_coefficients.size()) {
                continue;
            }
            const auto source_speaker = std::find(
                downmix.source_speaker_masks.begin(),
                downmix.source_speaker_masks.end(),
                decoded.waveform_speaker_masks[waveform]);
            if (source_speaker
                == downmix.source_speaker_masks.end()) {
                continue;
            }
            const std::size_t source_column =
                static_cast<std::size_t>(std::distance(
                    downmix.source_speaker_masks.begin(),
                    source_speaker));
            for (std::size_t row = 0U;
                 row < downmix.reference_speaker_masks.size();
                 ++row) {
                std::size_t output = layout.channels.size();
                for (std::size_t candidate = 0U;
                     candidate < layout.channels.size();
                     ++candidate) {
                    std::uint32_t speaker_mask = 0U;
                    if (!dtsx::standard_speaker_mask(
                            layout.channels[candidate],
                            speaker_mask)) {
                        continue;
                    }
                    if (speaker_mask
                            == downmix.reference_speaker_masks[row]
                        || (downmix.reference_speaker_masks[row]
                                == (1U << 9U)
                            && layout.channels[candidate] == "SL")
                        || (downmix.reference_speaker_masks[row]
                                == (1U << 10U)
                            && layout.channels[candidate] == "SR")) {
                        output = candidate;
                        break;
                    }
                }
                if (output == layout.channels.size()) {
                    continue;
                }
                const std::uint8_t reference_depth =
                    downmix.reference_storage_bit_depths[row];
                const std::uint8_t source_depth =
                    downmix.source_storage_bit_depth;
                if (reference_depth < source_depth
                    || reference_depth == 0U
                    || source_depth == 0U
                    || reference_depth > 24U
                    || source_depth > 24U) {
                    return false;
                }
                const std::size_t coefficient_index =
                    row * downmix.source_channel_count
                    + source_column;
                const std::int32_t current_coefficient =
                    downmix.current_coefficients[
                        coefficient_index];
                const std::int32_t previous_coefficient =
                    downmix.previous_coefficients[
                        coefficient_index];
                std::uint8_t interpolation_bits = 0U;
                std::uint64_t interpolation_rounding = 0U;
                if (decoded.samples_per_channel > 1U) {
                    std::uint64_t interpolation_length = 1U;
                    while (interpolation_length
                           < decoded.samples_per_channel) {
                        interpolation_length <<= 1U;
                        ++interpolation_bits;
                    }
                    interpolation_rounding =
                        interpolation_length >> 1U;
                }
                const std::int64_t coefficient_delta =
                    static_cast<std::int64_t>(
                        current_coefficient)
                    - previous_coefficient;
                std::int64_t interpolation_accumulator = 0;
                const std::uint8_t source_output_shift =
                    static_cast<std::uint8_t>(
                        24U - source_depth);
                const std::uint8_t source_to_reference_shift =
                    static_cast<std::uint8_t>(
                        reference_depth - source_depth);
                const std::uint8_t reference_output_shift =
                    static_cast<std::uint8_t>(
                        24U - reference_depth);
                for (std::size_t sample = 0U;
                     sample < decoded.samples_per_channel;
                     ++sample) {
                    const std::int32_t coefficient =
                        coefficient_delta == 0
                        ? current_coefficient
                        : static_cast<std::int32_t>(
                              previous_coefficient
                              + ((interpolation_accumulator
                                  + static_cast<std::int64_t>(
                                      interpolation_rounding))
                                 >> interpolation_bits));
                    interpolation_accumulator +=
                        coefficient_delta;
                    const std::int32_t source_sample =
                        decoded.waveform_channels[waveform][sample];
                    const std::int32_t unshifted_source =
                        source_output_shift == 0U
                        ? source_sample
                        : source_sample
                              / static_cast<std::int32_t>(
                                    1U << source_output_shift);
                    const std::int32_t shifted_source =
                        static_cast<std::int32_t>(
                            static_cast<std::uint32_t>(
                                unshifted_source)
                            << source_to_reference_shift);
                    const std::int32_t correction =
                        static_cast<std::int32_t>(
                            (static_cast<std::int64_t>(
                                 shifted_source)
                                 * coefficient
                             + 0x4000)
                            >> 15);
                    const std::int32_t output_correction =
                        static_cast<std::int32_t>(
                            static_cast<std::uint32_t>(
                                correction)
                            << reference_output_shift);
                    planar[output][sample] =
                        static_cast<std::int32_t>(
                            static_cast<std::uint32_t>(
                                planar[output][sample])
                            + static_cast<std::uint32_t>(
                                output_correction));
                }
            }
            folded = true;
            break;
        }
        if (!folded) {
            return false;
        }
    }
    return supplemental_available;
}

void apply_native_soft_linear_peak_limiter(
    std::vector<std::vector<std::int32_t>>& channels) noexcept {
    // libdtsx.so: dtsxPeakLimiter_SoftLinear, 0x9bdf0.
    // dtsxPlayerInitConfig leaves MixerPeakLimiter at zero, selecting this
    // limiter in the final player output stage.
    constexpr float kLinearLimit = 6710900.0F;
    constexpr float kSaturationLimit = 10486000.0F;
    constexpr float kSlope = 0.44444F;
    constexpr float kOffset = 3728300.0F;
    for (auto& channel : channels) {
        for (std::int32_t& sample : channel) {
            const bool negative = sample < 0;
            const float magnitude = negative
                ? -static_cast<float>(sample)
                : static_cast<float>(sample);
            std::int32_t limited = 0;
            if (magnitude > kSaturationLimit) {
                limited = negative ? -0x800000 : 0x7FFFFF;
            } else if (magnitude > kLinearLimit) {
                const std::int32_t compressed =
                    static_cast<std::int32_t>(
                        magnitude * kSlope + kOffset + 0.5F);
                limited = negative ? -compressed : compressed;
            } else {
                limited = sample;
            }
            sample = std::max<std::int32_t>(
                -0x800000,
                std::min<std::int32_t>(0x7FFFFF, limited));
        }
    }
}

void apply_native_object_output_limiters(
    std::vector<std::vector<std::int32_t>>& channels) noexcept {
    // libdtsx.so: DTSHD_UHDAssetDecoder_DecodeSubframe clamps the decoded
    // bed before dtsPlayerObjectRenderer_RenderObjects adds the final object
    // signals.  sub_64298 then calls dtsxPeakLimitMixOut on that unclamped
    // 32-bit mix.  Clamping here before the soft limiter collapses every
    // over-range value to one constant plateau.
    apply_native_soft_linear_peak_limiter(channels);
}

std::uint64_t render_object_stream(
    const Options& options,
    const ChannelLayout& layout,
    std::uint32_t output_sample_rate,
    const std::filesystem::path& output_path,
    RenderMode render_mode) {
    DtsFrameReader reader(options);
    ObjectFrameDecoder decoder;
    DcaBedDecoder dca_bed_decoder;
    ObjectAudioRenderer renderer(layout, options.verbose);
    ProgressReporter progress;
    std::error_code size_error;
    const std::uint64_t elementary_size =
        std::filesystem::file_size(options.input, size_error);
    std::uint64_t elementary_bytes = 0U;
    progress.update(
        "decode",
        size_error ? -1 : 0);
    std::unique_ptr<WavWriter> writer;
    std::uint64_t frame_limit =
        std::numeric_limits<std::uint64_t>::max();
    const auto write_limited =
        [&writer, &frame_limit](
            const std::vector<std::vector<std::int32_t>>& channels) {
            const std::uint64_t remaining =
                frame_limit - std::min(
                    frame_limit, writer->frames_written());
            const std::size_t frames_to_write =
                static_cast<std::size_t>(
                    std::min<std::uint64_t>(
                        remaining,
                        channels.empty() ? 0U : channels.front().size()));
            writer->write_planar_24(channels, frames_to_write);
            return writer->frames_written() >= frame_limit;
        };
    dtsx::ElementaryFrame elementary;
    while (reader.read(elementary)) {
        elementary_bytes += elementary.bytes.size();
        progress.update(
            "decode",
            size_error
                ? -1
                : decode_percent(elementary_bytes, elementary_size));
        if (elementary.packing
                != dtsx::StreamPacking::ExtensionBigEndian
            && elementary.packing
                != dtsx::StreamPacking::ExtensionLittleEndian) {
            dca_bed_decoder.remember_core(elementary);
            continue;
        }
        DcaDecodedBed decoded_bed;
        const bool decoded_bed_available =
            dca_bed_decoder.decode_extension(
                elementary, decoded_bed);
        DecodedObjectAudioFrame decoded;
        const auto lossy_base = make_xll_lossy_base(
            dca_bed_decoder.decoded_core());
        const ObjectFrameDecodeResult decode_result =
            decoder.decode(
                elementary, decoded, lossy_base);
        if (decode_result != ObjectFrameDecodeResult::Decoded) {
            if (decoded_bed_available) {
                decoded.bed_channels =
                    std::move(decoded_bed.channels);
                decoded.bed_speaker_activity_mask =
                    decoded_bed.speaker_activity_mask;
                decoded.sample_rate = decoded_bed.sample_rate;
                decoded.samples_per_channel =
                    decoded_bed.samples_per_channel;
            } else {
                throw std::runtime_error(
                    decode_result == ObjectFrameDecodeResult::Malformed
                        ? "internal DTS:X ExSS/XLL frame and bed are malformed"
                        : "internal DTS:X ExSS frame has no decodable audio");
            }
        } else if (decoded_bed_available
                   && decoded.bed_channels.empty()) {
            decoded.bed_channels =
                std::move(decoded_bed.channels);
            decoded.bed_speaker_activity_mask =
                decoded_bed.speaker_activity_mask;
            decoded.sample_rate = decoded_bed.sample_rate;
            decoded.samples_per_channel =
                decoded_bed.samples_per_channel;
        } else if (decoded.bed_channels.empty()) {
            throw std::runtime_error(
                "internal DTS Core/XLL bed decode failed: "
                + dca_bed_decoder.last_error());
        }
        if (output_sample_rate != 0U
            && decoded.sample_rate != output_sample_rate) {
            throw std::runtime_error(
                "native DTS:X XLL sample rate differs from requested output "
                "sample rate");
        }
        if (writer == nullptr) {
            writer = std::make_unique<WavWriter>(
                output_path,
                layout,
                decoded.sample_rate,
                options.overwrite);
            frame_limit =
                duration_frame_limit(options, decoded.sample_rate);
        }
        if (render_mode != RenderMode::Bed
            && !decoded.objects.empty()
            && !renderer.remove_embedded_object_fold_down(decoded)) {
            throw std::runtime_error(
                "native DTS:X embedded object fold-down removal failed");
        }
        std::vector<std::vector<std::int32_t>> bed;
        const bool rendered_supplemental =
            render_mode != RenderMode::Bed
            && map_supplemental_xll_to_height_layout(
                decoded, layout, bed);
        if (!rendered_supplemental
            && !map_xll_bed_to_layout(decoded, layout, bed)) {
            throw std::runtime_error(
                "native DTS:X XLL bed cannot be mapped to the requested "
                "layout");
        }
        if (render_mode == RenderMode::Bed) {
            if (write_limited(bed)) {
                break;
            }
            continue;
        }
        if (rendered_supplemental
            && decoded.objects.empty()) {
            if (write_limited(bed)) {
                break;
            }
            continue;
        }
        std::vector<std::vector<std::int32_t>> rendered;
        if (!renderer.render(decoded, rendered)) {
            // Preserve a usable channel bed when an object is unresolved or
            // its metadata cannot be rendered for the selected layout.
            if (write_limited(bed)) {
                break;
            }
            continue;
        }
        if (render_mode == RenderMode::ObjectsOnly) {
            apply_native_object_output_limiters(rendered);
            if (write_limited(rendered)) {
                break;
            }
            continue;
        }
        for (std::size_t channel = 0U;
             channel < rendered.size();
             ++channel) {
            for (std::size_t sample = 0U;
                 sample < rendered[channel].size();
                 ++sample) {
                const std::int64_t mixed =
                    static_cast<std::int64_t>(bed[channel][sample])
                    + rendered[channel][sample];
                rendered[channel][sample] =
                    static_cast<std::int32_t>(
                        std::max<std::int64_t>(
                            (std::numeric_limits<std::int32_t>::min)(),
                            std::min<std::int64_t>(
                                (std::numeric_limits<std::int32_t>::max)(),
                                mixed)));
            }
        }
        apply_native_object_output_limiters(rendered);
        if (write_limited(rendered)) {
            break;
        }
    }
    if (writer == nullptr) {
        throw std::runtime_error(
            "internal decoder produced no PCM frames");
    }
    writer->close();
    progress.done("decode");
    return writer->frames_written();
}

} // namespace

int run_pipeline(const Options& requested_options) {
    Options options = requested_options;
    if (!std::filesystem::exists(options.input)) {
        throw std::runtime_error("input file does not exist");
    }
    if (!supported_input_extension(options.input)) {
        throw std::runtime_error(
            "input extension must be .mkv, .mp4, .m2ts, .dts or .dtshd");
    }
    if (!is_elementary_dts(options.input)
        && !options.audio_track_explicit) {
        options.audio_track =
            select_default_dts_track(options);
    }
    if (options.coordinates_output_explicit
        && !options.metadata_output_explicit) {
        throw std::runtime_error(
            "--coordinates-output requires --metadata-output");
    }
    const bool object_stems_only =
        options.objects_output_directory_explicit
        && options.layout.empty()
        && !options.output_explicit;
    if (options.layout.empty()
        && !options.probe
        && !object_stems_only) {
        throw std::runtime_error(
            "--layout is required; stream metadata and DTS:X rendering "
            "are determined by the internal decoder");
    }
    const std::optional<ChannelLayout> layout =
        options.layout.empty()
        ? std::nullopt
        : find_layout(options.layout);
    if (!options.layout.empty() && !layout) {
        throw std::runtime_error("unsupported --layout; supported: "
            + supported_layouts_text());
    }
    AudioProbe probe;
    probe.stream_index = options.audio_track;
    probe.sample_rate = options.sample_rate;
    probe.channels = layout
        ? static_cast<std::uint32_t>(layout->channels.size())
        : 0U;
    probe.channel_layout = layout ? options.layout : std::string{};
    probe.codec_name = "dts";

    if (options.channels_check != 0U && !layout) {
        throw std::runtime_error("--channels requires --layout");
    }
    if (options.channels_check != 0
        && options.channels_check != layout->channels.size()) {
        throw std::runtime_error("--channels does not match the selected layout");
    }
    const std::uint32_t sample_rate = options.sample_rate;
    if (!options.probe && sample_rate != 0U
        && (sample_rate < 8000 || sample_rate > 384000)) {
        throw std::runtime_error("output sample rate is outside 8000..384000 Hz");
    }
    if (options.probe) {
        run_internal_probe(
            options,
            probe,
            layout ? &*layout : nullptr);
        return 0;
    }
    const bool dts_uhd = input_is_dts_uhd(options);
    if (dts_uhd) {
        if (options.metadata_output_explicit
            || options.coordinates_output_explicit
            || options.objects_output_directory_explicit) {
            throw std::runtime_error(
                "DTS-UHD full channel-based mix does not expose "
                "dynamic object sidecars");
        }
    }

    Options decode_options = options;
    ScopedTemporaryFile elementary_file;
    if (!is_elementary_dts(options.input)) {
        elementary_file.set(temporary_elementary_path());
        demux_container(options, elementary_file.path());
        decode_options.input = elementary_file.path();
    }
    ChannelCapacity capacity;
    std::string capacity_warning;
    if (layout) {
        capacity = inspect_channel_capacity(decode_options);
        capacity_warning =
            channel_capacity_warning(
                *layout,
                capacity.physical_speakers,
                options.render_mode != RenderMode::Bed
                    && capacity.dynamic_objects);
    }

    if (dts_uhd) {
        const std::filesystem::path output =
            options.output_explicit
            ? options.output
            : generated_output_path(options.input, *layout);
        print_decode_summary(
            options, probe, *layout, output, capacity_warning);
        decode_p2_stream(decode_options, output);
        return 0;
    }
    if (options.metadata_output_explicit) {
        dump_metadata(decode_options, options.metadata_output);
    }
    if (options.objects_output_directory_explicit) {
        export_object_stems(decode_options);
        if (object_stems_only) {
            return 0;
        }
    }

    const std::filesystem::path output = options.output_explicit
        ? options.output
        : generated_output_path(options.input, *layout);

    print_decode_summary(
        options, probe, *layout, output, capacity_warning);
    const std::uint64_t frames = render_object_stream(
        decode_options, *layout, sample_rate, output, options.render_mode);
    std::cerr << "Frames: " << frames << '\n';
    return 0;
}

} // namespace dtsx_decode
