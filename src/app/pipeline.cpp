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
#include "../render/object_gain.hpp"
#include "../render/imax_post_processor.hpp"
#include "../render/parma_blind_renderer.hpp"
#include "../render/parma_filterbank.hpp"
#include "../render/parma_guided_controls.hpp"
#include "../render/parma_guided_renderer.hpp"
#include "../render/parma_guided_topology.hpp"
#include "../render/parma_layout.hpp"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cwctype>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace dtsx_decode {
namespace {

template <typename T>
class OrderedDecodeSlot final {
public:
    [[nodiscard]] bool submit(T&& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
            return !value_.has_value() || stopped_;
        });
        if (stopped_) {
            return false;
        }
        value_.emplace(std::move(value));
        condition_.notify_all();
        return true;
    }

    [[nodiscard]] bool take(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
            return value_.has_value() || finished_ || stopped_;
        });
        if (stopped_ || !value_.has_value()) {
            return false;
        }
        value = std::move(*value_);
        value_.reset();
        condition_.notify_all();
        return true;
    }

    void finish() {
        std::lock_guard<std::mutex> lock(mutex_);
        finished_ = true;
        condition_.notify_all();
    }

    void stop(std::exception_ptr error = nullptr) {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
        if (error != nullptr && error_ == nullptr) {
            error_ = error;
        }
        condition_.notify_all();
    }

    [[nodiscard]] bool stopped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_;
    }

    [[nodiscard]] std::exception_ptr error() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return error_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<T> value_;
    bool finished_ = false;
    bool stopped_ = false;
    std::exception_ptr error_;
};

class BooleanTaskWorker final {
public:
    BooleanTaskWorker()
        : thread_([this] { run(); }) {
    }

    ~BooleanTaskWorker() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
            condition_.notify_all();
        }
        thread_.join();
    }

    BooleanTaskWorker(const BooleanTaskWorker&) = delete;
    BooleanTaskWorker& operator=(const BooleanTaskWorker&) = delete;

    [[nodiscard]] std::future<bool> submit(
        std::function<bool()> function) {
        std::packaged_task<bool()> task(std::move(function));
        std::future<bool> result = task.get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (task_.has_value()) {
                throw std::logic_error("parallel decode task overlap");
            }
            task_.emplace(std::move(task));
        }
        condition_.notify_all();
        return result;
    }

private:
    void run() {
        for (;;) {
            std::packaged_task<bool()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] {
                    return task_.has_value() || stopping_;
                });
                if (!task_.has_value()) {
                    return;
                }
                task = std::move(*task_);
                task_.reset();
            }
            task();
        }
    }

    std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<std::packaged_task<bool()>> task_;
    bool stopping_ = false;
    std::thread thread_;
};

std::wstring lower_extension(std::filesystem::path path) {
    std::wstring extension = path.extension().wstring();
    for (wchar_t& c : extension) {
        c = std::towlower(c);
    }
    return extension;
}

bool supported_input_extension(const std::filesystem::path& path) {
    const std::wstring extension = lower_extension(path);
    return extension == L".mkv" || extension == L".mka"
        || extension == L".mp4" || extension == L".m4a"
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
    DtsHdHra = 2U,
    DtsHdMa = 3U,
    DtsX = 4U,
};

bool has_core_hra_dtsx_envelope(
    const dtsx::ElementaryFrame& frame) noexcept {
    if (frame.bytes.size() < 10U) {
        return false;
    }
    for (std::size_t i = 0U; i + 9U < frame.bytes.size(); ++i) {
        if (frame.bytes[i] == 0x3AU
            && frame.bytes[i + 1U] == 0x42U
            && frame.bytes[i + 2U] == 0x9BU
            && frame.bytes[i + 3U] == 0x0AU
            && frame.bytes[i + 6U] == 0x02U
            && frame.bytes[i + 7U] == 0x00U
            && frame.bytes[i + 8U] == 0x08U
            && frame.bytes[i + 9U] == 0x50U) {
            return true;
        }
    }
    return false;
}

bool has_lossy_object_asset(
    const dtsx::ElementaryFrame& frame) noexcept {
    if (frame.packing != dtsx::StreamPacking::ExtensionBigEndian
        && frame.packing != dtsx::StreamPacking::ExtensionLittleEndian) {
        return false;
    }
    const bool swap =
        frame.packing == dtsx::StreamPacking::ExtensionBigEndian;
    dtsx::bitstream::WordBuffer words(frame.bytes, swap);
    dtsx::bitstream::Cursor header_source = words.cursor();
    dtsx::ExssHeader header;
    if (!dtsx::unpack_exss_header(header_source, header)) {
        return false;
    }
    for (std::uint32_t ordinal = 0U;
         ordinal < header.asset_count
             && ordinal < header.asset_header_bit_offsets.size();
         ++ordinal) {
        dtsx::bitstream::Cursor asset_source = words.cursor();
        asset_source.fast_forward(static_cast<std::int32_t>(
            header.asset_header_bit_offsets[ordinal]));
        dtsx::ExssAssetSummary asset;
        if (dtsx::unpack_exss_asset_summary(
                asset_source, header, asset, ordinal)
            && asset.object_audio_type == 1U
            && (asset.coding_components
                & ((1U << 6U) | (1U << 8U))) != 0U
            && (asset.coding_components & (1U << 9U)) == 0U) {
            return true;
        }
    }
    return false;
}

bool has_lbr_asset(const dtsx::ElementaryFrame& frame) noexcept {
    if (frame.packing != dtsx::StreamPacking::ExtensionBigEndian
        && frame.packing != dtsx::StreamPacking::ExtensionLittleEndian) {
        return false;
    }
    const bool swap =
        frame.packing == dtsx::StreamPacking::ExtensionBigEndian;
    dtsx::bitstream::WordBuffer words(frame.bytes, swap);
    dtsx::bitstream::Cursor header_source = words.cursor();
    dtsx::ExssHeader header;
    if (!dtsx::unpack_exss_header(header_source, header)) {
        return false;
    }
    for (std::uint32_t ordinal = 0U;
         ordinal < header.asset_count
             && ordinal < header.asset_header_bit_offsets.size();
         ++ordinal) {
        dtsx::bitstream::Cursor asset_source = words.cursor();
        asset_source.fast_forward(static_cast<std::int32_t>(
            header.asset_header_bit_offsets[ordinal]));
        dtsx::ExssAssetSummary asset;
        if (dtsx::unpack_exss_asset_summary(
                asset_source, header, asset, ordinal)
            && asset.coding_mode_available
            && asset.coding_mode == 2U) {
            return true;
        }
    }
    return false;
}

bool is_core_stream_packing(dtsx::StreamPacking packing) noexcept {
    return packing == dtsx::StreamPacking::Core16BitBigEndian
        || packing == dtsx::StreamPacking::Core16BitLittleEndian
        || packing == dtsx::StreamPacking::Core14BitBigEndian
        || packing == dtsx::StreamPacking::Core14BitLittleEndian;
}

bool core_swap_byte_pairs(dtsx::StreamPacking packing) noexcept {
    return packing == dtsx::StreamPacking::Core16BitLittleEndian
        || packing == dtsx::StreamPacking::Core14BitLittleEndian;
}

void write_metadata_chunks_json(
    std::ostream& output,
    const char* field_name,
    const std::vector<dtsx::MetadataChunkLocation>& chunks,
    const dtsx::bitstream::WordBuffer& fallback_words) {
    output << ",\"" << field_name << "\":[";
    for (std::size_t index = 0; index < chunks.size(); ++index) {
        if (index != 0U) {
            output << ',';
        }
        const dtsx::MetadataChunkLocation& chunk = chunks[index];
        output << "{\"offset\":" << chunk.byte_offset
               << ",\"elements\":" << chunk.envelope.element_sizes.size()
               << ",\"payloadBytes\":"
               << chunk.envelope.element_payload_size
               << ",\"crcValid\":"
               << (chunk.envelope.crc_valid ? "true" : "false")
               << ",\"chunkIds\":[";
        const auto& elements = chunk.envelope.elements;
        for (std::size_t element_index = 0;
             element_index < elements.size();
             ++element_index) {
            if (element_index != 0U) {
                output << ',';
            }
            const auto& element = elements[element_index];
            output << "{\"id\":" << static_cast<unsigned>(element.chunk_id)
                   << ",\"primary\":"
                   << (element.primary ? "true" : "false")
                   << ",\"association\":"
                   << static_cast<unsigned>(element.association_index);
            if (chunk.envelope.crc_valid
                && element.chunk_id == 241U
                && element.primary) {
                const std::uint32_t element_byte_offset =
                    chunk.byte_offset
                    + chunk.element_prefix_bytes
                    + element.byte_offset
                    + 2U;
                dtsx::bitstream::Cursor presentation_source =
                    chunk.source_words != nullptr
                    ? chunk.source_words->cursor()
                    : fallback_words.cursor();
                presentation_source.fast_forward(
                    static_cast<std::int32_t>(8U * element_byte_offset));
                dtsx::PreliminaryMetadataHeader preliminary;
                preliminary.chunk_id = element.chunk_id;
                preliminary.raw_flags = element.flags;
                preliminary.primary = element.primary;
                preliminary.short_form = element.short_form;
                preliminary.alternate_association =
                    element.alternate_association;
                preliminary.association_type = element.association_type;
                preliminary.association_index = element.association_index;
                dtsx::AudioPresentationMetadata presentation;
                if (dtsx::unpack_audio_presentation_metadata(
                        presentation_source,
                        preliminary,
                        preliminary.short_form,
                        presentation)) {
                    std::vector<bool> waveform_decoder_available(
                        presentation.objects.size(), false);
                    for (std::size_t object_index = 0U;
                         object_index < presentation.objects.size();
                         ++object_index) {
                        waveform_decoder_available[object_index] =
                            presentation.objects[object_index]
                                .waveform_id_available;
                    }
                    const bool bodies_parsed =
                        dtsx::unpack_audio_presentation_object_bodies(
                            presentation_source,
                            presentation,
                            &waveform_decoder_available);
                    output << ",\"presentationIndex\":"
                           << static_cast<unsigned>(
                                  presentation.presentation_index)
                           << ",\"objectCount\":"
                           << static_cast<unsigned>(
                                  presentation.object_count)
                           << ",\"speakerMask\":"
                           << presentation.speaker_activity_mask
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
                        output << "{\"id\":"
                               << (object.object_id_available
                                       ? object.object_id
                                       : static_cast<std::uint32_t>(
                                             object_index))
                               << ",\"metadataPresent\":"
                               << (object.metadata_present
                                       ? "true"
                                       : "false")
                               << ",\"mode\":"
                               << static_cast<unsigned>(
                                      object.preamble.metadata_mode)
                               << ",\"waveformIdAvailable\":"
                               << (object.waveform_id_available
                                       ? "true"
                                       : "false")
                               << ",\"waveformId\":"
                               << static_cast<unsigned>(object.waveform_id)
                               << ",\"pointSources\":"
                               << object.points.size() << '}';
                    }
                    output << ']';
                }
            }
            output << '}';
        }
        output << "]}";
    }
    output << ']';
}

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
        const bool hra_dtsx = has_core_hra_dtsx_envelope(frame);
        rank = (std::max)(
            rank,
            hra_dtsx ? DtsTrackRank::DtsX : DtsTrackRank::Dts);
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
        } else if ((asset.coding_components & 0x00E0U) != 0U) {
            rank = (std::max)(rank, DtsTrackRank::DtsHdHra);
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

DtsTrackRank inspect_selected_dts_profile(const Options& options) {
    if (!is_elementary_dts(options.input)) {
        return inspect_audio_track(
            options, options.audio_track).rank;
    }
    constexpr std::uint32_t kMaximumFrames = 256U;
    Options probe_options = options;
    probe_options.probe = true;
    probe_options.full_probe = false;
    DtsFrameReader reader(probe_options);
    DtsTrackRank rank = DtsTrackRank::None;
    dtsx::ElementaryFrame frame;
    std::uint32_t frames = 0U;
    while (frames++ < kMaximumFrames && reader.read(frame)) {
        classify_dts_frame(frame, rank);
        if (rank == DtsTrackRank::DtsX) {
            break;
        }
    }
    return rank;
}

bool inspect_core_hra_dtsx(const Options& options) {
    Options probe_options = options;
    probe_options.probe = true;
    probe_options.full_probe = false;
    DtsFrameReader reader(probe_options);
    dtsx::ElementaryFrame frame;
    std::uint32_t frames = 0U;
    while (frames++ < 256U && reader.read(frame)) {
        if (has_core_hra_dtsx_envelope(frame)) {
            return true;
        }
    }
    return false;
}

bool inspect_lbr_stream(const Options& options) {
    Options probe_options = options;
    probe_options.probe = true;
    probe_options.full_probe = false;
    DtsFrameReader reader(probe_options);
    dtsx::ElementaryFrame frame;
    std::uint32_t frames = 0U;
    while (frames++ < 256U && reader.read(frame)) {
        if (has_lbr_asset(frame)) {
            return true;
        }
    }
    return false;
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
    std::uint32_t coded_bed_speakers = 0U;
    std::uint32_t reference_speakers = 0U;
    std::uint32_t supplemental_speakers = 0U;
    std::uint32_t maximum_supplemental_channels = 0U;
    std::uint32_t maximum_unmapped_supplemental_waveforms = 0U;
    std::uint32_t sampled_windows = 0U;
    bool any_objects = false;
    bool any_waveforms = false;
    bool dynamic_objects = false;
    bool any_positive_height_object = false;
    bool imax_metadata = false;
    struct ObjectRecommendationState final {
        bool observed = false;
        bool position_available = false;
        bool single_point = true;
        bool centered = true;
        bool static_position = true;
        float azimuth = 0.0F;
        float elevation = 0.0F;
        float distance = 0.0F;
    };
    std::map<std::uint32_t, ObjectRecommendationState>
        object_recommendation_states;
};

struct SupplementalLayoutObservation final {
    std::uint32_t channels = 0U;
    std::uint32_t unmapped_waveforms = 0U;
    std::uint32_t speaker_activity_mask = 0U;
};

struct SupplementalLayoutEvidence final {
    std::uint32_t channels = 0U;
    std::uint32_t unmapped_waveforms = 0U;
    std::uint32_t speaker_activity_mask = 0U;
    std::uint32_t consecutive_frames = 0U;
    bool confirmed = false;
};

std::uint32_t alternate_profile_unmapped_waveform_count(
    std::uint32_t extension_sync_word) noexcept {
    // D1-D4 samples use a stable N+4 topology.  D0 does not encode N in the
    // sync suffix: Arcam accepts both one-channel point sources and
    // three-channel multi-point sources, so no count may be inferred from a
    // D0 sync when its XLL headers were not decoded.
    switch (extension_sync_word) {
    case 0xF14000D0U:
        return 0U;
    case 0xF14000D1U:
        return 6U;
    case 0xF14000D2U:
        return 7U;
    case 0xF14000D3U:
        return 8U;
    case 0xF14000D4U:
        return 9U;
    default:
        return 0U;
    }
}

const char* alternate_profile_name(
    std::uint32_t extension_sync_word) noexcept {
    switch (extension_sync_word) {
    case 0xF14000D0U:
        return "D0";
    case 0xF14000D1U:
        return "D1";
    case 0xF14000D2U:
        return "D2";
    case 0xF14000D3U:
        return "D3";
    case 0xF14000D4U:
        return "D4";
    default:
        return nullptr;
    }
}

std::vector<bool> semantically_mapped_waveforms(
    const DecodedObjectAudioFrame& decoded) {
    std::vector<bool> mapped(decoded.waveform_channels.size(), false);
    for (std::size_t waveform = 0U;
         waveform < mapped.size()
             && waveform < decoded.waveform_speaker_masks.size();
         ++waveform) {
        mapped[waveform] =
            decoded.waveform_speaker_masks[waveform] != 0U;
    }
    for (const dtsx::ObjectMetadataBlock& object : decoded.objects) {
        std::vector<std::uint32_t> channels;
        if (!dtsx::object_waveform_channel_indices(
                object,
                channels,
                &decoded.waveform_base_by_id)) {
            continue;
        }
        for (const std::uint32_t channel : channels) {
            if (channel < mapped.size()) {
                mapped[channel] = true;
            }
        }
    }
    return mapped;
}

std::vector<bool> object_mapped_waveforms(
    const DecodedObjectAudioFrame& decoded) {
    std::vector<bool> mapped(decoded.waveform_channels.size(), false);
    for (const dtsx::ObjectMetadataBlock& object : decoded.objects) {
        std::vector<std::uint32_t> channels;
        if (!dtsx::object_waveform_channel_indices(
                object,
                channels,
                &decoded.waveform_base_by_id)) {
            continue;
        }
        for (const std::uint32_t channel : channels) {
            if (channel < mapped.size()) {
                mapped[channel] = true;
            }
        }
    }
    return mapped;
}

bool duplicates_semantically_mapped_waveform(
    const DecodedObjectAudioFrame& decoded,
    std::size_t waveform,
    const std::vector<bool>& mapped) noexcept {
    if (waveform >= decoded.waveform_channels.size()) {
        return false;
    }
    const auto& samples = decoded.waveform_channels[waveform];
    for (std::size_t candidate = 0U;
         candidate < mapped.size();
         ++candidate) {
        if (candidate == waveform
            || !mapped[candidate]
            || candidate >= decoded.waveform_channels.size()
            || decoded.waveform_channels[candidate].size()
                   != samples.size()) {
            continue;
        }
        if (samples == decoded.waveform_channels[candidate]) {
            return true;
        }
    }
    return false;
}

SupplementalLayoutObservation supplemental_layout_observation(
    const DecodedObjectAudioFrame& decoded) {
    SupplementalLayoutObservation observation;
    const std::vector<bool> mapped =
        semantically_mapped_waveforms(decoded);
    const std::vector<bool> object_mapped =
        object_mapped_waveforms(decoded);
    observation.speaker_activity_mask =
        decoded.supplemental_speaker_activity_mask;
    for (std::size_t waveform = 0U;
         waveform < decoded.waveform_is_supplemental.size();
         ++waveform) {
        if (!decoded.waveform_is_supplemental[waveform]) {
            continue;
        }
        if (waveform < object_mapped.size()
            && object_mapped[waveform]) {
            continue;
        }
        const std::uint32_t speaker_mask =
            waveform < decoded.waveform_speaker_masks.size()
            ? decoded.waveform_speaker_masks[waveform]
            : 0U;
        if (speaker_mask == 0U
            && duplicates_semantically_mapped_waveform(
                decoded, waveform, mapped)) {
            continue;
        }
        ++observation.channels;
        if (speaker_mask == 0U) {
            ++observation.unmapped_waveforms;
        }
        observation.speaker_activity_mask |=
            dtsx::speaker_mask_to_activity_mask(speaker_mask);
    }
    return observation;
}

bool confirm_supplemental_layout(
    SupplementalLayoutEvidence& evidence,
    const SupplementalLayoutObservation& observation) noexcept {
    if (observation.channels == 0U) {
        evidence = {};
        return false;
    }
    if (evidence.channels != observation.channels
        || evidence.unmapped_waveforms
               != observation.unmapped_waveforms
        || evidence.speaker_activity_mask
               != observation.speaker_activity_mask) {
        evidence.channels = observation.channels;
        evidence.unmapped_waveforms = observation.unmapped_waveforms;
        evidence.speaker_activity_mask =
            observation.speaker_activity_mask;
        evidence.consecutive_frames = 1U;
        evidence.confirmed = false;
        return false;
    }
    if (evidence.consecutive_frames
        < (std::numeric_limits<std::uint32_t>::max)()) {
        ++evidence.consecutive_frames;
    }
    // A channel layout is persistent XLL structure. Requiring the same
    // CRC-valid set in the following frame prevents a coincidental header
    // match while scanning private type-69 chunk storage from becoming a
    // stream-wide layout declaration.
    evidence.confirmed = evidence.consecutive_frames >= 2U;
    return evidence.confirmed;
}

void merge_channel_capacity(
    ChannelCapacity& destination,
    const ChannelCapacity& source) noexcept {
    destination.physical_speakers |= source.physical_speakers;
    destination.coded_bed_speakers |= source.coded_bed_speakers;
    destination.reference_speakers |= source.reference_speakers;
    destination.supplemental_speakers |= source.supplemental_speakers;
    destination.maximum_supplemental_channels = (std::max)(
        destination.maximum_supplemental_channels,
        source.maximum_supplemental_channels);
    destination.maximum_unmapped_supplemental_waveforms = (std::max)(
        destination.maximum_unmapped_supplemental_waveforms,
        source.maximum_unmapped_supplemental_waveforms);
    destination.any_objects =
        destination.any_objects || source.any_objects;
    destination.any_waveforms =
        destination.any_waveforms || source.any_waveforms;
    destination.dynamic_objects =
        destination.dynamic_objects || source.dynamic_objects;
    destination.any_positive_height_object =
        destination.any_positive_height_object
        || source.any_positive_height_object;
    destination.imax_metadata =
        destination.imax_metadata || source.imax_metadata;
    for (const auto& entry : source.object_recommendation_states) {
        auto& state = destination.object_recommendation_states[entry.first];
        state.observed = state.observed || entry.second.observed;
        state.position_available =
            state.position_available || entry.second.position_available;
        state.single_point = state.single_point && entry.second.single_point;
        state.centered = state.centered && entry.second.centered;
        state.static_position =
            state.static_position && entry.second.static_position;
        state.azimuth = entry.second.azimuth;
        state.elevation = entry.second.elevation;
        state.distance = entry.second.distance;
    }
}

void observe_object_recommendation_metadata(
    ChannelCapacity& capacity,
    const DecodedObjectAudioFrame& decoded) {
    capacity.imax_metadata = capacity.imax_metadata || decoded.imax_enhanced;
    for (std::size_t index = 0U; index < decoded.objects.size(); ++index) {
        const dtsx::ObjectMetadataBlock& object = decoded.objects[index];
        const std::uint32_t id = object.object_id_available
            ? object.object_id
            : static_cast<std::uint32_t>(index);
        auto& state = capacity.object_recommendation_states[id];
        state.observed = true;
        state.single_point = state.single_point && object.points.size() == 1U;
        if (object.points.empty()) {
            state.centered = false;
            state.static_position = false;
            continue;
        }
        if (object.points.size() != 1U) {
            state.centered = false;
        }
        for (const dtsx::PointSourceMetadata& point : object.points) {
            const dtsx::RendererCoordinates& coordinates = point.coordinates;
            capacity.any_positive_height_object =
                capacity.any_positive_height_object
                || coordinates.elevation_degrees > 0.0F;
            state.centered = state.centered
                && std::fabs(coordinates.azimuth_degrees) <= 0.5F
                && std::fabs(coordinates.elevation_degrees) <= 0.5F;
            if (state.position_available
                && (std::fabs(coordinates.azimuth_degrees - state.azimuth) > 0.01F
                    || std::fabs(coordinates.elevation_degrees - state.elevation) > 0.01F
                    || std::fabs(coordinates.distance - state.distance) > 0.001F)) {
                state.static_position = false;
            }
            state.azimuth = coordinates.azimuth_degrees;
            state.elevation = coordinates.elevation_degrees;
            state.distance = coordinates.distance;
            state.position_available = true;
        }
    }
}

bool has_single_static_center_imax_object(
    const ChannelCapacity& capacity) noexcept {
    if (!capacity.imax_metadata
        || capacity.object_recommendation_states.size() != 1U) {
        return false;
    }
    const auto& state = capacity.object_recommendation_states.begin()->second;
    return state.observed && state.single_point && state.centered
        && state.static_position;
}

ChannelCapacity inspect_channel_capacity_window(
    const Options& options,
    std::uint64_t elementary_byte_offset,
    std::uint64_t container_start_milliseconds,
    std::uint64_t container_duration_milliseconds,
    bool extended_window) {
    const std::uint32_t extension_frame_limit =
        extended_window ? 2816U : 16U;
    const std::uint32_t frame_limit =
        extended_window ? 8192U : 256U;
    DtsFrameReader reader(
        options,
        elementary_byte_offset,
        container_start_milliseconds,
        container_duration_milliseconds);
    ObjectFrameDecoder object_decoder;
    DcaBedDecoder bed_decoder;
    dtsx::UhdFrameParserState uhd_state;
    ChannelCapacity capacity;
    dtsx::ElementaryFrame frame;
    std::uint32_t frames = 0U;
    std::uint32_t extension_frames = 0U;
    SupplementalLayoutEvidence supplemental_evidence;
    while (frames++ < frame_limit && reader.read(frame)) {
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
                add_activity_speakers(
                    header.speaker_activity_mask,
                    capacity.reference_speakers);
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
            object_decoder.remember_core_metadata(frame);
            continue;
        }
        ++extension_frames;
        DcaDecodedBed decoded_bed;
        std::vector<DcaDecodedObjectAsset> lossy_object_assets;
        const bool bed_available =
            bed_decoder.decode_extension(
                frame,
                decoded_bed,
                !has_lbr_asset(frame),
                &lossy_object_assets);
        if (bed_available) {
            add_activity_speakers(
                decoded_bed.speaker_activity_mask,
                capacity.physical_speakers);
            add_activity_speakers(
                decoded_bed.speaker_activity_mask,
                capacity.coded_bed_speakers);
        }
        DecodedObjectAudioFrame decoded;
        const auto lossy_base =
            make_xll_lossy_base(bed_decoder.decoded_core());
        if (object_decoder.decode(
                frame,
                decoded,
                lossy_base,
                lossy_object_assets)
            == ObjectFrameDecodeResult::Decoded) {
            observe_object_recommendation_metadata(capacity, decoded);
            for (std::size_t waveform = 0U;
                 waveform < decoded.waveform_channels.size();
                 ++waveform) {
                if (waveform
                        >= decoded.waveform_is_supplemental.size()
                    || !decoded.waveform_is_supplemental[waveform]) {
                    capacity.any_waveforms = true;
                    break;
                }
            }
            add_activity_speakers(
                decoded.bed_speaker_activity_mask,
                capacity.physical_speakers);
            add_activity_speakers(
                decoded.bed_speaker_activity_mask,
                capacity.coded_bed_speakers);
            if (decoded.metadata_speaker_activity_mask != 0U) {
                add_activity_speakers(
                    decoded.metadata_speaker_activity_mask,
                    capacity.reference_speakers);
            }
            for (const dtsx::XllHierarchicalDownmixOutput& downmix :
                 decoded.bed_downmix_outputs) {
                for (const std::uint32_t speaker :
                     downmix.speaker_masks) {
                    capacity.physical_speakers |= speaker;
                }
            }
            const SupplementalLayoutObservation supplemental =
                supplemental_layout_observation(decoded);
            if (confirm_supplemental_layout(
                    supplemental_evidence, supplemental)) {
                capacity.maximum_supplemental_channels = (std::max)(
                    capacity.maximum_supplemental_channels,
                    supplemental.channels);
                capacity.maximum_unmapped_supplemental_waveforms =
                    (std::max)(
                        capacity
                            .maximum_unmapped_supplemental_waveforms,
                        supplemental.unmapped_waveforms);
                add_activity_speakers(
                    supplemental.speaker_activity_mask,
                    capacity.physical_speakers);
                add_activity_speakers(
                    supplemental.speaker_activity_mask,
                    capacity.supplemental_speakers);
            }
            if (!decoded.objects.empty()) {
                capacity.any_objects = true;
                capacity.dynamic_objects = true;
            }
        }
        if (extension_frames >= extension_frame_limit) {
            break;
        }
    }
    reader.finish();
    return capacity;
}

ChannelCapacity inspect_channel_capacity(
    const Options& options,
    bool distributed) {
    constexpr std::size_t kWindowCount = 5U;
    constexpr std::uint64_t kTailWindowBytes = 16U * 1024U * 1024U;
    constexpr std::uint64_t kContainerWindowMilliseconds = 30000U;
    constexpr std::array<std::uint32_t, kWindowCount>
        kPositionNumerators = {{0U, 1U, 1U, 1U, 1U}};
    constexpr std::array<std::uint32_t, kWindowCount>
        kPositionDenominators = {{1U, 32U, 4U, 2U, 1U}};

    if (!distributed) {
        ChannelCapacity capacity = inspect_channel_capacity_window(
            options, 0U, 0U, 0U, false);
        capacity.sampled_windows = 1U;
        return capacity;
    }

    ChannelCapacity capacity;
    if (is_elementary_dts(options.input)) {
        std::error_code size_error;
        const std::uint64_t size =
            std::filesystem::file_size(options.input, size_error);
        if (size_error) {
            throw std::runtime_error(
                "cannot determine DTS elementary stream size: "
                + size_error.message());
        }
        const std::uint64_t tail_window =
            (std::min)(size, kTailWindowBytes);
        const std::uint64_t span = size - tail_window;
        std::uint64_t previous_offset =
            (std::numeric_limits<std::uint64_t>::max)();
        for (std::size_t index = 0U; index < kWindowCount; ++index) {
            const std::uint64_t offset = span
                * kPositionNumerators[index]
                / kPositionDenominators[index];
            if (offset == previous_offset) {
                continue;
            }
            merge_channel_capacity(
                capacity,
                inspect_channel_capacity_window(
                    options, offset, 0U, 0U, true));
            ++capacity.sampled_windows;
            previous_offset = offset;
        }
        return capacity;
    }

    const std::uint64_t duration =
        ffmpeg_input_duration_milliseconds(options);
    if (duration == 0U) {
        capacity = inspect_channel_capacity_window(
            options,
            0U,
            0U,
            kContainerWindowMilliseconds,
            true);
        capacity.sampled_windows = 1U;
        return capacity;
    }
    const std::uint64_t window =
        (std::min)(duration, kContainerWindowMilliseconds);
    const std::uint64_t span = duration - window;
    std::uint64_t previous_start =
        (std::numeric_limits<std::uint64_t>::max)();
    for (std::size_t index = 0U; index < kWindowCount; ++index) {
        const std::uint64_t start = span
            * kPositionNumerators[index]
            / kPositionDenominators[index];
        if (start == previous_start) {
            continue;
        }
        merge_channel_capacity(
            capacity,
            inspect_channel_capacity_window(
                options, 0U, start, window, true));
        ++capacity.sampled_windows;
        previous_start = start;
    }
    return capacity;
}

std::uint32_t count_physical_speakers(
    std::uint32_t mask) noexcept {
    std::uint32_t count = 0U;
    while (mask != 0U) {
        count += mask & 1U;
        mask >>= 1U;
    }
    return count;
}

std::optional<ChannelLayout> infer_metadata_layout(
    const ChannelCapacity& capacity) {
    const std::uint32_t reference =
        capacity.reference_speakers != 0U
            ? capacity.reference_speakers
            : capacity.coded_bed_speakers;
    const std::uint32_t speakers =
        reference | capacity.supplemental_speakers;
    const std::uint32_t channel_count =
        count_physical_speakers(speakers);
    constexpr std::array<std::string_view, 9U> kLayouts = {{
        "7.1.4",
        "5.1.4",
        "7.1.2",
        "5.1.2",
        "7.1",
        "5.1(side)",
        "5.1",
        "stereo",
        "mono",
    }};
    for (const std::string_view name : kLayouts) {
        const std::optional<ChannelLayout> candidate =
            find_layout(std::string(name));
        if (candidate
            && candidate->channels.size() == channel_count
            && missing_layout_channels(*candidate, speakers).empty()) {
            return candidate;
        }
    }
    return std::nullopt;
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
    if (options.dolby_output) {
        const bool color = console_style::color_enabled(stderr);
        console_style::paint(
            std::cerr, color, console_style::bold);
        console_style::paint(
            std::cerr, color, console_style::bright_yellow);
        std::cerr
            << "Order:  DOLBY; channel order differs from "
               "WAVEFORMATEXTENSIBLE, channel mask omitted";
        console_style::reset(std::cerr, color);
        std::cerr << '\n';
    }
    if (options.mono_tracks) {
        std::cerr << "Mono:   "
                  << options.mono_tracks_directory.string()
                  << '\n';
    }
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

void print_layout_recommendations(
    const ChannelLayout& layout,
    const ChannelCapacity& capacity) {
    const bool is_five_point_one =
        layout.name == "5.1" || layout.name == "5.1(side)";
    const bool is_seven_point_one = layout.name == "7.1";
    if (!is_five_point_one && !is_seven_point_one) {
        return;
    }
    if (capacity.any_positive_height_object) {
        std::cerr
            << "Recommendation: objects with positive elevation were "
               "detected; use the "
            << (is_five_point_one ? "5.1.4" : "7.1.4") << ".\n";
    }
    if (has_single_static_center_imax_object(capacity)) {
        std::cerr
            << "Warning: one static center object was detected with the "
               "IMAX metadata flag. This is a virtual IMAX height channel; "
               "the recommended layout is "
            << (is_five_point_one ? "5.1.4" : "7.1.4") << ".\n";
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

std::string format_probe_bytes(std::uint64_t bytes) {
    constexpr std::uint64_t kKiB = 1024U;
    constexpr std::uint64_t kMiB = 1024U * 1024U;
    std::ostringstream output;
    if (bytes >= kMiB) {
        output << std::fixed << std::setprecision(2)
               << static_cast<long double>(bytes)
                      / static_cast<long double>(kMiB)
               << " MiB";
    } else if (bytes >= kKiB) {
        output << std::fixed << std::setprecision(2)
               << static_cast<long double>(bytes)
                      / static_cast<long double>(kKiB)
               << " KiB";
    } else {
        output << bytes << " B";
    }
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
        if (speakers[index] == (1U << 6U)) {
            output << "Cs";
            continue;
        }
        std::string_view name;
        if (dtsx::standard_speaker_name(speakers[index], name)) {
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
    std::size_t waveform_count = 0U;
    bool group_present = false;
    std::uint8_t group = 0U;
    bool spatial_group_present = false;
    std::uint8_t spatial_group = 0U;
    std::size_t maximum_point_sources = 0U;
    std::uint8_t maximum_extent_mode = 0U;
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
    info.waveform_count = (std::max)(
        info.waveform_count,
        static_cast<std::size_t>(object.preamble.waveform_count));
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
    info.maximum_extent_mode = (std::max)(
        info.maximum_extent_mode,
        object.preamble.extent_mode);
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

std::string format_probe_object_summary(
    const ProbeObjectInfo& object) {
    std::ostringstream output;
    if (object.maximum_extent_mode != 0U) {
        output << "extended source";
    } else if (object.maximum_point_sources > 1U) {
        output << "multi-point source";
    } else if (object.maximum_point_sources == 1U) {
        output << "point source";
    } else {
        output << "waveform object";
    }
    output << ", " << object.waveform_count
           << (object.waveform_count == 1U
                   ? " waveform"
                   : " waveforms");
    if (object.position_available) {
        output << ", "
               << (object.position_changed
                       ? "motion observed"
                       : "static in probe window");
        output << ", initial azimuth " << object.initial_azimuth
               << " deg, elevation " << object.initial_elevation
               << " deg, distance " << object.initial_distance;
    }
    if (object.group_present) {
        output << ", group "
               << static_cast<unsigned>(object.group);
    }
    if (object.spatial_group_present) {
        output << ", spatial group "
               << static_cast<unsigned>(object.spatial_group);
    }
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

template <typename Value>
void print_probe_warning_field(
    std::string_view name,
    const Value& value,
    bool color) {
    std::cout << "  " << std::left << std::setw(30)
              << std::setfill(' ') << name << ": ";
    console_style::paint(
        std::cout, color, console_style::bold);
    console_style::paint(
        std::cout, color, console_style::bright_red);
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

bool bed_channels_in_activity_order(
    const std::vector<std::vector<std::int32_t>>& channels,
    const std::vector<std::uint32_t>& channel_speaker_masks,
    std::uint32_t speaker_activity_mask,
    std::vector<std::vector<std::int32_t>>& ordered) {
    const std::vector<std::uint32_t> expected =
        dtsx::expand_speaker_activity_mask(speaker_activity_mask);
    if (expected.empty()
        || channels.size() != channel_speaker_masks.size()
        || channels.size() < expected.size()) {
        return false;
    }
    const std::size_t frame_count = channels.front().size();
    ordered.assign(
        expected.size(),
        std::vector<std::int32_t>(frame_count, 0));
    for (std::size_t output = 0U; output < expected.size(); ++output) {
        bool found = false;
        for (std::size_t source = 0U;
             source < channel_speaker_masks.size();
             ++source) {
            if (channel_speaker_masks[source] != expected[output]) {
                continue;
            }
            if (channels[source].size() != frame_count) {
                return false;
            }
            ordered[output] = channels[source];
            found = true;
            break;
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

bool remap_layout_channels(
    const std::vector<std::vector<std::int32_t>>& input,
    const ChannelLayout& input_layout,
    const ChannelLayout& output_layout,
    std::vector<std::vector<std::int32_t>>& output) {
    if (input.size() != input_layout.channels.size()
        || input.empty()) {
        return false;
    }
    const std::size_t frame_count = input.front().size();
    output.assign(
        output_layout.channels.size(),
        std::vector<std::int32_t>(frame_count, 0));
    for (std::size_t out_index = 0U;
         out_index < output_layout.channels.size();
         ++out_index) {
        const auto found = std::find(
            input_layout.channels.begin(),
            input_layout.channels.end(),
            output_layout.channels[out_index]);
        if (found == input_layout.channels.end()) {
            continue;
        }
        const std::size_t in_index = static_cast<std::size_t>(
            std::distance(input_layout.channels.begin(), found));
        if (input[in_index].size() != frame_count) {
            return false;
        }
        output[out_index] = input[in_index];
    }
    return true;
}

class ParmaObjectDelay final {
public:
    [[nodiscard]] bool process(
        std::vector<std::vector<std::int32_t>>& channels) {
        if (channels.empty()) {
            return false;
        }
        const std::size_t frame_count = channels.front().size();
        if (frame_count == 0U) {
            return false;
        }
        for (const auto& channel : channels) {
            if (channel.size() != frame_count) {
                return false;
            }
        }
        if (!initialized_) {
            delay_.assign(
                channels.size(),
                std::vector<std::int32_t>(kLatencySamples, 0));
            position_ = 0U;
            initialized_ = true;
        }
        if (delay_.size() != channels.size()) {
            return false;
        }
        for (std::size_t sample = 0U; sample < frame_count; ++sample) {
            for (std::size_t channel = 0U;
                 channel < channels.size();
                 ++channel) {
                const std::int32_t delayed = delay_[channel][position_];
                delay_[channel][position_] = channels[channel][sample];
                channels[channel][sample] = delayed;
            }
            position_ = (position_ + 1U) % kLatencySamples;
        }
        return true;
    }

private:
    // dtsPlayerParmaDelayController_GetZeroBed uses
    // DTS_ParmaDec_GetLatency_Samples() for the 1024/64 OSFB path.
    static constexpr std::size_t kLatencySamples =
        parma_filterbank_latency_samples();
    std::vector<std::vector<std::int32_t>> delay_;
    std::size_t position_ = 0U;
    bool initialized_ = false;
};

bool apply_parma_upmix(
    ParmaBlindRenderer& blind_upmixer,
    ParmaGuidedRenderer& guided_upmixer,
    const std::vector<std::vector<std::int32_t>>& bed_channels,
    std::uint32_t bed_speaker_activity_mask,
    const std::optional<dtsx::CombinedMixMetadata>& guided_metadata,
    std::uint32_t sample_rate,
    const ChannelLayout& output_layout,
    std::vector<std::vector<std::int32_t>>& upmixed) {
    if (dtsx::has_height_channels(bed_speaker_activity_mask)) {
        throw std::runtime_error(
            "--upmix requires a horizontal bed (2.0/5.1/7.1); "
            "source already has height channels");
    }
    const std::optional<ChannelLayout> layout_7_1_4 =
        find_layout("7.1.4");
    if (!layout_7_1_4) {
        return false;
    }
    ParmaGuidedControls guided_controls;
    if (guided_metadata) {
        if (!derive_parma_guided_controls(
                *guided_metadata,
                bed_speaker_activity_mask,
                layout_speaker_activity_mask(output_layout),
                guided_controls)) {
            throw std::runtime_error(
                "native PARMA SetGuided rejected the metadata matrix");
        }
        ParmaGuidedTopology topology;
        bool supported_topology =
            build_parma_guided_topology(guided_controls, topology);
        for (std::uint32_t node = 0U;
             supported_topology && node < topology.node_count;
             ++node) {
            supported_topology = topology.nodes[node].mode <= 3U;
        }
        if (!supported_topology) {
            throw std::runtime_error(
                "native PARMA SetGuided topology is unsupported");
        }
        return guided_upmixer.render(
            bed_channels,
            bed_speaker_activity_mask,
            output_layout,
            guided_controls,
            sample_rate,
            upmixed);
    }
    ParmaLayoutControls controls;
    if (!derive_parma_layout_controls(
            bed_speaker_activity_mask,
            layout_speaker_activity_mask(*layout_7_1_4),
            controls)
        || controls.blind_table_row < 9
        || controls.blind_table_row > 17
        || (controls.input_main_channel_mask != 0x01U
            && controls.input_main_channel_mask != 0x06U
            && controls.input_main_channel_mask != 0x07U
            && controls.input_main_channel_mask != 0x26U
            && controls.input_main_channel_mask != 0x27U
            && controls.input_main_channel_mask != 0x1EU
            && controls.input_main_channel_mask != 0x1FU
            && controls.input_main_channel_mask != 0x3FU
            && controls.input_main_channel_mask != 0xDFU)
        || controls.output_main_channel_mask != 0x360DFU) {
        throw std::runtime_error(
            "--upmix input bed is not represented by a verified PARMA "
            "blind row for 5.1.4/7.1.4 output");
    }
    std::vector<std::vector<std::int32_t>> upmixed_7_1_4;
    if (!blind_upmixer.render(
            bed_channels,
            bed_speaker_activity_mask,
            *layout_7_1_4,
            sample_rate,
            upmixed_7_1_4)) {
        return false;
    }
    if (output_layout.name == "7.1.4") {
        if (output_layout.channels == layout_7_1_4->channels) {
            upmixed = std::move(upmixed_7_1_4);
            return true;
        }
        return remap_layout_channels(
            upmixed_7_1_4,
            *layout_7_1_4,
            output_layout,
            upmixed);
    }
    if (output_layout.name == "5.1.4") {
        return remap_layout_channels(
            upmixed_7_1_4,
            *layout_7_1_4,
            output_layout,
            upmixed);
    }
    throw std::runtime_error(
        "--upmix currently supports --layout 5.1.4 or 7.1.4");
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
    const std::filesystem::path& input,
    const ChannelLayout& layout,
    OutputFormat format) {
    std::string safe_layout = layout.name;
    for (char& c : safe_layout) {
        if (c == '(' || c == ')') {
            c = '_';
        }
    }
    const wchar_t* extension =
        format == OutputFormat::Wave64 ? L".w64" : L".wav";
    return input.parent_path() / (input.stem().wstring() + L"_"
        + std::wstring(safe_layout.begin(), safe_layout.end()) + extension);
}

std::filesystem::path generated_ffmpeg_output_path(
    const Options& options) {
    if (options.output_explicit) {
        return options.output;
    }
    return options.input.parent_path()
        / (options.input.stem().wstring()
            + (options.output_format == OutputFormat::Wave64
                   ? L"_ffmpeg.w64"
                   : L"_ffmpeg.wav"));
}

std::string quoted_command_argument(
    const std::filesystem::path& value) {
    std::string text = value.u8string();
    std::string quoted;
    quoted.reserve(text.size() + 2U);
    quoted.push_back('"');
    quoted.append(text);
    quoted.push_back('"');
    return quoted;
}

void print_ffmpeg_decode_redirect(const Options& options) {
    const bool color = console_style::color_enabled(stderr);
    console_style::paint(
        std::cerr, color, console_style::bold);
    console_style::paint(
        std::cerr, color, console_style::bright_yellow);
    std::cerr << "This stream is not DTS:X. "
                 "Use FFmpeg for decoding:";
    console_style::reset(std::cerr, color);
    std::cerr << '\n';

    std::ostringstream command;
    command << "ffmpeg"
            << (options.overwrite ? " -y" : " -n")
            << " -i " << quoted_command_argument(options.input);
    if (!is_elementary_dts(options.input)) {
        command << " -map 0:a:" << options.audio_track;
    }
    if (options.duration_seconds != 0U) {
        command << " -t " << options.duration_seconds;
    }
    command << " -vn -sn -dn -c:a pcm_s24le ";
    if (options.output_format == OutputFormat::Wave64) {
        command << "-f w64 ";
    }
    command
            << quoted_command_argument(
                   generated_ffmpeg_output_path(options));

    console_style::paint(
        std::cerr, color, console_style::bold);
    console_style::paint(
        std::cerr, color, console_style::bright_cyan);
    std::cerr << "  " << command.str();
    console_style::reset(std::cerr, color);
    std::cerr << '\n';
}

void run_internal_probe(
    const Options& options,
    const AudioProbe& audio_probe,
    const ChannelLayout* layout) {
    constexpr std::uint64_t kQuickProbeDurationSeconds = 10U;

    ProgressReporter progress;
    const char* const progress_stage =
        options.full_probe ? "full probe" : "probe";
    progress.update(progress_stage, -1);
    std::uint64_t probe_total_bytes = 0U;
    std::uint64_t probe_total_milliseconds = 0U;
    if (options.full_probe) {
        if (is_elementary_dts(options.input)) {
            std::error_code size_error;
            probe_total_bytes =
                std::filesystem::file_size(options.input, size_error);
            if (size_error) {
                probe_total_bytes = 0U;
            }
        } else {
            probe_total_milliseconds =
                ffmpeg_input_duration_milliseconds(options);
        }
        if (probe_total_bytes != 0U
            || probe_total_milliseconds != 0U) {
            progress.update(progress_stage, 0);
        }
    }
    const ChannelCapacity sampled_capacity = options.full_probe
        ? ChannelCapacity{}
        : inspect_channel_capacity(options, true);
    DtsFrameReader reader(options);
    const bool probe_is_demuxer_bounded = reader.container_input();
    ObjectFrameDecoder object_decoder;
    DcaBedDecoder dca_bed_decoder;
    dtsx::ElementaryFrame frame;
    std::uint64_t frame_count = 0U;
    std::uint64_t core_frame_count = 0U;
    std::uint64_t core_byte_count = 0U;
    std::uint64_t extension_frame_count = 0U;
    std::uint64_t extension_byte_count = 0U;
    std::uint64_t uhd_frame_count = 0U;
    std::uint64_t uhd_byte_count = 0U;
    std::uint64_t xll_asset_count = 0U;
    std::uint64_t xll_hierarchical_channel_set_count = 0U;
    std::uint64_t xll_downmix_channel_set_count = 0U;
    std::uint32_t maximum_xll_channel_sets_per_asset = 0U;
    std::uint32_t detected_xll_pcm_bits = 0U;
    std::uint32_t detected_xll_storage_bits = 0U;
    std::uint64_t malformed_object_frame_count = 0U;
    std::uint64_t malformed_object_frame_bytes = 0U;
    std::uint64_t xll_pbr_fallback_frame_count = 0U;
    std::uint64_t xll_pbr_fallback_frame_bytes = 0U;
    std::uint32_t maximum_supplemental_xll_channels = 0U;
    std::uint32_t maximum_unmapped_supplemental_waveforms = 0U;
    std::uint64_t supplemental_activity_total_samples = 0U;
    std::uint64_t supplemental_activity_active_samples = 0U;
    std::uint64_t metadata_chunk_count = 0U;
    std::uint64_t metadata_body_parse_failure_count = 0U;
    std::uint64_t metadata_body_failure_declared_bytes = 0U;
    std::uint64_t raw_metadata_envelope_count = 0U;
    std::uint64_t raw_metadata_envelope_parse_failure_count = 0U;
    std::uint64_t raw_metadata_envelope_failure_bytes = 0U;
    std::uint64_t raw_metadata_crc_failure_count = 0U;
    std::uint64_t raw_metadata_crc_failure_bytes = 0U;
    std::uint64_t first_rejected_metadata_frame = 0U;
    std::uint64_t first_rejected_metadata_offset = 0U;
    std::uint64_t first_rejected_metadata_sample = 0U;
    std::uint64_t last_rejected_metadata_frame = 0U;
    std::uint64_t last_rejected_metadata_offset = 0U;
    std::uint64_t last_rejected_metadata_sample = 0U;
    bool nonempty_xll_metadata_chunk_seen = false;
    std::size_t maximum_object_count = 0U;
    std::set<std::uint32_t> object_ids;
    std::uint32_t detected_frame_duration = 0U;
    std::uint32_t detected_stream_sample_rate = 0U;
    std::uint32_t detected_bed_activity_mask = 0U;
    std::uint32_t detected_core_activity_mask = 0U;
    std::uint32_t detected_core_channel_count = 0U;
    std::uint32_t detected_core_pcm_bits = 0U;
    std::int32_t detected_core_bit_rate = 0;
    std::int32_t detected_core_profile = 0;
    std::int32_t detected_core_matrix_encoding = 0;
    bool detected_core_es_matrix_surround = false;
    bool detected_core_embedded_6ch = false;
    std::uint32_t detected_extension_channel_count = 0U;
    std::uint32_t detected_extension_pcm_bits = 0U;
    std::int32_t detected_extension_profile = 0;
    bool detected_extension_embedded_stereo = false;
    bool detected_extension_embedded_6ch = false;
    std::uint32_t detected_metadata_activity_mask = 0U;
    std::uint32_t detected_supplemental_activity_mask = 0U;
    SupplementalLayoutEvidence supplemental_layout_evidence;
    bool detected_imax_enhanced = false;
    bool detected_core_embedded_dtsx = false;
    std::uint64_t core_metadata_envelope_count = 0U;
    std::uint64_t core_metadata_element_count = 0U;
    std::uint64_t core_metadata_crc_failure_count = 0U;
    std::set<unsigned> core_metadata_chunk_ids;
    bool detected_uhd_type1_certified_content = false;
    bool detected_uhd_3d_object_metadata = false;
    std::uint32_t detected_dtsx_extension_sync_word = 0U;
    std::uint32_t ignored_unmapped_objects = 0U;
    std::uint64_t ignored_unmapped_object_occurrences = 0U;
    std::uint64_t frames_with_ignored_unmapped_objects = 0U;
    std::uint64_t inactive_object_frames_without_audio = 0U;
    std::uint64_t unmapped_missing_waveform_occurrences = 0U;
    std::uint64_t unmapped_channel_out_of_range_occurrences = 0U;
    std::uint64_t first_unmapped_frame = 0U;
    std::uint64_t first_unmapped_offset = 0U;
    std::uint64_t first_unmapped_sample = 0U;
    std::uint64_t last_unmapped_frame = 0U;
    std::uint64_t last_unmapped_offset = 0U;
    std::uint64_t last_unmapped_sample = 0U;
    std::uint32_t first_unmapped_waveform_id = 0U;
    std::uint32_t first_unmapped_requested_channel = 0U;
    std::uint32_t first_unmapped_available_channels = 0U;
    std::uint32_t detected_uhd_audio_chunk_id = 0U;
    std::map<std::uint32_t, ProbeObjectInfo> probe_objects;
    std::vector<std::vector<dtsx::XllChannelSetHeader>>
        probe_previous_xll_headers;
    dtsx::UhdFrameParserState probe_uhd_state;
    int last_probe_percent = 0;

    while (reader.read(frame)) {
        ++frame_count;
        if (options.full_probe) {
            int percent = -1;
            if (probe_total_bytes != 0U) {
                const std::uint64_t scanned_bytes = (std::min)(
                    probe_total_bytes,
                    frame.stream_offset + frame.bytes.size());
                percent = static_cast<int>(
                    scanned_bytes * 100U / probe_total_bytes);
            } else if (probe_total_milliseconds != 0U
                       && detected_frame_duration != 0U
                       && detected_stream_sample_rate != 0U) {
                std::uint64_t timed_frame_count =
                    uhd_frame_count != 0U
                        ? uhd_frame_count
                        : extension_frame_count != 0U
                        ? extension_frame_count
                        : core_frame_count;
                if (frame.packing == dtsx::StreamPacking::DtsUhd) {
                    ++timed_frame_count;
                } else if ((frame.packing
                                == dtsx::StreamPacking::ExtensionBigEndian
                            || frame.packing
                                == dtsx::StreamPacking::ExtensionLittleEndian)
                           && uhd_frame_count == 0U) {
                    ++timed_frame_count;
                } else if (extension_frame_count == 0U
                           && uhd_frame_count == 0U) {
                    ++timed_frame_count;
                }
                const std::uint64_t scanned_milliseconds =
                    timed_frame_count * detected_frame_duration * 1000U
                    / detected_stream_sample_rate;
                percent = static_cast<int>(
                    (std::min)(scanned_milliseconds,
                               probe_total_milliseconds)
                    * 100U / probe_total_milliseconds);
            }
            if (percent >= 0) {
                percent = (std::min)(percent, 99);
                last_probe_percent = (std::max)(
                    last_probe_percent, percent);
                progress.update(progress_stage, last_probe_percent);
            }
        }
        if (frame.packing == dtsx::StreamPacking::DtsUhd) {
            dtsx::UhdFrameHeader uhd;
            const dtsx::UhdHeaderParseResult result =
                dtsx::parse_uhd_frame_header(
                    frame.bytes, probe_uhd_state, uhd);
            if (result != dtsx::UhdHeaderParseResult::Complete) {
                ++malformed_object_frame_count;
                malformed_object_frame_bytes += frame.bytes.size();
                continue;
            }
            ++uhd_frame_count;
            uhd_byte_count += frame.bytes.size();
            detected_uhd_3d_object_metadata =
                detected_uhd_3d_object_metadata
                || uhd.has_3d_object_metadata;
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
                detected_uhd_type1_certified_content =
                    detected_uhd_type1_certified_content
                    || uhd.type1_certified_content;
                detected_imax_enhanced =
                    detected_imax_enhanced
                    || uhd.type1_certified_content;
            }
        } else if (frame.packing == dtsx::StreamPacking::ExtensionBigEndian
            || frame.packing == dtsx::StreamPacking::ExtensionLittleEndian) {
            ++extension_frame_count;
            extension_byte_count += frame.bytes.size();
            DcaDecodedBed decoded_bed;
            std::vector<DcaDecodedObjectAsset> lossy_object_assets;
            const bool internal_bed_decoded =
                dca_bed_decoder.decode_extension(
                    frame,
                    decoded_bed,
                    true,
                    &lossy_object_assets);
            const DcaExtensionStreamInfo& extension_info =
                dca_bed_decoder.extension_stream_info();
            if (extension_info.valid) {
                detected_extension_channel_count =
                    extension_info.channels;
                detected_extension_pcm_bits =
                    extension_info.source_pcm_bits;
                detected_extension_profile =
                    extension_info.profile;
                detected_extension_embedded_stereo =
                    extension_info.embedded_stereo;
                detected_extension_embedded_6ch =
                    extension_info.embedded_6ch;
                if (extension_info.sample_rate != 0U) {
                    detected_stream_sample_rate =
                        extension_info.sample_rate;
                }
                if (extension_info.speaker_activity_mask != 0U) {
                    detected_bed_activity_mask =
                        extension_info.speaker_activity_mask;
                }
            }
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
                malformed_object_frame_bytes += frame.bytes.size();
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
                    malformed_object_frame_bytes += frame.bytes.size();
                    asset_parse_failed = true;
                    break;
                }
                if ((asset.coding_components & (1U << 9U)) != 0U) {
                    ++xll_asset_count;
                    if (asset.xll_metadata_present
                        && !asset.xll_metadata_chunk_sizes.empty()) {
                        ++metadata_chunk_count;
                        nonempty_xll_metadata_chunk_seen =
                            nonempty_xll_metadata_chunk_seen
                            || std::any_of(
                                asset.xll_metadata_chunk_sizes.begin(),
                                asset.xll_metadata_chunk_sizes.end(),
                                [](std::uint32_t size) {
                                    return size != 0U;
                                });
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
                            detected_xll_pcm_bits =
                                (std::max)(
                                    detected_xll_pcm_bits,
                                    static_cast<std::uint32_t>(
                                        set_header.probe.bit_depth));
                            detected_xll_storage_bits =
                                (std::max)(
                                    detected_xll_storage_bits,
                                    static_cast<std::uint32_t>(
                                        set_header.probe
                                            .storage_bit_depth));
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
            }
            if (asset_parse_failed) {
                continue;
            }

            const std::vector<dtsx::MetadataChunkLocation>
                extension_chunks = dtsx::scan_metadata_chunks(
                    words.cursor(),
                    static_cast<std::uint32_t>(frame.bytes.size()),
                    static_cast<std::uint8_t>(header.asset_count));
            metadata_chunk_count += extension_chunks.size();
            if (has_core_hra_dtsx_envelope(frame)) {
                detected_core_embedded_dtsx = true;
                detected_dtsx_extension_sync_word = 0x02000850U;
                for (const dtsx::MetadataChunkLocation& chunk :
                     extension_chunks) {
                    ++core_metadata_envelope_count;
                    core_metadata_element_count +=
                        static_cast<std::uint32_t>(
                            chunk.envelope.elements.size());
                    if (!chunk.envelope.crc_valid) {
                        ++core_metadata_crc_failure_count;
                    }
                    for (const dtsx::MetadataElementHeader& element :
                         chunk.envelope.elements) {
                        core_metadata_chunk_ids.insert(
                            static_cast<unsigned>(element.chunk_id));
                    }
                }
            }

            DecodedObjectAudioFrame decoded;
            const auto lossy_base = make_xll_lossy_base(
                dca_bed_decoder.decoded_core());
            const ObjectFrameDecodeResult result =
                object_decoder.decode(
                    frame,
                    decoded,
                    lossy_base,
                    lossy_object_assets);
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
                    xll_pbr_fallback_frame_bytes += frame.bytes.size();
                    continue;
                }
                ++malformed_object_frame_count;
                malformed_object_frame_bytes += frame.bytes.size();
                continue;
            }
            const SupplementalLayoutObservation supplemental =
                supplemental_layout_observation(decoded);
            const bool supplemental_layout_confirmed =
                confirm_supplemental_layout(
                    supplemental_layout_evidence,
                    supplemental);
            if (supplemental_layout_confirmed) {
                maximum_supplemental_xll_channels =
                    (std::max)(
                        maximum_supplemental_xll_channels,
                        supplemental.channels);
                maximum_unmapped_supplemental_waveforms =
                    (std::max)(
                        maximum_unmapped_supplemental_waveforms,
                        supplemental.unmapped_waveforms);
                detected_supplemental_activity_mask |=
                    supplemental.speaker_activity_mask;
            }
            if (decoded.samples_per_channel != 0U) {
                supplemental_activity_total_samples +=
                    decoded.samples_per_channel;
                bool supplemental_active = false;
                for (std::size_t waveform = 0U;
                     waveform < decoded.waveform_channels.size()
                     && waveform
                            < decoded.waveform_is_supplemental.size();
                     ++waveform) {
                    if (!decoded.waveform_is_supplemental[waveform]) {
                        continue;
                    }
                    const auto& samples =
                        decoded.waveform_channels[waveform];
                    if (std::any_of(
                            samples.begin(),
                            samples.end(),
                            [](std::int32_t sample) {
                                return sample != 0;
                            })) {
                        supplemental_active = true;
                        break;
                    }
                }
                if (supplemental_layout_confirmed
                    && supplemental_active) {
                    supplemental_activity_active_samples +=
                        decoded.samples_per_channel;
                }
            }
            if (decoded.sample_rate != 0U) {
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
            ignored_unmapped_object_occurrences +=
                decoded.ignored_unmapped_objects;
            inactive_object_frames_without_audio +=
                decoded.inactive_objects_without_waveform_audio;
            unmapped_missing_waveform_occurrences +=
                decoded.unmapped_objects_missing_waveform;
            unmapped_channel_out_of_range_occurrences +=
                decoded.unmapped_objects_channel_out_of_range;
            if (decoded.ignored_unmapped_objects != 0U) {
                ++frames_with_ignored_unmapped_objects;
                const std::uint64_t sample_position =
                    extension_frame_count == 0U
                        ? 0U
                        : (extension_frame_count - 1U)
                            * detected_frame_duration;
                if (first_unmapped_frame == 0U) {
                    first_unmapped_frame = frame_count;
                    first_unmapped_offset = frame.stream_offset;
                    first_unmapped_sample = sample_position;
                    if (decoded.unmapped_object_detail_available) {
                        first_unmapped_waveform_id =
                            decoded.unmapped_object_waveform_id;
                        first_unmapped_requested_channel =
                            decoded.unmapped_object_requested_channel;
                        first_unmapped_available_channels =
                            decoded.unmapped_object_available_channels;
                    }
                }
                last_unmapped_frame = frame_count;
                last_unmapped_offset = frame.stream_offset;
                last_unmapped_sample = sample_position;
            }
            if (decoded.metadata_speaker_activity_mask != 0U) {
                detected_metadata_activity_mask =
                    decoded.metadata_speaker_activity_mask;
            }
            metadata_body_parse_failure_count +=
                decoded.metadata_body_parse_failures;
            metadata_body_failure_declared_bytes +=
                decoded.metadata_body_failure_declared_bytes;
            raw_metadata_envelope_count +=
                decoded.raw_metadata_envelopes;
            raw_metadata_envelope_parse_failure_count +=
                decoded.raw_metadata_envelope_parse_failures;
            raw_metadata_envelope_failure_bytes +=
                decoded.raw_metadata_envelope_failure_bytes;
            raw_metadata_crc_failure_count +=
                decoded.raw_metadata_crc_failures;
            raw_metadata_crc_failure_bytes +=
                decoded.raw_metadata_crc_failure_bytes;
            core_metadata_envelope_count +=
                decoded.core_metadata_envelopes;
            core_metadata_element_count +=
                decoded.core_metadata_elements;
            core_metadata_crc_failure_count +=
                decoded.core_metadata_crc_failures;
            for (const std::uint8_t chunk_id :
                 decoded.core_metadata_chunk_ids) {
                core_metadata_chunk_ids.insert(
                    static_cast<unsigned>(chunk_id));
            }
            const bool rejected_metadata_in_frame =
                decoded.metadata_body_parse_failures != 0U
                || decoded.raw_metadata_envelope_parse_failures != 0U
                || decoded.raw_metadata_crc_failures != 0U;
            if (rejected_metadata_in_frame) {
                const std::uint64_t sample_position =
                    extension_frame_count == 0U
                        ? 0U
                        : (extension_frame_count - 1U)
                            * detected_frame_duration;
                if (first_rejected_metadata_frame == 0U) {
                    first_rejected_metadata_frame = frame_count;
                    first_rejected_metadata_offset = frame.stream_offset;
                    first_rejected_metadata_sample = sample_position;
                }
                last_rejected_metadata_frame = frame_count;
                last_rejected_metadata_offset = frame.stream_offset;
                last_rejected_metadata_sample = sample_position;
            }
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
            core_byte_count += frame.bytes.size();
            if (has_core_hra_dtsx_envelope(frame)) {
                detected_core_embedded_dtsx = true;
                detected_dtsx_extension_sync_word = 0x02000850U;
            }
            dca_bed_decoder.remember_core(frame);
            object_decoder.remember_core_metadata(frame);
            const DcaCoreStreamInfo& core_info =
                dca_bed_decoder.core_stream_info();
            if (core_info.valid) {
                detected_core_activity_mask =
                    core_info.speaker_activity_mask;
                detected_core_channel_count = core_info.channels;
                detected_core_pcm_bits = core_info.source_pcm_bits;
                detected_core_bit_rate = core_info.bit_rate;
                detected_core_profile = core_info.profile;
                detected_core_matrix_encoding =
                    core_info.matrix_encoding;
                detected_core_es_matrix_surround =
                    core_info.es_matrix_surround;
                detected_core_embedded_6ch =
                    core_info.embedded_6ch;
                if (detected_stream_sample_rate == 0U) {
                    detected_stream_sample_rate =
                        core_info.sample_rate;
                }
                if (detected_frame_duration == 0U) {
                    detected_frame_duration =
                        core_info.samples_per_frame;
                }
            }
        }
        if (!options.full_probe
            && !probe_is_demuxer_bounded) {
            const std::uint64_t timed_frame_count =
                uhd_frame_count != 0U
                    ? uhd_frame_count
                    : extension_frame_count != 0U
                    ? extension_frame_count
                    : core_frame_count;
            if (detected_frame_duration != 0U
                && detected_stream_sample_rate != 0U
                && timed_frame_count * detected_frame_duration
                    >= kQuickProbeDurationSeconds
                        * detected_stream_sample_rate) {
                break;
            }
        }
    }

    if (!options.full_probe) {
        detected_bed_activity_mask |=
            dtsx::speaker_mask_to_activity_mask(
                sampled_capacity.coded_bed_speakers);
        detected_metadata_activity_mask |=
            dtsx::speaker_mask_to_activity_mask(
                sampled_capacity.reference_speakers);
        detected_supplemental_activity_mask |=
            dtsx::speaker_mask_to_activity_mask(
                sampled_capacity.supplemental_speakers);
        maximum_supplemental_xll_channels = (std::max)(
            maximum_supplemental_xll_channels,
            sampled_capacity.maximum_supplemental_channels);
        maximum_unmapped_supplemental_waveforms = (std::max)(
            maximum_unmapped_supplemental_waveforms,
            sampled_capacity
                .maximum_unmapped_supplemental_waveforms);
    }

    progress.done(progress_stage);

    const bool color = console_style::color_enabled(stdout);
    const std::uint32_t sample_rate =
        detected_stream_sample_rate != 0U
            ? detected_stream_sample_rate
            : audio_probe.sample_rate;
    if (detected_bed_activity_mask == 0U) {
        detected_bed_activity_mask =
            detected_core_activity_mask;
    }
    const std::uint64_t scanned_samples =
        (uhd_frame_count != 0U
             ? uhd_frame_count
             : extension_frame_count != 0U
             ? extension_frame_count
             : core_frame_count)
        * detected_frame_duration;
    const bool dtsx_profile =
        uhd_frame_count != 0U
        || metadata_chunk_count != 0U
        || raw_metadata_envelope_count != 0U
        || detected_dtsx_extension_sync_word != 0U
        || detected_core_embedded_dtsx
        || detected_imax_enhanced;
    const bool channel_based_dtsx =
        dtsx_profile
        && uhd_frame_count == 0U
        && maximum_object_count == 0U
        && maximum_supplemental_xll_channels != 0U;

    console_style::paint(
        std::cout, color, console_style::bold);
    console_style::paint(
        std::cout, color, console_style::white);
    std::cout << (dtsx_profile ? "DTS:X" : "DTS")
              << (options.full_probe
                      ? " full stream probe"
                      : " stream probe");
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
    constexpr std::int32_t kDcaProfileDs96_24 = 0x02;
    constexpr std::int32_t kDcaProfileDsEs = 0x04;
    constexpr std::int32_t kDcaProfileHdHra = 0x08;
    constexpr std::int32_t kDcaProfileExpress = 0x20;
    constexpr std::int32_t kDcaMatrixSurround = 1;
    const bool core_xch =
        detected_core_profile == kDcaProfileDsEs
        && detected_core_channel_count == 7U
        && (detected_core_activity_mask & (1U << 4U)) != 0U;
    if (detected_core_profile == kDcaProfileDsEs) {
        coding += core_xch
            ? " + DTS-ES XCh"
            : " + DTS-ES XXCh";
    } else if (detected_core_es_matrix_surround) {
        coding += " (DTS-ES Matrix)";
    }
    if (xll_asset_count != 0U) {
        coding += " + DTS-HD MA/XLL";
    } else if (detected_extension_profile == kDcaProfileHdHra) {
        coding += " + DTS-HD High Resolution";
    } else if (detected_extension_profile == kDcaProfileExpress) {
        coding += " + DTS Express/LBR";
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
            ? detected_uhd_type1_certified_content
                ? "DTS:X Profile 2 with IMAX Enhanced"
                : "DTS:X Profile 2"
            : detected_imax_enhanced
            ? "IMAX Enhanced"
            : metadata_chunk_count != 0U
                  || raw_metadata_envelope_count != 0U
                  || detected_dtsx_extension_sync_word != 0U
            ? "DTS:X"
            : xll_asset_count != 0U
            ? "DTS-HD Master Audio"
            : detected_extension_profile == kDcaProfileHdHra
            ? "DTS-HD High Resolution Audio"
            : detected_extension_profile == kDcaProfileExpress
            ? "DTS Express"
            : detected_core_profile == kDcaProfileDsEs
            ? core_xch
                ? "DTS-ES 6.1 Discrete"
                : "DTS-ES Discrete"
            : detected_core_es_matrix_surround
            ? "DTS-ES Matrix"
            : detected_core_profile == kDcaProfileDs96_24
            ? "DTS 96/24"
            : "DTS Digital Surround");
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
        if (const char* extension_profile = alternate_profile_name(
                detected_dtsx_extension_sync_word)) {
            print_probe_field(
                "DTS:X extension profile",
                extension_profile);
        }
    }
    if (detected_core_embedded_dtsx) {
        print_probe_field(
            "Core DTS:X envelope",
            "0x3a429b0a + 0x02000850 (HRA)");
        print_probe_field(
            "Core metadata envelopes",
            core_metadata_envelope_count);
        print_probe_field(
            "Core metadata elements",
            core_metadata_element_count);
        if (!core_metadata_chunk_ids.empty()) {
            std::ostringstream chunk_ids;
            bool first = true;
            for (const unsigned chunk_id : core_metadata_chunk_ids) {
                if (!first) {
                    chunk_ids << ',';
                }
                first = false;
                chunk_ids << chunk_id;
            }
            print_probe_field(
                "Core metadata chunk IDs",
                chunk_ids.str());
        }
        if (core_metadata_crc_failure_count != 0U) {
            print_probe_field(
                "Core metadata CRC failures",
                core_metadata_crc_failure_count);
        }
        const bool type241 =
            core_metadata_chunk_ids.find(241U)
            != core_metadata_chunk_ids.end();
        print_probe_field(
            "Core type-241 objects",
            type241
                ? "parsed"
                : core_metadata_envelope_count == 0U
                ? "envelope not parsed"
                : "absent (channel-based HRA)");
    }
    if (detected_imax_enhanced) {
        if (detected_uhd_type1_certified_content) {
            print_probe_field("IMAX certification", "T1-CC");
        } else {
            print_probe_field(
                "IMAX metadata version",
                "not exposed by the recognized D0 header");
        }
    }
    print_probe_field("Sample rate", sample_rate);
    print_probe_field(
        "Embedded DTS Core",
        core_frame_count != 0U && extension_frame_count != 0U
            ? "yes"
            : "no");
    if (detected_extension_channel_count != 0U) {
        print_probe_field(
            "Encoded channels",
            detected_extension_channel_count);
    }
    if (detected_core_channel_count != 0U) {
        print_probe_field(
            "Core output channels",
            detected_core_channel_count);
    }
    const std::uint32_t source_pcm_bits =
        detected_xll_pcm_bits != 0U
            ? detected_xll_pcm_bits
        : detected_extension_pcm_bits != 0U
            ? detected_extension_pcm_bits
            : detected_core_pcm_bits;
    if (source_pcm_bits != 0U) {
        print_probe_field(
            "Source PCM resolution",
            std::to_string(source_pcm_bits) + " bit");
    }
    if (detected_xll_storage_bits != 0U
        && detected_xll_storage_bits != source_pcm_bits) {
        print_probe_field(
            "XLL storage depth",
            std::to_string(detected_xll_storage_bits) + " bit");
    }
    if (detected_core_bit_rate > 0) {
        print_probe_field(
            "Core bit rate",
            std::to_string(detected_core_bit_rate / 1000)
                + " kb/s");
    }
    if (sample_rate != 0U && detected_frame_duration != 0U) {
        const std::uint64_t hd_stream_bytes =
            extension_byte_count != 0U
                ? core_byte_count + extension_byte_count
                : uhd_byte_count;
        const std::uint64_t hd_stream_frames =
            extension_frame_count != 0U
                ? extension_frame_count
                : uhd_frame_count;
        if (hd_stream_bytes != 0U && hd_stream_frames != 0U) {
            const std::uint64_t hd_stream_samples =
                hd_stream_frames * detected_frame_duration;
            const std::uint64_t hd_average_bit_rate =
                (hd_stream_bytes * 8ULL * sample_rate)
                / hd_stream_samples;
            print_probe_field(
                "HD average bit rate",
                std::to_string(hd_average_bit_rate / 1000ULL)
                    + " kb/s");
        }
    }
    if (detected_core_profile == kDcaProfileDsEs) {
        print_probe_field(
            "Core extension",
            core_xch ? "XCh discrete Cs" : "XXCh");
    }
    if ((detected_core_profile != kDcaProfileDsEs
         && detected_core_es_matrix_surround)
        || detected_core_matrix_encoding == kDcaMatrixSurround) {
        print_probe_field("Matrix surround", "DTS-ES encoded");
    }
    print_probe_field(
        options.full_probe
            ? "Scanned duration"
            : "Quick probe window",
        format_probe_duration(scanned_samples, sample_rate));
    if (!options.full_probe) {
        print_probe_field(
            "Layout scan windows",
            sampled_capacity.sampled_windows);
    }
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
        std::ostringstream supplemental_summary;
        supplemental_summary
            << maximum_supplemental_xll_channels;
        if (options.full_probe
            && supplemental_activity_total_samples != 0U) {
            const long double activity_percent =
                100.0L
                * static_cast<long double>(
                    supplemental_activity_active_samples)
                / static_cast<long double>(
                    supplemental_activity_total_samples);
            supplemental_summary << " (active frames "
                                 << std::fixed
                                 << std::setprecision(
                                        activity_percent < 0.01L
                                            ? 4
                                            : activity_percent < 1.0L
                                            ? 2
                                            : 1)
                                 << activity_percent << "%)";
        }
        print_probe_field(
            "Supplemental XLL channels",
            supplemental_summary.str());
        print_probe_field(
            "Supplemental layout",
            detected_supplemental_activity_mask != 0U
                ? format_probe_speaker_layout(
                      detected_supplemental_activity_mask)
                : "not signalled (no supplemental speaker mapping)");
        if (maximum_unmapped_supplemental_waveforms != 0U) {
            print_probe_field(
                "Unmapped supplemental waveforms",
                maximum_unmapped_supplemental_waveforms);
        }
    }
    if (maximum_unmapped_supplemental_waveforms == 0U
        && maximum_supplemental_xll_channels == 0U) {
        const std::uint32_t profile_waveforms =
            alternate_profile_unmapped_waveform_count(
                detected_dtsx_extension_sync_word);
        if (profile_waveforms != 0U) {
            std::ostringstream summary;
            summary << profile_waveforms
                    << " (profile topology; PCM not decoded)";
            print_probe_field(
                "Unmapped supplemental waveforms",
                summary.str());
        }
    }
    print_probe_field(
        "Height in coded bed",
        dtsx::has_height_channels(detected_bed_activity_mask)
            ? "yes"
            : "no");
    print_probe_field(
        "Embedded downmix",
        xll_downmix_channel_set_count != 0U
                || detected_core_embedded_6ch
                || detected_extension_embedded_stereo
                || detected_extension_embedded_6ch
            ? "present"
            : "not declared");
    if (xll_asset_count != 0U) {
        print_probe_field(
            "XLL channel-set structure",
            xll_hierarchical_channel_set_count != 0U
                ? "hierarchical"
                : "independent");
        print_probe_field(
            "Maximum sets per asset",
            maximum_xll_channel_sets_per_asset);
    }

    if (dtsx_profile) {
        print_probe_section("Dynamic objects", color);
        print_probe_highlighted_field(
            "Dynamic objects", object_ids.size(), color);
        if (channel_based_dtsx) {
            print_probe_field(
                "Object metadata",
                "absent (channel-based DTS:X)");
        }
        if (detected_uhd_3d_object_metadata) {
            print_probe_field(
                "3D object metadata",
                "present (native 3D representation)");
        }
        const std::size_t object_ids_available =
            static_cast<std::size_t>(std::count_if(
                probe_objects.begin(),
                probe_objects.end(),
                [](const auto& entry) {
                    return entry.second.id_available;
                }));
        print_probe_field(
            "Object IDs",
            object_ids_available == 0U
                ? "not signalled"
                : object_ids_available == probe_objects.size()
                    ? "signalled"
                    : std::to_string(object_ids_available)
                        + "/"
                        + std::to_string(probe_objects.size())
                        + " signalled");
        if (ignored_unmapped_objects != 0U) {
            print_probe_field(
                "Ignored/unmapped max per frame",
                ignored_unmapped_objects);
        }
        if (inactive_object_frames_without_audio != 0U) {
            print_probe_field(
                "Inactive object frames (no waveform)",
                inactive_object_frames_without_audio);
        }
        if (maximum_object_count != object_ids.size()) {
            print_probe_field(
                "Maximum objects per frame",
                maximum_object_count);
        }
        for (const auto& entry : probe_objects) {
            const ProbeObjectInfo& object = entry.second;
            std::ostringstream label;
            if (object.id_available) {
                label << "Object ID " << object.id;
            } else {
                label << "Object #" << object.id;
            }
            print_probe_field(
                label.str(),
                format_probe_object_summary(object));
        }
    }

    if (options.full_probe) {
        const std::uint64_t rejected_metadata_bytes =
            raw_metadata_envelope_failure_bytes
            +
            raw_metadata_crc_failure_bytes
            + metadata_body_failure_declared_bytes;
        const bool has_unresolved_data =
            rejected_metadata_bytes != 0U
            || malformed_object_frame_bytes != 0U
            || xll_pbr_fallback_frame_bytes != 0U
            || raw_metadata_crc_failure_bytes != 0U
            || raw_metadata_envelope_failure_bytes != 0U
            || metadata_body_failure_declared_bytes != 0U
            || ignored_unmapped_object_occurrences != 0U
            || frames_with_ignored_unmapped_objects != 0U
            || unmapped_missing_waveform_occurrences != 0U
            || unmapped_channel_out_of_range_occurrences != 0U;
        if (has_unresolved_data) {
        print_probe_section("Unresolved data", color);
        print_probe_field(
            "Rejected metadata bytes",
            format_probe_bytes(rejected_metadata_bytes));
        print_probe_field(
            "Malformed frame source bytes",
            format_probe_bytes(malformed_object_frame_bytes));
        print_probe_field(
            "XLL fallback source bytes",
            format_probe_bytes(xll_pbr_fallback_frame_bytes));
        print_probe_field(
            "CRC-failed metadata bytes",
            format_probe_bytes(raw_metadata_crc_failure_bytes));
        print_probe_field(
            "Malformed envelope bytes",
            format_probe_bytes(raw_metadata_envelope_failure_bytes));
        print_probe_field(
            "Failed body declared bytes",
            format_probe_bytes(metadata_body_failure_declared_bytes));
        print_probe_field(
            "Unmapped object-frame instances",
            ignored_unmapped_object_occurrences);
        print_probe_field(
            "Frames with unmapped objects",
            frames_with_ignored_unmapped_objects);
        print_probe_field(
            "Missing waveform mapping",
            unmapped_missing_waveform_occurrences);
        print_probe_field(
            "Waveform channel out of range",
            unmapped_channel_out_of_range_occurrences);
        if (first_unmapped_frame != 0U) {
            print_probe_field(
                "First unmapped object",
                format_probe_duration(
                    first_unmapped_sample, sample_rate)
                    + ", frame "
                    + std::to_string(first_unmapped_frame)
                    + ", offset "
                    + std::to_string(first_unmapped_offset));
            print_probe_field(
                "Last unmapped object",
                format_probe_duration(
                    last_unmapped_sample, sample_rate)
                    + ", frame "
                    + std::to_string(last_unmapped_frame)
                    + ", offset "
                    + std::to_string(last_unmapped_offset));
            print_probe_field(
                "First unmapped mapping",
                "waveform "
                    + std::to_string(first_unmapped_waveform_id)
                    + ", channel "
                    + std::to_string(first_unmapped_requested_channel)
                    + ", available "
                    + std::to_string(first_unmapped_available_channels));
        }
        if (first_rejected_metadata_frame != 0U) {
            print_probe_field(
                "First rejected metadata",
                format_probe_duration(
                    first_rejected_metadata_sample, sample_rate)
                    + ", frame "
                    + std::to_string(first_rejected_metadata_frame)
                    + ", offset "
                    + std::to_string(first_rejected_metadata_offset));
            print_probe_field(
                "Last rejected metadata",
                format_probe_duration(
                    last_rejected_metadata_sample, sample_rate)
                    + ", frame "
                    + std::to_string(last_rejected_metadata_frame)
                    + ", offset "
                    + std::to_string(last_rejected_metadata_offset));
        }
        print_probe_field(
            "Unparsed waveform bytes",
            "not separately measurable; XLL frame failures are counted above");
        }
    }

    const std::uint64_t probe_failures =
        malformed_object_frame_count
        + metadata_body_parse_failure_count
        + raw_metadata_envelope_parse_failure_count
        + raw_metadata_crc_failure_count
        + xll_pbr_fallback_frame_count;
        const bool dtsx_metadata_possibly_absent =
        uhd_frame_count == 0U
        && maximum_object_count == 0U
        && !channel_based_dtsx
        && !detected_core_embedded_dtsx
        && !nonempty_xll_metadata_chunk_seen
        && (metadata_chunk_count != 0U
            || raw_metadata_envelope_count != 0U
            || detected_dtsx_extension_sync_word != 0U);
    if (probe_failures != 0U
        || dtsx_metadata_possibly_absent) {
        print_probe_section("Probe warnings", color);
        if (dtsx_metadata_possibly_absent) {
            print_probe_warning_field(
                "DTS:X metadata",
                "possibly absent",
                color);
        }
        if (probe_failures != 0U) {
            print_probe_field(
                "Malformed object frames",
                malformed_object_frame_count);
            print_probe_field(
                "Metadata body failures",
                metadata_body_parse_failure_count);
            print_probe_field(
                "Metadata envelope failures",
                raw_metadata_envelope_parse_failure_count);
            print_probe_field(
                "Raw metadata CRC failures",
                raw_metadata_crc_failure_count);
            print_probe_field(
                "XLL PBR fallback frames",
                xll_pbr_fallback_frame_count);
        }
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
    ProgressReporter progress;
    std::error_code size_error;
    const std::uint64_t input_size =
        std::filesystem::file_size(options.input, size_error);
    std::uint64_t input_bytes = 0U;
    progress.update("metadata", size_error ? -1 : 0);
    std::unique_ptr<ObjectSidecarWriter> coordinate_output;
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
        input_bytes += frame.bytes.size();
        progress.update(
            "metadata",
            size_error
                ? -1
                : decode_percent(input_bytes, input_size));
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
                           << ",\"contentTypePresent\":"
                           << (asset.content_type_present
                                   ? "true" : "false")
                           << ",\"contentType\":"
                           << static_cast<unsigned>(asset.content_type)
                           << ",\"type1CertifiedContent\":"
                           << (asset.type1_certified_content
                                   ? "true" : "false")
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
                    std::vector<DcaDecodedObjectAsset> lossy_object_assets;
                    if (has_lossy_object_asset(frame)) {
                        DcaDecodedBed ignored_bed;
                        (void)coordinate_bed_decoder.decode_extension(
                            frame,
                            ignored_bed,
                            false,
                            &lossy_object_assets);
                    }
                    if (coordinate_decoder.decode(
                            frame,
                            decoded_coordinates,
                            lossy_base,
                            lossy_object_assets)
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
                                        std::nullopt,
                                        ObjectMetadataGain{
                                            object.spatial_header.gain_present,
                                            object.spatial_header.gain_present
                                                ? object.spatial_header.gain_code
                                                : static_cast<std::uint8_t>(61U),
                                            object.spatial_header.gain_present
                                                ? object.spatial_header.gain_exponent
                                                : static_cast<std::uint8_t>(0U),
                                            decoded_coordinates
                                                .presentation_gain_code,
                                            decode_object_source_gain_q23(
                                                object.spatial_header.gain_present
                                                    ? object.spatial_header.gain_code
                                                    : static_cast<std::uint8_t>(61U),
                                                object.spatial_header.gain_present
                                                    ? object.spatial_header.gain_exponent
                                                    : static_cast<std::uint8_t>(0U),
                                                point.gain_code,
                                                decode_object_presentation_gain_q23(
                                                    decoded_coordinates
                                                        .presentation_gain_code))});
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
        } else if (is_core_stream_packing(frame.packing)) {
            coordinate_bed_decoder.remember_core(frame);
            coordinate_decoder.remember_core_metadata(frame);
            const bool swap = core_swap_byte_pairs(frame.packing);
            auto core_words =
                std::make_shared<dtsx::bitstream::WordBuffer>(
                    frame.bytes, swap);
            dtsx::bitstream::Cursor cursor = core_words->cursor();
            std::uint32_t core_size = 0;
            if (dtsx::validate_core_frame_header(cursor, core_size)) {
                dtsx::CoreSubstreamInfo core;
                cursor = core_words->cursor();
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
            std::vector<dtsx::MetadataChunkLocation> core_chunks =
                dtsx::scan_metadata_chunks(
                    core_words->cursor(),
                    static_cast<std::uint32_t>(frame.bytes.size()),
                    1U,
                    1U);
            for (dtsx::MetadataChunkLocation& chunk : core_chunks) {
                chunk.source_words = core_words;
            }
            write_metadata_chunks_json(
                output,
                "coreMetadataChunks",
                core_chunks,
                *core_words);
        }
        output << "}\n";
    }
    if (coordinate_output != nullptr) {
        coordinate_output->close();
    }
    progress.done("metadata");
}

void export_object_stems(
    const Options& options,
    bool require_audio = true,
    bool show_progress = true) {
    DtsFrameReader reader(options);
    ObjectFrameDecoder decoder;
    DcaBedDecoder dca_bed_decoder;
    std::unique_ptr<ObjectStemWriter> writer;
    dtsx::ElementaryFrame elementary;
    std::uint64_t sample_position = 0U;
    std::uint64_t sample_limit =
        (std::numeric_limits<std::uint64_t>::max)();
    std::uint32_t last_extension_frame_duration = 0U;
    bool wrote_audio = false;
    ProgressReporter progress;
    std::error_code size_error;
    const std::uint64_t input_size =
        std::filesystem::file_size(options.input, size_error);
    std::uint64_t input_bytes = 0U;
    if (show_progress) {
        progress.update(
            "objects",
            size_error ? -1 : 0);
    }

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

    struct StemWork final {
        DecodedObjectAudioFrame decoded;
        std::uint64_t sample_position = 0U;
    };
    const bool parallel = options.threads != 1U;
    OrderedDecodeSlot<StemWork> slot;
    const auto write_frame = [&](StemWork& work) {
        if (writer == nullptr) {
            writer = std::make_unique<ObjectStemWriter>(
                options.objects_output_directory,
                work.decoded.sample_rate,
                options.overwrite);
        }
        if (!writer->write(
                work.decoded,
                work.sample_position,
                work.decoded.samples_per_channel)) {
            throw std::runtime_error(
                "decoded object waveform channel is unavailable");
        }
        wrote_audio = writer->has_audio_stems();
    };
    std::thread writer_thread;
    if (parallel) {
        writer_thread = std::thread([&] {
            try {
                StemWork work;
                while (slot.take(work)) {
                    write_frame(work);
                }
            } catch (...) {
                slot.stop(std::current_exception());
            }
        });
    }

    try {
    while (!slot.stopped() && reader.read(elementary)) {
        input_bytes += elementary.bytes.size();
        if (show_progress) {
            progress.update(
                "objects",
                size_error
                    ? -1
                    : decode_percent(input_bytes, input_size));
        }
        if (elementary.packing
                != dtsx::StreamPacking::ExtensionBigEndian
            && elementary.packing
                != dtsx::StreamPacking::ExtensionLittleEndian) {
            dca_bed_decoder.remember_core(elementary);
            decoder.remember_core_metadata(elementary);
            continue;
        }
        DecodedObjectAudioFrame decoded;
        const auto lossy_base = make_xll_lossy_base(
            dca_bed_decoder.decoded_core());
        std::vector<DcaDecodedObjectAsset> lossy_object_assets;
        if (has_lossy_object_asset(elementary)) {
            DcaDecodedBed ignored_bed;
            if (!dca_bed_decoder.decode_extension(
                    elementary,
                    ignored_bed,
                    false,
                    &lossy_object_assets)) {
                throw std::runtime_error(
                    "lossy DTS:X object asset decode failed: "
                    + dca_bed_decoder.last_error());
            }
        }
        const ObjectFrameDecodeResult decode_result =
            decoder.decode(
                elementary,
                decoded,
                lossy_base,
                lossy_object_assets);
        if (decode_result != ObjectFrameDecodeResult::Decoded) {
            if (options.verbose) {
                std::cerr
                    << "Object stem frame skipped: samples="
                    << sample_position
                    << " result="
                    << (decode_result
                                == ObjectFrameDecodeResult::Malformed
                            ? "malformed"
                            : "ignored");
                if (!decoder.last_error().empty()) {
                    std::cerr << " reason=" << decoder.last_error();
                }
                std::cerr << '\n';
            }
            if (elementary.packing == dtsx::StreamPacking::ExtensionBigEndian
                || elementary.packing
                    == dtsx::StreamPacking::ExtensionLittleEndian) {
                advance_ignored_extension(elementary);
            }
            continue;
        }
        last_extension_frame_duration = decoded.samples_per_channel;
        if (sample_limit
            == (std::numeric_limits<std::uint64_t>::max)()) {
            sample_limit = duration_frame_limit(
                options, decoded.sample_rate);
        }
        if (sample_position >= sample_limit) {
            break;
        }
        const std::uint32_t frame_duration = static_cast<std::uint32_t>(
            std::min<std::uint64_t>(
                decoded.samples_per_channel,
                sample_limit - sample_position));
        if (frame_duration == 0U) {
            break;
        }
        if (frame_duration != decoded.samples_per_channel) {
            for (std::vector<std::int32_t>& channel :
                 decoded.waveform_channels) {
                if (channel.size() > frame_duration) {
                    channel.resize(frame_duration);
                }
            }
            decoded.samples_per_channel = frame_duration;
        }
        StemWork work{std::move(decoded), sample_position};
        if (parallel) {
            if (!slot.submit(std::move(work))) {
                break;
            }
        } else {
            write_frame(work);
        }
        sample_position += frame_duration;
        if (sample_position >= sample_limit) {
            break;
        }
    }
    } catch (...) {
        slot.stop(std::current_exception());
    }
    if (parallel) {
        slot.finish();
        writer_thread.join();
    }
    if (const std::exception_ptr error = slot.error()) {
        std::rethrow_exception(error);
    }
    if (writer != nullptr) {
        writer->close();
    }
    if (show_progress) {
        progress.done("objects");
    }
    if (require_audio && !wrote_audio) {
        throw std::runtime_error(
            "no decodable DTS:X object waveforms were found");
    }
}

bool map_xll_bed_to_layout(
    const DecodedObjectAudioFrame& decoded,
    const ChannelLayout& layout,
    std::vector<std::vector<std::int32_t>>& planar) {
    const bool fold_side_pair_to_back =
        std::find(
            layout.channels.begin(),
            layout.channels.end(),
            "SL")
        == layout.channels.end();
    const auto find_source =
        [&layout, fold_side_pair_to_back](
            const std::vector<std::uint32_t>& speaker_masks,
            std::size_t output) {
            std::uint32_t speaker_mask = 0U;
            if (!dtsx::standard_speaker_mask(
                    layout.channels[output], speaker_mask)) {
                return speaker_masks.size();
            }
            auto found = std::find(
                speaker_masks.begin(),
                speaker_masks.end(),
                speaker_mask);
            const auto try_speaker =
                [&speaker_masks, &found](std::uint32_t bit) {
                    if (found == speaker_masks.end()) {
                        found = std::find(
                            speaker_masks.begin(),
                            speaker_masks.end(),
                            1U << bit);
                    }
                };
            if (layout.channels[output] == "SL") {
                try_speaker(9U);
            } else if (layout.channels[output] == "SR") {
                try_speaker(10U);
            } else if (fold_side_pair_to_back
                       && layout.channels[output] == "BL") {
                try_speaker(3U);
                try_speaker(9U);
            } else if (fold_side_pair_to_back
                       && layout.channels[output] == "BR") {
                try_speaker(4U);
                try_speaker(10U);
            }
            return found == speaker_masks.end()
                ? speaker_masks.size()
                : static_cast<std::size_t>(std::distance(
                      speaker_masks.begin(), found));
        };
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
            source_by_output[output] =
                find_source(downmix.speaker_masks, output);
            if (source_by_output[output]
                == downmix.speaker_masks.size()) {
                exact_layout = false;
                break;
            }
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
        const std::size_t source =
            find_source(physical_masks, output);
        if (source == physical_masks.size()) {
            continue;
        }
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
    const std::vector<bool> semantically_mapped =
        semantically_mapped_waveforms(decoded);
    const std::vector<bool> object_mapped =
        object_mapped_waveforms(decoded);
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
        // DTSHD_UHDAssetDecoder keeps speaker beds and registered object
        // waveform decoders on separate buses.  A type-241 object waveform
        // is consumed only by dtsPlayerObjectRenderer; publishing the same
        // decoder again as a supplemental speaker channel duplicates it in
        // both normal rendering and ReverseRenderObjects output.
        if (waveform < object_mapped.size()
            && object_mapped[waveform]) {
            continue;
        }
        supplemental_available = true;
        if (decoded.waveform_channels[waveform].size()
                != decoded.samples_per_channel) {
            return false;
        }
        if (decoded.waveform_speaker_masks[waveform] == 0U) {
            if (duplicates_semantically_mapped_waveform(
                    decoded, waveform, semantically_mapped)) {
                mapped_waveforms[waveform] = true;
                continue;
            }
            // No native speaker destination exists for this decoder.  Keep
            // it available to object-stem/unmapped export, but do not invent
            // a height speaker by waveform ordinal.
            continue;
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
    apply_native_soft_linear_peak_limiter(channels);
}

std::uint64_t render_object_stream(
    const Options& options,
    const ChannelLayout& layout,
    std::uint32_t output_sample_rate,
    const std::filesystem::path& output_path,
    RenderMode render_mode,
    std::uint32_t* rendered_sample_rate = nullptr,
    ChannelCapacity* recommendation_capacity = nullptr) {
    DtsFrameReader reader(options);
    ObjectFrameDecoder decoder;
    DcaBedDecoder dca_bed_decoder;
    std::unique_ptr<ObjectAudioRenderer> renderer;
    ParmaBlindRenderer parma_upmixer;
    const std::unique_ptr<ParmaGuidedRenderer> parma_guided_upmixer =
        options.upmix
        ? std::make_unique<ParmaGuidedRenderer>()
        : nullptr;
    ParmaObjectDelay parma_object_delay;
    ImaxPostProcessor imax_post_processor;
    bool imax_stream_active = false;
    bool imax_profile_reported = false;
    ProgressReporter progress;
    std::error_code size_error;
    const std::uint64_t elementary_size =
        std::filesystem::file_size(options.input, size_error);
    std::uint64_t elementary_bytes = 0U;
    progress.update(
        "decode",
        size_error ? -1 : 0);
    std::unique_ptr<WavWriter> writer;
    std::unique_ptr<MonoTrackWriter> mono_writer;
    const bool parallel_decode = options.threads != 1U;
    const std::unique_ptr<BooleanTaskWorker> bed_worker =
        parallel_decode
        ? std::make_unique<BooleanTaskWorker>()
        : nullptr;
    std::uint64_t frame_limit =
        std::numeric_limits<std::uint64_t>::max();
    const auto write_limited =
        [&writer, &mono_writer, &frame_limit, &options, &progress](
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
            if (mono_writer != nullptr) {
                mono_writer->write_planar_24(
                    channels, frames_to_write);
            }
            if (options.duration_seconds != 0U) {
                progress.update(
                    "decode",
                    decode_percent(
                        writer->frames_written(), frame_limit));
            }
            return writer->frames_written() >= frame_limit;
        };
    dtsx::ElementaryFrame elementary;
    while (reader.read(elementary)) {
        elementary_bytes += elementary.bytes.size();
        if (options.duration_seconds == 0U) {
            progress.update(
                "decode",
                size_error
                    ? -1
                    : decode_percent(elementary_bytes, elementary_size));
        }
        DcaDecodedBed decoded_bed;
        DecodedObjectAudioFrame decoded;
        ObjectFrameDecodeResult decode_result =
            ObjectFrameDecodeResult::Ignored;
        bool decoded_bed_available = false;
        if (elementary.packing
                != dtsx::StreamPacking::ExtensionBigEndian
            && elementary.packing
                != dtsx::StreamPacking::ExtensionLittleEndian) {
            dca_bed_decoder.remember_core(elementary);
            decoder.remember_core_metadata(elementary);
            // DTSXDecFramePlayer_SAPI_DecStream_CoreSubStream stores the
            // core substream and publishes PCM only after the matching ExSS
            // asset has completed the same frame transaction.  Emitting the
            // remembered core here produced one extra PCM block before every
            // HRA/XLL extension block, doubling duration and alternating the
            // lossy 5.1 core with the decoded HD bed.
            continue;
        } else {
            const auto lossy_base = make_xll_lossy_base(
                dca_bed_decoder.decoded_core());
            std::vector<DcaDecodedObjectAsset> lossy_object_assets;
            const bool lossy_objects =
                has_lossy_object_asset(elementary);
            const bool lbr_asset = has_lbr_asset(elementary);
            std::future<bool> decoded_bed_future;
            if (parallel_decode && !lossy_objects) {
                decoded_bed_future = bed_worker->submit(
                    [&dca_bed_decoder,
                     &elementary,
                     &decoded_bed,
                     &options,
                     lbr_asset] {
                        return dca_bed_decoder.decode_extension(
                            elementary,
                            decoded_bed,
                            !options.upmix && !lbr_asset);
                    });
            } else {
                decoded_bed_available =
                    dca_bed_decoder.decode_extension(
                        elementary,
                        decoded_bed,
                        !options.upmix && !lbr_asset,
                        lossy_objects
                            ? &lossy_object_assets
                            : nullptr);
            }
            decode_result = decoder.decode(
                elementary,
                decoded,
                lossy_base,
                lossy_object_assets);
            if (parallel_decode && !lossy_objects) {
                decoded_bed_available = decoded_bed_future.get();
            }
        }
        if (recommendation_capacity != nullptr
            && decode_result == ObjectFrameDecodeResult::Decoded) {
            observe_object_recommendation_metadata(
                *recommendation_capacity, decoded);
        }
        imax_stream_active =
            imax_stream_active || decoded.imax_enhanced;
        std::vector<std::vector<std::int32_t>> upmix_dca_bed;
        std::uint32_t upmix_dca_activity_mask = 0U;
        const bool upmix_dca_ready =
            options.upmix
            && decoded_bed_available
            && bed_channels_in_activity_order(
                decoded_bed.channels,
                decoded_bed.channel_speaker_masks,
                decoded_bed.speaker_activity_mask,
                upmix_dca_bed);
        if (upmix_dca_ready) {
            upmix_dca_activity_mask =
                decoded_bed.speaker_activity_mask;
        }
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
                const std::string object_error =
                    decoder.last_error().empty()
                    ? "unspecified object-frame error"
                    : decoder.last_error();
                const std::string bed_error =
                    dca_bed_decoder.last_error().empty()
                    ? "unspecified bed error"
                    : dca_bed_decoder.last_error();
                throw std::runtime_error(
                    decode_result == ObjectFrameDecodeResult::Malformed
                        ? "internal DTS:X ExSS/XLL frame and bed are malformed"
                            " (object: " + object_error
                            + "; bed: " + bed_error
                            + "; stream offset: "
                            + std::to_string(elementary.stream_offset)
                            + ")"
                        : "internal DTS:X ExSS frame has no decodable audio"
                            " (object: " + object_error
                            + "; bed: " + bed_error
                            + "; stream offset: "
                            + std::to_string(elementary.stream_offset)
                            + ")");
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
            if (rendered_sample_rate != nullptr) {
                *rendered_sample_rate = decoded.sample_rate;
            }
            writer = std::make_unique<WavWriter>(
                output_path,
                layout,
                decoded.sample_rate,
                options.overwrite,
                options.output_format,
                options.dolby_output);
            if (options.mono_tracks) {
                mono_writer = std::make_unique<MonoTrackWriter>(
                    options.mono_tracks_directory,
                    layout,
                    decoded.sample_rate,
                    options.overwrite);
            }
            frame_limit =
                duration_frame_limit(options, decoded.sample_rate);
        }
        const auto apply_imax_profile =
            [&](std::vector<std::vector<std::int32_t>>& channels) {
                if (!options.imax_dsp || !imax_stream_active) {
                    return;
                }
                if (!imax_post_processor.process(
                        channels,
                        layout,
                        decoded.sample_rate,
                        true,
                        options.imax_small_speakers)) {
                    throw std::runtime_error(
                        "IMAX DSP profile cannot process the output layout");
                }
                if (options.verbose && !imax_profile_reported) {
                    std::cerr
                        << "IMAX DSP: recovered AVRx0 profile; exact 70 Hz "
                        << "LR4 crossover, relative +10 dB LFE trim; small="
                        << options.imax_small_speakers << '\n';
                    imax_profile_reported = true;
                }
            };
        if (render_mode != RenderMode::Bed
            && !decoded.objects.empty()) {
            if (renderer == nullptr) {
                renderer = std::make_unique<ObjectAudioRenderer>(
                    layout, options.verbose);
            }
            if (!renderer->remove_embedded_object_fold_down(decoded)) {
                throw std::runtime_error(
                    "native DTS:X embedded object fold-down removal failed");
            }
        }
        std::vector<std::vector<std::int32_t>> bed;
        bool rendered_supplemental = false;
        if (options.upmix) {
            std::vector<std::vector<std::int32_t>> bed_input;
            std::uint32_t bed_activity_mask = 0U;
            const auto activity_channel_count =
                [](std::uint32_t mask) {
                    return dtsx::expand_speaker_activity_mask(mask)
                        .size();
                };
            // Prefer native XLL bed when the object decoder produced one.
            // Otherwise use the libdcadec bed captured before any moves.
            if (decode_result == ObjectFrameDecodeResult::Decoded
                && !decoded.bed_channels.empty()
                && decoded.bed_speaker_activity_mask != 0U
                && decoded.bed_channels.size()
                    == activity_channel_count(
                        decoded.bed_speaker_activity_mask)) {
                bed_input = decoded.bed_channels;
                bed_activity_mask =
                    decoded.bed_speaker_activity_mask;
            } else if (upmix_dca_ready) {
                bed_input = std::move(upmix_dca_bed);
                bed_activity_mask = upmix_dca_activity_mask;
            } else if (!decoded.bed_channels.empty()
                       && decoded.bed_speaker_activity_mask != 0U
                       && decoded.bed_channels.size()
                           == activity_channel_count(
                               decoded.bed_speaker_activity_mask)) {
                bed_input = decoded.bed_channels;
                bed_activity_mask =
                    decoded.bed_speaker_activity_mask;
            }
            if (bed_input.empty() || bed_activity_mask == 0U) {
                throw std::runtime_error(
                    "--upmix requires a decoded horizontal channel bed");
            }
            if (!apply_parma_upmix(
                    parma_upmixer,
                    *parma_guided_upmixer,
                    bed_input,
                    bed_activity_mask,
                    decoded.parma_guided_metadata,
                    decoded.sample_rate,
                    layout,
                    bed)) {
                throw std::runtime_error(
                    "PARMA upmix failed for the current frame");
            }
        } else {
            rendered_supplemental =
                render_mode != RenderMode::Bed
                && map_supplemental_xll_to_height_layout(
                    decoded, layout, bed);
            if (!rendered_supplemental
                && !map_xll_bed_to_layout(decoded, layout, bed)) {
                throw std::runtime_error(
                    "native DTS:X XLL bed cannot be mapped to the requested "
                    "layout");
            }
        }
        if (render_mode == RenderMode::Bed
            || render_mode == RenderMode::BedWithoutObjects) {
            apply_imax_profile(bed);
            if (write_limited(bed)) {
                break;
            }
            continue;
        }
        if (!options.upmix && decoded.objects.empty()) {
            apply_imax_profile(bed);
            if (write_limited(bed)) {
                break;
            }
            continue;
        }
        std::vector<std::vector<std::int32_t>> rendered;
        if (options.upmix && decoded.objects.empty()) {
            // DTS_ParmaDec_GetLatency_Samples delays the PARMA bed by 1024
            // samples. Keep the object bus on the same timeline even during
            // frames with no object metadata; otherwise every object gap
            // would collapse the native delay line and create discontinuities
            // when the next object frame arrives.
            rendered.assign(
                bed.size(),
                std::vector<std::int32_t>(bed.front().size(), 0));
        } else {
            if (renderer == nullptr) {
                renderer = std::make_unique<ObjectAudioRenderer>(
                    layout, options.verbose);
            }
            if (!renderer->render(decoded, rendered)) {
                if (options.upmix) {
                    // An unrenderable object frame is still a zero object bus
                    // in the native mixer. Advance the delay line rather than
                    // shortening the preceding object tail.
                    rendered.assign(
                        bed.size(),
                        std::vector<std::int32_t>(bed.front().size(), 0));
                } else {
                    // Preserve a usable channel bed when an object is
                    // unresolved or its metadata cannot be rendered for the
                    // selected layout.
                    apply_imax_profile(bed);
                    if (write_limited(bed)) {
                        break;
                    }
                    continue;
                }
            }
        }
        if (options.upmix
            && !parma_object_delay.process(rendered)) {
            throw std::runtime_error(
                "PARMA object-delay alignment failed");
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
        apply_imax_profile(rendered);
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
    if (mono_writer != nullptr) {
        mono_writer->close();
    }
    progress.done("decode");
    return writer->frames_written();
}

void write_object_bed_descriptor(
    const std::filesystem::path& path,
    const ChannelLayout& layout,
    std::uint32_t sample_rate,
    std::uint64_t frames,
    bool overwrite) {
    if (std::filesystem::exists(path) && !overwrite) {
        throw std::runtime_error(
            "object bed descriptor exists; pass --overwrite to replace it");
    }
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("cannot open object bed descriptor");
    }
    output << "{\"type\":\"dtsx-object-viewer-bed\","
           << "\"version\":1,"
           << "\"kind\":\"multichannel_bed\","
           << "\"audioFile\":\"bed.wav\","
           << "\"containsObjects\":false,"
           << "\"sampleRate\":" << sample_rate << ','
           << "\"bitsPerSample\":24,"
           << "\"frames\":" << frames << ','
           << "\"layout\":\"" << layout.name << "\","
           << "\"channels\":[";
    for (std::size_t index = 0U;
         index < layout.channels.size();
         ++index) {
        if (index != 0U) {
            output << ',';
        }
        output << '"' << layout.channels[index] << '"';
    }
    output << "]}\n";
}

} // namespace

int run_pipeline(const Options& requested_options) {
    Options options = requested_options;
    if (!std::filesystem::exists(options.input)) {
        throw std::runtime_error("input file does not exist");
    }
    if (options.objects_output_bed
        && !options.objects_output_directory_explicit) {
        throw std::runtime_error(
            "--objects-output-bed requires --objects-output-dir");
    }
    if (options.objects_output_directory_explicit
        && options.objects_output_directory.empty()) {
        options.objects_output_directory = options.input.stem();
    }
    if (options.objects_output_directory_explicit
        && !options.objects_output_directory.is_absolute()) {
        const std::filesystem::path input_directory =
            options.input.parent_path().empty()
            ? std::filesystem::path(L".")
            : options.input.parent_path();
        options.objects_output_directory =
            input_directory / options.objects_output_directory;
    }
    if (options.mono_tracks) {
        const std::filesystem::path input_directory =
            options.input.parent_path().empty()
            ? std::filesystem::path(L".")
            : options.input.parent_path();
        options.mono_tracks_directory =
            input_directory / options.input.stem();
    }
    if (!supported_input_extension(options.input)) {
        throw std::runtime_error(
            "input extension must be .mkv, .mka, .mp4, .m2ts, .dts or .dtshd");
    }
    if (!is_elementary_dts(options.input)
        && !options.audio_track_explicit) {
        options.audio_track =
            select_default_dts_track(options);
    }
    const bool object_stems_only =
        options.objects_output_directory_explicit
        && options.layout.empty()
        && !options.output_explicit
        && !options.mono_tracks;
    std::optional<ChannelLayout> layout =
        options.layout.empty()
        ? std::nullopt
        : find_layout(options.layout);
    if (!options.layout.empty() && !layout) {
        throw std::runtime_error("unsupported --layout; supported: "
            + supported_layouts_text());
    }
    if (layout && options.dolby_output) {
        layout = dolby_ordered_layout(*layout);
    }
    AudioProbe probe;
    probe.stream_index = options.audio_track;
    probe.sample_rate = options.sample_rate;
    probe.channels = layout
        ? static_cast<std::uint32_t>(layout->channels.size())
        : 0U;
    probe.channel_layout = layout ? options.layout : std::string{};
    probe.codec_name = "dts";

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
    const DtsTrackRank selected_rank =
        inspect_selected_dts_profile(options);
    const bool core_hra_dtsx =
        selected_rank != DtsTrackRank::DtsX
        && inspect_core_hra_dtsx(options);
    const bool dts_lbr =
        selected_rank != DtsTrackRank::DtsX
        && !core_hra_dtsx
        && inspect_lbr_stream(options);
    if (selected_rank != DtsTrackRank::DtsX
        && !core_hra_dtsx
        && !dts_lbr) {
        if (!options.upmix
            || selected_rank == DtsTrackRank::None) {
            print_ffmpeg_decode_redirect(options);
            return 1;
        }
        if (options.verbose) {
            std::cerr
                << "note: non-DTS:X stream accepted for --upmix "
                   "PARMA bed path\n";
        }
    }
    const bool dts_uhd = input_is_dts_uhd(options);
    // P2 ACEW can expose independent stream-set PCM buses.  Metadata sidecars
    // still require the DTS:X object parser, while --objects-output-dir is
    // handled by the P2 decoder's verified internal bus path.
    if (dts_uhd && options.metadata_output_explicit) {
        throw std::runtime_error(
            "DTS-UHD stream metadata sidecars are not exposed by ACEW");
    }

    Options decode_options = options;
    ScopedTemporaryFile elementary_file;
    const bool distributed_capacity_scan =
        !layout || options.objects_output_directory_explicit;
    ChannelCapacity capacity;
    bool capacity_scanned = false;
    if (!is_elementary_dts(options.input)
        && distributed_capacity_scan) {
        capacity = inspect_channel_capacity(options, true);
        capacity_scanned = true;
    }
    if (!is_elementary_dts(options.input)) {
        elementary_file.set(temporary_elementary_path());
        demux_container(options, elementary_file.path());
        decode_options.input = elementary_file.path();
    }
    if (!capacity_scanned) {
        capacity = inspect_channel_capacity(
            decode_options,
            distributed_capacity_scan);
    }
    if (!layout && (!object_stems_only || options.objects_output_bed)) {
        layout = infer_metadata_layout(capacity);
        if (!layout) {
            throw std::runtime_error(
                "DTS:X reference + supplemental metadata layout is "
                "unsupported; specify --layout");
        }
        options.layout = layout->name;
        decode_options.layout = layout->name;
        if (options.dolby_output) {
            layout = dolby_ordered_layout(*layout);
        }
        probe.channels =
            static_cast<std::uint32_t>(layout->channels.size());
        probe.channel_layout = layout->name;
    }
    std::string capacity_warning;
    if (layout) {
        capacity_warning =
            channel_capacity_warning(
                *layout,
                capacity.physical_speakers,
                options.upmix
                    || (options.render_mode != RenderMode::Bed
                        && capacity.dynamic_objects));
    }

    if (dts_uhd) {
        const std::filesystem::path output =
            options.output_explicit
            ? options.output
            : options.objects_output_bed
            ? (options.objects_output_directory / L"bed.wav")
            : generated_output_path(
                  options.input, *layout, options.output_format);
        print_decode_summary(
            options, probe, *layout, output, capacity_warning);
        if (options.objects_output_bed && !options.output_explicit) {
            decode_options.output_format = OutputFormat::Wav;
        }
        decode_p2_stream(
            decode_options, output, !object_stems_only);
        return 0;
    }
    if (options.metadata_output_explicit) {
        dump_metadata(decode_options, options.metadata_output);
    }
    if (options.objects_output_directory_explicit) {
        std::future<void> object_stems_future;
        bool object_stems_parallel = false;
        if (capacity.any_objects || capacity.any_waveforms) {
            object_stems_parallel =
                options.objects_output_bed
                && options.threads != 1U;
            if (object_stems_parallel) {
                Options stem_options = decode_options;
                stem_options.threads = 1U;
                object_stems_future = std::async(
                    std::launch::async,
                    [stem_options, &options] {
                        export_object_stems(
                            stem_options,
                            !options.objects_output_bed,
                            false);
                    });
            } else {
                export_object_stems(
                    decode_options,
                    !options.objects_output_bed);
            }
        } else {
            std::cerr
                << "warning: no object waveforms found; skipping WAV export\n";
        }
        if (options.objects_output_bed) {
            if (!layout) {
                throw std::runtime_error(
                    "cannot infer layout for object bed output");
            }
            std::error_code directory_error;
            std::filesystem::create_directories(
                options.objects_output_directory,
                directory_error);
            if (directory_error) {
                throw std::runtime_error(
                    "cannot create object output directory: "
                    + directory_error.message());
            }
            const std::filesystem::path bed_path =
                options.objects_output_directory / L"bed.wav";
            std::uint32_t bed_sample_rate = 0U;
            Options bed_decode_options = decode_options;
            if (object_stems_parallel) {
                bed_decode_options.threads = 1U;
            }
            bed_decode_options.output_format = OutputFormat::Wav;
            bed_decode_options.mono_tracks = false;
            bed_decode_options.dolby_output = false;
            const std::optional<ChannelLayout> standard_bed_layout =
                options.dolby_output
                ? find_layout(layout->name)
                : layout;
            if (!standard_bed_layout) {
                throw std::runtime_error(
                    "cannot restore standard object bed layout");
            }
            const std::uint64_t bed_frames = render_object_stream(
                bed_decode_options,
                *standard_bed_layout,
                sample_rate,
                bed_path,
                RenderMode::BedWithoutObjects,
                &bed_sample_rate,
                nullptr);
            if (object_stems_parallel) {
                object_stems_future.get();
            }
            write_object_bed_descriptor(
                options.objects_output_directory / L"bed.json",
                *standard_bed_layout,
                bed_sample_rate,
                bed_frames,
                options.overwrite);
        } else if (object_stems_parallel) {
            object_stems_future.get();
        }
        if (object_stems_only) {
            return 0;
        }
    }

    const std::filesystem::path output = options.output_explicit
        ? options.output
        : generated_output_path(
              options.input, *layout, options.output_format);

    print_decode_summary(
        options, probe, *layout, output, capacity_warning);
    const std::uint64_t frames = render_object_stream(
        decode_options,
        *layout,
        sample_rate,
        output,
        options.render_mode,
        nullptr,
        &capacity);
    std::cerr << "Frames: " << frames << '\n';
    print_layout_recommendations(*layout, capacity);
    return 0;
}

} // namespace dtsx_decode
