#include "io/object_stem_writer.hpp"

#include "audio/layout.hpp"
#include "dtsx/object_waveform_map.hpp"
#include "dtsx/speaker_mask.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace dtsx_decode {
namespace {

std::uint8_t sparse_gain_code(
    const std::vector<dtsx::SixBitUpdate>& values,
    std::size_t index) noexcept {
    for (const dtsx::SixBitUpdate& value : values) {
        if (value.index == index) {
            return value.value;
        }
    }
    return 0U;
}

std::uint32_t peak_absolute_sample(
    const std::vector<std::int32_t>& samples) noexcept {
    std::uint32_t peak = 0U;
    for (const std::int32_t sample : samples) {
        const std::uint32_t magnitude = sample < 0
            ? static_cast<std::uint32_t>(
                  -static_cast<std::int64_t>(sample))
            : static_cast<std::uint32_t>(sample);
        peak = std::max(peak, magnitude);
    }
    return peak;
}

} // namespace

ObjectStemWriter::ObjectStemWriter(
    std::filesystem::path directory,
    std::uint32_t sample_rate,
    bool overwrite)
    : directory_(std::move(directory))
    , sample_rate_(sample_rate)
    , overwrite_(overwrite) {
    if (directory_.empty() || sample_rate_ == 0U) {
        throw std::runtime_error("invalid object stem output settings");
    }
    std::filesystem::create_directories(directory_);
}

std::filesystem::path ObjectStemWriter::stem_path(
    const StemKey& key,
    const wchar_t* extension) const {
    std::wostringstream name;
    name << L"object_" << std::setw(3) << std::setfill(L'0')
         << key.first << L"_waveform_" << std::setw(2)
         << key.second << extension;
    return directory_ / name.str();
}

WavWriter& ObjectStemWriter::wav(const StemKey& key) {
    auto found = wavs_.find(key);
    if (found != wavs_.end()) {
        return *found->second;
    }
    const auto mono = find_layout("mono");
    if (!mono) {
        throw std::runtime_error("mono layout is unavailable");
    }
    auto writer = std::make_unique<WavWriter>(
        stem_path(key, L".wav"),
        *mono,
        sample_rate_,
        overwrite_);
    WavWriter& result = *writer;
    wavs_.emplace(key, std::move(writer));
    return result;
}

ObjectSidecarWriter& ObjectStemWriter::coordinates(
    const StemKey& key) {
    auto found = coordinate_writers_.find(key);
    if (found != coordinate_writers_.end()) {
        return *found->second;
    }
    auto writer = std::make_unique<ObjectSidecarWriter>(
        stem_path(key, L".coordinates.jsonl"), overwrite_);
    ObjectSidecarWriter& result = *writer;
    coordinate_writers_.emplace(key, std::move(writer));
    return result;
}

bool ObjectStemWriter::write(
    const DecodedObjectAudioFrame& frame,
    std::uint64_t sample_position,
    std::uint32_t duration_samples) {
    if (sample_position
        > std::numeric_limits<std::uint64_t>::max()
              - duration_samples) {
        return false;
    }
    timeline_end_ = std::max(
        timeline_end_,
        sample_position + duration_samples);
    if (frame.objects.empty()) {
        // Bed and supplemental channels without an object metadata block are
        // not object stems.  Keeping them here would expose the coded bed as
        // object_000_waveform_* in --objects-output-dir.
        return true;
    }
    for (std::size_t object_index = 0;
         object_index < frame.objects.size();
         ++object_index) {
        const dtsx::ObjectMetadataBlock& object =
            frame.objects[object_index];
        const std::uint32_t object_id =
            object.object_id_available
            ? object.object_id
            : static_cast<std::uint32_t>(object_index);
        std::vector<std::uint32_t> channel_indices;
        if (!dtsx::object_waveform_channel_indices(
                object,
                channel_indices,
                &frame.waveform_base_by_id)) {
            for (std::uint32_t waveform = 0U;
                 waveform < object.preamble.waveform_count;
                 ++waveform) {
                const StemKey key{object_id, waveform};
                bool wrote_coordinates = false;
                for (const dtsx::PointSourceMetadata& point : object.points) {
                    if (point.waveform_index != waveform) {
                        continue;
                    }
                    coordinates(key).write(
                        sample_position,
                        duration_samples,
                        sample_rate_,
                        object_id,
                        waveform,
                        point,
                        object.preamble.metadata_mode,
                        dtsx::point_source_is_renderable(
                            object.preamble.metadata_mode, point),
                        std::nullopt);
                    wrote_coordinates = true;
                }
                if (!wrote_coordinates) {
                    coordinates(key).write_unavailable(
                        sample_position,
                        duration_samples,
                        sample_rate_,
                        object_id,
                        waveform,
                        "waveform_decoder_unavailable");
                }
            }
            continue;
        }
        for (std::size_t waveform = 0;
             waveform < channel_indices.size();
             ++waveform) {
            const std::uint32_t channel_index =
                channel_indices[waveform];
            if (channel_index >= frame.waveform_channels.size()
                || frame.waveform_channels[channel_index].size()
                    != duration_samples) {
                const StemKey key{
                    object_id,
                    static_cast<std::uint32_t>(waveform)};
                bool wrote_coordinates = false;
                for (const dtsx::PointSourceMetadata& point : object.points) {
                    if (point.waveform_index != waveform) {
                        continue;
                    }
                    coordinates(key).write(
                        sample_position,
                        duration_samples,
                        sample_rate_,
                        object_id,
                        static_cast<std::uint32_t>(waveform),
                        point,
                        object.preamble.metadata_mode,
                        dtsx::point_source_is_renderable(
                            object.preamble.metadata_mode, point),
                        std::nullopt);
                    wrote_coordinates = true;
                }
                if (!wrote_coordinates) {
                    coordinates(key).write_unavailable(
                        sample_position,
                        duration_samples,
                        sample_rate_,
                        object_id,
                        static_cast<std::uint32_t>(waveform),
                        "waveform_channel_unavailable");
                }
                continue;
            }
            const StemKey key{
                object_id,
                static_cast<std::uint32_t>(waveform),
            };
            WavWriter& stem = wav(key);
            if (stem.frames_written() > sample_position) {
                return false;
            }
            constexpr std::size_t kSilenceBlockSamples = 4096U;
            while (stem.frames_written() < sample_position) {
                const std::uint64_t missing =
                    sample_position - stem.frames_written();
                const std::size_t block_size =
                    static_cast<std::size_t>(
                        std::min<std::uint64_t>(
                            missing, kSilenceBlockSamples));
                stem.write_planar_24({
                    std::vector<std::int32_t>(block_size, 0),
                });
            }
            stem.write_planar_24(
                {frame.waveform_channels[channel_index]});
            const std::uint32_t peak_sample =
                peak_absolute_sample(
                    frame.waveform_channels[channel_index]);
            ObjectSidecarWriter& sidecar = coordinates(key);
            bool wrote_metadata = false;
            for (const dtsx::PointSourceMetadata& point :
                 object.points) {
                if (point.waveform_index == waveform) {
                    sidecar.write(
                        sample_position,
                        duration_samples,
                        sample_rate_,
                        object_id,
                        static_cast<std::uint32_t>(waveform),
                        point,
                        object.preamble.metadata_mode,
                        dtsx::point_source_is_renderable(
                            object.preamble.metadata_mode, point),
                        peak_sample);
                    wrote_metadata = true;
                }
            }
            const std::vector<std::uint32_t> metadata_speakers =
                dtsx::expand_speaker_activity_mask(
                    frame.metadata_speaker_activity_mask);
            if ((object.preamble.metadata_mode == 1U
                    || object.preamble.metadata_mode == 2U)
                && waveform < object.updates.size()) {
                const dtsx::WaveformMetadataUpdate& update =
                    object.updates[waveform];
                for (std::size_t destination = 0U;
                     destination < metadata_speakers.size();
                     ++destination) {
                    const std::uint8_t coherent = sparse_gain_code(
                        update.first_values, destination);
                    const std::uint8_t noncoherent = sparse_gain_code(
                        update.second_values, destination);
                    if (coherent != 0U || noncoherent != 0U) {
                        sidecar.write_destination(
                            sample_position,
                            duration_samples,
                            sample_rate_,
                            object_id,
                            static_cast<std::uint32_t>(waveform),
                            metadata_speakers[destination],
                            coherent,
                            noncoherent);
                        wrote_metadata = true;
                    }
                }
            } else if (object.preamble.metadata_mode == 3U
                       && waveform
                           < object.mode_three.waveforms.size()) {
                const dtsx::ModeThreeWaveformValue& value =
                    object.mode_three.waveforms[waveform];
                const std::size_t destination_count = std::min(
                    metadata_speakers.size(), value.values.size());
                for (std::size_t destination = 0U;
                     destination < destination_count;
                     ++destination) {
                    if (value.values[destination] != 0U) {
                        sidecar.write_destination(
                            sample_position,
                            duration_samples,
                            sample_rate_,
                            object_id,
                            static_cast<std::uint32_t>(waveform),
                            metadata_speakers[destination],
                            value.values[destination],
                            0U);
                        wrote_metadata = true;
                    }
                }
            }
            if (!wrote_metadata) {
                sidecar.write_unavailable(
                    sample_position,
                    duration_samples,
                    sample_rate_,
                    object_id,
                    static_cast<std::uint32_t>(waveform),
                    "spatial_metadata_unavailable");
            }
        }
    }
    return true;
}

void ObjectStemWriter::close() {
    for (auto& entry : wavs_) {
        constexpr std::size_t kSilenceBlockSamples = 4096U;
        while (entry.second->frames_written() < timeline_end_) {
            const std::uint64_t missing =
                timeline_end_ - entry.second->frames_written();
            const std::size_t block_size =
                static_cast<std::size_t>(
                    std::min<std::uint64_t>(
                        missing, kSilenceBlockSamples));
            entry.second->write_planar_24({
                std::vector<std::int32_t>(block_size, 0),
            });
        }
        entry.second->close();
    }
    for (auto& entry : coordinate_writers_) {
        entry.second->close();
    }
}

} // namespace dtsx_decode
