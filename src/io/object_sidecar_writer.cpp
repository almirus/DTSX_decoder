#include "io/object_sidecar_writer.hpp"

#include "dtsx/speaker_mask.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace dtsx_decode {
namespace {

bool contiguous(
    std::uint64_t start,
    std::uint64_t duration,
    std::uint64_t next) noexcept {
    return duration <= (std::numeric_limits<std::uint64_t>::max)() - start
        && start + duration == next;
}

bool same_coordinates(
    const dtsx::PointSourceMetadata& left,
    const dtsx::PointSourceMetadata& right) noexcept {
    return left.azimuth_code == right.azimuth_code
        && left.elevation_code == right.elevation_code
        && left.distance_code == right.distance_code
        && left.extended == right.extended
        && left.width_degrees == right.width_degrees
        && left.height_degrees == right.height_degrees
        && left.rotation_degrees == right.rotation_degrees;
}

bool same_point_state(
    const dtsx::PointSourceMetadata& left,
    const dtsx::PointSourceMetadata& right,
    std::uint8_t left_mode,
    std::uint8_t right_mode,
    bool left_renderable,
    bool right_renderable,
    const ObjectMetadataGain& left_gain,
    const ObjectMetadataGain& right_gain) noexcept {
    return same_coordinates(left, right)
        && left.point_source_index == right.point_source_index
        && left.waveform_index == right.waveform_index
        && left.coherent_rendering == right.coherent_rendering
        && left.source_type == right.source_type
        && left.gain_code == right.gain_code
        && left.snap_to_nearest_speaker
               == right.snap_to_nearest_speaker
        && left.coordinates.coordinate_system
               == right.coordinates.coordinate_system
        && left_mode == right_mode
        && left_renderable == right_renderable
        && left_gain.object_gain_present
               == right_gain.object_gain_present
        && left_gain.object_gain_code == right_gain.object_gain_code
        && left_gain.object_gain_exponent
               == right_gain.object_gain_exponent
        && left_gain.presentation_gain_code
               == right_gain.presentation_gain_code
        && left_gain.effective_gain_q23
               == right_gain.effective_gain_q23;
}

} // namespace

ObjectSidecarWriter::ObjectSidecarWriter(
    const std::filesystem::path& path, bool overwrite) {
    if (std::filesystem::exists(path) && !overwrite) {
        throw std::runtime_error(
            "object coordinate sidecar exists; pass --overwrite to replace it");
    }
    output_.open(path, std::ios::binary | std::ios::trunc);
    if (!output_) {
        throw std::runtime_error("cannot create object coordinate sidecar");
    }
    output_ << std::fixed << std::setprecision(6);
}

void ObjectSidecarWriter::ensure_header(
    std::uint32_t sample_rate,
    std::uint32_t object_id,
    std::uint32_t waveform_id) {
    if (header_written_) {
        if (sample_rate_ != sample_rate
            || object_id_ != object_id
            || waveform_id_ != waveform_id) {
            throw std::runtime_error(
                "object coordinate sidecar identity changed");
        }
        return;
    }
    sample_rate_ = sample_rate;
    object_id_ = object_id;
    waveform_id_ = waveform_id;
    header_written_ = true;
    output_ << "{\"type\":\"header\""
            << ",\"schema\":\"dtsx-object-coordinates\""
            << ",\"version\":2"
            << ",\"sampleRate\":" << sample_rate_
            << ",\"objectId\":" << object_id_
            << ",\"waveformId\":" << waveform_id_
            << ",\"coordinateSystem\":\"dtsx_spherical\"}\n";
}

void ObjectSidecarWriter::update_activity(
    std::uint64_t pts_samples,
    std::uint32_t duration_samples,
    std::optional<std::uint32_t> peak_sample) {
    if (!peak_sample.has_value()
        || activity_last_pts_samples_ == pts_samples) {
        return;
    }
    activity_last_pts_samples_ = pts_samples;
    const bool active = *peak_sample != 0U;
    if (!activity_open_
        || activity_active_ != active
        || !contiguous(
            activity_start_samples_,
            activity_duration_samples_,
            pts_samples)) {
        flush_activity();
        activity_open_ = true;
        activity_start_samples_ = pts_samples;
        activity_duration_samples_ = duration_samples;
        activity_peak_sample_ = *peak_sample;
        activity_active_ = active;
        return;
    }
    activity_duration_samples_ += duration_samples;
    activity_peak_sample_ = (std::max)(
        activity_peak_sample_, *peak_sample);
}

void ObjectSidecarWriter::flush_activity() {
    if (!activity_open_) {
        return;
    }
    output_ << "{\"type\":\"activity\""
            << ",\"startSamples\":" << activity_start_samples_
            << ",\"durationSamples\":" << activity_duration_samples_
            << ",\"active\":"
            << (activity_active_ ? "true" : "false")
            << ",\"peakSample\":" << activity_peak_sample_
            << "}\n";
    activity_open_ = false;
    activity_duration_samples_ = 0U;
    activity_peak_sample_ = 0U;
    activity_active_ = false;
}

void ObjectSidecarWriter::write_point_interval(
    const PendingPointState& state,
    std::uint64_t start_samples,
    std::uint64_t duration_samples,
    std::string_view interpolation) {
    const dtsx::PointSourceMetadata& point = state.point;
    const ObjectMetadataGain& gain = state.metadata_gain;
    output_ << "{\"type\":\"state\""
            << ",\"startSamples\":" << start_samples
            << ",\"durationSamples\":" << duration_samples
            << ",\"pointSourceIndex\":" << point.point_source_index
            << ",\"coordinateStatus\":\"decoded\""
            << ",\"azimuthCode\":" << point.azimuth_code
            << ",\"elevationCode\":" << point.elevation_code
            << ",\"distanceCode\":" << point.distance_code
            << ",\"azimuthDeg\":" << point.coordinates.azimuth_degrees
            << ",\"elevationDeg\":"
            << point.coordinates.elevation_degrees
            << ",\"distance\":" << point.coordinates.distance
            << ",\"sourceType\":"
            << static_cast<unsigned>(point.source_type)
            << ",\"coherentRendering\":"
            << (point.coherent_rendering ? "true" : "false")
            << ",\"metadataMode\":"
            << static_cast<unsigned>(state.metadata_mode)
            << ",\"renderable\":"
            << (state.renderable ? "true" : "false")
            << ",\"snapToNearestSpeaker\":"
            << (point.snap_to_nearest_speaker ? "true" : "false")
            << ",\"gainCode\":" << static_cast<unsigned>(point.gain_code)
            << ",\"objectGainPresent\":"
            << (gain.object_gain_present ? "true" : "false")
            << ",\"objectGainCode\":"
            << static_cast<unsigned>(gain.object_gain_code)
            << ",\"objectGainExponent\":"
            << static_cast<unsigned>(gain.object_gain_exponent)
            << ",\"presentationGainCode\":"
            << static_cast<unsigned>(gain.presentation_gain_code)
            << ",\"metadataGainQ23\":" << gain.effective_gain_q23
            << ",\"metadataGainLinear\":"
            << static_cast<double>(gain.effective_gain_q23) / 8388608.0;
    if (gain.effective_gain_q23 > 0) {
        output_ << ",\"metadataGainDb\":"
                << 20.0 * std::log10(
                       static_cast<double>(gain.effective_gain_q23)
                       / 8388608.0);
    } else {
        output_ << ",\"metadataGainDb\":null";
    }
    output_ << ",\"widthDeg\":" << point.width_degrees
            << ",\"heightDeg\":" << point.height_degrees
            << ",\"rotationDeg\":" << point.rotation_degrees
            << ",\"interpolation\":\"" << interpolation << "\"}\n";
}

void ObjectSidecarWriter::flush_point_state(
    PendingPointState state,
    bool coordinates_change) {
    if (state.duration_samples == 0U) {
        return;
    }
    if (coordinates_change
        && state.last_frame_duration != 0U
        && state.duration_samples > state.last_frame_duration) {
        const std::uint64_t hold_duration =
            state.duration_samples - state.last_frame_duration;
        write_point_interval(
            state, state.start_samples, hold_duration, "hold");
        write_point_interval(
            state,
            state.start_samples + hold_duration,
            state.last_frame_duration,
            "linear");
        return;
    }
    write_point_interval(
        state,
        state.start_samples,
        state.duration_samples,
        coordinates_change ? "linear" : "hold");
}

void ObjectSidecarWriter::write(
    std::uint64_t pts_samples,
    std::uint32_t duration_samples,
    std::uint32_t sample_rate,
    std::uint32_t object_id,
    std::uint32_t waveform_id,
    const dtsx::PointSourceMetadata& point,
    std::uint8_t metadata_mode,
    bool renderable,
    std::optional<std::uint32_t> peak_sample,
    const ObjectMetadataGain& metadata_gain) {
    if (closed_) {
        throw std::runtime_error("object coordinate sidecar is closed");
    }
    ensure_header(sample_rate, object_id, waveform_id);
    update_activity(pts_samples, duration_samples, peak_sample);

    const auto found = point_states_.find(point.point_source_index);
    if (found == point_states_.end()) {
        point_states_.emplace(
            point.point_source_index,
            PendingPointState{
                pts_samples,
                duration_samples,
                duration_samples,
                point,
                metadata_gain,
                metadata_mode,
                renderable});
        return;
    }
    PendingPointState& pending = found->second;
    const bool adjacent = contiguous(
        pending.start_samples,
        pending.duration_samples,
        pts_samples);
    if (adjacent
        && same_point_state(
            pending.point,
            point,
            pending.metadata_mode,
            metadata_mode,
            pending.renderable,
            renderable,
            pending.metadata_gain,
            metadata_gain)) {
        pending.duration_samples += duration_samples;
        pending.last_frame_duration = duration_samples;
        return;
    }
    const bool coordinates_change = adjacent
        && !same_coordinates(pending.point, point);
    flush_point_state(pending, coordinates_change);
    pending = PendingPointState{
        pts_samples,
        duration_samples,
        duration_samples,
        point,
        metadata_gain,
        metadata_mode,
        renderable};
}

void ObjectSidecarWriter::flush_destination_state(
    const PendingDestinationState& state) {
    if (state.duration_samples == 0U) {
        return;
    }
    float azimuth = 0.0F;
    float elevation = 0.0F;
    std::string_view speaker_name;
    const bool coordinates_available =
        dtsx::standard_speaker_coordinates(
            state.speaker_mask, azimuth, elevation, speaker_name);
    output_ << "{\"type\":\"destination\""
            << ",\"startSamples\":" << state.start_samples
            << ",\"durationSamples\":" << state.duration_samples
            << ",\"coordinateStatus\":\""
            << (coordinates_available
                    ? "speaker_destination"
                    : "speaker_mask_only")
            << "\",\"speakerMask\":" << state.speaker_mask;
    if (coordinates_available) {
        output_ << ",\"speaker\":\"" << speaker_name
                << "\",\"azimuthDeg\":" << azimuth
                << ",\"elevationDeg\":" << elevation;
    }
    output_ << ",\"coherentGainCode\":"
            << static_cast<unsigned>(state.coherent_gain_code)
            << ",\"noncoherentGainCode\":"
            << static_cast<unsigned>(state.noncoherent_gain_code)
            << "}\n";
}

void ObjectSidecarWriter::write_destination(
    std::uint64_t pts_samples,
    std::uint32_t duration_samples,
    std::uint32_t sample_rate,
    std::uint32_t object_id,
    std::uint32_t waveform_id,
    std::uint32_t speaker_mask,
    std::uint8_t coherent_gain_code,
    std::uint8_t noncoherent_gain_code) {
    if (closed_) {
        throw std::runtime_error("object coordinate sidecar is closed");
    }
    ensure_header(sample_rate, object_id, waveform_id);
    const auto found = destination_states_.find(speaker_mask);
    if (found == destination_states_.end()) {
        destination_states_.emplace(
            speaker_mask,
            PendingDestinationState{
                pts_samples,
                duration_samples,
                speaker_mask,
                coherent_gain_code,
                noncoherent_gain_code});
        return;
    }
    PendingDestinationState& pending = found->second;
    if (contiguous(
            pending.start_samples,
            pending.duration_samples,
            pts_samples)
        && pending.coherent_gain_code == coherent_gain_code
        && pending.noncoherent_gain_code == noncoherent_gain_code) {
        pending.duration_samples += duration_samples;
        return;
    }
    flush_destination_state(pending);
    pending = PendingDestinationState{
        pts_samples,
        duration_samples,
        speaker_mask,
        coherent_gain_code,
        noncoherent_gain_code};
}

void ObjectSidecarWriter::flush_unavailable_state(
    const PendingUnavailableState& state) {
    if (state.duration_samples == 0U) {
        return;
    }
    output_ << "{\"type\":\"unavailable\""
            << ",\"startSamples\":" << state.start_samples
            << ",\"durationSamples\":" << state.duration_samples
            << ",\"coordinateStatus\":\"unavailable\""
            << ",\"reason\":\"" << state.reason << "\"}\n";
}

void ObjectSidecarWriter::write_unavailable(
    std::uint64_t pts_samples,
    std::uint32_t duration_samples,
    std::uint32_t sample_rate,
    std::uint32_t object_id,
    std::uint32_t waveform_id,
    std::string_view reason) {
    if (closed_) {
        throw std::runtime_error("object coordinate sidecar is closed");
    }
    ensure_header(sample_rate, object_id, waveform_id);
    if (unavailable_state_.has_value()
        && unavailable_state_->reason == reason
        && contiguous(
            unavailable_state_->start_samples,
            unavailable_state_->duration_samples,
            pts_samples)) {
        unavailable_state_->duration_samples += duration_samples;
        return;
    }
    if (unavailable_state_.has_value()) {
        flush_unavailable_state(*unavailable_state_);
    }
    unavailable_state_ = PendingUnavailableState{
        pts_samples, duration_samples, std::string(reason)};
}

void ObjectSidecarWriter::close() {
    if (closed_) {
        return;
    }
    for (const auto& entry : point_states_) {
        flush_point_state(entry.second, false);
    }
    for (const auto& entry : destination_states_) {
        flush_destination_state(entry.second);
    }
    if (unavailable_state_.has_value()) {
        flush_unavailable_state(*unavailable_state_);
    }
    flush_activity();
    output_.flush();
    if (!output_) {
        throw std::runtime_error("cannot finalize object coordinate sidecar");
    }
    closed_ = true;
}

} // namespace dtsx_decode
