#include "io/object_sidecar_writer.hpp"

#include "dtsx/speaker_mask.hpp"

#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace dtsx_decode {

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
    output_ << "{\"ptsSamples\":" << pts_samples
            << ",\"durationSamples\":" << duration_samples
            << ",\"sampleRate\":" << sample_rate
            << ",\"objectId\":" << object_id
            << ",\"waveformId\":" << waveform_id
            << ",\"pointSourceIndex\":" << point.point_source_index
            << ",\"coordinateStatus\":\"decoded\""
            << ",\"coordinateSystem\":\"dtsx_spherical\""
            << ",\"azimuthDeg\":" << point.coordinates.azimuth_degrees
            << ",\"elevationDeg\":" << point.coordinates.elevation_degrees
            << ",\"distance\":" << point.coordinates.distance
            << ",\"sourceType\":"
            << static_cast<unsigned>(point.source_type)
            << ",\"coherentRendering\":"
            << (point.coherent_rendering ? "true" : "false")
            << ",\"metadataMode\":"
            << static_cast<unsigned>(metadata_mode)
            << ",\"renderable\":"
            << (renderable ? "true" : "false");
    if (peak_sample.has_value()) {
        output_ << ",\"audioActive\":"
                << (*peak_sample != 0U ? "true" : "false")
                << ",\"peakSample\":" << *peak_sample;
    }
    output_ << ",\"snapToNearestSpeaker\":"
            << (point.snap_to_nearest_speaker ? "true" : "false")
            << ",\"gainCode\":" << static_cast<unsigned>(point.gain_code)
            << ",\"objectGainPresent\":"
            << (metadata_gain.object_gain_present ? "true" : "false")
            << ",\"objectGainCode\":"
            << static_cast<unsigned>(metadata_gain.object_gain_code)
            << ",\"objectGainExponent\":"
            << static_cast<unsigned>(metadata_gain.object_gain_exponent)
            << ",\"presentationGainCode\":"
            << static_cast<unsigned>(
                   metadata_gain.presentation_gain_code)
            << ",\"metadataGainQ23\":"
            << metadata_gain.effective_gain_q23
            << ",\"metadataGainLinear\":"
            << static_cast<double>(metadata_gain.effective_gain_q23)
                   / 8388608.0;
    if (metadata_gain.effective_gain_q23 > 0) {
        output_ << ",\"metadataGainDb\":"
                << 20.0 * std::log10(
                       static_cast<double>(
                           metadata_gain.effective_gain_q23)
                       / 8388608.0);
    } else {
        output_ << ",\"metadataGainDb\":null";
    }
    output_
            << ",\"widthDeg\":" << point.width_degrees
            << ",\"heightDeg\":" << point.height_degrees
            << ",\"rotationDeg\":" << point.rotation_degrees
            << "}\n";
    if (!output_) {
        throw std::runtime_error("cannot write object coordinate sidecar");
    }
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
    float azimuth = 0.0F;
    float elevation = 0.0F;
    std::string_view speaker_name;
    const bool coordinates_available =
        dtsx::standard_speaker_coordinates(
            speaker_mask, azimuth, elevation, speaker_name);
    output_ << "{\"ptsSamples\":" << pts_samples
            << ",\"durationSamples\":" << duration_samples
            << ",\"sampleRate\":" << sample_rate
            << ",\"objectId\":" << object_id
            << ",\"waveformId\":" << waveform_id
            << ",\"coordinateStatus\":\""
            << (coordinates_available
                    ? "speaker_destination"
                    : "speaker_mask_only")
            << "\",\"speakerMask\":" << speaker_mask;
    if (coordinates_available) {
        output_ << ",\"speaker\":\"" << speaker_name
                << "\",\"azimuthDeg\":" << azimuth
                << ",\"elevationDeg\":" << elevation;
    }
    output_ << ",\"coherentGainCode\":"
            << static_cast<unsigned>(coherent_gain_code)
            << ",\"noncoherentGainCode\":"
            << static_cast<unsigned>(noncoherent_gain_code)
            << "}\n";
    if (!output_) {
        throw std::runtime_error("cannot write object coordinate sidecar");
    }
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
    output_ << "{\"ptsSamples\":" << pts_samples
            << ",\"durationSamples\":" << duration_samples
            << ",\"sampleRate\":" << sample_rate
            << ",\"objectId\":" << object_id
            << ",\"waveformId\":" << waveform_id
            << ",\"coordinateStatus\":\"unavailable\""
            << ",\"reason\":\"" << reason << "\"}\n";
    if (!output_) {
        throw std::runtime_error("cannot write object coordinate sidecar");
    }
}

void ObjectSidecarWriter::close() {
    if (closed_) {
        return;
    }
    output_.flush();
    if (!output_) {
        throw std::runtime_error("cannot finalize object coordinate sidecar");
    }
    closed_ = true;
}

} // namespace dtsx_decode
