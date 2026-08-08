#pragma once

#include "dtsx/object_spatial_metadata.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace dtsx_decode {

struct ObjectMetadataGain final {
    bool object_gain_present = false;
    std::uint8_t object_gain_code = 61U;
    std::uint8_t object_gain_exponent = 0U;
    std::uint8_t presentation_gain_code = 61U;
    std::int32_t effective_gain_q23 = 0x800000;
};

class ObjectSidecarWriter final {
public:
    ObjectSidecarWriter(const std::filesystem::path& path, bool overwrite);

    void write(std::uint64_t pts_samples,
               std::uint32_t duration_samples,
               std::uint32_t sample_rate,
               std::uint32_t object_id,
               std::uint32_t waveform_id,
               const dtsx::PointSourceMetadata& point,
               std::uint8_t metadata_mode,
               bool renderable,
               std::optional<std::uint32_t> peak_sample,
               const ObjectMetadataGain& metadata_gain);
    void write_destination(std::uint64_t pts_samples,
                           std::uint32_t duration_samples,
                           std::uint32_t sample_rate,
                           std::uint32_t object_id,
                           std::uint32_t waveform_id,
                           std::uint32_t speaker_mask,
                           std::uint8_t coherent_gain_code,
                           std::uint8_t noncoherent_gain_code);
    void write_unavailable(std::uint64_t pts_samples,
                           std::uint32_t duration_samples,
                           std::uint32_t sample_rate,
                           std::uint32_t object_id,
                           std::uint32_t waveform_id,
                           std::string_view reason);
    void close();

private:
    struct PendingPointState final {
        std::uint64_t start_samples = 0U;
        std::uint64_t duration_samples = 0U;
        std::uint32_t last_frame_duration = 0U;
        dtsx::PointSourceMetadata point;
        ObjectMetadataGain metadata_gain;
        std::uint8_t metadata_mode = 0U;
        bool renderable = false;
    };

    struct PendingDestinationState final {
        std::uint64_t start_samples = 0U;
        std::uint64_t duration_samples = 0U;
        std::uint32_t speaker_mask = 0U;
        std::uint8_t coherent_gain_code = 0U;
        std::uint8_t noncoherent_gain_code = 0U;
    };

    struct PendingUnavailableState final {
        std::uint64_t start_samples = 0U;
        std::uint64_t duration_samples = 0U;
        std::string reason;
    };

    void ensure_header(std::uint32_t sample_rate,
                       std::uint32_t object_id,
                       std::uint32_t waveform_id);
    void update_activity(std::uint64_t pts_samples,
                         std::uint32_t duration_samples,
                         std::optional<std::uint32_t> peak_sample);
    void flush_activity();
    void flush_point_state(PendingPointState state,
                           bool coordinates_change);
    void write_point_interval(const PendingPointState& state,
                              std::uint64_t start_samples,
                              std::uint64_t duration_samples,
                              std::string_view interpolation);
    void flush_destination_state(
        const PendingDestinationState& state);
    void flush_unavailable_state(
        const PendingUnavailableState& state);

    std::ofstream output_;
    std::map<std::uint32_t, PendingPointState> point_states_;
    std::map<std::uint32_t, PendingDestinationState>
        destination_states_;
    std::optional<PendingUnavailableState> unavailable_state_;
    std::uint64_t activity_start_samples_ = 0U;
    std::uint64_t activity_duration_samples_ = 0U;
    std::uint64_t activity_last_pts_samples_ =
        static_cast<std::uint64_t>(-1);
    std::uint32_t activity_peak_sample_ = 0U;
    std::uint32_t sample_rate_ = 0U;
    std::uint32_t object_id_ = 0U;
    std::uint32_t waveform_id_ = 0U;
    bool header_written_ = false;
    bool activity_open_ = false;
    bool activity_active_ = false;
    bool closed_ = false;
};

} // namespace dtsx_decode
