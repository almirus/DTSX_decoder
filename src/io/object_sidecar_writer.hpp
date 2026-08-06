#pragma once

#include "dtsx/object_spatial_metadata.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
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
    std::ofstream output_;
    bool closed_ = false;
};

} // namespace dtsx_decode
