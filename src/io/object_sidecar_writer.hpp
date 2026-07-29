#pragma once

#include "dtsx/object_spatial_metadata.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string_view>

namespace dtsx_decode {

class ObjectSidecarWriter final {
public:
    ObjectSidecarWriter(const std::filesystem::path& path, bool overwrite);

    void write(std::uint64_t pts_samples,
               std::uint32_t duration_samples,
               std::uint32_t sample_rate,
               std::uint32_t object_id,
               std::uint32_t waveform_id,
               const dtsx::PointSourceMetadata& point);
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
