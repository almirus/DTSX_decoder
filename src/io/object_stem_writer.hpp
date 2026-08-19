#pragma once

#include "app/object_frame_decoder.hpp"
#include "io/object_sidecar_writer.hpp"
#include "io/wav_writer.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <utility>

namespace dtsx_decode {

class ObjectStemWriter final {
public:
    ObjectStemWriter(
        std::filesystem::path directory,
        std::uint32_t sample_rate,
        bool overwrite);

    [[nodiscard]] bool write(
        const DecodedObjectAudioFrame& frame,
        std::uint64_t sample_position,
        std::uint32_t duration_samples);
    [[nodiscard]] bool has_audio_stems() const noexcept {
        return !wavs_.empty() || !supplemental_wavs_.empty();
    }
    void close();

private:
    using StemKey = std::pair<std::uint32_t, std::uint32_t>;

    WavWriter& wav(const StemKey& key);
    WavWriter& supplemental_wav(std::uint32_t waveform);
    ObjectSidecarWriter& coordinates(const StemKey& key);
    [[nodiscard]] std::filesystem::path stem_path(
        const StemKey& key,
        const wchar_t* extension) const;

    std::filesystem::path directory_;
    std::uint32_t sample_rate_ = 0;
    bool overwrite_ = false;
    std::uint64_t timeline_end_ = 0;
    std::map<StemKey, std::unique_ptr<WavWriter>> wavs_;
    std::map<std::uint32_t, std::unique_ptr<WavWriter>>
        supplemental_wavs_;
    std::map<StemKey, std::unique_ptr<ObjectSidecarWriter>>
        coordinate_writers_;
};

} // namespace dtsx_decode
