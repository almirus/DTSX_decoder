#pragma once

#include "../app/options.hpp"
#include "process.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace dtsx_decode {

void require_ffmpeg_in_path();
[[nodiscard]] const std::filesystem::path& ffmpeg_executable();

struct AudioProbe {
    unsigned stream_index = 0;
    std::string codec_name;
    std::string profile;
    std::string codec_tag;
    std::uint32_t sample_rate = 0;
    std::uint32_t channels = 0;
    std::string channel_layout;
};

class FfmpegDtsReader {
public:
    explicit FfmpegDtsReader(const Options& options);

    std::size_t read(void* destination, std::size_t capacity);
    void finish();

private:
    std::unique_ptr<ProcessReader> process_;
    bool finished_ = false;
};

} // namespace dtsx_decode
