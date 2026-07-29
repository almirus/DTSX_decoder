#include "ffmpeg.hpp"

#include <stdexcept>
#include <vector>

namespace dtsx_decode {
namespace {

std::wstring unsigned_text(unsigned value) {
    return std::to_wstring(value);
}

} // namespace

FfmpegDtsReader::FfmpegDtsReader(const Options& options) {
    const std::vector<std::wstring> arguments = {
        L"-hide_banner",
        L"-nostdin",
        L"-v",
        L"error",
        L"-i",
        options.input.wstring(),
        L"-map",
        L"0:a:" + unsigned_text(options.audio_track),
        L"-vn",
        L"-sn",
        L"-dn",
        L"-c:a",
        L"copy",
        L"-f",
        // The data muxer writes the selected packet payloads unchanged.
        // FFmpeg's DTS muxer rejects valid DTS:X packets whose decoded
        // channel metadata exceeds the legacy DTS muxer constraints.
        L"data",
        L"pipe:1",
    };
    process_ = std::make_unique<ProcessReader>(
        options.ffmpeg, arguments, options.verbose);
}

std::size_t FfmpegDtsReader::read(void* destination, std::size_t capacity) {
    return process_->read(destination, capacity);
}

void FfmpegDtsReader::finish() {
    if (finished_) {
        return;
    }
    const unsigned code = process_->wait();
    finished_ = true;
    if (code != 0) {
        throw std::runtime_error("ffmpeg DTS demux failed with exit code "
            + std::to_string(code));
    }
}

} // namespace dtsx_decode
