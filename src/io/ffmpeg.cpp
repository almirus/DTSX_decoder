#include "ffmpeg.hpp"

#include <stdexcept>
#include <vector>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

namespace dtsx_decode {
namespace {

std::wstring unsigned_text(unsigned value) {
    return std::to_wstring(value);
}

std::filesystem::path find_ffmpeg_in_path() {
    const DWORD path_length =
        GetEnvironmentVariableW(L"PATH", nullptr, 0U);
    if (path_length == 0U) {
        throw std::runtime_error(
            "ffmpeg.exe was not found: PATH is empty");
    }
    std::vector<wchar_t> path_value(path_length);
    if (GetEnvironmentVariableW(
            L"PATH", path_value.data(), path_length) == 0U) {
        throw std::runtime_error(
            "cannot read PATH while locating ffmpeg.exe");
    }

    const DWORD executable_length = SearchPathW(
        path_value.data(),
        L"ffmpeg.exe",
        nullptr,
        0U,
        nullptr,
        nullptr);
    if (executable_length == 0U) {
        throw std::runtime_error(
            "ffmpeg.exe was not found in PATH; install FFmpeg and add "
            "its bin directory to PATH");
    }
    std::vector<wchar_t> executable(executable_length + 1U);
    const DWORD copied = SearchPathW(
        path_value.data(),
        L"ffmpeg.exe",
        nullptr,
        static_cast<DWORD>(executable.size()),
        executable.data(),
        nullptr);
    if (copied == 0U || copied >= executable.size()) {
        throw std::runtime_error(
            "cannot resolve ffmpeg.exe from PATH");
    }
    return std::filesystem::path(
        std::wstring(executable.data(), copied));
}

} // namespace

const std::filesystem::path& ffmpeg_executable() {
    static const std::filesystem::path executable =
        find_ffmpeg_in_path();
    return executable;
}

void require_ffmpeg_in_path() {
    (void)ffmpeg_executable();
}

FfmpegDtsReader::FfmpegDtsReader(const Options& options) {
    std::vector<std::wstring> arguments = {
        L"-hide_banner",
        L"-nostdin",
        L"-v",
        options.audio_track_optional ? L"quiet" : L"error",
        L"-i",
        options.input.wstring(),
        L"-map",
        L"0:a:" + unsigned_text(options.audio_track)
            + (options.audio_track_optional ? L"?" : L""),
        L"-vn",
        L"-sn",
        L"-dn",
        L"-c:a",
        L"copy",
    };
    if (options.duration_seconds != 0U && !options.probe) {
        arguments.emplace_back(L"-t");
        arguments.emplace_back(
            std::to_wstring(options.duration_seconds));
    }
    if (options.probe && !options.full_probe) {
        arguments.emplace_back(L"-t");
        arguments.emplace_back(L"10");
    }
    arguments.insert(arguments.end(), {
        L"-f",
        // The data muxer writes the selected packet payloads unchanged.
        // FFmpeg's DTS muxer rejects valid DTS:X packets whose decoded
        // channel metadata exceeds the legacy DTS muxer constraints.
        L"data",
        L"pipe:1",
    });
    process_ = std::make_unique<ProcessReader>(
        ffmpeg_executable(), arguments, options.verbose);
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
