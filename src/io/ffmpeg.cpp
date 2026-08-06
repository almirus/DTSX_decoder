#include "ffmpeg.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <stdexcept>
#include <string_view>
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

std::wstring milliseconds_text(std::uint64_t value) {
    return std::to_wstring(value / 1000U)
        + L"."
        + std::to_wstring(1000U + value % 1000U).substr(1U);
}

std::uint64_t parse_duration_milliseconds(std::string_view output) {
    constexpr std::string_view marker = "Duration: ";
    const std::size_t marker_position = output.find(marker);
    if (marker_position == std::string_view::npos) {
        return 0U;
    }
    const std::size_t start = marker_position + marker.size();
    const std::size_t comma = output.find(',', start);
    if (comma == std::string_view::npos) {
        return 0U;
    }
    const std::string_view value = output.substr(start, comma - start);
    unsigned hours = 0U;
    unsigned minutes = 0U;
    double seconds = 0.0;
    if (value.size() < 8U
        || value[2] != ':'
        || value[5] != ':'
        || std::from_chars(
               value.data(), value.data() + 2U, hours).ec
               != std::errc{}
        || std::from_chars(
               value.data() + 3U, value.data() + 5U, minutes).ec
               != std::errc{}) {
        return 0U;
    }
    try {
        seconds = std::stod(std::string(value.substr(6U)));
    } catch (const std::exception&) {
        return 0U;
    }
    if (minutes >= 60U || seconds < 0.0 || seconds >= 60.0) {
        return 0U;
    }
    return static_cast<std::uint64_t>(std::llround(
        (static_cast<double>(hours) * 3600.0
         + static_cast<double>(minutes) * 60.0
         + seconds)
        * 1000.0));
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

std::uint64_t ffmpeg_input_duration_milliseconds(
    const Options& options) {
    const std::vector<std::wstring> arguments = {
        L"-hide_banner",
        L"-nostdin",
        L"-i",
        options.input.wstring(),
    };
    ProcessReader process(
        ffmpeg_executable(), arguments, options.verbose, true);
    std::array<char, 16U * 1024U> buffer{};
    std::string output;
    while (true) {
        const std::size_t size = process.read(
            buffer.data(), buffer.size());
        if (size == 0U) {
            break;
        }
        output.append(buffer.data(), size);
    }
    (void)process.wait();
    return parse_duration_milliseconds(output);
}

FfmpegDtsReader::FfmpegDtsReader(
    const Options& options,
    std::uint64_t start_milliseconds,
    std::uint64_t duration_milliseconds) {
    std::vector<std::wstring> arguments = {
        L"-hide_banner",
        L"-nostdin",
        L"-v",
        options.audio_track_optional ? L"quiet" : L"error",
    };
    if (start_milliseconds != 0U) {
        arguments.emplace_back(L"-ss");
        arguments.emplace_back(
            milliseconds_text(start_milliseconds));
    }
    arguments.insert(arguments.end(), {
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
    });
    if (duration_milliseconds != 0U) {
        arguments.emplace_back(L"-t");
        arguments.emplace_back(
            milliseconds_text(duration_milliseconds));
    } else if (options.duration_seconds != 0U && !options.probe) {
        arguments.emplace_back(L"-t");
        arguments.emplace_back(
            std::to_wstring(options.duration_seconds));
    } else if (options.probe && !options.full_probe) {
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
