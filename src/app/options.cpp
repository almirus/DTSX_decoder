#include "options.hpp"

#include "../audio/layout.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace dtsx_decode {
namespace {

std::string narrow_ascii(const std::wstring& value, const char* option) {
    std::string result;
    result.reserve(value.size());
    for (const wchar_t c : value) {
        if (static_cast<unsigned int>(c) > 0x7Fu) {
            throw std::runtime_error(std::string(option) + " accepts an ASCII value");
        }
        result.push_back(static_cast<char>(c));
    }
    return result;
}

std::uint32_t parse_u32(const std::wstring& value, const char* option) {
    try {
        std::size_t consumed = 0;
        const unsigned long parsed = std::stoul(value, &consumed, 10);
        if (consumed != value.size() || parsed > std::numeric_limits<std::uint32_t>::max()) {
            throw std::runtime_error("range");
        }
        return static_cast<std::uint32_t>(parsed);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("invalid numeric value for ") + option);
    }
}

std::uint64_t parse_duration(const std::wstring& value) {
    if (value.empty()) {
        throw std::runtime_error(
            "invalid --duration; expected 10s, 10m or 1h2m5s");
    }
    std::uint64_t total = 0U;
    std::size_t offset = 0U;
    int previous_rank = 4;
    while (offset < value.size()) {
        if (value[offset] < L'0' || value[offset] > L'9') {
            throw std::runtime_error(
                "invalid --duration; expected 10s, 10m or 1h2m5s");
        }
        std::uint64_t amount = 0U;
        do {
            const unsigned digit =
                static_cast<unsigned>(value[offset] - L'0');
            if (amount
                > (std::numeric_limits<std::uint64_t>::max() - digit) / 10U) {
                throw std::runtime_error("--duration is too large");
            }
            amount = amount * 10U + digit;
            ++offset;
        } while (offset < value.size()
                 && value[offset] >= L'0'
                 && value[offset] <= L'9');
        if (offset == value.size()) {
            throw std::runtime_error(
                "invalid --duration; expected 10s, 10m or 1h2m5s");
        }

        std::uint64_t multiplier = 0U;
        int rank = 0;
        switch (value[offset++]) {
        case L'h':
            multiplier = 3600U;
            rank = 3;
            break;
        case L'm':
            multiplier = 60U;
            rank = 2;
            break;
        case L's':
            multiplier = 1U;
            rank = 1;
            break;
        default:
            throw std::runtime_error(
                "invalid --duration; expected 10s, 10m or 1h2m5s");
        }
        if (rank >= previous_rank) {
            throw std::runtime_error(
                "invalid --duration; units must be ordered h, m, s");
        }
        previous_rank = rank;
        if (amount > std::numeric_limits<std::uint64_t>::max() / multiplier
            || total > std::numeric_limits<std::uint64_t>::max()
                - amount * multiplier) {
            throw std::runtime_error("--duration is too large");
        }
        total += amount * multiplier;
    }
    if (total == 0U) {
        throw std::runtime_error("--duration must be greater than zero");
    }
    return total;
}

const wchar_t* require_value(int argc, wchar_t** argv, int& i, const char* option) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + option);
    }
    return argv[++i];
}

} // namespace

Options parse_options(int argc, wchar_t** argv) {
    Options options;
    if (argc == 1) {
        options.help = true;
        return options;
    }

    for (int i = 1; i < argc; ++i) {
        const std::wstring arg = argv[i];
        if (arg == L"-h" || arg == L"--help") {
            options.help = true;
        } else if (arg == L"--version") {
            options.version = true;
        } else if (arg == L"--probe") {
            options.probe = true;
        } else if (arg == L"--full-probe") {
            options.probe = true;
            options.full_probe = true;
        } else if (arg == L"-i" || arg == L"--input") {
            options.input = require_value(argc, argv, i, "--input");
        } else if (arg == L"-o" || arg == L"--output") {
            options.output = require_value(argc, argv, i, "--output");
            options.output_explicit = true;
        } else if (arg == L"--metadata-output") {
            options.metadata_output = require_value(
                argc, argv, i, "--metadata-output");
            options.metadata_output_explicit = true;
        } else if (arg == L"--coordinates-output") {
            options.coordinates_output = require_value(
                argc, argv, i, "--coordinates-output");
            options.coordinates_output_explicit = true;
        } else if (arg == L"--objects-output-dir") {
            options.objects_output_directory = require_value(
                argc, argv, i, "--objects-output-dir");
            options.objects_output_directory_explicit = true;
        } else if (arg == L"--ffmpeg") {
            options.ffmpeg = require_value(argc, argv, i, "--ffmpeg");
        } else if (arg == L"--layout") {
            options.layout = narrow_ascii(require_value(argc, argv, i, "--layout"), "--layout");
        } else if (arg == L"--channels") {
            options.channels_check =
                parse_u32(require_value(argc, argv, i, "--channels"), "--channels");
        } else if (arg == L"--sample-rate") {
            options.sample_rate =
                parse_u32(require_value(argc, argv, i, "--sample-rate"), "--sample-rate");
        } else if (arg == L"--duration") {
            options.duration_seconds =
                parse_duration(require_value(argc, argv, i, "--duration"));
        } else if (arg == L"--audio-track") {
            options.audio_track =
                parse_u32(require_value(argc, argv, i, "--audio-track"), "--audio-track");
            options.audio_track_explicit = true;
        } else if (arg == L"--render") {
            const std::string mode =
                narrow_ascii(require_value(argc, argv, i, "--render"), "--render");
            if (mode == "bed") {
                options.render_mode = RenderMode::Bed;
            } else if (mode == "objects") {
                options.render_mode = RenderMode::Objects;
            } else if (mode == "objects-only") {
                options.render_mode = RenderMode::ObjectsOnly;
            } else {
                throw std::runtime_error(
                    "--render must be bed, objects or objects-only");
            }
        } else if (arg == L"--overwrite") {
            options.overwrite = true;
        } else if (arg == L"-v" || arg == L"--verbose") {
            options.verbose = true;
        } else {
            throw std::runtime_error("unknown option; use --help");
        }
    }

    if (!options.help && !options.version && options.input.empty()) {
        throw std::runtime_error("--input is required");
    }
    return options;
}

std::uint64_t duration_frame_limit(
    const Options& options,
    std::uint32_t sample_rate) {
    if (options.duration_seconds == 0U) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    if (sample_rate == 0U
        || options.duration_seconds
            > std::numeric_limits<std::uint64_t>::max() / sample_rate) {
        throw std::runtime_error("--duration sample count is too large");
    }
    return options.duration_seconds * sample_rate;
}

void print_help() {
    std::cout
        << "dtsx-decode - DTS:X object decoder/render pipeline\n\n"
        << "Usage:\n"
        << "  dtsx-decode -i INPUT [options]\n\n"
        << "Options:\n"
        << "  -i, --input PATH       Input .mkv, .mp4, .m2ts, .dts or .dtshd\n"
        << "  -o, --output PATH      Output PCM24 WAV\n"
        << "      --metadata-output PATH  Write parsed DTS:X metadata JSONL\n"
        << "      --coordinates-output PATH  Write decoded object coordinates JSONL\n"
        << "      --objects-output-dir PATH  Write per-object waveform WAV and coordinates\n"
        << "      --audio-track N    Audio track ordinal; default: best DTS track\n"
        << "      --layout NAME      Output layout; default: DTS:X metadata layout\n"
        << "      --channels N       Validate layout channel count only\n"
        << "      --sample-rate HZ   Output sample rate\n"
        << "      --duration TIME    Decode only this duration: 10s, 10m, 1h2m5s\n"
        << "      --render MODE      objects, objects-only or bed; default objects\n"
        << "      --ffmpeg PATH      ffmpeg executable for container demux only\n"
        << "      --overwrite        Replace existing output\n"
        << "  -v, --verbose          Detailed diagnostics\n"
        << "      --version          Print version\n"
        << "      --probe            Inspect a DTS stream; no output layout required\n"
        << "      --full-probe       Inspect the complete DTS stream\n"
        << "  -h, --help             Print help\n\n"
        << "Layouts: " << supported_layouts_text() << "\n"
        << "Bed mode decodes the channel bed without DTS:X objects.\n";
}

} // namespace dtsx_decode
