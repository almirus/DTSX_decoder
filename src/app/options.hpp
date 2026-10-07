#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace dtsx_decode {

enum class RenderMode {
    Bed,
    BedWithoutObjects,
    Objects,
    ObjectsOnly,
};

enum class OutputFormat {
    Wav,
    Wave64,
};

struct Options {
    std::filesystem::path input;
    std::filesystem::path output;
    std::filesystem::path metadata_output;
    std::filesystem::path objects_output_directory;
    std::filesystem::path mono_tracks_directory;
    std::string layout;
    std::string imax_small_speakers = "all";
    std::uint32_t sample_rate = 0;
    std::uint32_t threads = 0;
    std::uint64_t duration_seconds = 0;
    unsigned audio_track = 0;
    RenderMode render_mode = RenderMode::Objects;
    OutputFormat output_format = OutputFormat::Wav;
    bool output_explicit = false;
    bool metadata_output_explicit = false;
    bool objects_output_directory_explicit = false;
    bool objects_output_bed = false;
    bool mono_tracks = false;
    bool dolby_output = false;
    bool overwrite = false;
    bool verbose = false;
    bool help = false;
    bool version = false;
    bool probe = false;
    bool full_probe = false;
    bool upmix = false;
    bool imax_dsp = false;
    bool p2_internal = false;
    bool audio_track_explicit = false;
    bool audio_track_optional = false;
};

Options parse_options(int argc, wchar_t** argv);
std::uint64_t duration_frame_limit(
    const Options& options,
    std::uint32_t sample_rate);
void print_help();

} // namespace dtsx_decode
