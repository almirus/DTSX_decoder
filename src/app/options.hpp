#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace dtsx_decode {

enum class RenderMode {
    Bed,
    Objects,
    ObjectsOnly,
};

struct Options {
    std::filesystem::path input;
    std::filesystem::path output;
    std::filesystem::path metadata_output;
    std::filesystem::path coordinates_output;
    std::filesystem::path objects_output_directory;
    std::filesystem::path ffmpeg = L"ffmpeg.exe";
    std::string layout;
    std::uint32_t channels_check = 0;
    std::uint32_t sample_rate = 0;
    unsigned audio_track = 0;
    RenderMode render_mode = RenderMode::Objects;
    bool output_explicit = false;
    bool metadata_output_explicit = false;
    bool coordinates_output_explicit = false;
    bool objects_output_directory_explicit = false;
    bool overwrite = false;
    bool verbose = false;
    bool help = false;
    bool version = false;
    bool probe = false;
};

Options parse_options(int argc, wchar_t** argv);
void print_help();

} // namespace dtsx_decode
