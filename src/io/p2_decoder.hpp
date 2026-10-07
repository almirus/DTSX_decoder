#pragma once

#include "app/options.hpp"

#include <filesystem>

namespace dtsx_decode {

[[nodiscard]] bool input_is_dts_uhd(const Options& options);

void decode_p2_stream(
    const Options& options,
    const std::filesystem::path& output,
    bool write_main_output = true);

} // namespace dtsx_decode
