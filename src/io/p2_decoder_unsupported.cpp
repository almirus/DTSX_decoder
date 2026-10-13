// Non-Windows builds: DTS-UHD (P2) decoding needs the embedded Windows-only DTSXDecoder.dll, so
// build.sh compiles this file instead of p2_decoder.cpp. DTS core, DTS-HD and legacy DTS:X
// (ExSS/XLL) decoding are unaffected.
#ifndef _WIN32
#include "io/p2_decoder.hpp"

#include <stdexcept>

namespace dtsx_decode {

bool input_is_dts_uhd(const Options&) {
    return false;
}

void decode_p2_stream(const Options&, const std::filesystem::path&, bool) {
    throw std::runtime_error("DTS-UHD (P2) decoding needs DTSXDecoder.dll and is only available on Windows");
}

} // namespace dtsx_decode
#endif
