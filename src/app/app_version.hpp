#pragma once

#include <string>

namespace dtsx_decode {

inline constexpr const char* kVersion = "0.1.3 alfa";
inline constexpr const char* kAuthor = "almirus";

inline std::string make_decode_comment() {
    return std::string("Decoded by dtsx-decode ")
        + kVersion + ", @" + kAuthor;
}

} // namespace dtsx_decode
