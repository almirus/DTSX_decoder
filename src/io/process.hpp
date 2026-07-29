#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace dtsx_decode {

class ProcessReader {
public:
    ProcessReader(const std::filesystem::path& executable,
        const std::vector<std::wstring>& arguments,
        bool verbose);
    ~ProcessReader();

    ProcessReader(const ProcessReader&) = delete;
    ProcessReader& operator=(const ProcessReader&) = delete;

    std::size_t read(void* destination, std::size_t capacity);
    unsigned wait();

private:
#ifdef _WIN32
    HANDLE process_ = nullptr;
    HANDLE thread_ = nullptr;
    HANDLE stdout_read_ = nullptr;
#endif
    bool waited_ = false;
    unsigned exit_code_ = 0;
};

std::string run_capture_text(const std::filesystem::path& executable,
    const std::vector<std::wstring>& arguments,
    bool verbose);

} // namespace dtsx_decode
