#include "app_version.hpp"
#include "options.hpp"
#include "pipeline.hpp"
#include "progress.hpp"
#include "support_qr.hpp"
#include "../io/ffmpeg.hpp"

#include <exception>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

void configure_console() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    dtsx_decode::console_style::enable_virtual_terminal();
}

void print_version() {
    std::cout
        << "dtsx-decode " << dtsx_decode::kVersion << '\n'
        << "Author: " << dtsx_decode::kAuthor << '\n';
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    configure_console();
    try {
        const dtsx_decode::Options options = dtsx_decode::parse_options(argc, argv);
        if (options.help) {
            dtsx_decode::print_help();
            return 0;
        }
        if (options.version) {
            print_version();
            dtsx_decode::print_support_author();
            return 0;
        }
        print_version();
        dtsx_decode::print_support_author();
        dtsx_decode::require_ffmpeg_in_path();
        return dtsx_decode::run_pipeline(options);
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}

#ifndef _WIN32
#include <clocale>
#include <cstdlib>
#include <vector>
int main(int argc, char** argv) {
    std::setlocale(LC_ALL, "C.UTF-8");
    std::vector<std::wstring> storage;
    std::vector<wchar_t*> wargv;
    for (int i = 0; i < argc; ++i) {
        std::wstring w(std::mbstowcs(nullptr, argv[i], 0), L'\0');
        std::mbstowcs(w.data(), argv[i], w.size());
        storage.push_back(w);
    }
    for (auto& w : storage) wargv.push_back(w.data());
    wargv.push_back(nullptr);
    return wmain(argc, wargv.data());
}
#endif
