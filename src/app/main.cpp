#include "app_version.hpp"
#include "options.hpp"
#include "pipeline.hpp"

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
            std::cout << dtsx_decode::kVersion << '\n';
            return 0;
        }
        return dtsx_decode::run_pipeline(options);
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
