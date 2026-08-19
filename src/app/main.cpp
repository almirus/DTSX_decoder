#include "app_version.hpp"
#include "options.hpp"
#include "pipeline.hpp"
#include "progress.hpp"
#include "../io/ffmpeg.hpp"

#include <exception>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

constexpr unsigned kExpirationYear = 2026U;
constexpr unsigned kExpirationMonth = 8U;
constexpr unsigned kExpirationDay = 30U;

constexpr bool date_after_expiration(
    unsigned year,
    unsigned month,
    unsigned day) {
    return year > kExpirationYear
        || (year == kExpirationYear
            && (month > kExpirationMonth
                || (month == kExpirationMonth
                    && day > kExpirationDay)));
}

static_assert(!date_after_expiration(2026U, 8U, 30U));
static_assert(date_after_expiration(2026U, 8U, 31U));

void configure_console() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    dtsx_decode::console_style::enable_virtual_terminal();
}

bool application_expired() {
#ifdef _WIN32
    SYSTEMTIME current{};
    GetLocalTime(&current);
    return date_after_expiration(
        current.wYear,
        current.wMonth,
        current.wDay);
#else
    return false;
#endif
}

void print_version() {
    std::cout
        << "dtsx-decode " << dtsx_decode::kVersion << '\n'
        << "Author: " << dtsx_decode::kAuthor << '\n';
}

void print_support_author() {
    const bool color =
        dtsx_decode::console_style::color_enabled(stdout);
    std::cout << "Поддержать автора:  ";
    dtsx_decode::console_style::paint(
        std::cout, color, dtsx_decode::console_style::bold);
    dtsx_decode::console_style::paint(
        std::cout, color, dtsx_decode::console_style::bright_magenta);
    std::cout << "2200 7009 5155 4582";
    dtsx_decode::console_style::reset(std::cout, color);
    std::cout << '\n';
}

void print_expiration_message() {
    std::cerr
        << "error: срок действия этой beta-версии истёк.\n"
        << "Приложение более не работоспособно. Ищите обновления на сайте "
        << "https://touch-max.ru/\n";
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
            print_support_author();
            return 0;
        }
        if (application_expired()) {
            print_expiration_message();
            return 1;
        }
        print_version();
        print_support_author();
        dtsx_decode::require_ffmpeg_in_path();
        return dtsx_decode::run_pipeline(options);
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
