#include "process.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace dtsx_decode {
namespace {

#ifdef _WIN32
std::wstring quote_windows_argument(const std::wstring& value) {
    if (value.empty()) {
        return L"\"\"";
    }
    if (value.find_first_of(L" \t\n\v\"") == std::wstring::npos) {
        return value;
    }

    std::wstring result = L"\"";
    unsigned backslashes = 0;
    for (const wchar_t c : value) {
        if (c == L'\\') {
            ++backslashes;
            continue;
        }
        if (c == L'"') {
            result.append(backslashes * 2u + 1u, L'\\');
            result.push_back(L'"');
            backslashes = 0;
            continue;
        }
        result.append(backslashes, L'\\');
        backslashes = 0;
        result.push_back(c);
    }
    result.append(backslashes * 2u, L'\\');
    result.push_back(L'"');
    return result;
}

std::wstring command_line(const std::filesystem::path& executable,
    const std::vector<std::wstring>& arguments) {
    std::wstring result = quote_windows_argument(executable.wstring());
    for (const std::wstring& argument : arguments) {
        result.push_back(L' ');
        result += quote_windows_argument(argument);
    }
    return result;
}
#endif

} // namespace

ProcessReader::ProcessReader(const std::filesystem::path& executable,
    const std::vector<std::wstring>& arguments,
    bool verbose,
    bool merge_stderr) {
#ifdef _WIN32
    SECURITY_ATTRIBUTES security{};
    security.nLength = sizeof(security);
    security.bInheritHandle = TRUE;

    HANDLE stdout_write = nullptr;
    if (!CreatePipe(&stdout_read_, &stdout_write, &security, 0)) {
        throw std::runtime_error("CreatePipe failed");
    }
    if (!SetHandleInformation(stdout_read_, HANDLE_FLAG_INHERIT, 0)) {
        CloseHandle(stdout_read_);
        CloseHandle(stdout_write);
        stdout_read_ = nullptr;
        throw std::runtime_error("SetHandleInformation failed");
    }

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdOutput = stdout_write;
    startup.hStdError = merge_stderr
        ? stdout_write
        : GetStdHandle(STD_ERROR_HANDLE);
    startup.hStdInput = GetStdHandle(STD_INPUT_HANDLE);

    PROCESS_INFORMATION process_info{};
    std::wstring command = command_line(executable, arguments);
    if (verbose) {
        std::wcerr << L"[process] " << command << L'\n';
    }

    const BOOL created = CreateProcessW(nullptr,
        command.data(),
        nullptr,
        nullptr,
        TRUE,
        CREATE_NO_WINDOW,
        nullptr,
        nullptr,
        &startup,
        &process_info);
    CloseHandle(stdout_write);
    if (!created) {
        const DWORD error = GetLastError();
        CloseHandle(stdout_read_);
        stdout_read_ = nullptr;
        throw std::runtime_error("cannot start child process, Windows error "
            + std::to_string(error));
    }
    process_ = process_info.hProcess;
    thread_ = process_info.hThread;
#else
    (void)executable;
    (void)arguments;
    (void)verbose;
    (void)merge_stderr;
    throw std::runtime_error("ProcessReader is currently implemented for Windows only");
#endif
}

ProcessReader::~ProcessReader() {
#ifdef _WIN32
    if (stdout_read_) {
        CloseHandle(stdout_read_);
    }
    if (thread_) {
        CloseHandle(thread_);
    }
    if (process_) {
        CloseHandle(process_);
    }
#endif
}

std::size_t ProcessReader::read(void* destination, std::size_t capacity) {
#ifdef _WIN32
    if (!stdout_read_ || capacity == 0) {
        return 0;
    }
    const DWORD request = static_cast<DWORD>(
        std::min<std::size_t>(capacity, std::numeric_limits<DWORD>::max()));
    DWORD received = 0;
    if (!ReadFile(stdout_read_, destination, request, &received, nullptr)) {
        const DWORD error = GetLastError();
        if (error == ERROR_BROKEN_PIPE) {
            return 0;
        }
        throw std::runtime_error("ReadFile from child process failed");
    }
    return static_cast<std::size_t>(received);
#else
    (void)destination;
    (void)capacity;
    return 0;
#endif
}

unsigned ProcessReader::wait() {
    if (waited_) {
        return exit_code_;
    }
#ifdef _WIN32
    if (WaitForSingleObject(process_, INFINITE) != WAIT_OBJECT_0) {
        throw std::runtime_error("waiting for child process failed");
    }
    DWORD code = 0;
    if (!GetExitCodeProcess(process_, &code)) {
        throw std::runtime_error("GetExitCodeProcess failed");
    }
    exit_code_ = static_cast<unsigned>(code);
#endif
    waited_ = true;
    return exit_code_;
}

} // namespace dtsx_decode
