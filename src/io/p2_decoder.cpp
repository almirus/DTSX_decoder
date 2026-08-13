#include "io/p2_decoder.hpp"

#include "app/progress.hpp"
#include "audio/layout.hpp"
#include "dtsx/speaker_mask.hpp"
#include "io/dts_frame_reader.hpp"
#include "io/p2_resources.hpp"
#include "io/wav_writer.hpp"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cwctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace dtsx_decode {
namespace {

using NativeGetSize = unsigned(__cdecl*)(int);
using NativeCreate = int(__cdecl*)(void**, int, void*);
using NativeDecode = int(__cdecl*)(void*, void*, void**, void*);
using NativeSetParameter =
    int(__cdecl*)(void*, int, unsigned*, unsigned);
using NativeDestroy = int(__cdecl*)(void*);

constexpr std::uintptr_t kGetSizeRva = 0x1A7D0U;
constexpr std::uintptr_t kCreateRva = 0x1A870U;
constexpr std::uintptr_t kDecodeRva = 0x1A3F0U;
constexpr std::uintptr_t kSetParameterRva = 0x1AB30U;
constexpr std::uintptr_t kDestroyRva = 0x1A780U;

template <class Function>
Function native_function(HMODULE module, std::uintptr_t rva) {
    return reinterpret_cast<Function>(
        reinterpret_cast<std::uintptr_t>(module) + rva);
}

struct EmbeddedRuntimeFile final {
    int resource_id;
    const wchar_t* file_name;
};

constexpr std::array<EmbeddedRuntimeFile, 5> kEmbeddedRuntimeFiles{{
    {IDR_P2_DTSX_DECODER, L"DTSXDecoder.dll"},
    {IDR_P2_MSVCP140_APP, L"MSVCP140_APP.dll"},
    {IDR_P2_VCCORLIB140_APP, L"VCCORLIB140_APP.dll"},
    {IDR_P2_VCRUNTIME140_1_APP, L"VCRUNTIME140_1_APP.dll"},
    {IDR_P2_VCRUNTIME140_APP, L"VCRUNTIME140_APP.dll"},
}};

class EmbeddedP2Runtime final {
public:
    EmbeddedP2Runtime() {
        create_directory();
        try {
            for (const EmbeddedRuntimeFile& file :
                 kEmbeddedRuntimeFiles) {
                extract(file);
            }
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~EmbeddedP2Runtime() {
        cleanup();
    }

    EmbeddedP2Runtime(const EmbeddedP2Runtime&) = delete;
    EmbeddedP2Runtime& operator=(const EmbeddedP2Runtime&) = delete;

    [[nodiscard]] std::filesystem::path decoder_path() const {
        return directory_ / L"DTSXDecoder.dll";
    }

private:
    void create_directory() {
        std::array<wchar_t, 32768> temporary_path{};
        const DWORD length = GetTempPathW(
            static_cast<DWORD>(temporary_path.size()),
            temporary_path.data());
        if (length == 0U || length >= temporary_path.size()) {
            throw std::runtime_error(
                "cannot determine temporary directory for P2 runtime");
        }
        const std::filesystem::path root(
            std::wstring(temporary_path.data(), length));
        const DWORD process_id = GetCurrentProcessId();
        const ULONGLONG seed = GetTickCount64();
        for (unsigned attempt = 0U; attempt < 128U; ++attempt) {
            directory_ = root /
                (L"dtsx-decode-p2-"
                 + std::to_wstring(process_id)
                 + L"-"
                 + std::to_wstring(seed + attempt));
            if (CreateDirectoryW(directory_.c_str(), nullptr) != 0) {
                return;
            }
            if (GetLastError() != ERROR_ALREADY_EXISTS) {
                throw std::runtime_error(
                    "cannot create temporary P2 runtime directory, "
                    "Windows error "
                    + std::to_string(GetLastError()));
            }
        }
        throw std::runtime_error(
            "cannot allocate a unique temporary P2 runtime directory");
    }

    void extract(const EmbeddedRuntimeFile& file) const {
        const HMODULE executable = GetModuleHandleW(nullptr);
        const HRSRC resource = FindResourceW(
            executable,
            MAKEINTRESOURCEW(file.resource_id),
            MAKEINTRESOURCEW(10));
        if (resource == nullptr) {
            throw std::runtime_error(
                "embedded P2 runtime resource is missing, Windows error "
                + std::to_string(GetLastError()));
        }
        const DWORD size = SizeofResource(executable, resource);
        const HGLOBAL loaded = LoadResource(executable, resource);
        const void* data = loaded != nullptr
            ? LockResource(loaded)
            : nullptr;
        if (size == 0U || data == nullptr) {
            throw std::runtime_error(
                "cannot read embedded P2 runtime resource, Windows error "
                + std::to_string(GetLastError()));
        }
        const std::filesystem::path output_path =
            directory_ / file.file_name;
        std::ofstream output(
            output_path,
            std::ios::binary | std::ios::trunc);
        if (!output) {
            throw std::runtime_error(
                "cannot create temporary P2 runtime file");
        }
        output.write(
            static_cast<const char*>(data),
            static_cast<std::streamsize>(size));
        if (!output) {
            throw std::runtime_error(
                "cannot write temporary P2 runtime file");
        }
    }

    void cleanup() noexcept {
        if (directory_.empty()) {
            return;
        }
        for (const EmbeddedRuntimeFile& file : kEmbeddedRuntimeFiles) {
            const std::filesystem::path path =
                directory_ / file.file_name;
            DeleteFileW(path.c_str());
        }
        RemoveDirectoryW(directory_.c_str());
        directory_.clear();
    }

    std::filesystem::path directory_;
};

bool is_elementary_dts(const std::filesystem::path& path) {
    std::wstring extension = path.extension().wstring();
    std::transform(
        extension.begin(),
        extension.end(),
        extension.begin(),
        [](wchar_t value) {
            return static_cast<wchar_t>(std::towlower(value));
        });
    return extension == L".dts" || extension == L".dtshd";
}

std::uint32_t requested_activity_mask(
    const ChannelLayout& layout) {
    std::uint32_t physical_mask = 0U;
    const bool has_side_pair =
        std::find(
            layout.channels.begin(),
            layout.channels.end(),
            "SL") != layout.channels.end();
    const bool has_back_pair =
        std::find(
            layout.channels.begin(),
            layout.channels.end(),
            "BL") != layout.channels.end();
    for (const std::string& name : layout.channels) {
        std::uint32_t speaker = 0U;
        const std::string_view native_name =
            !has_side_pair && name == "BL" ? "SL"
            : !has_side_pair && name == "BR" ? "SR"
            : has_back_pair && name == "SL" ? "LSS"
            : has_back_pair && name == "SR" ? "RSS"
            : std::string_view(name);
        if (!dtsx::standard_speaker_mask(native_name, speaker)) {
            throw std::runtime_error(
                "P2 output layout contains an unknown speaker");
        }
        physical_mask |= speaker;
    }
    return dtsx::speaker_mask_to_activity_mask(physical_mask);
}

std::uint32_t output_slot(
    const std::string& name,
    std::uint32_t physical_mask) {
    std::uint32_t speaker = 0U;
    if (!dtsx::standard_speaker_mask(name, speaker)) {
        return 32U;
    }
    if ((physical_mask & speaker) != 0U) {
        for (std::uint32_t bit = 0U; bit < 32U; ++bit) {
            if (speaker == (1U << bit)) {
                return bit;
            }
        }
    }
    if (name == "BL") {
        return (physical_mask & (1U << 3U)) != 0U ? 3U : 32U;
    }
    if (name == "BR") {
        return (physical_mask & (1U << 4U)) != 0U ? 4U : 32U;
    }
    if (name == "SL") {
        if ((physical_mask & (1U << 9U)) != 0U) {
            return 9U;
        }
        return (physical_mask & (1U << 7U)) != 0U ? 7U : 32U;
    }
    if (name == "SR") {
        if ((physical_mask & (1U << 10U)) != 0U) {
            return 10U;
        }
        return (physical_mask & (1U << 8U)) != 0U ? 8U : 32U;
    }
    return 32U;
}

struct NativeInputPacket final {
    const void* data = nullptr;
    std::uint32_t size = 0U;
    std::uint32_t flags = 0U;
    std::uintptr_t reserved = 0U;
};

struct NativePacketList final {
    NativeInputPacket* packets = nullptr;
    std::uintptr_t count = 0U;
};

struct NativeOutputRecord final {
    std::uint32_t bit_depth = 0U;
    std::uint32_t sample_rate = 0U;
    std::uint32_t physical_mask = 0U;
    std::uint32_t channel_count = 0U;
    std::uint32_t sample_count = 0U;
    std::uint32_t alignment = 0U;
    std::array<const std::int32_t*, 32> channels{};
};

static_assert(offsetof(NativeInputPacket, flags) == 12U);
static_assert(offsetof(NativeOutputRecord, channels) == 24U);

class NativeP2Decoder final {
public:
    explicit NativeP2Decoder(std::uint32_t output_activity_mask) {
        try {
            const std::filesystem::path dll =
                runtime_.decoder_path();
            module_ = LoadLibraryExW(
                dll.c_str(),
                nullptr,
                LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR
                    | LOAD_LIBRARY_SEARCH_SYSTEM32);
            if (module_ == nullptr) {
                throw std::runtime_error(
                    "cannot load embedded 64-bit DTSXDecoder.dll, "
                    "Windows error "
                    + std::to_string(GetLastError()));
            }
            get_size_ =
                native_function<NativeGetSize>(module_, kGetSizeRva);
            create_ =
                native_function<NativeCreate>(module_, kCreateRva);
            decode_ =
                native_function<NativeDecode>(module_, kDecodeRva);
            set_parameter_ =
                native_function<NativeSetParameter>(
                    module_, kSetParameterRva);
            destroy_ =
                native_function<NativeDestroy>(module_, kDestroyRva);

            memory_.resize(get_size_(5) + 8U);
            const int created =
                create_(&decoder_, 5, memory_.data());
            if (created < 0 || decoder_ == nullptr) {
                throw std::runtime_error(
                    "native P2 decoder creation failed: "
                    + std::to_string(created));
            }
            unsigned mask = output_activity_mask;
            const int configured =
                set_parameter_(decoder_, 102, &mask, sizeof(mask));
            if (configured < 0) {
                throw std::runtime_error(
                    "native P2 speaker configuration failed: "
                    + std::to_string(configured));
            }
            unsigned mode = 0U;
            const int output_mode =
                set_parameter_(decoder_, 100, &mode, sizeof(mode));
            if (output_mode < 0) {
                throw std::runtime_error(
                    "native P2 output mode configuration failed: "
                    + std::to_string(output_mode));
            }
            const int decode_mode =
                set_parameter_(decoder_, 106, &mode, sizeof(mode));
            if (decode_mode < 0) {
                throw std::runtime_error(
                    "native P2 decode mode configuration failed: "
                    + std::to_string(decode_mode));
            }
        } catch (...) {
            release();
            throw;
        }
    }

    ~NativeP2Decoder() {
        release();
    }

    NativeP2Decoder(const NativeP2Decoder&) = delete;
    NativeP2Decoder& operator=(const NativeP2Decoder&) = delete;

    void decode(
        const dtsx::ElementaryFrame& frame,
        const ChannelLayout& layout,
        std::uint32_t requested_sample_rate,
        std::vector<std::vector<std::int32_t>>& planar,
        std::uint32_t& sample_rate) {
        NativeInputPacket packet{
            frame.bytes.data(),
            static_cast<std::uint32_t>(frame.bytes.size()),
            16U,
            0U,
        };
        NativePacketList packets{&packet, 1U};
        std::array<std::uint8_t, 512> output_storage{};
        void* output = output_storage.data();
        std::array<std::uint8_t, 160> decode_status{};
        const int result = decode_(
            decoder_,
            &packets,
            &output,
            decode_status.data());
        if (result < 0 || output == nullptr) {
            throw std::runtime_error(
                "native P2 frame decode failed: "
                + std::to_string(result));
        }

        const auto* record =
            static_cast<const NativeOutputRecord*>(output);
        if (record->bit_depth != 24U
            || record->sample_rate == 0U
            || record->sample_count == 0U
            || record->sample_count > 8192U) {
            throw std::runtime_error(
                "native P2 decoder returned an invalid PCM record");
        }
        if (requested_sample_rate != 0U
            && requested_sample_rate != record->sample_rate) {
            throw std::runtime_error(
                "native P2 sample rate differs from --sample-rate");
        }
        sample_rate = record->sample_rate;
        planar.assign(
            layout.channels.size(),
            std::vector<std::int32_t>(record->sample_count, 0));
        for (std::size_t channel = 0U;
             channel < layout.channels.size();
             ++channel) {
            const std::uint32_t slot =
                output_slot(
                    layout.channels[channel],
                    record->physical_mask);
            if (slot >= record->channels.size()
                || record->channels[slot] == nullptr) {
                continue;
            }
            std::copy_n(
                record->channels[slot],
                record->sample_count,
                planar[channel].begin());
        }
    }

private:
    void release() noexcept {
        if (decoder_ != nullptr && destroy_ != nullptr) {
            destroy_(decoder_);
            decoder_ = nullptr;
        }
        if (module_ != nullptr) {
            FreeLibrary(module_);
            module_ = nullptr;
        }
    }

    EmbeddedP2Runtime runtime_;
    HMODULE module_ = nullptr;
    NativeGetSize get_size_ = nullptr;
    NativeCreate create_ = nullptr;
    NativeDecode decode_ = nullptr;
    NativeSetParameter set_parameter_ = nullptr;
    NativeDestroy destroy_ = nullptr;
    std::vector<std::uint8_t> memory_;
    void* decoder_ = nullptr;
};

} // namespace

bool input_is_dts_uhd(const Options& options) {
    Options probe_options = options;
    probe_options.probe = true;
    DtsFrameReader reader(probe_options);
    dtsx::ElementaryFrame frame;
    if (!reader.read(frame)) {
        return false;
    }
    const bool dts_uhd =
        frame.packing == dtsx::StreamPacking::DtsUhd;
    if (!is_elementary_dts(options.input)) {
        while (reader.read(frame)) {
        }
    }
    return dts_uhd;
}

void decode_p2_stream(
    const Options& options,
    const std::filesystem::path& output) {
    if (options.layout.empty()) {
        throw std::runtime_error("--layout is required for P2 decode");
    }
    std::optional<ChannelLayout> layout =
        find_layout(options.layout);
    if (!layout) {
        throw std::runtime_error(
            "unsupported --layout; supported: "
            + supported_layouts_text());
    }
    if (options.dolby_output) {
        layout = dolby_ordered_layout(*layout);
    }
    if (options.render_mode == RenderMode::ObjectsOnly) {
        throw std::runtime_error(
            "DTS-UHD full channel-based mix has no dynamic objects");
    }

    NativeP2Decoder decoder(requested_activity_mask(*layout));
    DtsFrameReader reader(options);
    ProgressReporter progress;
    std::error_code size_error;
    const std::uint64_t elementary_size =
        std::filesystem::file_size(options.input, size_error);
    progress.update("decode P2", size_error ? -1 : 0);
    dtsx::ElementaryFrame frame;
    std::unique_ptr<WavWriter> writer;
    std::unique_ptr<MonoTrackWriter> mono_writer;
    std::vector<std::vector<std::int32_t>> planar;
    std::uint32_t sample_rate = 0U;
    std::uint64_t access_unit_count = 0U;
    std::uint64_t encoded_byte_count = 0U;
    while (reader.read(frame)) {
        if (frame.packing != dtsx::StreamPacking::DtsUhd) {
            throw std::runtime_error(
                "P2 decoder received a non-DTS-UHD frame");
        }
        decoder.decode(
            frame,
            *layout,
            options.sample_rate,
            planar,
            sample_rate);
        if (writer == nullptr) {
            writer = std::make_unique<WavWriter>(
                output,
                *layout,
                sample_rate,
                options.overwrite,
                options.output_format,
                options.dolby_output);
            if (options.mono_tracks) {
                mono_writer = std::make_unique<MonoTrackWriter>(
                    options.mono_tracks_directory,
                    *layout,
                    sample_rate,
                    options.overwrite);
            }
        }
        const std::uint64_t frame_limit =
            duration_frame_limit(options, sample_rate);
        const std::uint64_t remaining =
            frame_limit - std::min(
                frame_limit, writer->frames_written());
        const std::size_t frames_to_write =
            static_cast<std::size_t>(
                std::min<std::uint64_t>(
                    remaining,
                    planar.empty() ? 0U : planar.front().size()));
        writer->write_planar_24(planar, frames_to_write);
        if (mono_writer != nullptr) {
            mono_writer->write_planar_24(
                planar, frames_to_write);
        }
        encoded_byte_count += frame.bytes.size();
        if (options.duration_seconds != 0U) {
            progress.update(
                "decode P2",
                static_cast<int>(
                    std::min<std::uint64_t>(
                        99U,
                        writer->frames_written() * 100U
                            / frame_limit)));
        } else if (!size_error && elementary_size != 0U) {
            progress.update(
                "decode P2",
                static_cast<int>(
                    std::min<std::uint64_t>(
                        99U,
                        encoded_byte_count * 100U
                            / elementary_size)));
        }
        ++access_unit_count;
        if (writer->frames_written() >= frame_limit) {
            break;
        }
    }
    if (writer == nullptr) {
        throw std::runtime_error(
            "P2 decoder produced no PCM frames");
    }
    writer->close();
    if (mono_writer != nullptr) {
        mono_writer->close();
    }
    progress.done("decode P2");
    std::cerr << "Frames: " << writer->frames_written() << '\n';
    std::cerr << "P2 access units: " << access_unit_count << '\n';
    if (options.verbose) {
        std::cerr << "P2 elementary bytes: "
                  << encoded_byte_count << '\n';
    }
}

} // namespace dtsx_decode
