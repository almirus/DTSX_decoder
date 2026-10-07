#include "io/p2_decoder.hpp"

#include "app/object_frame_decoder.hpp"
#include "app/progress.hpp"
#include "audio/layout.hpp"
#include "dtsx/ace_frame.hpp"
#include "dtsx/ace_lfe.hpp"
#include "dtsx/ace_stream.hpp"
#include "dtsx/speaker_mask.hpp"
#include "dtsx/uhd_frame.hpp"
#include "io/dts_frame_reader.hpp"
#include "io/p2_resources.hpp"
#include "dtsx/ace_spectral_payload.hpp"
#include "dtsx/ace_mdct.hpp"
#include "dtsx/ace_lts.hpp"
#include "dtsx/ace_deemphasis.hpp"
#include "io/object_stem_writer.hpp"
#include "io/wav_writer.hpp"
#include "render/object_audio_renderer.hpp"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cwctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <map>
#include <set>

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

std::string_view layout_native_speaker_name(
    const ChannelLayout& layout,
    const std::string& name) {
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
    if (!has_side_pair && name == "BL") {
        return "SL";
    }
    if (!has_side_pair && name == "BR") {
        return "SR";
    }
    if (has_back_pair && name == "SL") {
        return "LSS";
    }
    if (has_back_pair && name == "SR") {
        return "RSS";
    }
    return name;
}

std::uint32_t layout_speaker_mask(
    const ChannelLayout& layout,
    const std::string& name) {
    std::uint32_t speaker = 0U;
    if (!dtsx::standard_speaker_mask(
            layout_native_speaker_name(layout, name), speaker)) {
        throw std::runtime_error(
            "P2 output layout contains an unknown speaker");
    }
    return speaker;
}

std::uint32_t requested_activity_mask(
    const ChannelLayout& layout) {
    std::uint32_t physical_mask = 0U;
    for (const std::string& name : layout.channels) {
        physical_mask |= layout_speaker_mask(layout, name);
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

std::array<std::uint32_t, 2> ace_routing_slots(
    std::uint32_t physical_mask,
    std::uint32_t channel_ordinal,
    bool stereo,
    std::uint32_t effective_channel_count = 0U,
    bool skip_lfe = false) {
    std::array<std::uint32_t, 2> result{32U, 32U};
    const auto speakers = skip_lfe
        ? dtsx::expand_speaker_activity_mask_without_lfe(physical_mask)
        : dtsx::expand_speaker_activity_mask(physical_mask);
    const std::size_t width = stereo ? 2U : 1U;
    for (std::size_t channel = 0U; channel < speakers.size(); ++channel) {
        const std::size_t source = static_cast<std::size_t>(channel_ordinal)
            + channel;
        if (channel >= width || source >= speakers.size()
            || (effective_channel_count != 0U
                && channel >= effective_channel_count)) {
            continue;
        }
        for (std::uint32_t bit = 0U; bit < 32U; ++bit) {
            if (speakers[source] == (1U << bit)) {
                result[channel] = bit;
                break;
            }
        }
    }
    return result;
}

std::int32_t ace_float_to_q31(float value) noexcept {
    if (!std::isfinite(value)) {
        return 0;
    }
    const float clamped = (std::max)(-1.0F, (std::min)(1.0F, value));
    const double scaled = static_cast<double>(clamped) * 2147483648.0;
    if (scaled >= 2147483647.0) {
        return 2147483647;
    }
    if (scaled <= -2147483648.0) {
        return static_cast<std::int32_t>(-2147483647 - 1);
    }
    return static_cast<std::int32_t>(std::lrint(scaled));
}

std::int32_t ace_float_to_pcm24(float value) noexcept {
    if (!std::isfinite(value)) {
        return 0;
    }
    // Homatic/Sony dts_flib_pcmcopy_f32_to_i32_24bit_decoder uses a
    // 8388600.0 scale, sign-dependent +/-0.5 rounding and truncation before
    // the 24-bit clamp.  Using lrint(value*2^23) here creates avoidable
    // one-LSB differential mismatches against the native PCM path.
    const double scaled = static_cast<double>(value) * 8388600.0;
    const double rounded = scaled + (scaled >= 0.0 ? 0.5 : -0.5);
    if (rounded >= 8388607.0) {
        return 8388607;
    }
    if (rounded <= -8388608.0) {
        return -8388608;
    }
    return static_cast<std::int32_t>(rounded);
}

std::size_t ace_short_transform_block_size(
    const std::array<std::uint32_t, 22>& block_counts) noexcept {
    std::uint32_t block_count = 1U;
    for (const std::uint32_t value : block_counts) {
        block_count = (std::max)(block_count, value == 0U ? 1U : value);
    }
    if (block_count == 0U || 1024U % block_count != 0U
        || (block_count & (block_count - 1U)) != 0U) {
        return 0U;
    }
    // DTSAceMdct_SetWindowSize(1 << v61) after the ACE short-transform bit
    // (v61 = 7) yields 128.  BandDequant a1[2] = 1024 >> 7 = 8, so
    // Inverse_Process runs eight 128-point transforms.  The previous
    // 1024/(factor/2) mapping produced 256 when every band used the default
    // TF of 8 and deinterleaved with stride 4 against a stride-8 reshape.
    (void)block_count;
    return 128U;
}

std::uint32_t ace_block_shift(
    const std::array<std::uint32_t, 22>& block_counts) noexcept {
    std::uint32_t block_count = 1U;
    for (const std::uint32_t value : block_counts) {
        block_count = (std::max)(block_count, value == 0U ? 1U : value);
    }
    std::uint32_t shift = 0U;
    while (block_count > 1U && (block_count & 1U) == 0U) {
        block_count >>= 1U;
        ++shift;
    }
    return block_count == 1U ? shift : 0U;
}

// DTSAceStreamDecoder: v22 starts at -1 and increments while
// transform_size < (1024 >> v22).  THF SetCommonParam stores this as a2[5].
std::uint32_t ace_thf_transform_shift(std::size_t transform_size) noexcept {
    if (transform_size == 0U || transform_size > 1024U) {
        return 0U;
    }
    std::uint32_t shift = static_cast<std::uint32_t>(-1);
    do {
        ++shift;
        if (shift >= 31U) {
            return 0U;
        }
    } while (transform_size < (1024U >> shift));
    return shift;
}

bool apply_internal_mdct_overlap(
    dtsx::AceMdctHistoryBank& history,
    std::size_t channel,
    std::vector<float>& samples,
    std::size_t transform_size,
    std::size_t previous_transform_size) noexcept {
    if (samples.size() != dtsx::AceMdctHistoryBank::kHistorySamples
        || (transform_size != 128U && transform_size != 256U
            && transform_size != 512U && transform_size != 1024U)
        || samples.size() % transform_size != 0U
        || channel >= history.channel_count()) {
        return false;
    }
    const auto window_size_for = [](const std::size_t size) {
        return size == 128U ? dtsx::AceMdctWindowSize::Size128
            : size == 256U ? dtsx::AceMdctWindowSize::Size256
            : size == 512U ? dtsx::AceMdctWindowSize::Size512
            : dtsx::AceMdctWindowSize::Size1024;
    };
    const auto load_window = [&](const std::size_t size,
                                 std::vector<float>& target) {
        std::vector<std::int32_t> table;
        if (!dtsx::ace_mdct_window_coefficients(
                window_size_for(size), table)
            || table.size() != size) {
            return false;
        }
        target.resize(size);
        for (std::size_t index = 0U; index < size; ++index) {
            target[index] = static_cast<float>(table[index]) / 2147483648.0F;
        }
        return true;
    };
    float* const state = history.float_plane(channel);
    if (state == nullptr) {
        return false;
    }
    // DTSAceMdct_Inverse_Process: the first block overlaps against the tail
    // of the persistent plane using a5 = min(current, previous); every later
    // block takes the preceding block's second half and the full current
    // size.  OverlapAdd only touches a2[0, a5/2) and a3[0, a5/2).
    const std::size_t previous_transform = previous_transform_size == 0U
        ? 1024U : previous_transform_size;
    std::size_t overlap_size =
        (std::min)(transform_size, previous_transform);
    std::vector<float> window;
    if (!load_window(overlap_size, window)) {
        return false;
    }
    float* overlap_plane = state
        + (dtsx::AceMdctHistoryBank::kHistorySamples - overlap_size / 2U);
    const std::size_t block_count = samples.size() / transform_size;
    for (std::size_t block = 0U; block < block_count; ++block) {
        float* const current = samples.data() + block * transform_size;
        if (!dtsx::ace_overlap_add_f32_native(
                current, overlap_plane, window.data(), overlap_size)) {
            return false;
        }
        overlap_plane = current + transform_size / 2U;
        if (overlap_size != transform_size) {
            overlap_size = transform_size;
            if (!load_window(overlap_size, window)) {
                return false;
            }
        }
    }
    // The frame handed to the caller is the plane as it stood before this
    // access unit; the freshly overlapped scratch only becomes visible one
    // frame later.  This is the native copy order in Inverse_Process.
    std::vector<float> emitted(state,
        state + dtsx::AceMdctHistoryBank::kHistorySamples);
    std::copy(samples.begin(), samples.end(), state);
    samples = std::move(emitted);
    history.set_transform_size(transform_size);
    return true;
}

bool use_native_mdct_candidate() noexcept {
    const char* value = std::getenv("DTSX_P2_NATIVE_MDCT_CANDIDATE");
    return value != nullptr && value[0] == '1';
}


bool apply_internal_lts(
    dtsx::AceLtsHistory& state,
    std::vector<float>& samples,
    bool enabled,
    std::uint32_t lag,
    std::uint32_t filter_index) noexcept {
    return dtsx::ace_lts_process_f32(state, samples, enabled, lag, filter_index);
}

bool apply_internal_deemphasis(
    std::unordered_map<std::uint64_t, float>& states,
    std::uint64_t key,
    std::vector<float>& samples) noexcept {
    if (samples.empty()) {
        return false;
    }
    const float previous = states[key];
    states[key] = dtsx::ace_deemphasis_channel(samples, previous);
    return true;
}

// DTSAceStreamDecoder_Process keeps the de-emphasis enable flag delayed by
// one frame: v47 = state[+20]; state[+16] = v47; state[+20] = (a5[4] == 0);
// FilterProcess runs only when v47 is set.  Zero-initialized state therefore
// skips the first frame even if the current ACE header enables de-emphasis.
bool consume_delayed_deemphasis_flag(
    std::unordered_map<std::uint64_t, bool>& delayed,
    std::uint64_t stream_key,
    bool current_enabled) noexcept {
    const bool apply = delayed[stream_key];
    delayed[stream_key] = current_enabled;
    return apply;
}

std::uint64_t stream_channel_state_key(
    std::uint64_t stream_key,
    std::size_t channel) noexcept {
    // Preserve the ACE namespace (stereo bit and audio-chunk index) in the
    // upper bits.  Channel-local state is mixed only into the stream index;
    // this lets a geometry reset discard one native ACEW instance without
    // touching the other chunk's MDCT/LTS/de-emphasis history.
    constexpr std::uint64_t kLowMask = 0x00000000FFFFFFFFULL;
    const std::uint64_t salt = static_cast<std::uint64_t>(channel + 1U)
        * 0x9E3779B97F4A7C15ULL;
    return (stream_key & ~kLowMask)
        | ((stream_key ^ salt) & kLowMask);
}

std::uint64_t ace_payload_state_key(
    std::uint32_t audio_chunk_index,
    std::uint32_t stream_set_id,
    std::uint32_t stream_index,
    bool stereo) noexcept {
    // homatic keeps main and object ACEW instances in separate decoder
    // states.  The FTOC/MDE audio-chunk index is the only source-level
    // discriminator available before the native object key is resolved;
    // retain it in the state namespace so two chunks cannot share MDCT/LTS
    // history when object-chunk decoding is enabled.
    return (static_cast<std::uint64_t>(stereo) << 63U)
        | (static_cast<std::uint64_t>(audio_chunk_index & 0x7FFFU) << 48U)
        | (static_cast<std::uint64_t>(stream_set_id) << 32U)
        | stream_index;
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
    struct AceChunkGeometry final {
        bool initialized = false;
        std::uint32_t sample_rate = 0U;
        std::uint32_t frame_duration = 0U;
        std::uint32_t bandwidth_mode = 0U;
        std::vector<dtsx::AceStreamSetHeader> stream_sets;
    };

    struct AceHoleFillHistory final {
        std::vector<std::vector<float>> left;
        std::vector<std::vector<float>> right;
        std::vector<std::vector<float>> work_left;
        std::vector<std::vector<float>> work_right;
        std::vector<std::uint32_t> stereo_coding;
    };

    struct InternalAceBus final {
        std::uint64_t stream_key = 0U;
        std::uint64_t frame_index = 0U;
        std::uint32_t audio_chunk_index = 0U;
        bool object_audio = false;
        bool object_mask_from_metadata = false;
        std::uint32_t stream_set_id = 0U;
        std::uint32_t stream_index = 0U;
        std::uint32_t channel_ordinal = 0U;
        std::uint32_t coding_mode = 0U;
        std::uint32_t effective_channel_count = 0U;
        std::uint32_t coded_channel_count = 0U;
        std::uint32_t physical_mask = 0U;
        std::uint32_t sample_rate = 0U;
        std::uint32_t frame_duration = 0U;
        std::uint32_t bandwidth_mode = 0U;
        dtsx::AceStreamType payload_type = dtsx::AceStreamType::Mono;
        bool stereo = false;
        bool short_transform = false;
        bool spectral_hole_fill = false;
        bool temporal_hole_fill = false;
        bool lts_enabled = false;
        std::uint32_t lts_lag = 0U;
        std::uint32_t lts_filter_index = 0U;
        bool lts_filter_reused = false;
        bool mdct_history_initialized = false;
        std::uint32_t previous_mdct_transform_size = 0U;
        std::int32_t lts_center_q31 = 0;
        std::int32_t lts_adjacent_q31 = 0;
        std::int32_t lts_outer_q31 = 0;
        std::array<std::uint32_t, 2> routing_slots{32U, 32U};
        std::array<std::uint32_t, 22> band_block_count{};
        std::array<std::uint32_t, 22> bit_allocation{};
        float peak = 0.0F;
        // Retain the pre-MDCT coefficient planes for the future aggregate
        // 7-channel temporal-hole-fill stage. Native THF operates on the
        // complete frame, not on the already synthesized per-stream PCM.
        std::vector<std::vector<float>> spectral_coefficients;
        // CalcBlockSqNorms state captured for each local ACE stream.  These
        // planes are kept separate until all streamsets in the access unit
        // have been assembled into the native aggregate channel order.
        std::vector<std::vector<float>> temporal_hole_fill_norms;
        std::vector<std::vector<std::uint8_t>> temporal_hole_fill_active;
        std::vector<std::vector<float>> pcm;
    };

    struct AggregateTemporalHoleFillState final {
        std::uint32_t audio_chunk_index = 0U;
        std::uint32_t sample_rate = 0U;
        std::uint32_t channel_count = 0U;
        std::uint32_t band_count = 0U;
        std::uint32_t bandwidth_mode = 0U;
        std::array<std::uint32_t, 10U> physical_slots{};
        std::vector<std::vector<float>> coefficients;
        std::vector<std::vector<float>> norms;
        std::vector<std::vector<std::uint8_t>> active;
        std::vector<std::vector<float>> norm_thresholds;
        std::vector<std::vector<std::uint32_t>> bit_allocation;
        std::vector<std::vector<std::uint8_t>> bit_allocation_indices;
        std::vector<std::vector<float>> bit_alloc_thresholds;
        std::vector<float> thf_noise_output_scales;
        std::vector<std::vector<float>> previous_norms;
        std::vector<std::vector<float>> history_norms;
    };

    explicit NativeP2Decoder(
        std::uint32_t output_activity_mask,
        bool internal_pcm_requested) {
        // Keep the embedded DLL authoritative by default.  This opt-in is a
        // differential diagnostic for the internal ACE path; it falls back
        // to the DLL whenever a complete internal channel map is unavailable.
        prefer_internal_pcm_ = internal_pcm_requested
            || std::getenv("DTSX_P2_INTERNAL_PCM") != nullptr;
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

    [[nodiscard]] std::uint64_t internal_spectral_payloads() const noexcept {
        return internal_spectral_payloads_;
    }

    [[nodiscard]] std::uint64_t internal_spectral_failures() const noexcept {
        return internal_spectral_failures_;
    }

    [[nodiscard]] const std::string& internal_last_failure() const noexcept {
        return internal_last_failure_;
    }

    [[nodiscard]] std::uint64_t internal_parse_failures() const noexcept {
        return internal_parse_failures_;
    }

    [[nodiscard]] std::uint64_t internal_bus_frames() const noexcept {
        return internal_bus_frames_;
    }

    [[nodiscard]] std::uint64_t internal_synthesis_frames() const noexcept {
        return internal_synthesis_frames_;
    }

    [[nodiscard]] std::uint64_t internal_synthesis_failures() const noexcept {
        return internal_synthesis_failures_;
    }

    [[nodiscard]] std::uint64_t internal_short_transform_frames() const noexcept {
        return internal_short_transform_frames_;
    }

    [[nodiscard]] std::uint64_t internal_routed_buses() const noexcept {
        return internal_routed_buses_;
    }

    [[nodiscard]] std::size_t internal_history_banks() const noexcept {
        return ace_mdct_histories_.size();
    }

    [[nodiscard]] std::uint64_t internal_lfe_checked() const noexcept {
        return internal_lfe_checked_;
    }

    [[nodiscard]] std::uint64_t internal_lfe_mismatches() const noexcept {
        return internal_lfe_mismatches_;
    }

    [[nodiscard]] std::uint64_t internal_pcm_selected() const noexcept {
        return internal_pcm_selected_;
    }

    [[nodiscard]] std::uint64_t internal_pcm_gate_rejections() const noexcept {
        return internal_pcm_gate_rejections_;
    }

    [[nodiscard]] std::uint64_t internal_temporal_hole_fill_frames() const noexcept {
        return internal_temporal_hole_fill_frames_;
    }

    [[nodiscard]] std::uint64_t
    internal_temporal_hole_fill_aggregate_frames() const noexcept {
        return internal_temporal_hole_fill_aggregate_frames_;
    }

    [[nodiscard]] std::uint64_t internal_long_transform_rejections() const noexcept {
        return internal_long_transform_rejections_;
    }

    [[nodiscard]] std::uint64_t internal_pcm_compared_samples() const noexcept {
        return internal_pcm_compared_samples_;
    }

    [[nodiscard]] std::uint64_t internal_pcm_mismatch_samples() const noexcept {
        return internal_pcm_mismatch_samples_;
    }

    [[nodiscard]] std::int64_t internal_pcm_max_abs_error() const noexcept {
        return internal_pcm_max_abs_error_;
    }

    [[nodiscard]] std::uint64_t internal_pcm_audible_samples() const noexcept {
        return internal_pcm_audible_samples_;
    }

    [[nodiscard]] std::uint64_t
    internal_pcm_audible_mismatches() const noexcept {
        return internal_pcm_audible_mismatches_;
    }

    [[nodiscard]] double internal_pcm_correlation() const noexcept {
        return internal_pcm_self_ > 0.0
            ? internal_pcm_cross_ / internal_pcm_self_ : 0.0;
    }

    [[nodiscard]] const std::vector<InternalAceBus>&
    last_internal_ace_buses() const noexcept {
        return last_internal_ace_buses_;
    }

    // Object ACE buses keyed by resolved MDE audio_chunk_index. Unresolved
    // chunks are omitted. This is not a DSP2/APE mix; the main bed stays DLL.
    [[nodiscard]] DecodedObjectAudioFrame last_object_audio_frame(
        std::uint32_t sample_rate) const {
        DecodedObjectAudioFrame decoded;
        decoded.sample_rate = sample_rate;
        std::set<std::uint32_t> unresolved(
            unresolved_object_audio_chunk_indices_.begin(),
            unresolved_object_audio_chunk_indices_.end());
        std::map<std::uint32_t, std::vector<const InternalAceBus*>> by_chunk;
        for (const InternalAceBus& bus : last_internal_ace_buses_) {
            if (!bus.object_audio
                || unresolved.find(bus.audio_chunk_index)
                    != unresolved.end()) {
                continue;
            }
            by_chunk[bus.audio_chunk_index].push_back(&bus);
        }
        std::set<std::uint32_t> emitted_chunks;
        for (const auto& association : metadata_object_associations_) {
            if (unresolved.find(association.audio_chunk_index)
                    != unresolved.end()
                || emitted_chunks.find(association.audio_chunk_index)
                    != emitted_chunks.end()) {
                continue;
            }
            const auto found = by_chunk.find(association.audio_chunk_index);
            if (found == by_chunk.end() || found->second.empty()) {
                continue;
            }
            std::vector<const InternalAceBus*> buses = found->second;
            std::sort(
                buses.begin(),
                buses.end(),
                [](const InternalAceBus* left, const InternalAceBus* right) {
                    if (left->stream_set_id != right->stream_set_id) {
                        return left->stream_set_id < right->stream_set_id;
                    }
                    return left->stream_index < right->stream_index;
                });
            dtsx::ObjectMetadataBlock object;
            object.metadata_present = true;
            object.object_id_available = true;
            object.object_id = association.object_id;
            object.waveform_id_available = true;
            object.waveform_id = 0U;
            object.waveform_channel_base_offset = static_cast<std::uint32_t>(
                decoded.waveform_channels.size());
            std::uint8_t waveform_offset = 0U;
            for (const InternalAceBus* bus : buses) {
                for (std::size_t channel = 0U;
                     channel < bus->pcm.size();
                     ++channel) {
                    std::vector<std::int32_t> samples;
                    samples.reserve(bus->pcm[channel].size());
                    for (const float value : bus->pcm[channel]) {
                        samples.push_back(ace_float_to_pcm24(value));
                    }
                    if (decoded.samples_per_channel == 0U
                        && !samples.empty()) {
                        decoded.samples_per_channel =
                            static_cast<std::uint32_t>(samples.size());
                    }
                    decoded.waveform_channels.push_back(std::move(samples));
                    const std::uint32_t slot = channel < bus->routing_slots.size()
                        ? bus->routing_slots[channel] : 32U;
                    decoded.waveform_speaker_masks.push_back(
                        slot < 32U ? (1U << slot) : 0U);
                    decoded.waveform_source_activity_masks.push_back(
                        bus->physical_mask);
                    decoded.waveform_is_supplemental.push_back(false);
                    object.waveform_channel_offsets.push_back(waveform_offset);
                    if (waveform_offset
                        < std::numeric_limits<std::uint8_t>::max()) {
                        ++waveform_offset;
                    }
                }
            }
            object.preamble.waveform_count = waveform_offset;
            object.preamble.waveform_types.assign(waveform_offset, 0U);
            decoded.objects.push_back(std::move(object));
            emitted_chunks.insert(association.audio_chunk_index);
        }
        return decoded;
    }

    [[nodiscard]] const std::set<std::uint32_t>&
    metadata_object_ids() const noexcept {
        return metadata_object_ids_;
    }

    [[nodiscard]] const std::vector<dtsx::UhdFrameHeader::MetadataObjectAssociation>&
    metadata_object_associations() const noexcept {
        return metadata_object_associations_;
    }

    [[nodiscard]] const std::vector<std::uint32_t>&
    unresolved_object_audio_chunk_indices() const noexcept {
        return unresolved_object_audio_chunk_indices_;
    }

    [[nodiscard]] bool has_3d_object_metadata() const noexcept {
        return has_3d_object_metadata_;
    }

    [[nodiscard]] const std::vector<dtsx::UhdFrameHeader::ThreeDObjectMetadata>&
    three_d_object_metadata() const noexcept {
        return three_d_object_metadata_;
    }

    void decode(
        const dtsx::ElementaryFrame& frame,
        const ChannelLayout& layout,
        std::uint32_t requested_sample_rate,
        std::vector<std::vector<std::int32_t>>& planar,
        std::uint32_t& sample_rate) {
        last_internal_ace_buses_.clear();
        const std::uint64_t frame_index = internal_frame_index_++;
        std::optional<dtsx::AceFrameHeader> ace_header;
        try {
            ace_header = parse_ace(frame);
            if (!last_internal_ace_buses_.empty()) {
                ++internal_bus_frames_;
            }
            // Homatic calls DTSHD_UHDAssetDecoder_SetObjPresentFlag from the
            // frame-segment control path, before object PCM is decoded.  A
            // failed/missing object ACE bus must therefore still keep the
            // internal bed candidate gated for this access unit; basing the
            // flag only on successfully synthesized buses could select an
            // incomplete bed as authoritative PCM.
            internal_object_audio_present_ = std::any_of(
                last_internal_ace_buses_.begin(),
                last_internal_ace_buses_.end(),
                [](const InternalAceBus& bus) noexcept {
                    return bus.object_audio;
                }) || !active_object_audio_chunk_indices_.empty();
        } catch (const std::exception& error) {
            // Internal P2 parsing is still a gated diagnostic path.  A
            // missing/unported ACE branch must never disable the native DLL
            // oracle/fallback that supplies authoritative PCM.
            ++internal_spectral_failures_;
            ++internal_parse_failures_;
            if (internal_last_failure_.empty()) {
                internal_last_failure_ = "frame " + std::to_string(frame_index)
                    + ": " + error.what();
            }
            // Keep persistent ACE stream/MDCT/LTS state across an AU that the
            // internal parser cannot consume.  The native wrapper retains
            // registration and predictive state while DLL decoding continues;
            // resetting here would make every following non-sync continuation
            // fail until another full sync frame.  Only discard per-AU
            // products assembled before the failure.
            last_internal_ace_buses_.clear();
            last_internal_lfe_pcm_.clear();
            aggregate_temporal_hole_fill_states_.clear();
            internal_object_audio_present_ = false;
            active_object_audio_chunk_indices_.clear();
        }
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
        int result = decode_(
            decoder_,
            &packets,
            &output,
            decode_status.data());
        // homatic P2 wrapper recovery: DSP1 decode retries once after
        // clearing parameter 200 when the frame reports -1022.  Keep the
        // retry bounded and use a fresh output pointer/status buffer exactly
        // as the native call sequence does.
        if (result == -1022 && set_parameter_ != nullptr) {
            std::uint32_t reset = 0U;
            if (set_parameter_(decoder_, 200, &reset, sizeof(reset)) >= 0) {
                output = output_storage.data();
                decode_status.fill(0U);
                result = decode_(
                    decoder_,
                    &packets,
                    &output,
                    decode_status.data());
            }
        }
        if (result < 0 || output == nullptr) {
            reset_internal_state();
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
        bool temporal_hole_fill_present = false;
        for (InternalAceBus& bus : last_internal_ace_buses_) {
            temporal_hole_fill_present = temporal_hole_fill_present
                || bus.temporal_hole_fill;
            if (!bus.object_mask_from_metadata) {
                // NativeOutputRecord reports a physical speaker mask, while
                // ACE waveform registration consumes the compact activity
                // mask. Keep the two domains explicit at this ABI boundary;
                // expanding physical bits as activity ordinals produces
                // invalid channel order and hides aggregate THF channels.
                bus.physical_mask = dtsx::speaker_mask_to_activity_mask(
                    record->physical_mask);
            }
            // Bed ACE waveforms omit LFE (separate stream type).  Indexing
            // the full expansion including activity 3 maps height buses onto
            // physical bit 5 and TFR, which is the slot 5/15 mismatch vs DLL.
            bus.frame_index = frame_index;
            bus.sample_rate = record->sample_rate;
            bus.frame_duration = record->sample_count;
            bus.routing_slots = ace_routing_slots(
                bus.physical_mask, bus.channel_ordinal, bus.stereo,
                bus.effective_channel_count,
                !bus.object_mask_from_metadata);
            if (bus.routing_slots[0] < 32U
                || (bus.stereo && bus.routing_slots[1] < 32U)) {
                ++internal_routed_buses_;
            }
        }
        if (temporal_hole_fill_present) {
            ++internal_temporal_hole_fill_frames_;
        }
        // Homatic THF has a distinct 10-channel branch which only advances
        // history; the 7-channel branch performs the not-yet-ported noise
        // synthesis.  Permit the internal candidate for the former only
        // after the per-frame differential comparison has also passed.
        internal_temporal_hole_fill_frame_safe_ =
            !temporal_hole_fill_present || record->channel_count == 10U;
        internal_pcm_frame_exact_ = false;
        compare_internal_pcm(*record, layout);
        if (ace_header.has_value()
            && (record->sample_rate != ace_header->sample_rate
                || record->sample_count != ace_header->frame_duration)) {
            throw std::runtime_error(
                "native P2 PCM geometry differs from internal ACE header");
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
        if (prefer_internal_pcm_ && use_internal_pcm_if_complete(
                layout, record->sample_count, planar)) {
            ++internal_pcm_selected_;
            if (std::getenv("DTSX_P2_INTERNAL_PCM_VERBOSE") != nullptr) {
                std::cerr << "Internal ACE PCM selected for this frame\n";
            }
        }
        const auto lfe_channel = std::find(
            layout.channels.begin(), layout.channels.end(), "LFE");
        if (lfe_channel != layout.channels.end()
            && !last_internal_lfe_pcm_.empty()
            && last_internal_lfe_pcm_.front().size() == record->sample_count) {
            const std::uint32_t lfe_slot = output_slot(
                "LFE", record->physical_mask);
            if (lfe_slot < record->channels.size()
                && record->channels[lfe_slot] != nullptr) {
                ++internal_lfe_checked_;
                for (std::size_t index = 0U;
                     index < record->sample_count; ++index) {
                    const auto value = static_cast<std::int64_t>(std::llround(
                        last_internal_lfe_pcm_.front()[index] * 8388608.0F));
                    if (value != record->channels[lfe_slot][index]) {
                        ++internal_lfe_mismatches_;
                        break;
                    }
                }
            }
        }
    }

private:
    void reset_internal_state() noexcept {
        uhd_state_ = {};
        ace_frame_states_.clear();
        ace_stream_states_.clear();
        ace_mdct_histories_.clear();
        ace_hole_fill_histories_.clear();
        ace_lts_histories_.clear();
        ace_deemphasis_states_.clear();
        ace_deemphasis_delayed_.clear();
        ace_chunk_geometries_.clear();
        ace_lfe_states_.clear();
        last_internal_ace_buses_.clear();
        last_internal_lfe_pcm_.clear();
        active_object_audio_chunk_indices_.clear();
        internal_temporal_hole_fill_frame_safe_ = false;
        internal_pcm_frame_exact_ = false;
        internal_object_audio_present_ = false;
        aggregate_temporal_hole_fill_states_.clear();
        aggregate_temporal_hole_fill_history_.clear();
        ace_thf_stream_histories_.clear();
        ace_streamset_prng_.clear();
    }

    void assemble_aggregate_temporal_hole_fill_state() {
        aggregate_temporal_hole_fill_states_.clear();
        std::unordered_map<std::uint32_t,
            std::vector<const InternalAceBus*>> grouped;
        for (const InternalAceBus& bus : last_internal_ace_buses_) {
            if (!bus.temporal_hole_fill
                || bus.spectral_coefficients.empty()) {
                continue;
            }
            grouped[bus.audio_chunk_index].push_back(&bus);
        }
        for (const auto& entry : grouped) {
            AggregateTemporalHoleFillState state;
            state.audio_chunk_index = entry.first;
            std::array<std::int32_t, 32U> slot_to_plane;
            slot_to_plane.fill(-1);
            std::vector<std::vector<float>> planes;
            std::vector<std::vector<std::uint32_t>> allocation_planes;
            std::array<std::uint32_t, 10U> slots{};
            std::uint32_t slot_count = 0U;
            std::uint32_t band_count = 0U;
            std::array<std::uint32_t, 22U> block_counts{};
            bool valid = true;
            for (const InternalAceBus* bus : entry.second) {
                if (bus == nullptr || bus->sample_rate == 0U
                    || bus->spectral_coefficients.empty()) {
                    valid = false;
                    break;
                }
                if (state.sample_rate != 0U
                    && state.sample_rate != bus->sample_rate) {
                    valid = false;
                    break;
                }
                if (bus->bandwidth_mode >= 4U
                    || (state.sample_rate != 0U
                        && state.bandwidth_mode != bus->bandwidth_mode)) {
                    valid = false;
                    break;
                }
                state.sample_rate = bus->sample_rate;
                state.bandwidth_mode = bus->bandwidth_mode;
                band_count = band_count == 0U
                    ? static_cast<std::uint32_t>(
                        bus->temporal_hole_fill_norms.size())
                    : (std::min)(band_count,
                        static_cast<std::uint32_t>(
                            bus->temporal_hole_fill_norms.size()));
                for (std::size_t source = 0U;
                     source < bus->spectral_coefficients.size()
                         && source < bus->routing_slots.size(); ++source) {
                    const std::uint32_t slot = bus->routing_slots[source];
                    if (slot >= slot_to_plane.size()
                        || bus->spectral_coefficients[source].empty()) {
                        valid = false;
                        break;
                    }
                    std::int32_t plane_index = slot_to_plane[slot];
                    if (plane_index < 0) {
                        if (slot_count >= slots.size()) {
                            valid = false;
                            break;
                        }
                        plane_index = static_cast<std::int32_t>(slot_count);
                        slot_to_plane[slot] = plane_index;
                        slots[slot_count++] = slot;
                        planes.emplace_back(
                            bus->spectral_coefficients[source].size(),
                            0.0F);
                        allocation_planes.emplace_back(
                            bus->bit_allocation.begin(),
                            bus->bit_allocation.end());
                    } else if (allocation_planes[
                                   static_cast<std::size_t>(plane_index)]
                            != std::vector<std::uint32_t>(
                                bus->bit_allocation.begin(),
                                bus->bit_allocation.end())) {
                        valid = false;
                        break;
                    }
                    auto& destination = planes[static_cast<std::size_t>(
                        plane_index)];
                    const auto& source_plane = bus->spectral_coefficients[
                        source];
                    if (destination.size() != source_plane.size()) {
                        valid = false;
                        break;
                    }
                    for (std::size_t sample = 0U;
                         sample < destination.size(); ++sample) {
                        destination[sample] += source_plane[sample];
                    }
                }
                if (!valid) {
                    break;
                }
                for (std::size_t index = 0U; index < block_counts.size();
                     ++index) {
                    if (bus->band_block_count[index] != 0U) {
                        block_counts[index] = bus->band_block_count[index];
                    }
                }
            }
            // Native THF has separate 7- and 10-channel state branches. Do
            // not manufacture a state for a partial or non-native topology.
            if (!valid || (slot_count != 7U && slot_count != 10U)
                || band_count == 0U) {
                continue;
            }
            std::vector<std::size_t> order(slot_count);
            std::iota(order.begin(), order.end(), 0U);
            std::sort(order.begin(), order.end(),
                [&slots](const std::size_t left,
                         const std::size_t right) noexcept {
                    return slots[left] < slots[right];
                });
            std::array<std::uint32_t, 10U> ordered_slots{};
            std::vector<std::vector<float>> ordered_planes;
            std::vector<std::vector<std::uint32_t>> ordered_allocations;
            ordered_planes.reserve(slot_count);
            ordered_allocations.reserve(slot_count);
            for (std::size_t index = 0U; index < order.size(); ++index) {
                ordered_slots[index] = slots[order[index]];
                ordered_planes.push_back(std::move(planes[order[index]]));
                ordered_allocations.push_back(std::move(
                    allocation_planes[order[index]]));
            }
            slots = ordered_slots;
            planes = std::move(ordered_planes);
            state.bit_allocation = std::move(ordered_allocations);
            state.bit_allocation_indices.assign(
                slot_count, std::vector<std::uint8_t>(band_count, 99U));
            state.bit_alloc_thresholds.assign(
                band_count, std::vector<float>(slot_count, 0.0F));
            state.thf_noise_output_scales.assign(band_count, 0.0F);
            for (std::uint32_t band = 0U; band < band_count; ++band) {
                const std::uint32_t reciprocal_sqrt =
                    dtsx::ace_temporal_hole_fill_recip_sqrt_num_bins_q30(
                        state.sample_rate, state.bandwidth_mode, band);
                state.thf_noise_output_scales[band] =
                    static_cast<float>(reciprocal_sqrt) / 1073741824.0F;
            }
            for (std::size_t channel = 0U; channel < slot_count; ++channel) {
                for (std::uint32_t band = 0U; band < band_count; ++band) {
                    const std::uint32_t reciprocal =
                        dtsx::ace_temporal_hole_fill_recip_num_total_bins_q28(
                            state.sample_rate, state.bandwidth_mode, band);
                    const std::int32_t allocation = band
                        < state.bit_allocation[channel].size()
                        ? static_cast<std::int32_t>(
                            state.bit_allocation[channel][band]) : -1;
                    const auto index =
                        dtsx::ace_temporal_hole_fill_bit_alloc_index_from_recip_q28(
                            reciprocal, allocation);
                    state.bit_allocation_indices[channel][band] =
                        static_cast<std::uint8_t>(index);
                }
            }
            state.channel_count = slot_count;
            state.band_count = band_count;
            state.physical_slots = slots;
            const std::uint32_t block_shift = ace_block_shift(block_counts);
            if (!dtsx::ace_temporal_hole_fill_calc_block_sq_norms(
                    planes, state.sample_rate, state.bandwidth_mode,
                    band_count, slot_count,
                    1U, block_shift, state.norms, state.active)) {
                continue;
            }
            for (std::size_t channel = 0U; channel < slot_count; ++channel) {
                for (std::uint32_t band = 0U; band < band_count; ++band) {
                    const std::uint8_t index =
                        state.bit_allocation_indices[channel][band];
                    state.bit_alloc_thresholds[band][channel] =
                        state.norms[band][channel]
                        * (static_cast<float>(
                            dtsx::ace_temporal_hole_fill_bit_alloc_threshold_q30(
                                index)) / 1073741824.0F);
                }
            }
            state.norm_thresholds.assign(
                band_count, std::vector<float>(slot_count, 0.0F));
            for (std::uint32_t band = 0U; band < band_count; ++band) {
                for (std::uint32_t channel = 0U; channel < slot_count;
                     ++channel) {
                    if (state.active[band][channel] != 0U) {
                        state.norm_thresholds[band][channel] = std::sqrt(
                            state.norms[band][channel]);
                    }
                }
            }
            dtsx::AceTemporalHoleFillHistory& history =
                aggregate_temporal_hole_fill_history_[state.audio_chunk_index];
            if (history.channel_count() != slot_count) {
                history.reset(slot_count);
            }
            state.previous_norms.assign(
                band_count, std::vector<float>(slot_count, 0.0F));
            state.history_norms.assign(
                band_count, std::vector<float>(slot_count, 0.0F));
            const auto& previous = history.previous();
            const auto& historical = history.history();
            for (std::uint32_t channel = 0U; channel < slot_count;
                 ++channel) {
                for (std::uint32_t band = 0U; band < band_count; ++band) {
                    const std::size_t offset =
                        static_cast<std::size_t>(channel) * 22U + band;
                    state.previous_norms[band][channel] = previous[offset];
                    state.history_norms[band][channel] = historical[offset];
                }
            }
            state.coefficients = std::move(planes);
            aggregate_temporal_hole_fill_states_.emplace(
                state.audio_chunk_index, std::move(state));
            std::vector<float> current(
                static_cast<std::size_t>(slot_count) * 22U, 0.0F);
            const auto& current_norms =
                aggregate_temporal_hole_fill_states_.at(entry.first).norms;
            for (std::uint32_t channel = 0U; channel < slot_count;
                 ++channel) {
                for (std::uint32_t band = 0U; band < band_count; ++band) {
                    current[static_cast<std::size_t>(channel) * 22U + band] =
                        current_norms[band][channel];
                }
            }
            history.update(current, band_count);
        }
        if (!aggregate_temporal_hole_fill_states_.empty()) {
            ++internal_temporal_hole_fill_aggregate_frames_;
        }
    }

    void compare_internal_pcm(
        const NativeOutputRecord& record,
        const ChannelLayout& layout) {
        internal_pcm_frame_exact_ = false;
        if (last_internal_ace_buses_.empty() || record.sample_count == 0U
            || record.channels.empty()) {
            return;
        }
        const std::size_t slot_count =
            (std::min)(record.channels.size(), layout.channels.size());
        std::vector<std::vector<std::int64_t>> accum(
            slot_count,
            std::vector<std::int64_t>(record.sample_count, 0));
        std::vector<std::vector<double>> raw_accum(
            slot_count,
            std::vector<double>(record.sample_count, 0.0));
        std::vector<bool> present(slot_count, false);
        std::vector<std::uint64_t> slot_mismatch_samples(slot_count, 0U);
        std::vector<std::int64_t> slot_max_abs_error(slot_count, 0);
        std::uint64_t frame_compared_samples = 0U;
        std::uint64_t frame_mismatch_samples = 0U;
        const bool skip_thf_compare =
            std::getenv("DTSX_P2_SKIP_THF_COMPARE") != nullptr;
        for (const InternalAceBus& bus : last_internal_ace_buses_) {
            if (bus.object_audio || (skip_thf_compare && bus.temporal_hole_fill)) {
                continue;
            }
            for (std::size_t source = 0U;
                 source < bus.pcm.size() && source < 2U; ++source) {
                const std::uint32_t bit = bus.routing_slots[source];
                if (bit >= 32U
                    || bus.pcm[source].size() != record.sample_count) {
                    continue;
                }
                const std::uint32_t mask = 1U << bit;
                for (std::size_t slot = 0U; slot < slot_count; ++slot) {
                    if (record.channels[slot] == nullptr
                        || layout_speaker_mask(layout, layout.channels[slot])
                            != mask) {
                        continue;
                    }
                    present[slot] = true;
                    for (std::size_t sample = 0U;
                         sample < record.sample_count; ++sample) {
                        const float value = bus.pcm[source][sample];
                        if (!std::isfinite(value)) {
                            continue;
                        }
                        accum[slot][sample] += static_cast<std::int64_t>(
                            ace_float_to_pcm24(value));
                        raw_accum[slot][sample] += static_cast<double>(value);
                    }
                }
            }
        }
        for (std::size_t slot = 0U; slot < slot_count; ++slot) {
            if (!present[slot] || record.channels[slot] == nullptr) {
                continue;
            }
            // Waveform correlation on the pre-quantisation floats is the only
            // metric that still moves while the synthesis level is wrong, so
            // it is what tracks progress towards a real match.  It is summed
            // as a magnitude per frame and slot because the current synthesis
            // still flips sign between frames, which would otherwise cancel.
            double cross = 0.0;
            double self = 0.0;
            double reference_energy = 0.0;
            for (std::size_t sample = 0U;
                 sample < record.sample_count; ++sample) {
                const double reference = static_cast<double>(
                    record.channels[slot][sample]);
                cross += raw_accum[slot][sample] * reference;
                self += raw_accum[slot][sample] * raw_accum[slot][sample];
                reference_energy += reference * reference;
            }
            if (self > 0.0 && reference_energy > 0.0) {
                internal_pcm_cross_ += std::fabs(
                    cross / std::sqrt(self * reference_energy));
                internal_pcm_self_ += 1.0;
                if (std::getenv("DTSX_P2_CORR_TRACE") != nullptr) {
                    std::cerr << "CORR "
                              << (internal_frame_index_ == 0U
                                  ? 0U : internal_frame_index_ - 1U)
                              << ' ' << slot << ' '
                              << (cross / std::sqrt(self * reference_energy))
                              << '\n';
                }
            }
            for (std::size_t sample = 0U;
                 sample < record.sample_count; ++sample) {
                const std::int64_t reference =
                    static_cast<std::int64_t>(record.channels[slot][sample]);
                const std::int64_t error = accum[slot][sample] - reference;
                const std::int64_t absolute = error < 0 ? -error : error;
                ++internal_pcm_compared_samples_;
                ++frame_compared_samples;
                // A reference of digital silence is matched by any decoder
                // that emits nothing, so silent samples cannot evidence
                // LSB-exactness.  Track the audible subset separately.
                if (reference != 0 || accum[slot][sample] != 0) {
                    ++internal_pcm_audible_samples_;
                    if (error != 0) {
                        ++internal_pcm_audible_mismatches_;
                    }
                }
                internal_pcm_max_abs_error_ =
                    (std::max)(internal_pcm_max_abs_error_, absolute);
                if (error != 0) {
                    ++internal_pcm_mismatch_samples_;
                    ++frame_mismatch_samples;
                    ++slot_mismatch_samples[slot];
                    slot_max_abs_error[slot] = (std::max)(
                        slot_max_abs_error[slot], absolute);
                    internal_pcm_frame_exact_ = false;
                }
            }
        }
        internal_pcm_frame_exact_ = frame_compared_samples != 0U
            && frame_mismatch_samples == 0U;
        if (!internal_pcm_frame_exact_
            && std::getenv("DTSX_P2_INTERNAL_PCM_VERBOSE") != nullptr) {
            std::cerr << "Internal ACE PCM mismatch frame "
                      << (internal_frame_index_ == 0U
                              ? 0U : internal_frame_index_ - 1U)
                      << " samples=" << frame_compared_samples
                      << " mismatches=" << frame_mismatch_samples;
            if (!last_internal_ace_buses_.empty()) {
                const InternalAceBus& bus = last_internal_ace_buses_.front();
                std::uint32_t max_block = 0U;
                for (const std::uint32_t value : bus.band_block_count) {
                    max_block = (std::max)(max_block, value);
                }
                std::cerr << " short=" << bus.short_transform
                          << " lts=" << bus.lts_enabled
                          << " max-block=" << max_block
                          << " prev-mdct="
                          << bus.previous_mdct_transform_size
                          << " thf=" << bus.temporal_hole_fill;
            }
            std::cerr << '\n';
            if (std::getenv("DTSX_P2_INTERNAL_PCM_VERBOSE_BUSES") != nullptr) {
                for (const InternalAceBus& bus : last_internal_ace_buses_) {
                    std::cerr << "  bus chunk=" << bus.audio_chunk_index
                              << " set=" << bus.stream_set_id
                              << " stream=" << bus.stream_index
                              << " channels=" << bus.coded_channel_count
                              << " stereo=" << bus.stereo
                              << " object=" << bus.object_audio
                              << " thf=" << bus.temporal_hole_fill
                              << " coding=" << bus.coding_mode
                              << " effective-ch="
                              << bus.effective_channel_count
                              << " current-mdct="
                              << (bus.short_transform
                                      ? ace_short_transform_block_size(
                                          bus.band_block_count)
                                      : 1024U)
                              << " prev-mdct="
                              << bus.previous_mdct_transform_size
                              << " slots=" << bus.routing_slots[0]
                              << ',' << bus.routing_slots[1] << '\n';
                }
            }
            for (std::size_t slot = 0U; slot < slot_count; ++slot) {
                if (slot_mismatch_samples[slot] == 0U) {
                    continue;
                }
                std::cerr << "  slot=" << slot
                          << " mismatch-samples="
                          << slot_mismatch_samples[slot]
                          << " max-abs-error=" << slot_max_abs_error[slot]
                          << '\n';
            }
            std::size_t printed = 0U;
            for (std::size_t slot = 0U;
                 slot < slot_count && printed < 16U; ++slot) {
                if (!present[slot] || record.channels[slot] == nullptr) {
                    continue;
                }
                for (std::size_t sample = 0U;
                     sample < record.sample_count && printed < 16U; ++sample) {
                    const std::int64_t error = accum[slot][sample]
                        - static_cast<std::int64_t>(record.channels[slot][sample]);
                    if (error != 0) {
                        std::cerr << "  slot=" << slot
                                  << " sample=" << sample
                                  << " error=" << error << '\n';
                        ++printed;
                    }
                }
            }
        }
    }

    bool use_internal_pcm_if_complete(
        const ChannelLayout& layout,
        std::uint32_t sample_count,
        std::vector<std::vector<std::int32_t>>& planar) {
        if (last_internal_ace_buses_.empty()
            || planar.size() != layout.channels.size()) {
            return false;
        }
        // DSP2 routes object chunks through a separate object processor and
        // APE_RenderObjects. Direct ACE buses cannot reproduce that final
        // mix, so never select them as authoritative PCM for such a frame.
        if (internal_object_audio_present_) {
            ++internal_pcm_gate_rejections_;
            return false;
        }
        // Internal PCM is a diagnostic candidate only until the current
        // access unit has passed the DLL differential gate.  Never replace
        // authoritative output based on a previous frame's counters.
        if (!internal_pcm_frame_exact_) {
            ++internal_pcm_gate_rejections_;
            return false;
        }
        // THF Process is wired before MDCT, but frames that still fail the
        // LSB gate (short overlap / THF bin levels) must keep DLL PCM.
        for (const InternalAceBus& bus : last_internal_ace_buses_) {
            if (bus.temporal_hole_fill
                && (!internal_temporal_hole_fill_frame_safe_
                    || !internal_pcm_frame_exact_)) {
                return false;
            }
        }
        std::vector<bool> present(planar.size(), false);
        std::vector<std::vector<std::int64_t>> accum(
            planar.size(), std::vector<std::int64_t>(sample_count, 0));
        for (const InternalAceBus& bus : last_internal_ace_buses_) {
            if (bus.object_audio) {
                continue;
            }
            for (std::size_t source = 0U;
                 source < bus.pcm.size() && source < 2U;
                 ++source) {
                const std::uint32_t mask = bus.routing_slots[source] < 32U
                    ? (1U << bus.routing_slots[source]) : 0U;
                if (mask == 0U || bus.pcm[source].size() != sample_count) {
                    continue;
                }
                for (std::size_t destination = 0U;
                     destination < layout.channels.size();
                     ++destination) {
                    const std::uint32_t destination_mask =
                        layout_speaker_mask(layout, layout.channels[destination]);
                    if (destination_mask != mask) {
                        continue;
                    }
                    present[destination] = true;
                    for (std::size_t sample = 0U; sample < sample_count;
                         ++sample) {
                        accum[destination][sample] += static_cast<std::int64_t>(
                            ace_float_to_pcm24(bus.pcm[source][sample]));
                    }
                }
            }
        }
        // ACE LFE is decoded by the dedicated native-equivalent LFE path and
        // is intentionally kept outside the generic streamset bus list.
        // Include it in the completeness test without inventing an object
        // association or exporting it under a colliding stream index.
        if (last_internal_lfe_pcm_.size() == 1U
            && last_internal_lfe_pcm_.front().size() == sample_count) {
            for (std::size_t destination = 0U;
                 destination < layout.channels.size();
                 ++destination) {
                if (layout.channels[destination] != "LFE") {
                    continue;
                }
                present[destination] = true;
                for (std::size_t sample = 0U; sample < sample_count; ++sample) {
                    accum[destination][sample] = static_cast<std::int64_t>(
                        std::llround(last_internal_lfe_pcm_.front()[sample]
                                     * 8388608.0F));
                }
            }
        }
        if (std::find(present.begin(), present.end(), false)
            != present.end()) {
            return false;
        }
        for (std::size_t channel = 0U; channel < planar.size(); ++channel) {
            for (std::size_t sample = 0U; sample < sample_count; ++sample) {
                planar[channel][sample] = static_cast<std::int32_t>(
                    (std::max)(static_cast<std::int64_t>(-8388608),
                        (std::min)(static_cast<std::int64_t>(8388607),
                            accum[channel][sample])));
            }
        }
        return true;
    }

    dtsx::AceFrameHeader parse_ace(
        const dtsx::ElementaryFrame& frame) {
        last_internal_lfe_pcm_.clear();
        dtsx::UhdFrameHeader uhd_header;
        if (dtsx::parse_uhd_frame_header(
                frame.bytes,
                uhd_state_,
                uhd_header)
            != dtsx::UhdHeaderParseResult::Complete) {
            throw std::runtime_error(
                "internal DTS-UHD FTOC parse failed during P2 decode");
        }
        metadata_object_ids_.insert(
            uhd_header.metadata_object_ids.begin(),
            uhd_header.metadata_object_ids.end());
        for (const auto& association :
             uhd_header.metadata_object_associations) {
            const auto known = std::find_if(
                metadata_object_associations_.begin(),
                metadata_object_associations_.end(),
                [&association](const auto& candidate) {
                    return candidate.object_id == association.object_id
                        && candidate.representation_type
                            == association.representation_type
                        && candidate.audio_chunk_index
                            == association.audio_chunk_index
                        && candidate.navigation_index
                            == association.navigation_index
                        && candidate.channel_layout_index
                            == association.channel_layout_index
                        && candidate.channel_activity_mask
                            == association.channel_activity_mask;
                });
            if (known == metadata_object_associations_.end()) {
                metadata_object_associations_.push_back(association);
            }
        }
        has_3d_object_metadata_ =
            has_3d_object_metadata_ || uhd_header.has_3d_object_metadata;
        for (const auto& object : uhd_header.three_d_object_metadata) {
            const auto found = std::find_if(
                three_d_object_metadata_.begin(),
                three_d_object_metadata_.end(),
                [&object](const auto& candidate) {
                    return candidate.object_id == object.object_id
                        && candidate.layout_mask == object.layout_mask
                        && candidate.speaker_indices == object.speaker_indices;
                });
            if (found == three_d_object_metadata_.end()) {
                three_d_object_metadata_.push_back(object);
            }
        }
        for (const std::uint32_t chunk_index :
             uhd_header.unresolved_object_audio_chunk_indices) {
            if (std::find(
                    unresolved_object_audio_chunk_indices_.begin(),
                    unresolved_object_audio_chunk_indices_.end(),
                    chunk_index)
                == unresolved_object_audio_chunk_indices_.end()) {
                unresolved_object_audio_chunk_indices_.push_back(chunk_index);
            }
        }
        // SetFrameSegments is per-access-unit state in the native wrapper;
        // object-audio chunk presence must not leak from a prior frame with
        // metadata into a frame that carries no associations.
        active_object_audio_chunk_indices_.clear();
        if (!uhd_header.metadata_object_associations.empty()) {
            for (const auto& association :
                 uhd_header.metadata_object_associations) {
                if (std::find(
                           active_object_audio_chunk_indices_.begin(),
                           active_object_audio_chunk_indices_.end(),
                           association.audio_chunk_index)
                        != active_object_audio_chunk_indices_.end()) {
                    continue;
                }
                active_object_audio_chunk_indices_.push_back(
                    association.audio_chunk_index);
            }
        }
        const auto main_chunk = std::find_if(
            uhd_header.audio_chunks.begin(),
            uhd_header.audio_chunks.end(),
            [](const dtsx::UhdChunk& candidate) {
                return candidate.id == 1U;
            });
        if (main_chunk == uhd_header.audio_chunks.end()) {
            throw std::runtime_error(
                "DTS-UHD frame has no valid ACE audio chunk");
        }
        const std::uint32_t main_audio_chunk_index = main_chunk->index;
        // Native DSP1 decodes every FTOC audio chunk through the asset
        // decoder, but the separate object decoder is entered only for
        // chunks returned by DTSX2_MDE_GetAudioChunk from an object record.
        // A non-main chunk is therefore not automatically object audio: it
        // may be an unmapped supplemental channel-set.  Keep this distinction
        // for bus routing/export instead of classifying all chunk indices
        // other than the main one as objects.
        const auto is_object_audio_chunk = [this, main_audio_chunk_index](
                                                std::uint32_t chunk_index) {
            if (chunk_index == main_audio_chunk_index) {
                return false;
            }
            return std::find(
                       active_object_audio_chunk_indices_.begin(),
                       active_object_audio_chunk_indices_.end(),
                       chunk_index)
                != active_object_audio_chunk_indices_.end();
        };
        const auto activity_mask_for_chunk = [&uhd_header,
                                               &is_object_audio_chunk](
                                                  std::uint32_t chunk_index) {
            if (!is_object_audio_chunk(chunk_index)) {
                return std::pair<std::uint32_t, bool>{
                    uhd_header.speaker_activity_mask, false};
            }
            std::uint32_t mask = 0U;
            bool found = false;
            bool consistent = true;
            for (const auto& association :
                 uhd_header.metadata_object_associations) {
                if (association.audio_chunk_index != chunk_index) {
                    continue;
                }
                if (!found) {
                    mask = association.channel_activity_mask;
                    found = true;
                } else if (association.channel_activity_mask != mask) {
                    consistent = false;
                }
            }
            // Native object decoder registers a channel mask from the object
            // record. If multiple records disagree, retain the FTOC mask and
            // keep the bus diagnostic rather than inventing a merge.
            if (!found || !consistent || mask == 0U) {
                return std::pair<std::uint32_t, bool>{
                    uhd_header.speaker_activity_mask, false};
            }
            return std::pair<std::uint32_t, bool>{mask, true};
        };
        std::vector<const dtsx::UhdChunk*> selected_chunks;
        selected_chunks.push_back(&*main_chunk);
        for (const auto& association :
             uhd_header.metadata_object_associations) {
            const auto object_chunk = std::find_if(
                uhd_header.audio_chunks.begin(),
                uhd_header.audio_chunks.end(),
                [&association](const dtsx::UhdChunk& candidate) {
                    return candidate.index == association.audio_chunk_index;
                });
            if (object_chunk != uhd_header.audio_chunks.end()
                && std::find(
                       selected_chunks.begin(), selected_chunks.end(),
                       &*object_chunk) == selected_chunks.end()) {
                selected_chunks.push_back(&*object_chunk);
            }
        }
        for (const std::uint32_t object_chunk_index :
             active_object_audio_chunk_indices_) {
            const auto object_chunk = std::find_if(
                uhd_header.audio_chunks.begin(),
                uhd_header.audio_chunks.end(),
                [object_chunk_index](const dtsx::UhdChunk& candidate) {
                    return candidate.index == object_chunk_index;
                });
            if (object_chunk != uhd_header.audio_chunks.end()
                && std::find(
                       selected_chunks.begin(), selected_chunks.end(),
                       &*object_chunk) == selected_chunks.end()) {
                selected_chunks.push_back(&*object_chunk);
            }
        }
        // DTSHD_UHDAssetDecoder_SetFrameSegments receives the complete FTOC
        // segment list and DecodeSubframe walks every ACE audio chunk.  Do
        // not make metadata association the admission criterion: a valid
        // supplemental/object chunk may be intentionally unmapped by MDE and
        // still has to produce an independent PCM bus.
        for (const dtsx::UhdChunk& candidate : uhd_header.audio_chunks) {
            if (candidate.id != 1U
                || std::find(
                       selected_chunks.begin(), selected_chunks.end(),
                       &candidate) != selected_chunks.end()) {
                continue;
            }
            selected_chunks.push_back(&candidate);
        }
        dtsx::AceFrameHeader main_ace_header;
        bool first_chunk = true;
        for (const dtsx::UhdChunk* chunk : selected_chunks) {
            if (chunk == nullptr
                || chunk->offset > frame.bytes.size()
                || chunk->size > frame.bytes.size() - chunk->offset) {
                throw std::runtime_error(
                    "internal DTS-UHD ACE audio chunk is malformed");
            }
            dtsx::AceFrameHeader ace_header;
            dtsx::AceFrameParserState& chunk_parser_state =
                ace_frame_states_[chunk->index];
            const auto frame_parse_result = dtsx::parse_ace_frame_header(
                    frame.bytes.data() + chunk->offset,
                    chunk->size,
                    chunk_parser_state,
                    ace_header);
            if (frame_parse_result != dtsx::AceFrameParseResult::Complete) {
                std::string prefix;
                const std::size_t preview = (std::min)(
                    static_cast<std::size_t>(8U),
                    static_cast<std::size_t>(chunk->size));
                static constexpr char hex[] = "0123456789abcdef";
                for (std::size_t index = 0U; index < preview; ++index) {
                    const std::uint8_t byte =
                        frame.bytes[chunk->offset + index];
                    prefix.push_back(hex[byte >> 4U]);
                    prefix.push_back(hex[byte & 0x0FU]);
                }
                throw std::runtime_error(
                    "internal DTS-UHD ACE frame header is malformed (chunk "
                    + std::to_string(chunk->index) + ", result "
                    + std::to_string(static_cast<int>(frame_parse_result))
                    + ", offset " + std::to_string(chunk->offset)
                    + ", size " + std::to_string(chunk->size)
                    + ", bytes " + prefix + ")");
            }
            if (first_chunk) {
                main_ace_header = ace_header;
                first_chunk = false;
            }
        // homatic_libHwAudio_dtsx.so updates frame segments before every
        // DecodeSubframe.  A changed ACE stream-set topology therefore cannot
        // reuse the previous ACEW registration/history even when the PCM
        // geometry (rate and duration) is unchanged.  Compare the fields that
        // define the native stream-set registration and reset state exactly at
        // that boundary.
        const auto same_stream_set_topology = [](
            const std::vector<dtsx::AceStreamSetHeader>& lhs,
            const std::vector<dtsx::AceStreamSetHeader>& rhs) noexcept {
            if (lhs.size() != rhs.size()) {
                return false;
            }
            for (std::size_t index = 0U; index < lhs.size(); ++index) {
                const auto& a = lhs[index];
                const auto& b = rhs[index];
                if (a.id != b.id || a.lfe_channels != b.lfe_channels
                    || a.mono_bandwidth_modes != b.mono_bandwidth_modes
                    || a.stereo_bandwidth_modes != b.stereo_bandwidth_modes) {
                    return false;
                }
            }
            return true;
        };
        AceChunkGeometry& previous_geometry =
            ace_chunk_geometries_[chunk->index];
        const bool topology_changed = previous_geometry.initialized
            && !same_stream_set_topology(
                previous_geometry.stream_sets, ace_header.stream_sets);
        if (previous_geometry.initialized
            && (previous_geometry.sample_rate != ace_header.sample_rate
                || previous_geometry.frame_duration != ace_header.frame_duration
                || topology_changed)) {
            // DTSX2_ACEW_SetFrameInitParams/Reset keeps streamset decoder,
            // MDCT, LTS and de-emphasis histories tied to frame geometry.
            // Never carry predictive state across a geometry reinitialization.
            const std::uint64_t chunk_namespace =
                (static_cast<std::uint64_t>(chunk->index & 0x7FFFU)
                 << 48U);
            const auto same_chunk = [chunk_namespace](std::uint64_t key) {
                return (key & 0x7FFF000000000000ULL) == chunk_namespace;
            };
            const auto erase_chunk = [same_chunk](auto& states) {
                for (auto iterator = states.begin(); iterator != states.end();) {
                    if (same_chunk(iterator->first)) {
                        iterator = states.erase(iterator);
                    } else {
                        ++iterator;
                    }
                }
            };
            erase_chunk(ace_stream_states_);
            erase_chunk(ace_mdct_histories_);
            erase_chunk(ace_hole_fill_histories_);
            erase_chunk(ace_lts_histories_);
            erase_chunk(ace_deemphasis_states_);
            erase_chunk(ace_deemphasis_delayed_);
            erase_chunk(ace_thf_stream_histories_);
            erase_chunk(ace_streamset_prng_);
            ace_frame_states_.erase(chunk->index);
        }
        previous_geometry.initialized = true;
        previous_geometry.sample_rate = ace_header.sample_rate;
        previous_geometry.frame_duration = ace_header.frame_duration;
        previous_geometry.stream_sets = ace_header.stream_sets;
        std::uint32_t chunk_channel_ordinal = 0U;
        for (const dtsx::AceStreamPayload& payload
             : ace_header.stream_payloads) {
            if (payload.stream_set_index >= ace_header.stream_sets.size()
                || payload.offset > chunk->size
                || payload.size > chunk->size - payload.offset) {
                throw std::runtime_error(
                    "internal DTS-UHD ACE stream payload is malformed");
            }
            // DTSAceStreamsetDecoder_Decode: dtsAce_PRNGRandUint on the
            // streamset decoder dword, then BandDequant_SetRandomSeed.
            const std::uint64_t streamset_prng_key =
                (static_cast<std::uint64_t>(chunk->index & 0x7FFFU) << 48U)
                | ace_header.stream_sets[payload.stream_set_index].id;
            const std::uint32_t band_dequant_seed =
                dtsx::ace_prng_rand_uint(
                    ace_streamset_prng_[streamset_prng_key]);
            if (payload.type != dtsx::AceStreamType::Lfe) {
                const bool stereo =
                    payload.type == dtsx::AceStreamType::Stereo;
                const std::uint64_t key = ace_payload_state_key(
                    chunk->index,
                    ace_header.stream_sets[payload.stream_set_index].id,
                    payload.stream_index,
                    stereo);
                auto found = ace_stream_states_.find(key);
                auto history = ace_mdct_histories_.find(key);
                if (history == ace_mdct_histories_.end()) {
                    history = ace_mdct_histories_.emplace(
                        key, dtsx::AceMdctHistoryBank{}).first;
                    history->second.reset(stereo ? 2U : 1U);
                }
                const dtsx::AceStreamPrefixState* previous =
                    found != ace_stream_states_.end()
                    ? &found->second : nullptr;
                const std::uint32_t previous_mdct_transform_size =
                    static_cast<std::uint32_t>(
                        history->second.previous_transform_size());
                dtsx::AceStreamPrefixState next = previous != nullptr
                    ? *previous : dtsx::AceStreamPrefixState{};
                dtsx::AceStreamPrefix prefix;
                if (dtsx::parse_ace_stream_prefix(
                        frame.bytes.data() + chunk->offset + payload.offset,
                        payload.size,
                        stereo,
                        ace_header.stream_sets[payload.stream_set_index]
                            .predictive,
                        ace_header.sample_rate,
                        payload.bandwidth_mode,
                        previous,
                        next,
                        prefix) != dtsx::AceStreamParseResult::Complete) {
                    throw std::runtime_error(
                        "internal DTS-UHD ACE stream prefix is malformed");
                }
                if (prefix.num_coded_bands == 0U
                    && prefix.coding_mode == dtsx::AceCodingMode::Off
                    && prefix.effective_channel_count != 0U) {
                    InternalAceBus bus;
                    bus.stream_key = key;
                    bus.audio_chunk_index = chunk->index;
                    bus.object_audio = is_object_audio_chunk(chunk->index);
                    const auto [activity_mask, from_metadata] =
                        activity_mask_for_chunk(chunk->index);
                    bus.stream_set_id = ace_header.stream_sets[
                        payload.stream_set_index].id;
                    bus.stream_index = payload.stream_index;
                    bus.channel_ordinal = chunk_channel_ordinal;
                    bus.coding_mode = static_cast<std::uint32_t>(
                        prefix.coding_mode);
                    bus.effective_channel_count =
                        prefix.effective_channel_count;
                    bus.coded_channel_count = payload.channel_count;
                    bus.physical_mask = activity_mask;
                    bus.object_mask_from_metadata = from_metadata;
                    bus.payload_type = payload.type;
                    bus.bandwidth_mode = payload.bandwidth_mode;
                    bus.stereo = stereo;
                    bus.frame_index = internal_frame_index_;
                    bus.sample_rate = ace_header.sample_rate;
                    bus.frame_duration = ace_header.frame_duration;
                    bus.routing_slots = ace_routing_slots(
                        bus.physical_mask, bus.channel_ordinal, stereo,
                        bus.effective_channel_count,
                        !bus.object_mask_from_metadata);
                    bus.pcm.assign(
                        stereo ? 2U : 1U,
                        std::vector<float>(ace_header.frame_duration, 0.0F));
                    bus.spectral_coefficients.assign(
                        stereo ? 2U : 1U, {});
                    last_internal_ace_buses_.push_back(std::move(bus));
                }
                // Empty mono/stereo service payloads carry no spectral bands
                // and are not an ACE PCM candidate (native skips them too).
                if (prefix.num_coded_bands != 0U) {
                    bool spectral_ok = false;
                    if (stereo) {
                        dtsx::AceStereoSpectralPayload spectral;
                        const auto shf_prev =
                            ace_hole_fill_histories_.find(key);
                        std::uint32_t hole_fill_seed = band_dequant_seed;
                        spectral_ok = dtsx::consume_ace_stereo_spectral_payload(
                            frame.bytes.data() + chunk->offset + payload.offset,
                            payload.size, ace_header.sample_rate, prefix,
                            spectral,
                            prefix.spectral_hole_fill
                                ? &hole_fill_seed : nullptr,
                            shf_prev != ace_hole_fill_histories_.end()
                                ? &shf_prev->second.left : nullptr,
                            shf_prev != ace_hole_fill_histories_.end()
                                ? &shf_prev->second.right : nullptr,
                            shf_prev != ace_hole_fill_histories_.end()
                                ? &shf_prev->second.stereo_coding : nullptr,
                            shf_prev != ace_hole_fill_histories_.end()
                                ? &shf_prev->second.work_left : nullptr,
                            shf_prev != ace_hole_fill_histories_.end()
                                ? &shf_prev->second.work_right : nullptr);
                        if (spectral_ok) {
                            AceHoleFillHistory stored;
                            stored.left.reserve(spectral.bands.size());
                            stored.right.reserve(spectral.bands.size());
                            stored.work_left.reserve(spectral.bands.size());
                            stored.work_right.reserve(spectral.bands.size());
                            stored.stereo_coding.reserve(spectral.bands.size());
                            for (const auto& band : spectral.bands) {
                                stored.left.push_back(band.hole_fill_left);
                                stored.right.push_back(band.hole_fill_right);
                                stored.work_left.push_back(
                                    band.hole_fill_work_left);
                                stored.work_right.push_back(
                                    band.hole_fill_work_right);
                                stored.stereo_coding.push_back(band.coding);
                            }
                            ace_hole_fill_histories_[key] = std::move(stored);
                            std::vector<std::vector<float>> left;
                            std::vector<std::vector<float>> right;
                            left.reserve(spectral.bands.size());
                            right.reserve(spectral.bands.size());
                            for (const auto& band : spectral.bands) {
                                left.push_back(band.left_spectrum);
                                right.push_back(band.right_spectrum);
                            }
                            std::vector<float> left_coefficients;
                            std::vector<float> right_coefficients;
                            const bool left_ok =
                                dtsx::ace_pack_spectral_frame_with_blocks(
                                    ace_header.sample_rate, left,
                                    spectral.band_block_count,
                                    left_coefficients,
                                    spectral.short_transform);
                            const bool right_ok =
                                dtsx::ace_pack_spectral_frame_with_blocks(
                                    ace_header.sample_rate, right,
                                    spectral.band_block_count,
                                    right_coefficients,
                                    spectral.short_transform);
                            const bool packed_ok = left_ok && right_ok;
                            const std::size_t thf_transform =
                                spectral.short_transform
                                ? ace_short_transform_block_size(
                                      spectral.band_block_count)
                                : 1024U;
                            const std::uint32_t thf_shift =
                                ace_thf_transform_shift(thf_transform);
                            if (packed_ok) {
                                std::vector<std::vector<float>> thf_planes{
                                    left_coefficients, right_coefficients};
                                std::uint32_t thf_seed = band_dequant_seed;
                                std::array<std::int32_t, 22> thf_beta{};
                                for (std::size_t band = 0U;
                                     band < spectral.bands.size()
                                     && band < thf_beta.size(); ++band) {
                                    thf_beta[band] = spectral.bands[band]
                                        .mid_side_ratio.beta_q15;
                                }
                                if (dtsx::ace_temporal_hole_fill_process(
                                        thf_planes,
                                        prefix.allocation,
                                        ace_header.sample_rate,
                                        payload.bandwidth_mode,
                                        static_cast<std::uint32_t>(
                                            spectral.bands.size()),
                                        thf_shift,
                                        spectral.short_transform,
                                        prefix.temporal_hole_fill,
                                        ace_thf_stream_histories_[key],
                                        thf_seed,
                                        &prefix.stereo_coding,
                                        &spectral.finalized_lognorm_q10,
                                        prefix.coding_mode,
                                        &thf_beta)
                                    && thf_planes.size() == 2U) {
                                    left_coefficients = std::move(
                                        thf_planes[0]);
                                    right_coefficients = std::move(
                                        thf_planes[1]);
                                }
                            }
                            std::vector<float> left_samples;
                            std::vector<float> right_samples;
                            const std::size_t short_block_size =
                                ace_short_transform_block_size(
                                    spectral.band_block_count);
                            const bool transform_ok = packed_ok
                                && (spectral.short_transform
                                    ? (short_block_size != 0U
                                       && (use_native_mdct_candidate()
                                           ? dtsx::ace_inverse_transform_packed_blocks_native_candidate(
                                                 left_coefficients,
                                                 short_block_size, left_samples)
                                           : dtsx::ace_inverse_transform_packed_blocks(
                                                 left_coefficients,
                                                 short_block_size, left_samples))
                                       && (use_native_mdct_candidate()
                                           ? dtsx::ace_inverse_transform_packed_blocks_native_candidate(
                                                 right_coefficients,
                                                 short_block_size, right_samples)
                                           : dtsx::ace_inverse_transform_packed_blocks(
                                                 right_coefficients,
                                                 short_block_size, right_samples)))
                                    : (use_native_mdct_candidate()
                                       ? (dtsx::ace_inverse_transform_native_candidate(
                                              left_coefficients, left_samples)
                                          && dtsx::ace_inverse_transform_native_candidate(
                                              right_coefficients, right_samples))
                                       : (dtsx::ace_inverse_transform_reference(
                                              left_coefficients, left_samples)
                                          && dtsx::ace_inverse_transform_reference(
                                              right_coefficients, right_samples))));
                            const bool overlap_ok = transform_ok
                                && apply_internal_mdct_overlap(
                                    history->second, 0U, left_samples,
                                    spectral.short_transform
                                        ? short_block_size : 1024U,
                                    previous_mdct_transform_size)
                                && apply_internal_mdct_overlap(
                                    history->second, 1U, right_samples,
                                    spectral.short_transform
                                        ? short_block_size : 1024U,
                                    previous_mdct_transform_size);
                            if (!overlap_ok && left_samples.size() > 512U) {
                                ++internal_long_transform_rejections_;
                            }
                            const bool lts_ok = overlap_ok
                                && apply_internal_lts(
                                    ace_lts_histories_[
                                        stream_channel_state_key(key, 0U)],
                                    left_samples,
                                    prefix.lts_enabled,
                                    prefix.lts_lag,
                                    prefix.lts_filter_index)
                                && apply_internal_lts(
                                    ace_lts_histories_[
                                        stream_channel_state_key(key, 1U)],
                                    right_samples,
                                    prefix.lts_enabled,
                                    prefix.lts_lag,
                                    prefix.lts_filter_index);
                            const bool apply_deemphasis =
                                consume_delayed_deemphasis_flag(
                                    ace_deemphasis_delayed_, key,
                                    ace_header.deemphasis_enabled);
                            const bool deemphasis_ok = lts_ok
                                && (!apply_deemphasis
                                    || (apply_internal_deemphasis(
                                        ace_deemphasis_states_,
                                        stream_channel_state_key(key, 0U),
                                        left_samples)
                                        && apply_internal_deemphasis(
                                            ace_deemphasis_states_,
                                            stream_channel_state_key(key, 1U),
                                            right_samples)));
                            // The Sony x64 float path runs de-emphasis and
                            // then the 2^-15 PCM scale; the saturating left
                            // shift only exists in the ARM32 Q31 build.
                            if (deemphasis_ok) {
                                dtsx::ace_apply_native_pcm_scale(left_samples);
                                dtsx::ace_apply_native_pcm_scale(right_samples);
                                if (left_samples.size()
                                        != ace_header.frame_duration
                                    || right_samples.size()
                                        != ace_header.frame_duration) {
                                    ++internal_synthesis_failures_;
                                } else {
                                    float peak = 0.0F;
                                    for (const auto& channel :
                                         {left_samples, right_samples}) {
                                        for (const float sample : channel) {
                                            peak = (std::max)(peak,
                                                std::fabs(sample));
                                        }
                                    }
                                    InternalAceBus bus;
                                    bus.stream_key = key;
                                    bus.audio_chunk_index = chunk->index;
                                    bus.object_audio =
                                        is_object_audio_chunk(chunk->index);
                                    const auto [activity_mask, from_metadata] =
                                        activity_mask_for_chunk(chunk->index);
                                    bus.stream_set_id = ace_header.stream_sets[
                                    payload.stream_set_index].id;
                                    bus.stream_index = payload.stream_index;
                                    bus.channel_ordinal =
                                        chunk_channel_ordinal;
                                    bus.coding_mode = static_cast<std::uint32_t>(
                                        prefix.coding_mode);
                                    bus.effective_channel_count =
                                        prefix.effective_channel_count;
                                    bus.coded_channel_count = payload.channel_count;
                                    bus.physical_mask = activity_mask;
                                    bus.object_mask_from_metadata =
                                        from_metadata;
                                    bus.payload_type = payload.type;
                                    bus.bandwidth_mode = payload.bandwidth_mode;
                                    bus.stereo = true;
                                    bus.short_transform = spectral.short_transform;
                                    bus.spectral_hole_fill =
                                        prefix.spectral_hole_fill;
                                    bus.temporal_hole_fill =
                                        prefix.temporal_hole_fill;
                                    if (spectral.short_transform) {
                                        ++internal_short_transform_frames_;
                                    }
                                    bus.lts_enabled = prefix.lts_enabled;
                                    bus.lts_lag = prefix.lts_lag;
                                    bus.lts_filter_index = prefix.lts_filter_index;
                                    bus.lts_filter_reused = prefix.lts_filter_reused;
                                    (void)dtsx::ace_lts_filter_coefficients(
                                    prefix.lts_filter_index,
                                    bus.lts_center_q31,
                                    bus.lts_adjacent_q31,
                                bus.lts_outer_q31);
                                    bus.mdct_history_initialized =
                                    history->second.channel_count() != 0U;
                                    bus.previous_mdct_transform_size =
                                        previous_mdct_transform_size;
                                    bus.routing_slots = ace_routing_slots(
                                        bus.physical_mask,
                                        bus.channel_ordinal, true,
                                        bus.effective_channel_count,
                                        !bus.object_mask_from_metadata);
                                    bus.band_block_count = spectral.band_block_count;
                                    bus.bit_allocation = prefix.allocation;
                                    bus.peak = peak;
                                    {
                                        const std::vector<std::vector<float>>
                                            thf_coefficients{
                                                left_coefficients,
                                                right_coefficients};
                                        (void)dtsx::ace_temporal_hole_fill_calc_block_sq_norms(
                                            thf_coefficients,
                                            ace_header.sample_rate,
                                            payload.bandwidth_mode,
                                            static_cast<std::uint32_t>(
                                                spectral.bands.size()),
                                            2U,
                                            dtsx::ace_thf_block_sample_stride(
                                                0U),
                                            dtsx::ace_thf_energy_bin_shift(
                                                spectral.short_transform),
                                            bus.temporal_hole_fill_norms,
                                            bus.temporal_hole_fill_active);
                                    }
                                    bus.spectral_coefficients.emplace_back(
                                        std::move(left_coefficients));
                                    bus.spectral_coefficients.emplace_back(
                                        std::move(right_coefficients));
                                    bus.pcm.emplace_back(std::move(left_samples));
                                    bus.pcm.emplace_back(std::move(right_samples));
                                    last_internal_ace_buses_.push_back(std::move(bus));
                                }
                            }
                            ++internal_synthesis_frames_;
                            if (!left_ok || !right_ok || !transform_ok
                                || !overlap_ok || !lts_ok
                                || !deemphasis_ok) {
                                ++internal_synthesis_failures_;
                            }
                        }
                    } else {
                        dtsx::AceSpectralPayload spectral;
                        const auto shf_prev =
                            ace_hole_fill_histories_.find(key);
                        std::uint32_t hole_fill_seed = band_dequant_seed;
                        spectral_ok = dtsx::consume_ace_mono_spectral_payload(
                            frame.bytes.data() + chunk->offset + payload.offset,
                            payload.size, ace_header.sample_rate, prefix,
                            spectral,
                            prefix.spectral_hole_fill
                                ? &hole_fill_seed : nullptr,
                            shf_prev != ace_hole_fill_histories_.end()
                                ? &shf_prev->second.left : nullptr);
                        if (spectral_ok) {
                            ace_hole_fill_histories_[key] = AceHoleFillHistory{
                                spectral.hole_fill_state, {}};
                            std::vector<float> coefficients;
                            const bool packed_ok =
                                dtsx::ace_pack_spectral_frame_with_blocks(
                                    ace_header.sample_rate,
                                    spectral.spectra,
                                    spectral.band_block_count,
                                    coefficients,
                                    prefix.short_transform);
                            if (packed_ok) {
                                std::vector<std::vector<float>> thf_planes{
                                    coefficients};
                                std::uint32_t thf_seed = band_dequant_seed;
                                const std::size_t thf_transform =
                                    spectral.short_transform
                                    ? ace_short_transform_block_size(
                                          spectral.band_block_count)
                                    : 1024U;
                                const std::uint32_t thf_shift =
                                    ace_thf_transform_shift(thf_transform);
                                if (dtsx::ace_temporal_hole_fill_process(
                                        thf_planes,
                                        prefix.allocation,
                                        ace_header.sample_rate,
                                        payload.bandwidth_mode,
                                        static_cast<std::uint32_t>(
                                            spectral.spectra.size()),
                                        thf_shift,
                                        spectral.short_transform,
                                        prefix.temporal_hole_fill,
                                        ace_thf_stream_histories_[key],
                                        thf_seed,
                                        &prefix.stereo_coding,
                                        &spectral.finalized_lognorm_q10,
                                        prefix.coding_mode)
                                    && !thf_planes.empty()) {
                                    coefficients = std::move(thf_planes[0]);
                                }
                            }
                            std::vector<float> samples;
                            ++internal_synthesis_frames_;
                            const std::size_t short_block_size =
                                ace_short_transform_block_size(
                                    spectral.band_block_count);
                            const bool transform_ok = packed_ok
                                && (spectral.short_transform
                                    ? (short_block_size != 0U
                                       && (use_native_mdct_candidate()
                                           ? dtsx::ace_inverse_transform_packed_blocks_native_candidate(
                                                 coefficients, short_block_size,
                                                 samples)
                                           : dtsx::ace_inverse_transform_packed_blocks(
                                                 coefficients, short_block_size,
                                                 samples)))
                                    : (use_native_mdct_candidate()
                                       ? dtsx::ace_inverse_transform_native_candidate(
                                             coefficients, samples)
                                       : dtsx::ace_inverse_transform_reference(
                                             coefficients, samples)));
                            const bool overlap_ok = transform_ok
                                && apply_internal_mdct_overlap(
                                    history->second, 0U, samples,
                                    spectral.short_transform
                                        ? short_block_size : 1024U,
                                    previous_mdct_transform_size);
                            if (!overlap_ok && samples.size() > 512U) {
                                ++internal_long_transform_rejections_;
                            }
                            const bool lts_ok = overlap_ok
                                && apply_internal_lts(
                                    ace_lts_histories_[
                                        stream_channel_state_key(key, 0U)],
                                    samples,
                                    prefix.lts_enabled,
                                    prefix.lts_lag,
                                    prefix.lts_filter_index);
                            const bool apply_deemphasis =
                                consume_delayed_deemphasis_flag(
                                    ace_deemphasis_delayed_, key,
                                    ace_header.deemphasis_enabled);
                            const bool deemphasis_ok = lts_ok
                                && (!apply_deemphasis
                                    || apply_internal_deemphasis(
                                        ace_deemphasis_states_,
                                        stream_channel_state_key(key, 0U),
                                        samples));
                            // The Sony x64 float path runs de-emphasis and
                            // then the 2^-15 PCM scale; the saturating left
                            // shift only exists in the ARM32 Q31 build.
                            if (deemphasis_ok) {
                                dtsx::ace_apply_native_pcm_scale(samples);
                                if (samples.size() != ace_header.frame_duration) {
                                    ++internal_synthesis_failures_;
                                } else {
                                    float peak = 0.0F;
                                    for (const float sample : samples) {
                                        peak = (std::max)(peak, std::fabs(sample));
                                    }
                                    InternalAceBus bus;
                                    bus.stream_key = key;
                                    bus.audio_chunk_index = chunk->index;
                                    bus.object_audio =
                                        is_object_audio_chunk(chunk->index);
                                    const auto [activity_mask, from_metadata] =
                                        activity_mask_for_chunk(chunk->index);
                                    bus.stream_set_id = ace_header.stream_sets[
                                    payload.stream_set_index].id;
                                    bus.stream_index = payload.stream_index;
                                    bus.channel_ordinal =
                                        chunk_channel_ordinal;
                                    bus.coding_mode = static_cast<std::uint32_t>(
                                        prefix.coding_mode);
                                    bus.effective_channel_count =
                                        prefix.effective_channel_count;
                                    bus.coded_channel_count = payload.channel_count;
                                    bus.physical_mask = activity_mask;
                                    bus.object_mask_from_metadata =
                                        from_metadata;
                                    bus.payload_type = payload.type;
                                    bus.bandwidth_mode = payload.bandwidth_mode;
                                    bus.stereo = false;
                                    bus.short_transform = spectral.short_transform;
                                    bus.spectral_hole_fill =
                                        prefix.spectral_hole_fill;
                                    bus.temporal_hole_fill =
                                        prefix.temporal_hole_fill;
                                    if (spectral.short_transform) {
                                        ++internal_short_transform_frames_;
                                    }
                                    bus.lts_enabled = prefix.lts_enabled;
                                    bus.lts_lag = prefix.lts_lag;
                                    bus.lts_filter_index = prefix.lts_filter_index;
                                    bus.lts_filter_reused = prefix.lts_filter_reused;
                                    (void)dtsx::ace_lts_filter_coefficients(
                                    prefix.lts_filter_index,
                                    bus.lts_center_q31,
                                    bus.lts_adjacent_q31,
                                bus.lts_outer_q31);
                                    bus.mdct_history_initialized =
                                        history->second.channel_count() != 0U;
                                    bus.previous_mdct_transform_size =
                                        previous_mdct_transform_size;
                                    bus.routing_slots = ace_routing_slots(
                                        bus.physical_mask,
                                        bus.channel_ordinal, false,
                                        bus.effective_channel_count,
                                        !bus.object_mask_from_metadata);
                                    bus.band_block_count = spectral.band_block_count;
                                    bus.bit_allocation = prefix.allocation;
                                    bus.peak = peak;
                                    {
                                        const std::vector<std::vector<float>>
                                            thf_coefficients{coefficients};
                                        (void)dtsx::ace_temporal_hole_fill_calc_block_sq_norms(
                                            thf_coefficients,
                                            ace_header.sample_rate,
                                            payload.bandwidth_mode,
                                            static_cast<std::uint32_t>(
                                                spectral.spectra.size()),
                                            1U,
                                            dtsx::ace_thf_block_sample_stride(
                                                0U),
                                            dtsx::ace_thf_energy_bin_shift(
                                                spectral.short_transform),
                                            bus.temporal_hole_fill_norms,
                                            bus.temporal_hole_fill_active);
                                    }
                                    bus.spectral_coefficients.emplace_back(
                                        std::move(coefficients));
                                    bus.pcm.emplace_back(std::move(samples));
                                    last_internal_ace_buses_.push_back(std::move(bus));
                                }
                            } else {
                                ++internal_synthesis_failures_;
                            }
                        }
                    }
                    ++internal_spectral_payloads_;
                    if (!spectral_ok) {
                        ++internal_spectral_failures_;
                    }
                }
                ace_stream_states_[key] = next;
                chunk_channel_ordinal += prefix.effective_channel_count;
                continue;
            }
            dtsx::AceLfeStreamData lfe;
            if (dtsx::parse_ace_lfe_stream(
                    frame.bytes.data() + chunk->offset + payload.offset,
                    payload.size,
                    payload.channel_count,
                    ace_header.frame_duration,
                    ace_header.stream_sets[payload.stream_set_index]
                        .predictive,
                    lfe) != dtsx::AceLfeParseResult::Complete) {
                throw std::runtime_error(
                    "internal DTS-UHD ACE LFE stream is malformed");
            }
            const std::uint64_t key = ace_payload_state_key(
                chunk->index,
                ace_header.stream_sets[payload.stream_set_index].id,
                payload.stream_index,
                false);
            std::vector<std::vector<float>> lfe_pcm;
            if (!dtsx::decode_ace_lfe_stream(
                    lfe,
                    ace_header.frame_duration,
                    ace_lfe_states_[key],
                    lfe_pcm)) {
                throw std::runtime_error(
                    "internal DTS-UHD ACE LFE synthesis failed");
            }
            last_internal_lfe_pcm_.insert(
                last_internal_lfe_pcm_.end(),
                std::make_move_iterator(lfe_pcm.begin()),
                std::make_move_iterator(lfe_pcm.end()));
        }
        }
        assemble_aggregate_temporal_hole_fill_state();
        return main_ace_header;
    }

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
    bool prefer_internal_pcm_ = false;
    dtsx::UhdFrameParserState uhd_state_;
    std::unordered_map<std::uint32_t, dtsx::AceFrameParserState>
        ace_frame_states_;
    std::unordered_map<std::uint64_t, dtsx::AceStreamPrefixState>
        ace_stream_states_;
    std::unordered_map<std::uint64_t, dtsx::AceMdctHistoryBank>
        ace_mdct_histories_;
    std::unordered_map<std::uint64_t, AceHoleFillHistory>
        ace_hole_fill_histories_;
    std::unordered_map<std::uint64_t, dtsx::AceLtsHistory>
        ace_lts_histories_;
    std::unordered_map<std::uint64_t, float> ace_deemphasis_states_;
    std::unordered_map<std::uint64_t, bool> ace_deemphasis_delayed_;
    std::unordered_map<std::uint32_t, AceChunkGeometry>
        ace_chunk_geometries_;
    std::unordered_map<std::uint64_t, dtsx::AceLfeDecoderState>
        ace_lfe_states_;
    std::unordered_map<std::uint32_t, AggregateTemporalHoleFillState>
        aggregate_temporal_hole_fill_states_;
    std::unordered_map<std::uint32_t, dtsx::AceTemporalHoleFillHistory>
        aggregate_temporal_hole_fill_history_;
    std::unordered_map<std::uint64_t, dtsx::AceTemporalHoleFillHistory>
        ace_thf_stream_histories_;
    std::unordered_map<std::uint64_t, std::uint32_t> ace_streamset_prng_;
    std::uint64_t internal_spectral_payloads_ = 0U;
    std::uint64_t internal_spectral_failures_ = 0U;
    std::uint64_t internal_parse_failures_ = 0U;
    std::string internal_last_failure_;
    std::uint64_t internal_bus_frames_ = 0U;
    std::uint64_t internal_frame_index_ = 0U;
    std::uint64_t internal_synthesis_frames_ = 0U;
    std::uint64_t internal_synthesis_failures_ = 0U;
    std::uint64_t internal_short_transform_frames_ = 0U;
    std::uint64_t internal_routed_buses_ = 0U;
    std::uint64_t internal_lfe_checked_ = 0U;
    std::uint64_t internal_lfe_mismatches_ = 0U;
    std::uint64_t internal_pcm_selected_ = 0U;
    std::uint64_t internal_pcm_gate_rejections_ = 0U;
    bool internal_temporal_hole_fill_frame_safe_ = false;
    bool internal_pcm_frame_exact_ = false;
    bool internal_object_audio_present_ = false;
    std::uint64_t internal_temporal_hole_fill_frames_ = 0U;
    std::uint64_t internal_temporal_hole_fill_aggregate_frames_ = 0U;
    std::uint64_t internal_long_transform_rejections_ = 0U;
    std::uint64_t internal_pcm_compared_samples_ = 0U;
    std::uint64_t internal_pcm_mismatch_samples_ = 0U;
    std::uint64_t internal_pcm_audible_samples_ = 0U;
    std::uint64_t internal_pcm_audible_mismatches_ = 0U;
    double internal_pcm_cross_ = 0.0;
    double internal_pcm_self_ = 0.0;
    std::int64_t internal_pcm_max_abs_error_ = 0;
    std::vector<InternalAceBus> last_internal_ace_buses_;
    std::vector<std::vector<float>> last_internal_lfe_pcm_;
    std::set<std::uint32_t> metadata_object_ids_;
    std::vector<dtsx::UhdFrameHeader::MetadataObjectAssociation>
        metadata_object_associations_;
    std::vector<std::uint32_t> unresolved_object_audio_chunk_indices_;
    std::vector<std::uint32_t> active_object_audio_chunk_indices_;
    bool has_3d_object_metadata_ = false;
    std::vector<dtsx::UhdFrameHeader::ThreeDObjectMetadata>
        three_d_object_metadata_;
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
    const std::filesystem::path& output,
    bool write_main_output) {
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
    NativeP2Decoder decoder(
        requested_activity_mask(*layout), options.p2_internal);
    if (options.p2_internal) {
        std::cerr << "P2 backend: internal ACE (DLL fallback enabled)\n";
    }
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
    std::uint64_t decoded_frame_count = 0U;
    std::unordered_map<std::uint64_t, std::unique_ptr<WavWriter>> bus_writers;
    std::unordered_map<std::uint64_t, std::array<std::uint32_t, 10U>> bus_info;
    std::unordered_map<std::uint64_t, bool> bus_temporal_hole_fill;
    std::unordered_map<std::uint64_t, bool> bus_spectral_hole_fill;
    std::ofstream streamset_oracle;
    std::unique_ptr<ObjectStemWriter> object_stem_writer;
    std::uint64_t object_sample_position = 0U;
    const bool export_buses = options.objects_output_directory_explicit;
    if (export_buses) {
        std::error_code directory_error;
        std::filesystem::create_directories(
            options.objects_output_directory, directory_error);
        if (directory_error) {
            throw std::runtime_error(
                "cannot create P2 object output directory: "
                + directory_error.message());
        }
        streamset_oracle.open(
            options.objects_output_directory / L"streamset_pcm.jsonl",
            std::ios::binary | std::ios::trunc);
        if (!streamset_oracle) {
            throw std::runtime_error(
                "cannot write P2 stream-set oracle JSONL");
        }
    }
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
        if (!planar.empty()) {
            decoded_frame_count += planar.front().size();
        }
        std::unordered_map<std::uint64_t, bool> active_buses;
        if (export_buses && !decoder.last_internal_ace_buses().empty()) {
            const std::optional<ChannelLayout> mono_layout =
                find_layout("mono");
            if (!mono_layout) {
                throw std::runtime_error(
                    "internal error: mono layout is unavailable");
            }
            for (const NativeP2Decoder::InternalAceBus& bus :
                 decoder.last_internal_ace_buses()) {
                for (std::size_t channel = 0U;
                     channel < bus.pcm.size(); ++channel) {
                    const std::uint64_t key =
                        (static_cast<std::uint64_t>(bus.audio_chunk_index)
                         << 48U)
                        | (static_cast<std::uint64_t>(bus.stream_set_id) << 32U)
                        ^ (static_cast<std::uint64_t>(bus.stream_index) << 2U)
                        ^ (static_cast<std::uint64_t>(bus.stereo) << 1U)
                        ^ static_cast<std::uint64_t>(channel);
                    active_buses[key] = true;
                    const dtsx::AceWaveformRegistration registration =
                        dtsx::ace_make_waveform_registration(
                            bus.stream_set_id,
                            bus.stream_index,
                            bus.physical_mask,
                            bus.stereo);
                    std::uint32_t object_id = 0xFFFFFFFFU;
                    for (const auto& association :
                         decoder.metadata_object_associations()) {
                        if (association.audio_chunk_index
                                == bus.audio_chunk_index
                            && (bus.physical_mask == 0U
                                || association.channel_activity_mask == 0U
                                || (association.channel_activity_mask
                                    & bus.physical_mask) != 0U)) {
                            object_id = association.object_id;
                            break;
                        }
                    }
                    bus_info[key] = {
                        bus.audio_chunk_index,
                        registration.stream_set_id,
                        registration.stream_index,
                        static_cast<std::uint32_t>(channel),
                        bus.routing_slots[channel],
                        registration.first_channel,
                        registration.channel_count,
                        channel < registration.relabel_flags.size()
                            ? registration.relabel_flags[channel]
                            : ((registration.left_relabel ? 1U : 0U)
                               | (registration.right_relabel ? 2U : 0U)),
                        bus.object_audio ? 1U : 0U,
                        object_id,
                    };
                    bus_temporal_hole_fill[key] = bus.temporal_hole_fill;
                    bus_spectral_hole_fill[key] = bus.spectral_hole_fill;
                    auto found = bus_writers.find(key);
                    if (found == bus_writers.end()) {
                        std::wstring suffix =
                            bus.stereo ? L"_stereo" : L"_mono";
                        suffix += channel == 0U ? L"_L" : L"_R";
                        const auto path = options.objects_output_directory
                            / (L"p2_chunk_"
                               + std::to_wstring(bus.audio_chunk_index)
                               + L"_streamset_"
                               + std::to_wstring(bus.stream_set_id)
                               + L"_stream_"
                               + std::to_wstring(bus.stream_index)
                               + suffix + L".wav");
                        auto writer_ptr = std::make_unique<WavWriter>(
                            path, *mono_layout, sample_rate,
                            options.overwrite, OutputFormat::Wav, false);
                        found = bus_writers.emplace(
                            key, std::move(writer_ptr)).first;
                    }
                    std::vector<std::int32_t> pcm;
                    pcm.reserve(bus.pcm[channel].size());
                    for (const float value : bus.pcm[channel]) {
                        pcm.push_back(ace_float_to_pcm24(value));
                    }
                    found->second->write_mono_24(pcm, pcm.size());
                }
            }
        }
        if (export_buses && !bus_writers.empty() && !planar.empty()) {
            const std::vector<std::int32_t> silence(planar.front().size(), 0);
            for (auto& entry : bus_writers) {
                if (active_buses.find(entry.first) == active_buses.end()) {
                    entry.second->write_mono_24(silence, silence.size());
                }
            }
        }
        if (export_buses && streamset_oracle) {
            for (const auto& entry : active_buses) {
                const auto info = bus_info.find(entry.first);
                if (info == bus_info.end()) {
                    continue;
                }
                const auto& value = info->second;
                const bool stereo = (entry.first & 2U) != 0U;
                streamset_oracle
                    << "{\"accessUnit\":" << access_unit_count
                    << ",\"audioChunkIndex\":" << value[0]
                    << ",\"streamSetId\":" << value[1]
                    << ",\"streamIndex\":" << value[2]
                    << ",\"channel\":" << value[3]
                    << ",\"routingSlot\":" << value[4]
                    << ",\"stereo\":" << (stereo ? "true" : "false")
                    << ",\"objectAudio\":"
                    << (value[8] != 0U ? "true" : "false")
                    << ",\"objectId\":";
                if (value[9] == 0xFFFFFFFFU) {
                    streamset_oracle << "null";
                } else {
                    streamset_oracle << value[9];
                }
                streamset_oracle
                    << ",\"sampleRate\":" << sample_rate
                    << ",\"sampleCount\":"
                    << (planar.empty() ? 0U : planar.front().size())
                    << ",\"pcmFile\":\"p2_chunk_"
                    << value[0] << "_streamset_" << value[1]
                    << "_stream_" << value[2]
                    << (stereo ? "_stereo" : "_mono")
                    << (value[3] == 0U ? "_L.wav" : "_R.wav")
                    << "\"}\n";
            }
            streamset_oracle.flush();
        }
        if (export_buses && sample_rate != 0U) {
            DecodedObjectAudioFrame object_frame =
                decoder.last_object_audio_frame(sample_rate);
            const std::uint32_t object_duration =
                object_frame.samples_per_channel;
            if (object_duration != 0U && !object_frame.objects.empty()) {
                if (object_stem_writer == nullptr) {
                    object_stem_writer = std::make_unique<ObjectStemWriter>(
                        options.objects_output_directory,
                        sample_rate,
                        options.overwrite);
                }
                if (!object_stem_writer->write(
                        object_frame,
                        object_sample_position,
                        object_duration)) {
                    throw std::runtime_error(
                        "cannot write P2 object ACE stems");
                }
                object_sample_position += object_duration;
                // Existing renderer API only; result is not mixed into the
                // DLL-authoritative bed for this access unit.
                ObjectAudioRenderer object_renderer(*layout);
                std::vector<std::vector<std::int32_t>> unused_object_mix;
                (void)object_renderer.render(
                    object_frame, unused_object_mix);
            }
        }
        if (writer == nullptr && write_main_output) {
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
        std::size_t frames_to_write = 0U;
        if (write_main_output && writer != nullptr) {
            const std::uint64_t remaining =
                frame_limit - std::min(
                    frame_limit, writer->frames_written());
            frames_to_write = static_cast<std::size_t>(
                std::min<std::uint64_t>(
                    remaining,
                    planar.empty() ? 0U : planar.front().size()));
            writer->write_planar_24(planar, frames_to_write);
        }
        if (mono_writer != nullptr && frames_to_write != 0U) {
            mono_writer->write_planar_24(planar, frames_to_write);
        }
        encoded_byte_count += frame.bytes.size();
        if (options.duration_seconds != 0U) {
            progress.update(
                "decode P2",
                static_cast<int>(
                    std::min<std::uint64_t>(
                        99U,
                    (writer != nullptr ? writer->frames_written()
                                       : decoded_frame_count) * 100U
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
        if (write_main_output && writer->frames_written() >= frame_limit) {
            break;
        }
    }
    if (write_main_output && writer == nullptr) {
        throw std::runtime_error(
            "P2 decoder produced no PCM frames");
    }
    if (writer != nullptr) {
        writer->close();
    }
    if (mono_writer != nullptr) {
        mono_writer->close();
    }
    for (auto& entry : bus_writers) {
        entry.second->close();
    }
    if (object_stem_writer != nullptr) {
        object_stem_writer->close();
    }
    if (streamset_oracle) {
        streamset_oracle.close();
    }
    if (export_buses && !bus_info.empty()) {
        const auto sidecar = options.objects_output_directory / L"p2-buses.json";
        std::ofstream metadata(sidecar, std::ios::binary | std::ios::trunc);
        if (!metadata) {
            throw std::runtime_error("cannot create P2 bus metadata");
        }
        metadata << "{\"pcmSource\":\"internal-ace-diagnostic\","
                    "\"associationStatus\":\"mde-chunk-object-stems\","
                    "\"has3DObjectMetadata\":"
                 << (decoder.has_3d_object_metadata() ? "true" : "false")
                 << ","
                    "\"sampleRate\":" << sample_rate
                 << ",\"metadataObjectIds\":[";
        bool first_object_id = true;
        for (const std::uint32_t object_id : decoder.metadata_object_ids()) {
            if (!first_object_id) {
                metadata << ',';
            }
            first_object_id = false;
            metadata << object_id;
        }
        metadata << "],\"metadataObjectAssociations\":[";
        bool first_association = true;
        for (const auto& association : decoder.metadata_object_associations()) {
            if (!first_association) {
                metadata << ',';
            }
            first_association = false;
            metadata << "{\"objectId\":" << association.object_id
                     << ",\"representationType\":"
                     << association.representation_type
                     << ",\"audioChunkIndex\":"
                     << association.audio_chunk_index
                     << ",\"navigationIndex\":"
                     << association.navigation_index
                     << ",\"channelLayoutIndex\":"
                     << association.channel_layout_index
                     << ",\"channelActivityMask\":"
                     << association.channel_activity_mask
                     << ",\"registrationChannelCount\":"
                     << association.registration_channel_count
                     << ",\"registrationLayoutMask\":"
                     << association.registration_layout_mask
                     << ",\"registrationDescriptor\":["
                     << association.registration_descriptor[0] << ','
                     << association.registration_descriptor[1] << ','
                     << association.registration_descriptor[2] << "]}";
        }
        metadata << "],\"unresolvedObjectAudioChunkIndices\":[";
        for (std::size_t index = 0U;
             index < decoder.unresolved_object_audio_chunk_indices().size();
             ++index) {
            if (index != 0U) {
                metadata << ',';
            }
            metadata << decoder.unresolved_object_audio_chunk_indices()[index];
        }
        metadata << "],\"threeDObjectMetadata\":[";
        bool first_3d = true;
        for (const auto& object : decoder.three_d_object_metadata()) {
            if (!first_3d) {
                metadata << ',';
            }
            first_3d = false;
            metadata << "{\"objectId\":" << object.object_id
                     << ",\"layoutMask\":" << object.layout_mask
                     << ",\"speakerIndices\":[";
            for (std::size_t index = 0U;
                 index < object.speaker_indices.size(); ++index) {
                if (index != 0U) {
                    metadata << ',';
                }
                metadata << object.speaker_indices[index];
            }
            metadata << "]}";
        }
        metadata << "],\"buses\":[";
        bool first_bus = true;
        for (const auto& entry : bus_info) {
            if (!first_bus) {
                metadata << ',';
            }
            first_bus = false;
            const auto& info = entry.second;
            const std::uint32_t physical_mask = info[4] < 32U
                ? (1U << info[4]) : 0U;
            const bool stereo = (entry.first & 2U) != 0U;
            metadata << "{\"audioChunkIndex\":" << info[0]
                     << ",\"streamSetId\":" << info[1]
                     << ",\"streamIndex\":" << info[2]
                     << ",\"objectAudio\":"
                     << (info[8] != 0U ? "true" : "false")
                     << ",\"objectId\":";
            if (info[9] == 0xFFFFFFFFU) {
                metadata << "null";
            } else {
                metadata << info[9];
            }
            metadata << ",\"objectIds\":[";
            bool first_bus_object_id = true;
            for (const auto& association :
                 decoder.metadata_object_associations()) {
                if (association.audio_chunk_index != info[0]
                    || std::find(
                           decoder.metadata_object_ids().begin(),
                           decoder.metadata_object_ids().end(),
                           association.object_id)
                        == decoder.metadata_object_ids().end()
                    || (info[4] < 32U
                        && association.channel_activity_mask != 0U
                        && (association.channel_activity_mask
                            & (1U << info[4])) == 0U)) {
                    continue;
                }
                if (!first_bus_object_id) {
                    metadata << ',';
                }
                first_bus_object_id = false;
                metadata << association.object_id;
            }
            metadata << ']'
                     << ",\"stereo\":" << (stereo ? "true" : "false")
                     << ",\"channel\":" << info[3]
                     << ",\"physicalSpeakerMask\":" << physical_mask
                     << ",\"registrationFirstChannel\":" << info[5]
                     << ",\"registrationChannelCount\":" << info[6]
                     << ",\"leftRelabel\":"
                     << ((info[7] & 1U) != 0U ? "true" : "false")
                     << ",\"rightRelabel\":"
                     << ((info[7] & 2U) != 0U ? "true" : "false")
                     << ",\"temporalHoleFill\":"
                     << (bus_temporal_hole_fill.find(entry.first)
                                 != bus_temporal_hole_fill.end()
                             && bus_temporal_hole_fill.at(entry.first)
                         ? "true" : "false")
                     << ",\"spectralHoleFill\":"
                     << (bus_spectral_hole_fill.find(entry.first)
                                 != bus_spectral_hole_fill.end()
                             && bus_spectral_hole_fill.at(entry.first)
                         ? "true" : "false")
                     << ",\"file\":\"p2_chunk_"
                     << info[0] << "_streamset_" << info[1]
                     << "_stream_" << info[2]
                     << (stereo ? "_stereo" : "_mono")
                     << (info[3] == 0U ? "_L.wav\"}" : "_R.wav\"}");
        }
        metadata << "]}\n";
    } else if (export_buses) {
        std::cerr << "warning: no internal ACE waveform buses observed\n";
    }
    progress.done("decode P2");
    std::cerr << "Frames: " << (writer != nullptr
        ? writer->frames_written() : 0U) << '\n';
    std::cerr << "P2 access units: " << access_unit_count << '\n';
    if (decoder.internal_parse_failures() != 0U) {
        std::cerr << "Internal ACE parse fallback: "
                  << decoder.internal_parse_failures()
                  << " access units delegated to DLL\n";
        if (!decoder.internal_last_failure().empty()) {
            std::cerr << "Internal ACE first failure: "
                      << decoder.internal_last_failure() << '\n';
        }
    }
    if (decoder.internal_bus_frames() != 0U) {
        std::cerr << "Internal ACE PCM buses: "
                  << decoder.internal_bus_frames()
                  << " access units\n";
    }
    if (decoder.internal_spectral_payloads() != 0U) {
        const std::uint64_t spectral_failures =
            decoder.internal_spectral_failures();
        const std::uint64_t spectral_passed =
            decoder.internal_spectral_payloads() > spectral_failures
            ? decoder.internal_spectral_payloads() - spectral_failures : 0U;
        std::cerr << "Internal ACE spectral gate: "
                  << spectral_passed
                  << '/' << decoder.internal_spectral_payloads()
                  << " payloads passed";
        if (spectral_failures != 0U) {
            std::cerr << " (" << spectral_failures
                      << " pending synthesis/parsing)";
        }
        std::cerr << '\n';
    }
    if (decoder.internal_synthesis_frames() != 0U) {
        const std::uint64_t synthesis_failures =
            decoder.internal_synthesis_failures();
        const std::uint64_t synthesis_passed =
            decoder.internal_synthesis_frames() > synthesis_failures
            ? decoder.internal_synthesis_frames() - synthesis_failures : 0U;
        std::cerr << "Internal ACE synthesis packing: "
                  << synthesis_passed
                  << '/' << decoder.internal_synthesis_frames();
        if (synthesis_failures != 0U) {
            std::cerr << " (" << synthesis_failures
                      << " failed)";
        }
        std::cerr << '\n';
    }
    if (decoder.internal_short_transform_frames() != 0U) {
        std::cerr << "Internal ACE short-transform frames: "
                  << decoder.internal_short_transform_frames() << '\n';
    }
    if (decoder.internal_routed_buses() != 0U) {
        std::cerr << "Internal ACE routed buses: "
                  << decoder.internal_routed_buses() << '\n';
    }
    if (decoder.internal_history_banks() != 0U) {
        std::cerr << "Internal ACE MDCT history banks: "
                  << decoder.internal_history_banks() << '\n';
    }
    if (decoder.internal_lfe_checked() != 0U) {
        const std::uint64_t lfe_mismatches = decoder.internal_lfe_mismatches();
        const std::uint64_t lfe_exact = decoder.internal_lfe_checked()
            > lfe_mismatches ? decoder.internal_lfe_checked() - lfe_mismatches
                             : 0U;
        std::cerr << "Internal ACE LFE oracle: "
                  << lfe_exact
                  << '/' << decoder.internal_lfe_checked()
                  << " frames exact";
        if (lfe_mismatches != 0U) {
            std::cerr << " (" << lfe_mismatches
                      << " mismatch)";
        }
        std::cerr << '\n';
    }
    if (decoder.internal_pcm_selected() != 0U) {
        std::cerr << "Internal ACE PCM selected: "
                  << decoder.internal_pcm_selected() << " frames\n";
    }
    if (decoder.internal_pcm_gate_rejections() != 0U) {
        std::cerr << "Internal ACE PCM gate rejections: "
                  << decoder.internal_pcm_gate_rejections() << " frames\n";
    }
    if (decoder.internal_pcm_compared_samples() != 0U) {
        std::cerr << "Internal ACE PCM differential: "
                  << (decoder.internal_pcm_compared_samples()
                      - decoder.internal_pcm_mismatch_samples())
                  << '/' << decoder.internal_pcm_compared_samples()
                  << " samples exact, max abs error "
                  << decoder.internal_pcm_max_abs_error() << " LSB\n";
        std::cerr << "Internal ACE PCM audible differential: "
                  << (decoder.internal_pcm_audible_samples()
                      - decoder.internal_pcm_audible_mismatches())
                  << '/' << decoder.internal_pcm_audible_samples()
                  << " non-silent samples exact\n";
        std::cerr << "Internal ACE PCM mean waveform correlation: "
                  << decoder.internal_pcm_correlation() << '\n';
    }
    if (decoder.internal_temporal_hole_fill_frames() != 0U) {
        std::cerr << "Internal ACE temporal-hole-fill frames: "
                  << decoder.internal_temporal_hole_fill_frames()
                  << " (DLL PCM retained)\n";
    }
    if (decoder.internal_temporal_hole_fill_aggregate_frames() != 0U) {
        std::cerr << "Internal ACE aggregate THF states: "
                  << decoder.internal_temporal_hole_fill_aggregate_frames()
                  << " (diagnostic; DLL PCM retained)\n";
    }
    if (decoder.internal_long_transform_rejections() != 0U) {
        std::cerr << "Internal ACE long-transform overlap rejected: "
                  << decoder.internal_long_transform_rejections()
                  << " frames\n";
    }
    if (options.verbose) {
        std::cerr << "P2 elementary bytes: "
                  << encoded_byte_count << '\n';
    }
}

} // namespace dtsx_decode
