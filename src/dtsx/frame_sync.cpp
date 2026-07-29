#include "dtsx/frame_sync.hpp"

namespace dtsx {
namespace {

constexpr std::uint32_t kCore16BitBigEndian = 0x7FFE8001U;
constexpr std::uint32_t kCore16BitLittleEndian = 0xFE7F0180U;
constexpr std::uint32_t kCore14BitBigEndian = 0x1FFFE800U;
constexpr std::uint32_t kCore14BitLittleEndian = 0xFF1F00E8U;
constexpr std::uint32_t kExtensionBigEndian = 0x58642520U;
constexpr std::uint32_t kExtensionLittleEndian = 0x64582025U;

} // namespace

std::optional<StreamPacking> classify_sync_word(
    std::uint32_t sync_word) noexcept {
    // libdtsx.so: DTSXDecParser_SAPI_CaptureFrame, 0x44240..0x44610.
    switch (sync_word) {
    case kCore16BitBigEndian:
        return StreamPacking::Core16BitBigEndian;
    case kCore16BitLittleEndian:
        return StreamPacking::Core16BitLittleEndian;
    case kCore14BitBigEndian:
        return StreamPacking::Core14BitBigEndian;
    case kCore14BitLittleEndian:
        return StreamPacking::Core14BitLittleEndian;
    case kExtensionBigEndian:
        return StreamPacking::ExtensionBigEndian;
    case kExtensionLittleEndian:
        return StreamPacking::ExtensionLittleEndian;
    default:
        return std::nullopt;
    }
}

bool is_valid_core_sync(std::uint32_t word,
                        std::uint32_t byte_swapped_word) noexcept {
    // libdtsx.so: DTSFrameScanner_IsValidCoreSync, 0x2ee40.
    return byte_swapped_word == kCore16BitBigEndian
        || byte_swapped_word == kCore16BitLittleEndian
        || word == kCore16BitBigEndian;
}

bool is_valid_core_substream_sync(std::uint32_t word,
                                  std::uint32_t byte_swapped_word) noexcept {
    // libdtsx.so: DTSFrameScanner_IsValidCoreSS, 0x2ef28.
    return byte_swapped_word == kCore14BitBigEndian
        || byte_swapped_word == kCore14BitLittleEndian;
}

FrameSyncScanner::FrameSyncScanner(SyncAlignment alignment) noexcept
    : alignment_(alignment) {}

std::optional<SyncMatch> FrameSyncScanner::push(std::uint8_t byte) noexcept {
    // The native parser shifts the previous value by eight and ORs the next
    // byte before applying its packing-specific sync tests.
    shift_register_ = (shift_register_ << 8U) | byte;
    ++bytes_seen_;
    if (alignment_ == SyncAlignment::FourByte && (bytes_seen_ & 3U) != 0U) {
        return std::nullopt;
    }

    const std::optional<StreamPacking> packing =
        classify_sync_word(shift_register_);
    if (!packing) {
        return std::nullopt;
    }
    return SyncMatch{*packing, bytes_seen_ - 4U, shift_register_};
}

void FrameSyncScanner::reset() noexcept {
    shift_register_ = 0;
    bytes_seen_ = 0;
}

} // namespace dtsx
