#pragma once

#include <cstdint>
#include <optional>

namespace dtsx {

enum class StreamPacking {
    Core16BitBigEndian,
    Core16BitLittleEndian,
    Core14BitBigEndian,
    Core14BitLittleEndian,
    ExtensionBigEndian,
    ExtensionLittleEndian,
};

enum class SyncAlignment {
    FourByte,
    AnyByte,
};

struct SyncMatch final {
    StreamPacking packing;
    std::uint64_t byte_offset = 0;
    std::uint32_t sync_word = 0;
};

class FrameSyncScanner final {
public:
    explicit FrameSyncScanner(
        SyncAlignment alignment = SyncAlignment::FourByte) noexcept;

    [[nodiscard]] std::optional<SyncMatch> push(std::uint8_t byte) noexcept;
    void reset() noexcept;

private:
    SyncAlignment alignment_;
    std::uint32_t shift_register_ = 0;
    std::uint64_t bytes_seen_ = 0;
};

[[nodiscard]] std::optional<StreamPacking> classify_sync_word(
    std::uint32_t sync_word) noexcept;

[[nodiscard]] bool is_valid_core_sync(std::uint32_t word,
                                      std::uint32_t byte_swapped_word) noexcept;

[[nodiscard]] bool is_valid_core_substream_sync(
    std::uint32_t word, std::uint32_t byte_swapped_word) noexcept;

} // namespace dtsx
