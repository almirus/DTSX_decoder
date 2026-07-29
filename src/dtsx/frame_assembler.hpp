#pragma once

#include "dtsx/frame_sync.hpp"
#include "dtsx/uhd_frame.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace dtsx {

struct ElementaryFrame final {
    StreamPacking packing = StreamPacking::Core16BitBigEndian;
    std::uint64_t stream_offset = 0;
    std::vector<std::uint8_t> bytes;
};

class FrameAssembler final {
public:
    explicit FrameAssembler(
        SyncAlignment alignment = SyncAlignment::AnyByte);

    [[nodiscard]] std::optional<ElementaryFrame> push(std::uint8_t byte);
    [[nodiscard]] bool has_partial_frame() const noexcept;
    void reset() noexcept;

private:
    [[nodiscard]] bool header_complete() const noexcept;
    [[nodiscard]] bool set_frame_size_from_header() noexcept;
    void start_frame(const SyncMatch& match);
    void clear_capture() noexcept;

    FrameSyncScanner scanner_;
    std::optional<StreamPacking> packing_;
    std::uint64_t stream_bytes_seen_ = 0;
    std::uint64_t frame_offset_ = 0;
    std::uint32_t expected_size_ = 0;
    std::vector<std::uint8_t> bytes_;
    UhdFrameParserState uhd_state_;
};

} // namespace dtsx
