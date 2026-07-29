#include "dtsx/frame_assembler.hpp"

#include "dtsx/frame_header.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

namespace dtsx {
namespace {

constexpr std::uint32_t kNativeCaptureCapacity = 0x8004U;

bool is_extension(StreamPacking packing) noexcept {
    return packing == StreamPacking::ExtensionBigEndian
        || packing == StreamPacking::ExtensionLittleEndian;
}

bool is_14bit_core(StreamPacking packing) noexcept {
    return packing == StreamPacking::Core14BitBigEndian
        || packing == StreamPacking::Core14BitLittleEndian;
}

} // namespace

FrameAssembler::FrameAssembler(SyncAlignment alignment) : scanner_(alignment) {
    bytes_.reserve(kNativeCaptureCapacity);
}

std::optional<ElementaryFrame> FrameAssembler::push(std::uint8_t byte) {
    ++stream_bytes_seen_;
    if (!packing_) {
        const std::optional<SyncMatch> match = scanner_.push(byte);
        if (match) {
            start_frame(*match);
        }
        return std::nullopt;
    }

    bytes_.push_back(byte);
    if (expected_size_ == 0U && header_complete()
        && !set_frame_size_from_header()) {
        clear_capture();
        return std::nullopt;
    }
    if (expected_size_ == 0U || bytes_.size() < expected_size_) {
        return std::nullopt;
    }

    ElementaryFrame result;
    result.packing = *packing_;
    result.stream_offset = frame_offset_;
    result.bytes = std::move(bytes_);
    clear_capture();
    return result;
}

bool FrameAssembler::has_partial_frame() const noexcept {
    return packing_.has_value();
}

void FrameAssembler::reset() noexcept {
    clear_capture();
    stream_bytes_seen_ = 0;
}

void FrameAssembler::clear_capture() noexcept {
    scanner_.reset();
    packing_.reset();
    frame_offset_ = 0;
    expected_size_ = 0;
    bytes_.clear();
}

bool FrameAssembler::header_complete() const noexcept {
    if (!packing_) {
        return false;
    }
    const std::size_t header_size = is_extension(*packing_)
        ? 12U
        : (is_14bit_core(*packing_) ? 10U : 8U);
    return bytes_.size() >= header_size;
}

bool FrameAssembler::set_frame_size_from_header() noexcept {
    if (!packing_) {
        return false;
    }
    if (is_extension(*packing_)) {
        std::array<std::uint8_t, 12> header{};
        std::copy_n(bytes_.begin(), header.size(), header.begin());
        const ExtensionFrameSizes sizes = unpack_extension_frame_sizes(header);
        if (sizes.header_size <= 10U || sizes.frame_size <= 10U
            || (sizes.frame_size & 3U) != 0U) {
            return false;
        }
        expected_size_ = sizes.frame_size;
    } else if (is_14bit_core(*packing_)) {
        std::array<std::uint8_t, 10> header{};
        std::copy_n(bytes_.begin(), header.size(), header.begin());
        expected_size_ = unpack_core_14bit_frame_size(
            header,
            *packing_ == StreamPacking::Core14BitLittleEndian);
        if (expected_size_ < header.size()) {
            return false;
        }
    } else {
        std::array<std::uint8_t, 8> header{};
        std::copy_n(bytes_.begin(), header.size(), header.begin());
        expected_size_ = unpack_core_frame_size(header);
        if (expected_size_ < header.size()) {
            return false;
        }
    }
    return expected_size_ <= kNativeCaptureCapacity;
}

void FrameAssembler::start_frame(const SyncMatch& match) {
    packing_ = match.packing;
    frame_offset_ = stream_bytes_seen_ - 4U;
    expected_size_ = 0;
    bytes_.clear();
    bytes_.push_back(static_cast<std::uint8_t>(match.sync_word >> 24U));
    bytes_.push_back(static_cast<std::uint8_t>(match.sync_word >> 16U));
    bytes_.push_back(static_cast<std::uint8_t>(match.sync_word >> 8U));
    bytes_.push_back(static_cast<std::uint8_t>(match.sync_word));
}

} // namespace dtsx
