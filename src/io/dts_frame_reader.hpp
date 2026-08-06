#pragma once

#include "app/options.hpp"
#include "dtsx/frame_assembler.hpp"
#include "io/ffmpeg.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>

namespace dtsx_decode {

class DtsFrameReader final {
public:
    explicit DtsFrameReader(
        const Options& options,
        std::uint64_t elementary_byte_offset = 0U,
        std::uint64_t container_start_milliseconds = 0U,
        std::uint64_t container_duration_milliseconds = 0U);

    [[nodiscard]] bool read(dtsx::ElementaryFrame& frame);
    void finish();
    [[nodiscard]] bool container_input() const noexcept {
        return demuxer_ != nullptr;
    }

private:
    std::unique_ptr<FfmpegDtsReader> demuxer_;
    std::ifstream elementary_stream_;
    dtsx::FrameAssembler assembler_;
    std::array<std::uint8_t, 256U * 1024U> input_{};
    std::size_t input_size_ = 0;
    std::size_t input_position_ = 0;
    bool finished_ = false;
    bool saw_input_bytes_ = false;
    bool saw_frame_ = false;
};

} // namespace dtsx_decode
