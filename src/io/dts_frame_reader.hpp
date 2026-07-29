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
    explicit DtsFrameReader(const Options& options);

    [[nodiscard]] bool read(dtsx::ElementaryFrame& frame);

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
