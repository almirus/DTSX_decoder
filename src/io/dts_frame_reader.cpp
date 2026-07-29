#include "io/dts_frame_reader.hpp"

#include <algorithm>
#include <cwctype>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dtsx_decode {
namespace {

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

} // namespace

DtsFrameReader::DtsFrameReader(const Options& options)
    : assembler_(dtsx::SyncAlignment::AnyByte) {
    if (is_elementary_dts(options.input)) {
        elementary_stream_.open(options.input, std::ios::binary);
        if (!elementary_stream_) {
            throw std::runtime_error("cannot open DTS elementary stream");
        }
    } else {
        demuxer_ = std::make_unique<FfmpegDtsReader>(options);
    }
}

bool DtsFrameReader::read(dtsx::ElementaryFrame& frame) {
    while (true) {
        while (input_position_ < input_size_) {
            const std::optional<dtsx::ElementaryFrame> completed =
                assembler_.push(input_[input_position_++]);
            if (completed) {
                frame = std::move(*completed);
                saw_frame_ = true;
                return true;
            }
        }

        if (finished_) {
            return false;
        }
        if (demuxer_ != nullptr) {
            input_size_ = demuxer_->read(input_.data(), input_.size());
        } else {
            elementary_stream_.read(
                reinterpret_cast<char*>(input_.data()),
                static_cast<std::streamsize>(input_.size()));
            input_size_ =
                static_cast<std::size_t>(elementary_stream_.gcount());
            if (elementary_stream_.bad()) {
                throw std::runtime_error(
                    "failed to read DTS elementary stream");
            }
        }
        input_position_ = 0;
        if (input_size_ == 0U) {
            if (demuxer_ != nullptr) {
                demuxer_->finish();
            }
            finished_ = true;
            if (saw_input_bytes_ && !saw_frame_) {
                throw std::runtime_error(
                    "input contains no DTS elementary frames; input may be "
                    "DTS-UHD/P2 or an unsupported MP4 payload");
            }
            if (assembler_.has_partial_frame()) {
                throw std::runtime_error(
                    "truncated DTS elementary frame at end of stream");
            }
        } else {
            saw_input_bytes_ = true;
        }
    }
}

} // namespace dtsx_decode
