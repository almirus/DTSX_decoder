#include "wav_writer.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace dtsx_decode {
namespace {

void write_fourcc(std::ostream& output, const char* value) {
    output.write(value, 4);
}

void write_u16(std::ostream& output, std::uint16_t value) {
    const std::array<char, 2> bytes = {
        static_cast<char>(value & 0xFFu),
        static_cast<char>((value >> 8u) & 0xFFu),
    };
    output.write(bytes.data(), bytes.size());
}

void write_u32(std::ostream& output, std::uint32_t value) {
    const std::array<char, 4> bytes = {
        static_cast<char>(value & 0xFFu),
        static_cast<char>((value >> 8u) & 0xFFu),
        static_cast<char>((value >> 16u) & 0xFFu),
        static_cast<char>((value >> 24u) & 0xFFu),
    };
    output.write(bytes.data(), bytes.size());
}

void write_u64(std::ostream& output, std::uint64_t value) {
    write_u32(output, static_cast<std::uint32_t>(value & 0xFFFFFFFFull));
    write_u32(output, static_cast<std::uint32_t>(value >> 32u));
}

} // namespace

WavWriter::WavWriter(const std::filesystem::path& path,
    const ChannelLayout& layout,
    std::uint32_t sample_rate,
    bool overwrite)
    : path_(path)
    , layout_(layout)
    , sample_rate_(sample_rate) {
    if (layout_.channels.empty() || layout_.channels.size() > 0xFFFFu) {
        throw std::runtime_error("invalid WAV channel count");
    }
    if (sample_rate_ == 0) {
        throw std::runtime_error("invalid WAV sample rate");
    }
    if (std::filesystem::exists(path_) && !overwrite) {
        throw std::runtime_error("output already exists; use --overwrite");
    }
    block_align_ = static_cast<std::uint16_t>(layout_.channels.size() * 3u);
    output_.open(path_, std::ios::binary | std::ios::trunc);
    if (!output_) {
        throw std::runtime_error("cannot create output WAV");
    }
    write_header();
}

WavWriter::~WavWriter() {
    if (!closed_) {
        try {
            close();
        } catch (const std::exception&) {
        }
    }
}

void WavWriter::write_header() {
    write_fourcc(output_, "RIFF");
    write_u32(output_, 0);
    write_fourcc(output_, "WAVE");

    write_fourcc(output_, "JUNK");
    write_u32(output_, 28);
    const std::array<char, 28> reserved{};
    output_.write(reserved.data(), reserved.size());

    write_fourcc(output_, "fmt ");
    write_u32(output_, 40);
    write_u16(output_, 0xFFFEu);
    write_u16(output_, static_cast<std::uint16_t>(layout_.channels.size()));
    write_u32(output_, sample_rate_);
    write_u32(output_, sample_rate_ * block_align_);
    write_u16(output_, block_align_);
    write_u16(output_, 24);
    write_u16(output_, 22);
    write_u16(output_, 24);
    write_u32(output_, layout_.wave_mask);
    write_u32(output_, 0x00000001u);
    write_u16(output_, 0x0000u);
    write_u16(output_, 0x0010u);
    const std::array<unsigned char, 8> pcm_guid_tail = {
        0x80u, 0x00u, 0x00u, 0xAAu, 0x00u, 0x38u, 0x9Bu, 0x71u,
    };
    output_.write(reinterpret_cast<const char*>(pcm_guid_tail.data()), pcm_guid_tail.size());

    write_fourcc(output_, "data");
    write_u32(output_, 0);
    if (!output_) {
        throw std::runtime_error("cannot write WAV header");
    }
}

void WavWriter::write(const void* data, std::size_t size) {
    if (closed_) {
        throw std::runtime_error("WAV writer is closed");
    }
    if (size == 0) {
        return;
    }
    output_.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
    if (!output_) {
        throw std::runtime_error("cannot write WAV samples");
    }
    data_bytes_ += size;
}

void WavWriter::write_planar_24(
    const std::vector<std::vector<std::int32_t>>& channels) {
    write_planar_24(
        channels,
        channels.empty() ? 0U : channels.front().size());
}

void WavWriter::write_planar_24(
    const std::vector<std::vector<std::int32_t>>& channels,
    std::size_t frame_count) {
    if (channels.size() != layout_.channels.size()) {
        throw std::runtime_error(
            "planar PCM channel count does not match WAV layout");
    }
    if (channels.empty()) {
        return;
    }
    for (const auto& channel : channels) {
        if (channel.size() < frame_count) {
            throw std::runtime_error(
                "planar PCM channel is shorter than requested frame count");
        }
    }

    constexpr std::size_t kBlockFrames = 4096U;
    std::vector<unsigned char> interleaved;
    interleaved.reserve(
        kBlockFrames * channels.size() * 3U);
    for (std::size_t frame_offset = 0;
         frame_offset < frame_count;
         frame_offset += kBlockFrames) {
        const std::size_t end =
            std::min(frame_count, frame_offset + kBlockFrames);
        interleaved.clear();
        for (std::size_t frame = frame_offset; frame < end; ++frame) {
            for (const auto& channel : channels) {
                const std::int32_t limited =
                    std::max<std::int32_t>(
                        -0x800000,
                        std::min<std::int32_t>(
                            0x7FFFFF, channel[frame]));
                const std::uint32_t sample =
                    static_cast<std::uint32_t>(limited);
                interleaved.push_back(
                    static_cast<unsigned char>(sample & 0xFFU));
                interleaved.push_back(
                    static_cast<unsigned char>((sample >> 8U) & 0xFFU));
                interleaved.push_back(
                    static_cast<unsigned char>((sample >> 16U) & 0xFFU));
            }
        }
        write(interleaved.data(), interleaved.size());
    }
}

void WavWriter::finalize_header() {
    constexpr std::uint64_t kHeaderSize = 104;
    constexpr std::uint64_t kDataSizeOffset = 100;
    const std::uint64_t padded_data_size = data_bytes_ + (data_bytes_ & 1u);
    const std::uint64_t riff_size = kHeaderSize + padded_data_size - 8u;
    const bool rf64 = data_bytes_ > std::numeric_limits<std::uint32_t>::max()
        || riff_size > std::numeric_limits<std::uint32_t>::max();

    output_.seekp(0, std::ios::beg);
    write_fourcc(output_, rf64 ? "RF64" : "RIFF");
    write_u32(output_, rf64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(riff_size));

    output_.seekp(12, std::ios::beg);
    write_fourcc(output_, rf64 ? "ds64" : "JUNK");
    write_u32(output_, 28);
    if (rf64) {
        write_u64(output_, riff_size);
        write_u64(output_, data_bytes_);
        write_u64(output_, frames_written());
        write_u32(output_, 0);
    } else {
        const std::array<char, 28> reserved{};
        output_.write(reserved.data(), reserved.size());
    }

    output_.seekp(static_cast<std::streamoff>(kDataSizeOffset), std::ios::beg);
    write_u32(output_, rf64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(data_bytes_));
    if (!output_) {
        throw std::runtime_error("cannot finalize WAV header");
    }
}

void WavWriter::close() {
    if (closed_) {
        return;
    }
    if (data_bytes_ % block_align_ != 0) {
        throw std::runtime_error("PCM byte count is not aligned to complete frames");
    }
    if ((data_bytes_ & 1u) != 0u) {
        output_.put('\0');
        if (!output_) {
            throw std::runtime_error("cannot write WAV alignment byte");
        }
    }
    finalize_header();
    output_.close();
    closed_ = true;
}

std::uint64_t WavWriter::frames_written() const {
    return block_align_ == 0 ? 0 : data_bytes_ / block_align_;
}

} // namespace dtsx_decode
