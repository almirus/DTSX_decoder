#include "wav_writer.hpp"

#include "app/app_version.hpp"

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

constexpr std::array<unsigned char, 16> kWave64RiffGuid = {
    'r', 'i', 'f', 'f', 0x2Eu, 0x91u, 0xCFu, 0x11u,
    0xA5u, 0xD6u, 0x28u, 0xDBu, 0x04u, 0xC1u, 0x00u, 0x00u,
};
constexpr std::array<unsigned char, 16> kWave64WaveGuid = {
    'w', 'a', 'v', 'e', 0xF3u, 0xACu, 0xD3u, 0x11u,
    0x8Cu, 0xD1u, 0x00u, 0xC0u, 0x4Fu, 0x8Eu, 0xDBu, 0x8Au,
};
constexpr std::array<unsigned char, 16> kWave64FmtGuid = {
    'f', 'm', 't', ' ', 0xF3u, 0xACu, 0xD3u, 0x11u,
    0x8Cu, 0xD1u, 0x00u, 0xC0u, 0x4Fu, 0x8Eu, 0xDBu, 0x8Au,
};
constexpr std::array<unsigned char, 16> kWave64DataGuid = {
    'd', 'a', 't', 'a', 0xF3u, 0xACu, 0xD3u, 0x11u,
    0x8Cu, 0xD1u, 0x00u, 0xC0u, 0x4Fu, 0x8Eu, 0xDBu, 0x8Au,
};
constexpr std::array<unsigned char, 16> kWave64SummaryListGuid = {
    0xBCu, 0x94u, 0x5Fu, 0x92u, 0x5Au, 0x52u, 0xD2u, 0x11u,
    0x86u, 0xDCu, 0x00u, 0xC0u, 0x4Fu, 0x8Eu, 0xDBu, 0x8Au,
};

void write_guid(
    std::ostream& output,
    const std::array<unsigned char, 16>& guid) {
    output.write(
        reinterpret_cast<const char*>(guid.data()),
        static_cast<std::streamsize>(guid.size()));
}

void write_pcm24_format(
    std::ostream& output,
    const ChannelLayout& layout,
    std::uint32_t sample_rate,
    std::uint16_t block_align,
    bool extensible) {
    write_u16(output, extensible ? 0xFFFEu : 0x0001u);
    write_u16(output, static_cast<std::uint16_t>(layout.channels.size()));
    write_u32(output, sample_rate);
    write_u32(output, sample_rate * block_align);
    write_u16(output, block_align);
    write_u16(output, 24);
    if (!extensible) {
        return;
    }
    write_u16(output, 22);
    write_u16(output, 24);
    write_u32(output, layout.wave_mask);
    write_u32(output, 0x00000001u);
    write_u16(output, 0x0000u);
    write_u16(output, 0x0010u);
    const std::array<unsigned char, 8> pcm_guid_tail = {
        0x80u, 0x00u, 0x00u, 0xAAu, 0x00u, 0x38u, 0x9Bu, 0x71u,
    };
    output.write(
        reinterpret_cast<const char*>(pcm_guid_tail.data()),
        static_cast<std::streamsize>(pcm_guid_tail.size()));
}

void write_list_info_comment(
    std::ostream& output,
    const std::string& comment) {
    if (comment.empty()) {
        return;
    }
    if (comment.size()
        >= std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error("WAV comment is too large");
    }
    const std::uint32_t comment_size =
        static_cast<std::uint32_t>(comment.size() + 1U);
    const std::uint32_t comment_padding = comment_size & 1U;
    const std::uint32_t list_size =
        4U + 8U + comment_size + comment_padding;

    write_fourcc(output, "LIST");
    write_u32(output, list_size);
    write_fourcc(output, "INFO");
    write_fourcc(output, "ICMT");
    write_u32(output, comment_size);
    output.write(
        comment.data(),
        static_cast<std::streamsize>(comment.size()));
    output.put('\0');
    if (comment_padding != 0U) {
        output.put('\0');
    }
    if (!output) {
        throw std::runtime_error(
            "cannot write WAV LIST/INFO comment");
    }
}

void write_wave64_summary_comment(
    std::ostream& output,
    const std::string& comment) {
    if (comment.empty()) {
        return;
    }
    if (comment.size()
        > (std::numeric_limits<std::uint32_t>::max() / 2U) - 1U) {
        throw std::runtime_error("Wave64 comment is too large");
    }

    const std::uint32_t comment_size =
        static_cast<std::uint32_t>((comment.size() + 1U) * 2U);
    const std::uint64_t payload_size = 12U + comment_size;
    const std::uint64_t chunk_size = 24U + payload_size;

    write_guid(output, kWave64SummaryListGuid);
    write_u64(output, chunk_size);
    write_u32(output, 1U);
    write_fourcc(output, "ICMT");
    write_u32(output, comment_size);
    for (const unsigned char character : comment) {
        write_u16(output, character);
    }
    write_u16(output, 0U);

    const std::uint64_t padding =
        (8U - (chunk_size & 7U)) & 7U;
    for (std::uint64_t index = 0U; index < padding; ++index) {
        output.put('\0');
    }
    if (!output) {
        throw std::runtime_error(
            "cannot write Wave64 summary metadata");
    }
}

} // namespace

WavWriter::WavWriter(const std::filesystem::path& path,
    const ChannelLayout& layout,
    std::uint32_t sample_rate,
    bool overwrite,
    OutputFormat format,
    bool dolby_output)
    : path_(path)
    , layout_(layout)
    , sample_rate_(sample_rate)
    , format_(format)
    , extensible_(!dolby_output) {
    if (layout_.channels.empty() || layout_.channels.size() > 0xFFFFu) {
        throw std::runtime_error("invalid WAV channel count");
    }
    const ChannelLayout source_layout = layout_;
    if (dolby_output) {
        layout_ = dolby_ordered_layout(source_layout);
    }
    channel_order_.reserve(layout_.channels.size());
    for (const std::string& channel : layout_.channels) {
        auto source = std::find(
            source_layout.channels.begin(),
            source_layout.channels.end(),
            channel);
        if (source == source_layout.channels.end()
            && channel == "SL") {
            source = std::find(
                source_layout.channels.begin(),
                source_layout.channels.end(),
                "BL");
        } else if (source == source_layout.channels.end()
                   && channel == "SR") {
            source = std::find(
                source_layout.channels.begin(),
                source_layout.channels.end(),
                "BR");
        }
        if (source == source_layout.channels.end()) {
            throw std::runtime_error(
                "cannot map output channel order");
        }
        channel_order_.push_back(
            static_cast<std::size_t>(std::distance(
                source_layout.channels.begin(), source)));
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
        throw std::runtime_error("cannot create PCM output file");
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
    if (format_ == OutputFormat::Wave64) {
        write_guid(output_, kWave64RiffGuid);
        write_u64(output_, 0U);
        write_guid(output_, kWave64WaveGuid);
        write_guid(output_, kWave64FmtGuid);
        write_u64(output_, extensible_ ? 64U : 40U);
        write_pcm24_format(
            output_, layout_, sample_rate_, block_align_, extensible_);
        write_wave64_summary_comment(
            output_, make_decode_comment());
        data_chunk_start_ = static_cast<std::uint64_t>(
            output_.tellp());
        write_guid(output_, kWave64DataGuid);
        data_size_offset_ = static_cast<std::uint64_t>(
            output_.tellp());
        write_u64(output_, 0U);
        if (!output_) {
            throw std::runtime_error("cannot write Wave64 header");
        }
        return;
    }

    write_fourcc(output_, "RIFF");
    write_u32(output_, 0);
    write_fourcc(output_, "WAVE");

    write_fourcc(output_, "JUNK");
    write_u32(output_, 28);
    const std::array<char, 28> reserved{};
    output_.write(reserved.data(), reserved.size());

    write_fourcc(output_, "fmt ");
    write_u32(output_, extensible_ ? 40U : 16U);
    write_pcm24_format(
        output_, layout_, sample_rate_, block_align_, extensible_);

    data_chunk_start_ = static_cast<std::uint64_t>(
        output_.tellp());
    write_fourcc(output_, "data");
    data_size_offset_ = static_cast<std::uint64_t>(
        output_.tellp());
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
            for (const std::size_t source_channel : channel_order_) {
                const auto& channel = channels[source_channel];
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

void WavWriter::write_mono_24(
    const std::vector<std::int32_t>& samples,
    std::size_t frame_count) {
    if (layout_.channels.size() != 1U) {
        throw std::runtime_error(
            "mono PCM can only be written to a one-channel output");
    }
    if (samples.size() < frame_count) {
        throw std::runtime_error(
            "mono PCM is shorter than requested frame count");
    }
    constexpr std::size_t kBlockFrames = 4096U;
    std::vector<unsigned char> packed;
    packed.reserve(kBlockFrames * 3U);
    for (std::size_t frame_offset = 0U;
         frame_offset < frame_count;
         frame_offset += kBlockFrames) {
        const std::size_t end =
            std::min(frame_count, frame_offset + kBlockFrames);
        packed.clear();
        for (std::size_t frame = frame_offset; frame < end; ++frame) {
            const std::int32_t limited =
                std::max<std::int32_t>(
                    -0x800000,
                    std::min<std::int32_t>(
                        0x7FFFFF, samples[frame]));
            const std::uint32_t sample =
                static_cast<std::uint32_t>(limited);
            packed.push_back(
                static_cast<unsigned char>(sample & 0xFFU));
            packed.push_back(
                static_cast<unsigned char>((sample >> 8U) & 0xFFU));
            packed.push_back(
                static_cast<unsigned char>((sample >> 16U) & 0xFFU));
        }
        write(packed.data(), packed.size());
    }
}

void WavWriter::finalize_header() {
    const std::streampos file_end_position = output_.tellp();
    if (file_end_position < 0) {
        throw std::runtime_error("cannot determine final WAV size");
    }
    const std::uint64_t file_size =
        static_cast<std::uint64_t>(file_end_position);
    if (format_ == OutputFormat::Wave64) {
        finalize_wave64_header(file_size);
    } else {
        finalize_wav_header(file_size);
    }
}

void WavWriter::finalize_wav_header(std::uint64_t file_size) {
    if (data_size_offset_ == 0U
        || file_size < data_size_offset_ + 4U) {
        throw std::runtime_error("invalid final WAV size");
    }
    const std::uint64_t riff_size = file_size - 8U;
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

    output_.seekp(
        static_cast<std::streamoff>(data_size_offset_),
        std::ios::beg);
    write_u32(output_, rf64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(data_bytes_));
    if (!output_) {
        throw std::runtime_error("cannot finalize WAV header");
    }
}

void WavWriter::finalize_wave64_header(std::uint64_t file_size) {
    constexpr std::uint64_t kRiffSizeOffset = 16U;
    if (data_chunk_start_ == 0U
        || data_size_offset_ == 0U
        || file_size < data_size_offset_ + 8U) {
        throw std::runtime_error("invalid final Wave64 size");
    }
    output_.seekp(
        static_cast<std::streamoff>(kRiffSizeOffset),
        std::ios::beg);
    write_u64(output_, file_size);
    output_.seekp(
        static_cast<std::streamoff>(data_size_offset_),
        std::ios::beg);
    write_u64(output_, file_size - data_chunk_start_);
    if (!output_) {
        throw std::runtime_error("cannot finalize Wave64 header");
    }
}

void WavWriter::close() {
    if (closed_) {
        return;
    }
    if (data_bytes_ % block_align_ != 0) {
        throw std::runtime_error("PCM byte count is not aligned to complete frames");
    }
    if (format_ == OutputFormat::Wave64) {
        const std::uint64_t padding =
            (8U - (data_bytes_ & 7U)) & 7U;
        for (std::uint64_t index = 0U; index < padding; ++index) {
            output_.put('\0');
        }
        if (!output_) {
            throw std::runtime_error(
                "cannot write Wave64 alignment bytes");
        }
    } else {
        if ((data_bytes_ & 1U) != 0U) {
            output_.put('\0');
            if (!output_) {
                throw std::runtime_error(
                    "cannot write WAV alignment byte");
            }
        }
        write_list_info_comment(output_, make_decode_comment());
    }
    finalize_header();
    output_.close();
    closed_ = true;
}

std::uint64_t WavWriter::frames_written() const {
    return block_align_ == 0 ? 0 : data_bytes_ / block_align_;
}

MonoTrackWriter::MonoTrackWriter(
    const std::filesystem::path& directory,
    const ChannelLayout& source_layout,
    std::uint32_t sample_rate,
    bool overwrite) {
    if (directory.empty() || source_layout.channels.empty()) {
        throw std::runtime_error("invalid mono-track output settings");
    }
    std::vector<std::filesystem::path> paths;
    paths.reserve(source_layout.channels.size());
    for (const std::string& channel : source_layout.channels) {
        const std::wstring name(channel.begin(), channel.end());
        paths.push_back(directory / (name + L".wav"));
    }
    if (!overwrite) {
        for (const std::filesystem::path& path : paths) {
            if (std::filesystem::exists(path)) {
                throw std::runtime_error(
                    "mono track already exists; use --overwrite");
            }
        }
    }
    std::error_code directory_error;
    std::filesystem::create_directories(
        directory, directory_error);
    if (directory_error) {
        throw std::runtime_error(
            "cannot create mono-track directory: "
            + directory_error.message());
    }

    writers_.reserve(source_layout.channels.size());
    for (std::size_t channel = 0U;
         channel < source_layout.channels.size();
         ++channel) {
        const std::uint32_t speaker_wave_mask =
            wave_mask_for_channel(source_layout.channels[channel]);
        if (speaker_wave_mask == 0U) {
            throw std::runtime_error(
                "cannot determine mono-track speaker mask");
        }
        ChannelLayout mono;
        mono.name = source_layout.channels[channel];
        mono.channels.push_back(source_layout.channels[channel]);
        mono.wave_mask = speaker_wave_mask;
        writers_.push_back(std::make_unique<WavWriter>(
            paths[channel],
            mono,
            sample_rate,
            overwrite,
            OutputFormat::Wav));
    }
}

void MonoTrackWriter::write_planar_24(
    const std::vector<std::vector<std::int32_t>>& channels,
    std::size_t frame_count) {
    if (closed_) {
        throw std::runtime_error("mono-track writer is closed");
    }
    if (channels.size() != writers_.size()) {
        throw std::runtime_error(
            "PCM channel count does not match mono-track writers");
    }
    for (std::size_t channel = 0U;
         channel < channels.size();
         ++channel) {
        writers_[channel]->write_mono_24(
            channels[channel], frame_count);
    }
}

void MonoTrackWriter::close() {
    if (closed_) {
        return;
    }
    for (const auto& writer : writers_) {
        writer->close();
    }
    closed_ = true;
}

} // namespace dtsx_decode
