#pragma once

#include "app/options.hpp"
#include "../audio/layout.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <vector>

namespace dtsx_decode {

class WavWriter {
public:
    WavWriter(const std::filesystem::path& path,
        const ChannelLayout& layout,
        std::uint32_t sample_rate,
        bool overwrite,
        OutputFormat format = OutputFormat::Wav,
        bool dolby_output = false);
    ~WavWriter();

    WavWriter(const WavWriter&) = delete;
    WavWriter& operator=(const WavWriter&) = delete;

    void write(const void* data, std::size_t size);
    void write_planar_24(
        const std::vector<std::vector<std::int32_t>>& channels);
    void write_planar_24(
        const std::vector<std::vector<std::int32_t>>& channels,
        std::size_t frame_count);
    void write_mono_24(
        const std::vector<std::int32_t>& samples,
        std::size_t frame_count);
    void close();
    std::uint64_t frames_written() const;

private:
    void write_header();
    void finalize_header();
    void finalize_wav_header(std::uint64_t file_size);
    void finalize_wave64_header(std::uint64_t file_size);

    std::filesystem::path path_;
    ChannelLayout layout_;
    std::uint32_t sample_rate_ = 0;
    std::uint16_t block_align_ = 0;
    OutputFormat format_ = OutputFormat::Wav;
    std::vector<std::size_t> channel_order_;
    std::uint64_t data_chunk_start_ = 0U;
    std::uint64_t data_size_offset_ = 0U;
    bool extensible_ = true;
    std::ofstream output_;
    std::uint64_t data_bytes_ = 0;
    bool closed_ = false;
};

class MonoTrackWriter {
public:
    MonoTrackWriter(
        const std::filesystem::path& directory,
        const ChannelLayout& source_layout,
        std::uint32_t sample_rate,
        bool overwrite);

    void write_planar_24(
        const std::vector<std::vector<std::int32_t>>& channels,
        std::size_t frame_count);
    void close();

private:
    std::vector<std::unique_ptr<WavWriter>> writers_;
    bool closed_ = false;
};

} // namespace dtsx_decode
