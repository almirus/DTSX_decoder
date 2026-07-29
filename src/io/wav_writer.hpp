#pragma once

#include "../audio/layout.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

namespace dtsx_decode {

class WavWriter {
public:
    WavWriter(const std::filesystem::path& path,
        const ChannelLayout& layout,
        std::uint32_t sample_rate,
        bool overwrite);
    ~WavWriter();

    WavWriter(const WavWriter&) = delete;
    WavWriter& operator=(const WavWriter&) = delete;

    void write(const void* data, std::size_t size);
    void write_planar_24(
        const std::vector<std::vector<std::int32_t>>& channels);
    void write_planar_24(
        const std::vector<std::vector<std::int32_t>>& channels,
        std::size_t frame_count);
    void close();
    std::uint64_t frames_written() const;

private:
    void write_header();
    void finalize_header();

    std::filesystem::path path_;
    ChannelLayout layout_;
    std::uint32_t sample_rate_ = 0;
    std::uint16_t block_align_ = 0;
    std::ofstream output_;
    std::uint64_t data_bytes_ = 0;
    bool closed_ = false;
};

} // namespace dtsx_decode
