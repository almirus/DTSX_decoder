#pragma once

#include "audio/layout.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace dtsx_decode {

// ARCAM AVRx0 IMAX crossover/bass profile recovered from the TI C6000 firmware.
class ImaxPostProcessor final {
public:
    bool process(
        std::vector<std::vector<std::int32_t>>& channels,
        const ChannelLayout& layout,
        std::uint32_t sample_rate,
        bool imax_enabled,
        const std::string& small_speakers);

    void reset() noexcept;

private:
    struct Biquad final {
        double b0 = 1.0;
        double b1 = 0.0;
        double b2 = 0.0;
        double a1 = 0.0;
        double a2 = 0.0;
        double z1 = 0.0;
        double z2 = 0.0;

        double process(double sample) noexcept;
    };

    std::array<Biquad, 2U> lfe_lp_sections_{};
    std::vector<std::array<Biquad, 2U>> main_hp_sections_;
    std::vector<std::array<Biquad, 2U>> main_lp_sections_;
    std::vector<bool> small_channels_;
    std::uint32_t sample_rate_ = 0U;
    std::vector<std::string> channel_names_;
    std::string small_speakers_;
    std::size_t lfe_channel_ = 0U;
    bool configured_ = false;
};

} // namespace dtsx_decode
