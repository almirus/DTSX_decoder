#pragma once

#include "audio/layout.hpp"
#include "render/parma_filterbank.hpp"
#include "render/parma_pairwise_analysis.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx_decode {

class ParmaBlindRenderer final {
public:
    [[nodiscard]] bool render(
        const std::vector<std::vector<std::int32_t>>& input,
        std::uint32_t input_speaker_activity_mask,
        const ChannelLayout& output_layout,
        std::uint32_t sample_rate,
        std::vector<std::vector<std::int32_t>>& output) noexcept;

private:
    [[nodiscard]] bool initialize(std::uint32_t sample_rate) noexcept;

    std::array<ParmaAnalysisFilterBank, 28U> analysis_filters_{};
    std::array<ParmaSynthesisFilterBank, 28U> synthesis_filters_{};
    ParmaPairwiseAnalysis left_analysis_;
    ParmaPairwiseAnalysis right_analysis_;
    std::array<std::array<float, 16U>, 4U>
        intermediate_gain_{};
    std::array<float, 16U> output_gain_{};
    std::array<std::array<std::int32_t, 960U>, 7U>
        bed_delay_{};
    std::size_t bed_delay_position_ = 0U;
    std::array<float, 960U> lfe_delay_{};
    std::size_t lfe_position_ = 0U;
    std::uint32_t sample_rate_ = 0U;
    float energy_attack_ = 0.0F;
    float energy_release_ = 0.0F;
    bool initialized_ = false;
};

} // namespace dtsx_decode
