#pragma once

#include "audio/layout.hpp"
#include "render/parma_filterbank.hpp"
#include "render/parma_guided_controls.hpp"
#include "render/parma_pairwise_analysis.hpp"
#include "render/parma_quadruplet.hpp"
#include "render/parma_triplet.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace dtsx_decode {

// Native DTS_ParmaDec_SetGuided execution path.  It intentionally owns its
// state: SetGuided controls may change per frame, while the OSFB and spatial
// analysis state must survive those changes until Flush.
class ParmaGuidedRenderer final {
public:
    [[nodiscard]] bool render(
        const std::vector<std::vector<std::int32_t>>& input,
        std::uint32_t input_speaker_activity_mask,
        const ChannelLayout& output_layout,
        const ParmaGuidedControls& controls,
        std::uint32_t sample_rate,
        std::vector<std::vector<std::int32_t>>& output) noexcept;
    void flush() noexcept;

#ifdef DTSX_PARMA_TESTING
    [[nodiscard]] const std::array<float, 16U>& testing_output_gain() const noexcept {
        return output_gain_;
    }
    [[nodiscard]] const std::array<float, 64U>& testing_gain_trace() const noexcept {
        return gain_trace_;
    }
    [[nodiscard]] std::size_t testing_gain_trace_count() const noexcept {
        return gain_trace_count_;
    }
    [[nodiscard]] const ParmaPairwiseAnalysis&
    testing_pair_analysis(std::size_t index) const noexcept {
        return pair_analysis_[index];
    }
#endif

private:
    [[nodiscard]] bool initialize(std::uint32_t sample_rate) noexcept;

    std::array<ParmaAnalysisFilterBank, 28U> analysis_filters_{};
    std::array<ParmaSynthesisFilterBank, 28U> synthesis_filters_{};
    std::array<std::array<float, 64U>, 28U> analysis_delay_{};
    std::array<ParmaPairwiseAnalysis, 11U> pair_analysis_{};
    std::array<ParmaTripletAnalysis, 11U> triplet_analysis_{};
    std::array<ParmaQuadrupletAnalysis, 11U> quadruplet_analysis_{};
    std::array<std::array<float, 1024U>, 28U> direct_delay_{};
    std::array<float, 1024U> lfe_delay_{};
    std::array<float, 16U> output_gain_{};
    std::size_t delay_position_ = 0U;
    std::uint32_t sample_rate_ = 0U;
    float energy_attack_ = 0.0F;
    float energy_release_ = 0.0F;
    bool initialized_ = false;
#ifdef DTSX_PARMA_TESTING
    std::array<float, 64U> gain_trace_{};
    std::size_t gain_trace_count_ = 0U;
#endif
};

} // namespace dtsx_decode
