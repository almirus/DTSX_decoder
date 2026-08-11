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

    // DTS_ParmaDec_Flush clears the analysis/synthesis state but does not
    // generate a tail. The caller keeps the PCM duration unchanged.
    void flush() noexcept;

#ifdef DTSX_PARMA_TESTING
    [[nodiscard]] const std::array<float, 16U>&
    testing_output_gain() const noexcept {
        return output_gain_;
    }

    [[nodiscard]] const std::array<std::array<float, 16U>, 4U>&
    testing_intermediate_gain() const noexcept {
        return intermediate_gain_;
    }

    [[nodiscard]] const ParmaPairwiseAnalysis&
    testing_left_analysis() const noexcept {
        return left_analysis_;
    }
    [[nodiscard]] const ParmaPairwiseAnalysis&
    testing_front_analysis() const noexcept {
        return front_analysis_;
    }
    [[nodiscard]] const std::array<float, 16U>&
    testing_stereo_primary() const noexcept { return stereo_primary_state_; }
    [[nodiscard]] const std::array<float, 16U>&
    testing_stereo_secondary() const noexcept { return stereo_secondary_state_; }
    [[nodiscard]] const std::array<float, 16U>&
    testing_stereo_repan() const noexcept { return stereo_repan_state_; }
    [[nodiscard]] const std::array<float, 16U>&
    testing_stereo_output() const noexcept { return stereo_output_state_; }
#endif

private:
    [[nodiscard]] bool initialize(std::uint32_t sample_rate) noexcept;

    std::array<ParmaAnalysisFilterBank, 28U> analysis_filters_{};
    std::array<ParmaSynthesisFilterBank, 28U> synthesis_filters_{};
    // DTS_ParmaDec_Process calls dts_flib_osfb_f32_t_analysis[_x2] before
    // copying the current PCM hop into the OSFB input cache.  This explicit
    // hop is therefore part of the public 1024-sample PARMA latency.
    std::array<std::array<float, 64U>, 28U> analysis_input_delay_{};
    ParmaPairwiseAnalysis left_analysis_;
    ParmaPairwiseAnalysis right_analysis_;
    ParmaPairwiseAnalysis front_analysis_;
    ParmaPairwiseAnalysis rear_analysis_;
    ParmaPairwiseAnalysis auxiliary_left_analysis_;
    ParmaPairwiseAnalysis auxiliary_right_analysis_;
    std::array<float, 16U> stereo_primary_state_{};
    std::array<float, 16U> stereo_secondary_state_{};
    std::array<float, 16U> stereo_repan_state_{};
    std::array<float, 16U> stereo_output_state_{};
    std::array<std::array<float, 16U>, 4U>
        intermediate_gain_{};
    std::array<float, 16U> output_gain_{};
    // Direct/pass-through channels do not traverse OSFB synthesis.  Native
    // DTS_ParmaDec still delays them by the public 1024-sample latency so
    // that they remain aligned with the reconstructed channels.
    std::array<std::array<float, parma_filterbank_latency_samples()>, 28U>
        direct_delay_{};
    std::size_t direct_delay_position_ = 0U;
    std::array<float, parma_filterbank_latency_samples()> lfe_delay_{};
    std::size_t lfe_position_ = 0U;
    std::array<float, 512U> mono_delay_{};
    std::size_t mono_delay_position_ = 0U;
    std::uint32_t sample_rate_ = 0U;
    float energy_attack_ = 0.0F;
    float energy_release_ = 0.0F;
    bool initialized_ = false;
};

} // namespace dtsx_decode
