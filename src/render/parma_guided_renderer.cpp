#include "render/parma_guided_renderer.hpp"

#include "dtsx/speaker_mask.hpp"
#include "render/parma_blind_config.hpp"
#include "render/parma_critical_bands.hpp"
#include "render/parma_guided_topology.hpp"
#include "render/parma_layout.hpp"
#include "render/parma_pairwise.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#ifdef DTSX_PARMA_TESTING
#include <cstdio>
#include <cstdlib>
#endif

namespace dtsx_decode {
namespace {
constexpr std::size_t kHop = 64U;
constexpr float kScale = 8388608.0F;
constexpr float kDenormal = 1.0e-18F;
constexpr float kMinimumSoundfieldGain = 0.31622776F;
constexpr float kMaximumSoundfieldGain = 3.1622777F;

std::int32_t pcm24(float value) noexcept {
    value = std::max(-1.0F, std::min(0.99999988F, value));
    return static_cast<std::int32_t>(value >= 0.0F
        ? std::floor(value * kScale + 0.5F)
        : std::ceil(value * kScale - 0.5F));
}

std::int32_t slot_for_speaker(std::uint32_t speaker) noexcept {
    return parma_channel_slot_from_speaker_mask(speaker);
}

} // namespace

bool ParmaGuidedRenderer::initialize(std::uint32_t sample_rate) noexcept {
    for (ParmaPairwiseAnalysis& analysis : pair_analysis_) {
        if (!analysis.initialize(sample_rate)) return false;
    }
    for (ParmaTripletAnalysis& analysis : triplet_analysis_) {
        if (!analysis.initialize(sample_rate)) return false;
    }
    for (ParmaQuadrupletAnalysis& analysis : quadruplet_analysis_) {
        if (!analysis.initialize(sample_rate)) return false;
    }
    for (auto& value : analysis_delay_) value.fill(0.0F);
    for (auto& value : direct_delay_) value.fill(0.0F);
    lfe_delay_.fill(0.0F);
    output_gain_.fill(1.0F);
    energy_attack_ = 1.0F - std::exp(-64.0F / (static_cast<float>(sample_rate) * 0.0063275F));
    energy_release_ = 1.0F - std::exp(-64.0F / (static_cast<float>(sample_rate) * 0.012655F));
    for (ParmaAnalysisFilterBank& filter : analysis_filters_) filter.reset();
    for (ParmaSynthesisFilterBank& filter : synthesis_filters_) filter.reset();
    delay_position_ = 0U;
    sample_rate_ = sample_rate;
    initialized_ = true;
#ifdef DTSX_PARMA_TESTING
    gain_trace_.fill(0.0F); gain_trace_count_ = 0U;
#endif
    return true;
}

void ParmaGuidedRenderer::flush() noexcept {
    if (!initialized_) return;
    for (ParmaPairwiseAnalysis& analysis : pair_analysis_) analysis.reset();
    for (ParmaTripletAnalysis& analysis : triplet_analysis_) analysis.reset();
    for (ParmaQuadrupletAnalysis& analysis : quadruplet_analysis_) analysis.reset();
    for (auto& value : analysis_delay_) value.fill(0.0F);
    for (auto& value : direct_delay_) value.fill(0.0F);
    lfe_delay_.fill(0.0F);
    output_gain_.fill(1.0F);
    for (ParmaAnalysisFilterBank& filter : analysis_filters_) filter.reset();
    for (ParmaSynthesisFilterBank& filter : synthesis_filters_) filter.reset();
    delay_position_ = 0U;
    sample_rate_ = 0U;
    initialized_ = false;
#ifdef DTSX_PARMA_TESTING
    gain_trace_.fill(0.0F); gain_trace_count_ = 0U;
#endif
}

bool ParmaGuidedRenderer::render(
    const std::vector<std::vector<std::int32_t>>& input,
    std::uint32_t input_mask, const ChannelLayout& output_layout,
    const ParmaGuidedControls& controls, std::uint32_t sample_rate,
    std::vector<std::vector<std::int32_t>>& output) noexcept {
    ParmaGuidedTopology topology;
    if (!build_parma_guided_topology(controls, topology)
        || controls.downmix_speaker_activity_mask != input_mask
        || topology.node_count == 0U || input.empty()
        || input.front().empty() || input.front().size() % kHop != 0U) return false;
    if (!initialized_) {
        if (!initialize(sample_rate)) return false;
    } else if (sample_rate_ != sample_rate) return false;

    std::array<std::int32_t, 28U> in_index{}; in_index.fill(-1);
    std::array<std::int32_t, 28U> out_index{}; out_index.fill(-1);
    std::int32_t in_lfe = -1, out_lfe = -1;
    const auto speakers = dtsx::expand_speaker_activity_mask(input_mask);
    if (speakers.size() != input.size()) return false;
    for (std::size_t i = 0; i < input.size(); ++i) {
        if (input[i].size() != input.front().size()) return false;
        const std::int32_t slot = slot_for_speaker(speakers[i]);
        if (slot == -2) in_lfe = static_cast<std::int32_t>(i);
        else if (slot >= 0 && slot < 28) in_index[slot] = static_cast<std::int32_t>(i);
    }
    for (std::size_t i = 0; i < output_layout.channels.size(); ++i) {
        std::uint32_t speaker = 0U;
        if (!dtsx::standard_speaker_mask(output_layout.channels[i], speaker)) return false;
        const std::int32_t slot = slot_for_speaker(speaker);
        if (slot == -2) out_lfe = static_cast<std::int32_t>(i);
        else if (slot >= 0 && slot < 28) out_index[slot] = static_cast<std::int32_t>(i);
        else return false;
    }
    if (in_lfe < 0 || out_lfe < 0) return false;
    for (std::uint32_t i = 0; i < topology.node_count; ++i) {
        const ParmaGuidedNode& n = topology.nodes[i];
        if (n.target >= 28U || out_index[n.target] < 0 || n.mode > 3U) return false;
        if (n.mode == 0U && in_index[n.sources[0U]] < 0) return false;
        if (n.mode == 1U && (in_index[n.sources[0U]] < 0 || in_index[n.sources[1U]] < 0)) return false;
        if (n.mode == 2U && (in_index[n.sources[0U]] < 0
            || in_index[n.sources[1U]] < 0
            || in_index[n.sources[2U]] < 0)) return false;
        if (n.mode == 3U && (in_index[n.sources[0U]] < 0
            || in_index[n.sources[1U]] < 0 || in_index[n.sources[2U]] < 0
            || in_index[n.sources[3U]] < 0)) return false;
    }

    output.assign(output_layout.channels.size(), std::vector<std::int32_t>(input.front().size()));
    std::vector<std::uint32_t> width_vector; std::vector<float> bark_scale;
    if (!parma_initialize_critical_band_partitions(sample_rate, kHop, 16U, width_vector, bark_scale) || width_vector.size() != 16U) return false;
    std::array<std::uint32_t, 16U> widths{}; std::copy(width_vector.begin(), width_vector.end(), widths.begin());
    std::array<std::array<float, kHop>, 28U> in_re{}, in_im{}, out_re{}, out_im{};
    std::array<std::array<float, kHop>, 11U> pair_pos{};
    std::array<std::array<float, kHop>, 11U> pair_bal{};
    std::array<std::array<float, kHop>, 11U> pair_dif{};
    std::array<std::array<std::uint32_t, kHop>, 11U> triplet_order{};
    std::array<std::array<float, kHop>, 11U> triplet_position{};
    std::array<std::array<float, kHop>, 11U> triplet_diffuseness{};
    std::array<std::array<float, kHop>, 11U> triplet_coherence{};
    std::array<std::array<std::uint32_t, kHop>, 11U> quadruplet_order{};
    std::array<std::array<float, kHop>, 11U> quadruplet_position{};
    std::array<std::array<float, kHop>, 11U> quadruplet_diffuseness{};
    std::array<std::array<float, kHop>, 11U> quadruplet_coherence{};
    std::array<float, kHop> synthesized{};
    std::array<bool, 28U> soundfield_input{};
    for (std::uint32_t i = 0; i < topology.node_count; ++i) {
        const ParmaGuidedNode& node = topology.nodes[i];
        for (std::uint32_t source = 0U; source < node.mode + 1U; ++source)
            if (node.mode != 0U) soundfield_input[node.sources[source]] = true;
    }
    for (std::size_t off = 0; off < input.front().size(); off += kHop) {
        for (std::uint32_t slot = 0; slot < 28U; ++slot) if (in_index[slot] >= 0) {
            analysis_filters_[slot].process(analysis_delay_[slot].data(), in_re[slot].data(), in_im[slot].data());
            for (std::size_t s = 0; s < kHop; ++s) analysis_delay_[slot][s] = static_cast<float>(input[static_cast<std::size_t>(in_index[slot])][off + s]) / kScale;
        }
        for (auto& a : out_re) a.fill(0.0F);
        for (auto& a : out_im) a.fill(0.0F);
        std::array<bool, 28U> reconstructed{};
        for (std::uint32_t i = 0; i < topology.node_count; ++i) {
            const ParmaGuidedNode& n = topology.nodes[i];
            if (n.mode == 0U) { out_re[n.target] = in_re[n.sources[0U]]; out_im[n.target] = in_im[n.sources[0U]]; continue; }
            if (n.mode == 3U) {
                const std::array<std::uint8_t, 4U> sources = {
                    n.sources[0U], n.sources[1U], n.sources[2U], n.sources[3U]};
                std::uint32_t first = i;
                for (std::uint32_t j = 0U; j < i; ++j) {
                    const auto& q = topology.nodes[j];
                    if (q.mode == 3U && q.sources == n.sources) { first = j; break; }
                }
                const std::array<const float*, 4U> source_real = {
                    in_re[sources[0U]].data(), in_re[sources[1U]].data(),
                    in_re[sources[2U]].data(), in_re[sources[3U]].data()};
                const std::array<const float*, 4U> source_imaginary = {
                    in_im[sources[0U]].data(), in_im[sources[1U]].data(),
                    in_im[sources[2U]].data(), in_im[sources[3U]].data()};
                if (first == i)
                    quadruplet_analysis_[first].process(source_real,
                        source_imaginary, quadruplet_order[first].data(),
                        quadruplet_position[first].data(),
                        quadruplet_diffuseness[first].data(),
                        quadruplet_coherence[first].data());
#ifdef DTSX_PARMA_TESTING
                if (first == i && std::getenv("PARMA_TRACE_QUADRUPLET") != nullptr
                    && (off / kHop == 1U || off / kHop == 16U)) {
                    std::printf("candidate_quadruplet[%zu]", off / kHop);
                    for (std::size_t bin = 0U; bin < 16U; ++bin)
                        std::printf(" %u:%.9g,%.9g,%.9g",
                            quadruplet_order[first][bin],
                            quadruplet_position[first][bin],
                            quadruplet_diffuseness[first][bin],
                            quadruplet_coherence[first][bin]);
                    std::putchar('\n');
                }
#endif
                parma_extract_matrixed_quadruplet_channel(
                    quadruplet_order[first].data(),
                    quadruplet_position[first].data(),
                    quadruplet_diffuseness[first].data(), source_real,
                    source_imaginary, out_re[n.target].data(),
                    out_im[n.target].data(), kHop);
                bool final = true;
                for (std::uint32_t j = i + 1U; j < topology.node_count; ++j) {
                    const auto& q = topology.nodes[j];
                    if (q.mode == 3U && q.sources == n.sources) final = false;
                }
                if (final) {
                    const std::array<float*, 4U> target_real = {
                        out_re[sources[0U]].data(), out_re[sources[1U]].data(),
                        out_re[sources[2U]].data(), out_re[sources[3U]].data()};
                    const std::array<float*, 4U> target_imaginary = {
                        out_im[sources[0U]].data(), out_im[sources[1U]].data(),
                        out_im[sources[2U]].data(), out_im[sources[3U]].data()};
                    parma_repan_quadruplet_channels(
                        quadruplet_order[first].data(),
                        quadruplet_position[first].data(),
                        quadruplet_diffuseness[first].data(),
                        quadruplet_coherence[first].data(), source_real,
                        source_imaginary, target_real, target_imaginary, kHop);
                    for (const std::uint8_t source : sources)
                        reconstructed[source] = true;
                }
                reconstructed[n.target] = true;
                continue;
            }
            if (n.mode == 2U) {
                const std::array<std::uint8_t, 3U> sources = {
                    n.sources[0U], n.sources[1U], n.sources[2U]};
                std::uint32_t first = i;
                for (std::uint32_t j = 0U; j < i; ++j) {
                    const auto& q = topology.nodes[j];
                    if (q.mode == 2U && q.sources[0U] == sources[0U]
                        && q.sources[1U] == sources[1U]
                        && q.sources[2U] == sources[2U]) {
                        first = j;
                        break;
                    }
                }
                const std::array<const float*, 3U> source_real = {
                    in_re[sources[0U]].data(), in_re[sources[1U]].data(),
                    in_re[sources[2U]].data()};
                const std::array<const float*, 3U> source_imaginary = {
                    in_im[sources[0U]].data(), in_im[sources[1U]].data(),
                    in_im[sources[2U]].data()};
                if (first == i) {
#ifdef DTSX_PARMA_TESTING
                    if (std::getenv("PARMA_TRACE_TRIPLET") != nullptr
                        && off / kHop == 16U) {
                        std::printf("candidate_triplet_sources[%u]", first);
                        for (std::size_t source = 0U; source < 3U; ++source) {
                            float energy = 0.0F;
                            for (std::size_t bin = 0U; bin < kHop; ++bin)
                                energy += source_real[source][bin]
                                        * source_real[source][bin]
                                    + source_imaginary[source][bin]
                                        * source_imaginary[source][bin];
                            std::printf(" %u:%.9g", sources[source], energy);
                        }
                        std::putchar('\n');
                    }
#endif
                    triplet_analysis_[first].process(source_real,
                        source_imaginary, triplet_order[first].data(),
                        triplet_position[first].data(),
                        triplet_diffuseness[first].data(),
                        triplet_coherence[first].data());
#ifdef DTSX_PARMA_TESTING
                    if (std::getenv("PARMA_TRACE_TRIPLET") != nullptr
                        && (off / kHop == 1U || off / kHop == 16U)) {
                        std::printf("candidate_triplet[%zu]", off / kHop);
                        for (std::size_t bin = 0U; bin < 16U; ++bin)
                            std::printf(" %u:%.9g,%.9g,%.9g",
                                triplet_order[first][bin],
                                triplet_position[first][bin],
                                triplet_diffuseness[first][bin],
                                triplet_coherence[first][bin]);
                        std::putchar('\n');
                    }
#endif
                }
                parma_extract_matrixed_triplet_channel(
                    triplet_order[first].data(),
                    triplet_position[first].data(),
                    triplet_diffuseness[first].data(), source_real,
                    source_imaginary, out_re[n.target].data(),
                    out_im[n.target].data(), kHop);
                bool final = true;
                for (std::uint32_t j = i + 1U; j < topology.node_count; ++j) {
                    const auto& q = topology.nodes[j];
                    if (q.mode == 2U && q.sources[0U] == sources[0U]
                        && q.sources[1U] == sources[1U]
                        && q.sources[2U] == sources[2U]) final = false;
                }
                if (final) {
                    const std::array<float*, 3U> target_real = {
                        out_re[sources[0U]].data(), out_re[sources[1U]].data(),
                        out_re[sources[2U]].data()};
                    const std::array<float*, 3U> target_imaginary = {
                        out_im[sources[0U]].data(), out_im[sources[1U]].data(),
                        out_im[sources[2U]].data()};
                    parma_repan_triplet_channels(
                        triplet_order[first].data(),
                        triplet_position[first].data(),
                        triplet_diffuseness[first].data(),
                        triplet_coherence[first].data(), source_real,
                        source_imaginary, target_real, target_imaginary, kHop);
                    for (const std::uint8_t source : sources)
                        reconstructed[source] = true;
                }
                reconstructed[n.target] = true;
                continue;
            }
            const std::uint8_t a = n.sources[0U], b = n.sources[1U];
            std::uint32_t first = i;
            for (std::uint32_t j = 0; j < i; ++j) { const auto& q = topology.nodes[j]; if (q.mode == 1U && q.sources[0U] == a && q.sources[1U] == b) { first = j; break; } }
            if (first == i) {
                pair_analysis_[first].process(in_re[a].data(), in_im[a].data(), in_re[b].data(), in_im[b].data(), pair_pos[first].data(), pair_bal[first].data(), pair_dif[first].data());
#ifdef DTSX_PARMA_TESTING
                if (first == 7U && std::getenv("PARMA_TRACE_ANALYSIS") != nullptr) {
                    std::printf("candidate_analysis[%zu] %.9f %.9f %.9f %.9f\n",
                        off / kHop,
                        pair_analysis_[first].testing_energy_ratio_state()[0U],
                        pair_analysis_[first].testing_position_state()[0U],
                        pair_analysis_[first].testing_balance_state()[0U],
                        pair_analysis_[first].testing_diffuseness_state()[0U]);
                }
#endif
            }
            float lower_angle = 0.0F;
            float upper_angle = 1.0F;
            bool all_angles_equal = true;
            for (std::uint32_t j = 0; j < topology.node_count; ++j) {
                const auto& q = topology.nodes[j];
                if (q.mode != 1U || q.sources[0U] != a
                    || q.sources[1U] != b) continue;
                all_angles_equal = all_angles_equal
                    && q.decoding_angle == n.decoding_angle;
                if (q.decoding_angle < n.decoding_angle)
                    lower_angle = std::max(lower_angle, q.decoding_angle);
                if (q.decoding_angle > n.decoding_angle)
                    upper_angle = std::min(upper_angle, q.decoding_angle);
            }
            if (all_angles_equal) lower_angle = n.decoding_angle;
            parma_extract_matrixed_pairwise_channel(pair_pos[first].data(), pair_bal[first].data(), pair_dif[first].data(), in_re[a].data(), in_im[a].data(), in_re[b].data(), in_im[b].data(), out_re[n.target].data(), out_im[n.target].data(), lower_angle, n.decoding_angle, upper_angle, kHop);
            // A group can have several targets.  Residual repanning is done
            // once, at its final target, using native min/max target angles.
            bool final = true; float minimum = n.decoding_angle, maximum = n.decoding_angle;
            for (std::uint32_t j = 0; j < topology.node_count; ++j) { const auto& q = topology.nodes[j]; if (q.mode == 1U && q.sources[0U] == a && q.sources[1U] == b) { minimum = std::min(minimum, q.decoding_angle); maximum = std::max(maximum, q.decoding_angle); if (j > i) final = false; } }
            if (final) { parma_repan_pairwise_channel(pair_pos[first].data(), pair_bal[first].data(), pair_dif[first].data(), in_re[a].data(), in_im[a].data(), in_re[b].data(), in_im[b].data(), out_re[a].data(), out_im[a].data(), out_re[b].data(), out_im[b].data(), minimum, 1.0F - maximum, kHop); reconstructed[a] = reconstructed[b] = true; }
            reconstructed[n.target] = true;
        }
        std::array<float, kHop> input_energy{}, output_energy{}, bin_gain{};
#ifdef DTSX_PARMA_TESTING
        if (std::getenv("PARMA_TRACE_PAIRWISE") != nullptr
            && (off / kHop == 8U || off / kHop == 16U)) {
            for (std::uint32_t slot = 0U; slot < 28U; ++slot) {
                if (!reconstructed[slot]) continue;
                float energy = 0.0F;
                for (std::size_t b = 0U; b < kHop; ++b)
                    energy += out_re[slot][b] * out_re[slot][b]
                        + out_im[slot][b] * out_im[slot][b];
                std::printf("candidate_pair_slot[%zu,%u] %.9g\n",
                    off / kHop, slot, energy);
            }
        }
#endif
        for (std::uint32_t slot = 0U; slot < 28U; ++slot) for (std::size_t b = 0U; b < kHop; ++b) {
            if (soundfield_input[slot] && in_index[slot] >= 0) input_energy[b] += in_re[slot][b] * in_re[slot][b] + in_im[slot][b] * in_im[slot][b];
            if (reconstructed[slot]) output_energy[b] += out_re[slot][b] * out_re[slot][b] + out_im[slot][b] * out_im[slot][b];
        }
        std::array<float, 16U> input_bands{}, output_bands{};
        parma_group_critical_bands(input_energy.data(), input_bands.data(), widths.data(), widths.size());
        parma_group_critical_bands(output_energy.data(), output_bands.data(), widths.data(), widths.size());
        for (std::size_t b = 0U; b < output_gain_.size(); ++b) { float target = std::sqrt(input_bands[b] / (output_bands[b] + 1.0e-12F)); target = std::max(kMinimumSoundfieldGain, std::min(kMaximumSoundfieldGain, target)); const float coefficient = output_gain_[b] >= target ? energy_release_ : energy_attack_; output_gain_[b] = ((1.0F - coefficient) * output_gain_[b] + coefficient * target + kDenormal) - kDenormal; }
#ifdef DTSX_PARMA_TESTING
        if (gain_trace_count_ < gain_trace_.size()) gain_trace_[gain_trace_count_++] = output_gain_[0U];
        if (std::getenv("PARMA_TRACE_ENERGY") != nullptr) {
            std::printf("candidate_energy[%zu] %.9g %.9g %.9g\n",
                off / kHop, input_bands[0U], output_bands[0U], output_gain_[0U]);
        }
#endif
        parma_ungroup_critical_bands(output_gain_.data(), bin_gain.data(), widths.data(), widths.size());
        for (std::uint32_t slot = 0; slot < 28U; ++slot) if (out_index[slot] >= 0 && reconstructed[slot]) {
            for (std::size_t b = 0U; b < kHop; ++b) { out_re[slot][b] *= bin_gain[b]; out_im[slot][b] *= bin_gain[b]; }
#ifdef DTSX_PARMA_TESTING
            if (std::getenv("PARMA_TRACE_SUBBAND") != nullptr && off / kHop == 8U) {
                float energy = 0.0F;
                for (std::size_t b = 0U; b < kHop; ++b)
                    energy += out_re[slot][b] * out_re[slot][b] + out_im[slot][b] * out_im[slot][b];
                std::printf("candidate_subband_energy[%u] %.9g\n", slot, energy);
            }
#endif
            synthesis_filters_[slot].process(out_re[slot].data(), out_im[slot].data(), synthesized.data());
            for (std::size_t s = 0; s < kHop; ++s) output[static_cast<std::size_t>(out_index[slot])][off + s] = pcm24(synthesized[s]);
        }
        for (std::uint32_t i = 0; i < topology.node_count; ++i) { const auto& n = topology.nodes[i]; if (n.mode != 0U || reconstructed[n.target]) continue; const auto oi = static_cast<std::size_t>(out_index[n.target]); const auto ii = static_cast<std::size_t>(in_index[n.sources[0U]]); for (std::size_t s = 0; s < kHop; ++s) { const std::size_t p = (delay_position_ + s) % 1024U; const float old = direct_delay_[n.target][p]; direct_delay_[n.target][p] = static_cast<float>(input[ii][off + s]) / kScale; output[oi][off + s] = pcm24(old); } }
        for (std::size_t s = 0; s < kHop; ++s) { const std::size_t p = (delay_position_ + s) % 1024U; const float old = lfe_delay_[p]; lfe_delay_[p] = static_cast<float>(input[static_cast<std::size_t>(in_lfe)][off + s]) / kScale; output[static_cast<std::size_t>(out_lfe)][off + s] = pcm24(old); }
        delay_position_ = (delay_position_ + kHop) % 1024U;
    }
    return true;
}

} // namespace dtsx_decode
