#pragma once

#include "dtsx/ace_band_scheduler.hpp"
#include "dtsx/ace_scalar_dequant.hpp"
#include "dtsx/ace_stream.hpp"
#include "dtsx/ace_vq.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace dtsx {

struct AceSpectralPayload final {
    bool short_transform = false;
    std::array<std::uint32_t, 22> band_block_count{};
    std::vector<AceBandScheduleEntry> bands;
    std::vector<AceVqPayloadResult> vq_bands;
    std::vector<std::vector<float>> spectra;
    std::size_t bits_consumed = 0U;
    std::size_t final_bit_offset = 0U;
    // Post-VQ / FillPartition, pre-UnNormalize band vectors in VQ order.
    // Native StoreCurrentBand keeps the TF-reshaped plane; the inverse
    // Buffer reshape of that plane is this layout, so the next AU can feed
    // FillPartition without a second TF pass.
    std::vector<std::vector<float>> hole_fill_state;
    AceScalarMatrix finalized_lognorm_q10{};
    std::uint32_t failed_band = std::numeric_limits<std::uint32_t>::max();
    std::size_t failed_bit_offset = 0U;
};

struct AceStereoSpectralBand final {
    AceBandScheduleEntry schedule{};
    std::uint32_t coding = 0U;
    bool left_partition_first = true;
    AceBandRatio mid_side_ratio{};
    AceVqPayloadResult left_or_mid{};
    AceVqPayloadResult right_or_side{};
    // Native output of ProcessOneBandStereo.  For coding 1/3 these are
    // post-StereoDecMidSideRevert L/R spectra; for coding 2 they are direct
    // L/R spectra.  They are still before scalar unnormalization and MDCT.
    std::vector<float> left_spectrum;
    std::vector<float> right_spectrum;
    std::vector<float> hole_fill_left;
    std::vector<float> hole_fill_right;
    // StoreCurrentBand working buffers a1[2]/a1[3]: M/S after L2, collapsed
    // when coding==1. PrepareSrc copies these for coding 1/3.
    std::vector<float> hole_fill_work_left;
    std::vector<float> hole_fill_work_right;
};

struct AceStereoSpectralPayload final {
    bool short_transform = false;
    std::array<std::uint32_t, 22> band_block_count{};
    std::vector<AceStereoSpectralBand> bands;
    std::size_t bits_consumed = 0U;
    std::size_t final_bit_offset = 0U;
    AceScalarMatrix finalized_lognorm_q10{};
    // Diagnostic only: retained on parse failure so regression can identify
    // the native band whose budget/control remains unported.
    std::uint32_t failed_band = std::numeric_limits<std::uint32_t>::max();
    std::size_t failed_bit_offset = 0U;
};

// Packs the dequantized ACE band vectors into the native 1024-bin MDCT input
// order.  This is the first synthesis boundary: it deliberately performs no
// speaker routing or PCM output until the native overlap/history stage is
// gated against the DLL.
[[nodiscard]] bool ace_pack_spectral_frame(
    std::uint32_t sample_rate,
    const std::vector<std::vector<float>>& bands,
    std::vector<float>& coefficients) noexcept;

[[nodiscard]] bool ace_pack_spectral_frame_with_blocks(
    std::uint32_t sample_rate,
    const std::vector<std::vector<float>>& bands,
    const std::array<std::uint32_t, 22>& block_counts,
    std::vector<float>& coefficients,
    bool short_transform) noexcept;

// Table 9-115 transpose branch used when the native time/frequency reshape
// mode is zero.  `time_factor` is the native a3 value and must divide the
// band vector length.
[[nodiscard]] bool ace_time_frequency_reshape(
    std::vector<float>& band,
    std::uint32_t time_factor) noexcept;

// Native TimeFreqReshapeBand: mode 0 transposes with a3 (8 or 1); non-zero
// mode transposes with a3<<mode then DTSAceBandDequant_Hadamard.
[[nodiscard]] bool ace_time_frequency_reshape_native(
    std::vector<float>& band,
    std::uint32_t transform_size,
    std::int32_t reshape_mode,
    std::uint32_t band_start = 0U) noexcept;

// Sony TimeFreqReshapeBuffer: inverse of Band. mode 0 inverse-transposes;
// non-zero mode is Hadamard(-mode) with unshifted ntime then inverse transpose.
[[nodiscard]] bool ace_time_frequency_reshape_buffer(
    std::vector<float>& band,
    std::uint32_t transform_size,
    std::int32_t reshape_mode,
    std::uint32_t buffer_start_div = 0U) noexcept;

// Unnormalised pair butterfly used by DTSAceBandDequant_Hadamard.  The
// native routine applies this operation to rectangular reshaped bands; this
// primitive intentionally exposes one power-of-two row so its ordering and
// sum/difference signs can be differentially tested before matrix routing is
// attached.
[[nodiscard]] bool ace_hadamard_butterfly_reference(
    std::vector<float>& row) noexcept;

[[nodiscard]] bool consume_ace_mono_spectral_payload(
    const std::uint8_t* bytes,
    std::size_t size,
    std::uint32_t sample_rate,
    const AceStreamPrefix& prefix,
    AceSpectralPayload& payload,
    std::uint32_t* spectral_hole_fill_seed = nullptr,
    const std::vector<std::vector<float>>* previous_hole_fill = nullptr) noexcept;

// Native counterpart: stereo cases 1/2/3 in DTSAceBandDequant_Process,
// through ProcessOneBandStereo / two ProcessOneBand calls. The result is
// coefficient payload before scalar dequant, M/S rotation and synthesis.
[[nodiscard]] bool consume_ace_stereo_spectral_payload(
    const std::uint8_t* bytes,
    std::size_t size,
    std::uint32_t sample_rate,
    const AceStreamPrefix& prefix,
    AceStereoSpectralPayload& payload,
    std::uint32_t* spectral_hole_fill_seed = nullptr,
    const std::vector<std::vector<float>>* previous_hole_fill_left = nullptr,
    const std::vector<std::vector<float>>* previous_hole_fill_right = nullptr,
    const std::vector<std::uint32_t>* previous_stereo_coding = nullptr,
    const std::vector<std::vector<float>>* previous_work_left = nullptr,
    const std::vector<std::vector<float>>* previous_work_right = nullptr) noexcept;

// Sony StoreCurrentBand / HandleStereo M/S: mid/side = gL*L ± gR*R, L2 to
// sqrt(N). collapse==true keeps the louder of the two (coding 1).
void ace_shf_lr_to_mid_side(
    std::vector<float>& left,
    std::vector<float>& right,
    std::int32_t left_lognorm_q10,
    std::int32_t right_lognorm_q10,
    bool collapse) noexcept;

// HandleStereoToMidSideMonoTransition: ace_shf_lr_to_mid_side(..., true).
void ace_shf_stereo_to_ms_mono_transition(
    std::vector<float>& left,
    std::vector<float>& right,
    std::int32_t left_lognorm_q10,
    std::int32_t right_lognorm_q10) noexcept;

// Sony PrepareSrcBand: BandSize samples from clamp(BandStart-BandSize) in the
// concatenated TF history (this AU overwrites bands 0..band-1).
[[nodiscard]] bool ace_shf_prepare_src_slice(
    const std::vector<std::vector<float>>* previous,
    const std::vector<std::vector<float>>& this_au,
    std::uint32_t band,
    std::size_t count,
    std::vector<float>& out) noexcept;

[[nodiscard]] bool ace_lr_stereo_bit_allocations(
    std::uint32_t allocation_select,
    std::uint32_t band,
    std::uint32_t total_bits,
    std::int32_t left_lognorm_q10,
    std::int32_t right_lognorm_q10,
    const std::array<std::uint32_t, 3>& region_modes,
    std::uint32_t& left_bits,
    std::uint32_t& right_bits) noexcept;

// Native counterpart: the quantizer-level branch in
// DTSAceBandDequant_StereoMidSideParamsCodec.  `log_band_q4` is the native
// table value returned by DTSAce_GetBandLogN; it is not a floating estimate.
[[nodiscard]] std::uint32_t ace_stereo_ms_ratio_quantization_level(
    std::uint32_t available_bits,
    std::uint32_t band_size,
    std::int32_t log_band_q4) noexcept;

// Native counterpart: DTSAceBandDequant_StereoMidSideParamsCodec.  The
// caller owns `persistent_beta_q15` because the native state carries it from
// one band to the next.  This is pre-synthesis stereo control only.
[[nodiscard]] bool ace_decode_stereo_mid_side_params(
    AceBitReader& source,
    std::uint32_t band_size,
    std::int32_t log_band_q4,
    bool coding_mode_1,
    std::uint32_t available_bits,
    std::int32_t& persistent_beta_q15,
    AceBandRatio& ratio,
    std::uint32_t& bits_consumed) noexcept;

// Native unnormalize step used after BandDequant: exp2(Q10 lognorm / 1024)
// is applied independently to each reconstructed channel/band spectrum.
[[nodiscard]] bool ace_apply_lognorm_gain(
    std::vector<float>& spectrum,
    std::int32_t lognorm_q10) noexcept;

// Sony DTSAceSpectralHoleFill_FillPartition: K!=0 is a no-op; a6==0 zeros
// the partition; otherwise copy a non-silent previous partition or draw
// dtsAce_PRNGRandInRange(-1,1) and L2-normalize to `scale`.
void ace_spectral_hole_fill_partition(
    float* dest,
    const float* previous,
    std::uint32_t count,
    std::uint32_t pulses,
    bool fill_enabled,
    float scale,
    std::uint32_t& seed) noexcept;

// Sony DTSAceBandDequant_InBandEnergySmoothing after PulseDecode:
// K!=0 && 2K < N && conditioning in 1..3. groups = 1 << (32-clz(ntime/2) + TF mode).
void ace_band_dequant_in_band_energy_smoothing(
    float* dest,
    std::uint32_t count,
    std::uint32_t groups,
    std::uint32_t pulses,
    std::uint32_t conditioning) noexcept;

[[nodiscard]] std::uint32_t ace_band_dequant_in_band_smoothing_groups(
    bool short_transform,
    std::int32_t reshape_mode) noexcept;

// Kept for THF seed alignment when FillPartition is not applied. Coding 3
// ProcessOneBandStereo passes a8=0, so those partitions do not fill.
void ace_band_dequant_advance_random_seed_after_vq(
    std::uint32_t& seed,
    bool spectral_hole_fill,
    const AceSpectralPayload& payload) noexcept;

void ace_band_dequant_advance_random_seed_after_vq(
    std::uint32_t& seed,
    bool spectral_hole_fill,
    const AceStereoSpectralPayload& payload) noexcept;

} // namespace dtsx
