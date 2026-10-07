#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <vector>

namespace dtsx {

struct AceWaveformRegistration final {
    std::uint32_t stream_set_id = 0U;
    std::uint32_t stream_index = 0U;
    std::uint32_t speaker_activity_mask = 0U;
    std::uint32_t first_channel = 0U;
    std::uint32_t channel_count = 0U;
    // DTSX2_ACEW_RegisterObjs stores relabel markers alongside the expanded
    // physical channel entries.  Keep one two-bit marker per exported
    // channel (bit 0 = activity 3, bit 1 = activity 12).
    std::vector<std::uint8_t> relabel_flags;
    bool left_relabel = false;
    bool right_relabel = false;
};

[[nodiscard]] AceWaveformRegistration ace_make_waveform_registration(
    std::uint32_t stream_set_id,
    std::uint32_t stream_index,
    std::uint32_t speaker_activity_mask,
    bool stereo);

[[nodiscard]] std::vector<std::uint32_t> ace_waveform_channel_masks(
    std::uint32_t speaker_activity_mask,
    std::uint32_t stream_index,
    bool stereo);

enum class AceCodingMode : std::uint32_t {
    Full = 0U,
    Off = 1U,
    Left = 2U,
    Right = 3U,
};

struct AceStreamPrefix final {
    bool stereo = false;
    bool predictive = false;
    std::uint32_t bandwidth_mode = 0U;
    std::uint32_t effective_bands = 0U;
    AceCodingMode coding_mode = AceCodingMode::Off;
    std::uint32_t first_effective_channel = 0U;
    std::uint32_t effective_channel_count = 0U;
    bool lts_enabled = false;
    std::uint32_t lts_lag = 0U;
    std::uint32_t lts_filter_index = 0U;
    bool lts_filter_reused = false;
    bool high_resolution_vq = false;
    bool short_transform = false;
    bool spectral_hole_fill = false;
    bool temporal_hole_fill = false;
    std::uint32_t lognorm_k_boost = 0U;
    std::array<std::array<std::int32_t, 22>, 2>
        primary_lognorm_residual{};
    std::array<std::array<std::int32_t, 22>, 2>
        primary_lognorm_q10{};
    std::array<std::array<std::int32_t, 22>, 2>
        previous_lognorm_q10{};
    std::array<std::array<std::int32_t, 22>, 2>
        refined_lognorm_q10{};
    std::array<std::uint32_t, 22> band_block_count{};
    std::uint32_t conditioning_level = 0U;
    std::uint32_t num_coded_bands = 0U;
    std::uint32_t num_ms_mono_bands = 0U;
    std::array<std::uint32_t, 22> stereo_coding{};
    std::array<std::uint32_t, 3> lr_allocation_mode{};
    std::uint32_t allocation_model = 0U;
    std::int32_t allocation_model_parameter = 0;
    std::array<std::int32_t, 22> allocation_delta{};
    std::array<std::int32_t, 22> allocation_adjustment{};
    std::array<std::int32_t, 22> allocation_lognorm_level{};
    std::array<std::uint32_t, 22> allocation{};
    std::array<std::uint32_t, 22> raw_allocation{};
    std::array<std::uint32_t, 22> refinement_allocation{};
    std::array<std::array<std::uint16_t, 22>, 2> refinement_code{};
    std::uint32_t allocation_total_bits = 0U;
    std::uint32_t allocation_available_bits = 0U;
    std::size_t lts_lag_bits_consumed = 0U;
    std::size_t lts_bits_consumed = 0U;
    std::size_t stream_flags_bits_consumed = 0U;
    std::size_t primary_lognorm_bits_consumed = 0U;
    std::size_t band_block_bits_consumed = 0U;
    std::size_t conditioning_bits_consumed = 0U;
    std::size_t allocation_header_bits_consumed = 0U;
    std::size_t allocation_bits_consumed = 0U;
    std::size_t refinement_bits_consumed = 0U;
    // The elementary spectral payload begins here.  Prefix parsing deliberately
    // stops before coarse residual/VQ data; retaining this exact bit boundary
    // prevents the internal P2 path from claiming that it decoded PCM when it
    // only reconstructed stream control state.
    std::size_t spectral_payload_bit_offset = 0U;
    std::size_t spectral_payload_bits_remaining = 0U;
    std::uint32_t parsed_stage = 0U;
    std::size_t bits_consumed = 0U;
};

struct AceStreamPrefixState final {
    bool initialized = false;
    std::uint32_t lts_lag = 0U;
    std::uint32_t lts_filter_index = 0U;
    std::array<std::array<std::int32_t, 22>, 2>
        primary_lognorm_q10{};
    std::uint32_t num_coded_bands = 0U;
    std::uint32_t num_ms_mono_bands = 0U;
    std::int32_t allocation_model_parameter = 0;
    std::array<std::uint32_t, 22> raw_allocation{};
    std::uint32_t guaranteed_allocation = 0U;
};

enum class AceStreamParseResult {
    Complete,
    Invalid,
    Unsupported,
};

// Native counterpart: DTSAce_GetBandSize().
[[nodiscard]] std::uint32_t ace_stream_band_size(
    std::uint32_t sample_rate,
    std::uint32_t band) noexcept;

// Sony DTSAce_GetLogBandSize: int16 Q4 table, band>=0x15 clamps to 21.
// Process passes shift 0 when hop is 1024 (`v66`).
[[nodiscard]] std::int32_t ace_log_band_size_q4(
    std::uint32_t sample_rate,
    std::uint32_t band) noexcept;

[[nodiscard]] std::int32_t ace_band_mean_lognorm_q10(
    std::uint32_t sample_rate,
    std::uint32_t band) noexcept;

[[nodiscard]] std::int32_t ace_unnormalize_lognorm_q10(
    std::int32_t decoded_q10,
    std::uint32_t sample_rate,
    std::uint32_t band) noexcept;

// Native DTSAceTemporalHoleFill_CalcNormThreshBitAlloc lookup.  The native
// code clamps the rounded (reciprocal-band-count * bit-allocation * 10)
// index to [0,99] before multiplying the per-bin norm threshold.
[[nodiscard]] float ace_temporal_hole_fill_bit_alloc_threshold(
    std::uint32_t index) noexcept;

// Fixed-point Q30 table used by ARM homatic/tcl THF implementations.
[[nodiscard]] std::uint32_t ace_temporal_hole_fill_bit_alloc_threshold_q30(
    std::uint32_t index) noexcept;

[[nodiscard]] std::uint32_t ace_temporal_hole_fill_bit_alloc_index_q30(
    std::int32_t scaled_allocation_q4) noexcept;

// Native Homatic weak tables. Total-bin reciprocals use Q28; square-root
// reciprocals use Q30. `bandwidth_mode` selects the native 22-entry row.
[[nodiscard]] std::uint32_t
ace_temporal_hole_fill_recip_num_total_bins_q28(
    std::uint32_t sample_rate,
    std::uint32_t bandwidth_mode,
    std::uint32_t band) noexcept;
[[nodiscard]] std::uint32_t
ace_temporal_hole_fill_recip_sqrt_num_bins_q30(
    std::uint32_t sample_rate,
    std::uint32_t bandwidth_mode,
    std::uint32_t band) noexcept;
[[nodiscard]] std::uint32_t
ace_temporal_hole_fill_bit_alloc_index_from_recip_q28(
    std::uint32_t reciprocal_q28,
    std::int32_t bit_allocation) noexcept;

[[nodiscard]] std::int32_t ace_fixed_mul_q31_round(
    std::int32_t left, std::int32_t right) noexcept;

[[nodiscard]] std::int32_t ace_temporal_hole_fill_norm_floor_q31(
    std::int32_t norm_q31) noexcept;

[[nodiscard]] std::int32_t ace_temporal_hole_fill_calc_offset_index_q30(
    std::int32_t ratio_q2) noexcept;

[[nodiscard]] std::int32_t ace_temporal_hole_fill_calc_offset_index_from_norms_q30(
    std::int32_t current_norm_q22,
    std::int32_t previous_norm_q22,
    std::int32_t history_norm_q22) noexcept;

void ace_temporal_hole_fill_apply_bit_alloc_threshold_q30(
    const std::int32_t* norms_q30,
    std::int32_t* thresholds_q30,
    std::uint32_t first_band,
    std::uint32_t last_band,
    std::uint32_t band_stride,
    std::uint32_t band_index,
    std::int32_t scaled_allocation_q4) noexcept;

// Native DTSAceTemporalHoleFill_CalcNormsThreshOffset.  `active_bins` is
// indexed as [band][channel] and uses the native one-byte active flag.
void ace_temporal_hole_fill_calc_norms_thresh_offset(
    const std::array<std::array<std::uint8_t, 2>, 22>& active_bins,
    const std::array<std::array<float, 2>, 22>& norms,
    std::array<std::array<float, 2>, 22>& thresholds,
    std::uint32_t channel_count,
    std::uint32_t first_band,
    std::uint32_t last_band) noexcept;

// Contiguous multi-channel form used by the native 7/10-channel THF state:
// each plane is [band][channel] with `channel_count` entries per band.
void ace_temporal_hole_fill_calc_norms_thresh_offset_flat(
    const std::uint8_t* active_bins,
    const float* norms,
    float* thresholds,
    std::uint32_t channel_count,
    std::uint32_t first_band,
    std::uint32_t last_band) noexcept;

void ace_temporal_hole_fill_calc_norm_thresh_bit_alloc(
    float reciprocal_band_bins,
    std::int32_t bit_allocation,
    const std::array<float, 22>& norms,
    std::array<float, 22>& thresholds,
    std::uint32_t first_band,
    std::uint32_t last_band) noexcept;

void ace_temporal_hole_fill_calc_norm_noise_predicted(
    const std::array<float, 22>& previous,
    const std::array<float, 22>& history,
    const std::array<float, 22>& predicted,
    std::array<float, 22>& output,
    std::uint32_t first_band,
    std::uint32_t last_band) noexcept;

void ace_temporal_hole_fill_calc_norm_thresh_low_confidence(
    std::uint32_t confidence_level,
    const std::array<float, 22>& channel_norms,
    std::array<float, 22>& thresholds,
    std::uint32_t first_band,
    std::uint32_t last_band) noexcept;

// Low-confidence profile whose sentinel is the integer 0x7fffffff rather
// than a float-domain value. Keep profiles explicit instead of mixing them.
void ace_temporal_hole_fill_calc_norm_thresh_low_confidence_homatic(
    std::uint32_t confidence_level,
    const std::array<float, 22>& channel_norms,
    std::array<float, 22>& thresholds,
    std::uint32_t first_band,
    std::uint32_t last_band) noexcept;

// Native counterpart: DTSAceTemporalHoleFill_CalcOffsetIndex.  Returns the
// linear interpolation factor used by the noise filler for one band/channel.
[[nodiscard]] float ace_temporal_hole_fill_calc_offset_index(
    float current_norm,
    float previous_norm,
    float history_norm) noexcept;

// Scalar body of DTSAceTemporalHoleFill_CalcBinNoiseLevels (Sony x64).
// Floor is current square-norm * 0.0001, not the reciprocal-bin table.
// `threshold` is min(predicted, bit-alloc, low-confidence). `bin_norm` is
// the per-bin value after CalcNormsThreshOffset (sqrt of that bin energy).
[[nodiscard]] float ace_temporal_hole_fill_calc_bin_noise_level(
    float current_sq_norm,
    float previous_norm,
    float history_norm,
    float threshold,
    float bin_norm,
    float output_scale) noexcept;

// Fixed-point body of Homatic/TCL CalcBinNoiseLevels. All norm arguments are
// native signed fixed-point values; reciprocal_sqrt_q30 and offset_q30 use
// Q30. The caller supplies the already-selected per-band thresholds.
[[nodiscard]] std::int32_t ace_temporal_hole_fill_calc_bin_noise_level_q30(
    std::int32_t bin_norm,
    std::int32_t predicted_threshold,
    std::int32_t bit_alloc_threshold,
    std::int32_t low_confidence_threshold,
    std::int32_t previous_norm,
    std::int32_t history_norm,
    std::int32_t reciprocal_sqrt_q30,
    std::int32_t output_scale_q30,
    std::int32_t bin_floor,
    std::int32_t offset_q30) noexcept;

// Scalar body of DTSAceTemporalHoleFill_CalcBlockSqNorms.  The native
// coefficient buffer is laid out as interleaved block rows; `sample_stride`
// is the row stride (the Process caller passes 1/2/4/8), and the returned
// norm is the sum of squares multiplied by that stride.
[[nodiscard]] float ace_temporal_hole_fill_calc_block_sq_norm(
    const float* coefficients,
    std::size_t coefficient_count,
    std::size_t band_start,
    std::size_t band_size,
    std::uint32_t sample_stride) noexcept;

// Fixed-point ARM counterpart used by homatic/tcl builds. The native
// `vmlal_s32` loop accumulates signed 32-bit coefficient squares in 64 bits
// and applies the sample-stride multiplier after the band scan.
[[nodiscard]] std::uint64_t ace_temporal_hole_fill_calc_block_sq_norm_i32(
    const std::int32_t* coefficients,
    std::size_t coefficient_count,
    std::size_t band_start,
    std::size_t band_size,
    std::uint32_t sample_stride) noexcept;

// dts_flib_sqrt_i64_to_i32. Used by FillHoles renormalize and
// CalcNormsThreshOffset.
[[nodiscard]] std::uint32_t ace_sqrt_i64_to_i32_native(
    std::uint32_t low,
    std::int32_t high) noexcept;

// Native Process a10 = (1024 >> THF a1[2] >> 7). SetCommonParam copies
// a2[5] from the stream-decoder loop `while (length < 1024 >> shift)`
// where length is the 1024-sample ACE frame, so a1[2] is 0 and a10 is 8
// for both long and short. Do not substitute the MDCT window size.
[[nodiscard]] inline std::uint32_t ace_thf_block_sample_stride(
    const std::uint32_t transform_shift) noexcept {
    if (transform_shift >= 10U) {
        return 1U;
    }
    const std::uint32_t stride = (1024U >> transform_shift) >> 7U;
    return stride == 0U ? 1U : stride;
}

[[nodiscard]] inline std::uint32_t ace_thf_energy_bin_shift(
    const bool short_transform) noexcept {
    return short_transform ? 3U : 0U;
}

// Aggregate form of CalcBlockSqNorms over native a10 sub-blocks.
// `energy_shift` is a11; `sample_stride` is a10.  `norms` keep the minimum
// positive sub-block energy; `active` is set if any sub-block is zero.
[[nodiscard]] bool ace_temporal_hole_fill_calc_block_sq_norms(
    const std::vector<std::vector<float>>& coefficients,
    std::uint32_t sample_rate,
    std::uint32_t bandwidth_mode,
    std::uint32_t band_count,
    std::uint32_t channel_count,
    std::uint32_t sample_stride,
    std::uint32_t energy_shift,
    std::vector<std::vector<float>>& norms,
    std::vector<std::vector<std::uint8_t>>& active) noexcept;

// Native counterpart: DTSAceTemporalHoleFill_UpdateHistory. Arrays use one
// contiguous 22-band plane per channel. For ten-channel streams `previous`
// and `history` are shifted; for seven-channel streams `previous` keeps the
// per-band minimum and both planes clear bands above `active_bands`.
void ace_temporal_hole_fill_update_history(
    std::uint32_t channel_count,
    std::uint32_t active_bands,
    const float* current,
    float* previous,
    float* history) noexcept;

// Same native 7/10 algorithms, but `plane_count` is the stream channel
// count (1..4) used by DTSAceStreamDecoder. `algorithm` is 7 (short /
// noise-fill) or 10 (long / history-shift).
void ace_temporal_hole_fill_update_history_planes(
    std::uint32_t plane_count,
    std::uint32_t algorithm,
    std::uint32_t active_bands,
    const float* current,
    float* previous,
    float* history) noexcept;

// Persistent wrapper for the native DTSAceTemporalHoleFill state planes.
// It deliberately exposes the previous/history arrays so the caller can
// initialize the native sentinel values before enabling the noise-fill stage.
class AceTemporalHoleFillHistory final {
public:
    void reset(std::uint32_t channel_count) noexcept;
    void reset(std::uint32_t plane_count, std::uint32_t algorithm) noexcept;
    // Native SetCommonParam writes algorithm 7/10 without clearing planes.
    void set_algorithm(std::uint32_t algorithm) noexcept;
    void note_process(std::uint32_t algorithm) noexcept;
    [[nodiscard]] std::uint32_t short_process_count() const noexcept;
    [[nodiscard]] std::uint32_t channel_count() const noexcept;
    [[nodiscard]] std::uint32_t algorithm() const noexcept;
    [[nodiscard]] std::vector<float>& previous() noexcept;
    [[nodiscard]] std::vector<float>& history() noexcept;
    [[nodiscard]] std::vector<float>& min_plane() noexcept;
    [[nodiscard]] const std::vector<float>& previous() const noexcept;
    [[nodiscard]] const std::vector<float>& history() const noexcept;
    [[nodiscard]] const std::vector<float>& min_plane() const noexcept;
    void update(const std::vector<float>& current,
        std::uint32_t active_bands) noexcept;
    // Sony a1[5]/a1[6]: UnNormalize pow2_i32 words. Algorithm 7 mins
    // previous with signed i32 compare; algorithm 10 shifts planes.
    void update_a7(const std::vector<std::int32_t>& current,
        std::uint32_t active_bands) noexcept;
    [[nodiscard]] const std::vector<std::int32_t>& previous_a7() const noexcept;
    [[nodiscard]] const std::vector<std::int32_t>& history_a7() const noexcept;

private:
    void fill_min_plane_sentinel() noexcept;

    std::uint32_t channel_count_ = 0U;
    std::uint32_t algorithm_ = 0U;
    std::uint32_t short_process_count_ = 0U;
    std::vector<float> previous_;
    std::vector<float> history_;
    std::vector<float> min_plane_;
    std::vector<std::int32_t> previous_a7_;
    std::vector<std::int32_t> history_a7_;
};

// DTSAceTemporalHoleFill_Process for one ACE stream. Algorithm 10 only
// advances history. Algorithm 7 (short-transform) runs the confirmed Sony
// float helpers and FillHolesWithNoise when `temporal_hole_fill` is set.
// The homatic Q30 CalcBinNoiseLevels / LAR-table path is not used.
[[nodiscard]] bool ace_temporal_hole_fill_process(
    std::vector<std::vector<float>>& coefficients,
    const std::array<std::uint32_t, 22>& bit_allocation,
    std::uint32_t sample_rate,
    std::uint32_t bandwidth_mode,
    std::uint32_t band_count,
    std::uint32_t block_shift,
    bool short_transform,
    bool temporal_hole_fill,
    AceTemporalHoleFillHistory& history,
    std::uint32_t& random_state,
    const std::array<std::uint32_t, 22>* stereo_coding = nullptr,
    const std::array<std::array<std::int32_t, 22>, 2>* lognorm_q10 = nullptr,
    AceCodingMode coding_mode = AceCodingMode::Full,
    const std::array<std::int32_t, 22>* band_beta = nullptr) noexcept;

// Native FillHolesWithNoise. `mode` is native a14 (last minus first plus
// one). Coherent fill runs when that value is 2 and
// (stereo_coding | 2) == 3. Coefficients are a channel-major plane; each
// channel must contain the complete transform buffer. `active_bins` and
// `noise_levels` are [band][channel] or packed [band][channel*8+sub]
// planes, while `target_norms` contains the native per-band norm used by
// the post-fill normalization.
[[nodiscard]] bool ace_temporal_hole_fill_fill_holes_with_noise(
    std::vector<std::vector<float>>& coefficients,
    const std::vector<std::vector<std::uint8_t>>& active_bins,
    const std::vector<std::vector<float>>& noise_levels,
    const std::vector<std::vector<float>>& target_norms,
    std::uint32_t sample_rate,
    std::uint32_t bandwidth_mode,
    std::uint32_t band,
    std::uint32_t first_channel,
    std::uint32_t last_channel,
    std::uint32_t sample_stride,
    std::uint32_t block_shift,
    std::int32_t mode,
    std::uint32_t& random_state,
    std::uint32_t subblock_index = 0U,
    bool renormalize = true,
    std::uint32_t stereo_coding = 3U,
    bool all_subblocks = false,
    std::int32_t band_beta = 0) noexcept;

[[nodiscard]] std::uint32_t ace_prng_rand_uint(std::uint32_t& state) noexcept;
[[nodiscard]] float ace_prng_rand(std::uint32_t& state) noexcept;
[[nodiscard]] float ace_prng_rand_in_range(
    std::uint32_t& state,
    float minimum,
    float maximum) noexcept;

// Integer counterpart of native dtsAce_PRNGRandInRange. The native routine
// uses the high half of an unsigned 32x32 product, not a floating-point
// interpolation.
[[nodiscard]] std::int32_t ace_prng_rand_in_range_i32(
    std::uint32_t& state,
    std::int32_t minimum,
    std::int32_t maximum) noexcept;

[[nodiscard]] AceStreamParseResult parse_ace_stream_prefix(
    const std::uint8_t* bytes,
    std::size_t size,
    bool stereo,
    bool predictive,
    std::uint32_t sample_rate,
    std::uint32_t bandwidth_mode,
    const AceStreamPrefixState* previous,
    AceStreamPrefixState& state,
    AceStreamPrefix& prefix) noexcept;

} // namespace dtsx
