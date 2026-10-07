#pragma once

#include <cstdint>
#include <array>
#include <vector>

namespace dtsx {

enum class AceMdctWindowSize : std::size_t {
    Size128 = 128U,
    Size256 = 256U,
    Size512 = 512U,
    Size1024 = 1024U,
};

[[nodiscard]] bool ace_mdct_set_window_size(
    std::size_t window_size,
    std::uint32_t& exponent) noexcept;

class AceMdctHistoryBank final {
public:
    static constexpr std::size_t kHistorySamples = 1024U;

    void reset(std::size_t channel_count) noexcept;
    [[nodiscard]] std::size_t channel_count() const noexcept;
    [[nodiscard]] std::int32_t* channel(std::size_t index) noexcept;
    [[nodiscard]] const std::int32_t* channel(
        std::size_t index) const noexcept;

    // Native DTSAceMdct state[0]/state[1] keeps the current and previous
    // window sizes for short/long transition handling.
    [[nodiscard]] std::size_t previous_transform_size() const noexcept;
    void set_transform_size(std::size_t size) noexcept;

    // DTSAceMdct_Inverse_Process keeps one 1024-sample float plane per
    // channel.  The plane is emitted as the current frame and only then
    // replaced by the freshly transformed scratch, which is the native
    // one-frame output delay.
    [[nodiscard]] float* float_plane(std::size_t index) noexcept;

    [[nodiscard]] bool overlap_add_q31(
        std::size_t channel_index,
        std::size_t history_offset,
        std::vector<std::int32_t>& transformed,
        const std::vector<std::int32_t>& low_window,
        const std::vector<std::int32_t>& high_window) noexcept;

private:
    std::vector<std::array<std::int32_t, kHistorySamples>> channels_;
    std::vector<std::array<float, kHistorySamples>> float_channels_;
    std::size_t previous_transform_size_ = 1024U;
};

[[nodiscard]] bool ace_mdct_window_coefficients(
    AceMdctWindowSize size,
    std::vector<std::int32_t>& coefficients) noexcept;

// Window halves in the exact address order consumed by
// DTS_ACE_MDCT_OverlapAdd: low starts at table[0], high walks the first half
// in reverse order.
[[nodiscard]] bool ace_mdct_overlap_window_halves(
    AceMdctWindowSize size,
    std::vector<std::int32_t>& low,
    std::vector<std::int32_t>& high) noexcept;

[[nodiscard]] bool ace_mdct_walsh_transform_reference(
    std::vector<std::int32_t>& values) noexcept;

// Literal Homatic DTS_ACE_MDCT_WalshTransform scheduler and packed radix
// stages on the 1024-word workspace.
[[nodiscard]] bool ace_mdct_walsh_native_reference(
    std::vector<std::int32_t>& values,
    std::uint32_t exponent) noexcept;

[[nodiscard]] bool ace_mdct_walsh_tcl_scalar_reference(
    std::vector<std::int32_t>& values,
    std::uint32_t exponent) noexcept;

// Literal first `v6 >= 16` pass of Homatic DTS_ACE_MDCT_WalshTransform.
// The caller supplies the native 1024-word workspace after bit reversal.
[[nodiscard]] bool ace_mdct_walsh_first_large_stage_native_reference(
    std::vector<std::int32_t>& workspace,
    std::uint32_t exponent) noexcept;

// DTS_ACE_MDCT_BitreversalPermutation: table-driven four-way swaps used by
// the native ACE transform (a3 = log2(transform size), 7..10).
[[nodiscard]] bool ace_mdct_bitreversal_permutation_reference(
    std::vector<std::int32_t>& values,
    std::uint32_t exponent) noexcept;

[[nodiscard]] bool ace_mdct_transform_ordering_reference(
    std::vector<std::int32_t>& values) noexcept;

// ARM NEON vqrdmulh_s32 used by the native MDCT butterflies.  This is the
// saturating, rounding, doubling-high multiply; keeping it explicit avoids
// relying on implementation-defined signed-shift behavior in callers.
[[nodiscard]] std::int32_t ace_qrdmulh_s32(
    std::int32_t left,
    std::int32_t right) noexcept;

// Native DTS_LTWIDLE/UTWIDLE tables extracted from the homatic ELF symbol
// bodies.  The tables are consumed by BackwardRotate; exposing them as a
// validated view keeps the future scalar port tied to the native data rather
// than regenerating twiddles from floating point.
[[nodiscard]] bool ace_mdct_idle_tables(
    std::uint32_t exponent,
    std::vector<std::int32_t>& ltwidle,
    std::vector<std::int32_t>& utwidle) noexcept;

// The first (8-sample) native BackwardRotate butterfly.  This is an exact
// scalar transcription of the homatic SIMD loop; later radix stages remain
// separate so they cannot silently be replaced by a different transform.
[[nodiscard]] bool ace_mdct_backward_rotate_initial_stage(
    std::vector<std::int32_t>& values,
    std::uint32_t exponent) noexcept;

// DTS_ACE_MDCT_ReverseOrdering: exact in-place reversal of the complete
// transform vector, applied after BackwardRotate.  Kept separate from the
// legacy float helper until the complete native inverse path is gated.
[[nodiscard]] bool ace_mdct_reverse_ordering_native_reference(
    std::vector<std::int32_t>& values) noexcept;

// Complete scalar transcription of DTS_ACE_MDCT_BackwardRotate, including
// the native middle radix stages and tail stage.  This is an isolated
// differential reference until its PCM output is gated against the native
// implementation.
[[nodiscard]] bool ace_mdct_backward_rotate_native_reference(
    std::vector<std::int32_t>& values,
    std::uint32_t exponent) noexcept;

// DTS_ACE_MDCT_TRANSFORM: native integer stage order on the 1024-sample
// workspace (the exponent selects the active swap/twiddle geometry)
// WalshTransform -> BackwardRotate -> ReverseOrdering.  This candidate is
// kept separate from the float PCM path until its output is differentially
// matched against the DLL.
[[nodiscard]] bool ace_mdct_native_transform_reference(
    std::vector<std::int32_t>& values,
    std::uint32_t exponent) noexcept;

// Sony x64/ARM64 float DTS_ACE_MDCT_TRANSFORM on the 1024-bin Inverse_Process
// workspace: BitreversalPermutation + Walsh (add/sub) + BackwardRotate with
// LTWIDLE/UTWIDLE as IEEE floats (Q31 tables / 2^31) + ReverseOrdering of
// the active prefix.  Kept behind DTSX_P2_NATIVE_MDCT_F32 until the
// Kinsetsu differential meets DCT-IV; not the Q31 candidate.
[[nodiscard]] bool ace_mdct_native_transform_f32(
    std::vector<float>& values,
    std::uint32_t exponent) noexcept;

// DTS_INVSQRTWINDOW_LENGTH from the native ACE MDCT state.  Returns zero for
// unsupported transform sizes.
[[nodiscard]] float ace_inverse_sqrt_window(
    std::size_t transform_size) noexcept;

// Final native DTSAceStreamDecoder_Process PCM scale (`0x3A800000`,
// 1/32768), applied after LTS/de-emphasis in the native pipeline.
void ace_apply_native_pcm_scale(std::vector<float>& samples) noexcept;

// DTSAceStreamDecoder_Sat_LeftShift: saturating left shift followed by the
// native round/quantize sequence (right 8, saturating left 8, right 8).
// Kept integer-only so it can be differentially gated before touching the
// float synthesis path.
void ace_sat_left_shift_native(
    std::vector<std::int32_t>& samples,
    std::uint32_t shift) noexcept;

// ETSI TS 103 491 clause 9.10.5: ACE inverse transform starts with a block
// DCT4. Kept as a reference stage until native ordering/scaling is gated.
[[nodiscard]] bool ace_dct4_reference(
    const std::vector<float>& input,
    std::vector<float>& output) noexcept;

// DTS_ACE_MDCT_ReverseOrdering: in-place reversal of the two half-blocks
// used by the native ACE MDCT core.
[[nodiscard]] bool ace_reverse_ordering_reference(
    std::vector<float>& values) noexcept;

// Scalar form of the native DTS_ACE_MDCT_OverlapAdd butterfly. Coefficients
// are Q31 window values; this keeps the fixed-point rounding order explicit
// until the native window tables are connected.
[[nodiscard]] bool ace_overlap_add_q31_reference(
    std::vector<std::int32_t>& history,
    std::vector<std::int32_t>& output,
    const std::vector<std::int32_t>& low_window,
    const std::vector<std::int32_t>& high_window) noexcept;

// Exact scalar transcription of the pairwise ARM body in
// DTS_ACE_MDCT_OverlapAdd. `transformed` is the complete N-sample block,
// `history` is the N/2-sample overlap area, and `window` is the native N-sample
// table (low half forward, high half addressed from its tail).
[[nodiscard]] bool ace_overlap_add_q31_native_pairwise_reference(
    std::vector<std::int32_t>& history,
    std::vector<std::int32_t>& transformed,
    const std::vector<std::int32_t>& window) noexcept;

// Sony x64/ARM64 float DTS_ACE_MDCT_OverlapAdd.  `block` is the native `a2`
// transform buffer and `history` the `a3` overlap plane; the routine reads and
// writes `block[0, overlap_size/2)` and `history[0, overlap_size/2)` only.
// `window` holds `overlap_size` coefficients: the low half is walked forward
// and the high half backwards from its tail.
[[nodiscard]] bool ace_overlap_add_f32_native(
    float* block,
    float* history,
    const float* window,
    std::size_t overlap_size) noexcept;

// Native transition form: `overlap_size` may be smaller than the current
// transform buffer on the first short-to-long block.
[[nodiscard]] bool ace_overlap_add_q31_native_sized_reference(
    std::vector<std::int32_t>& history,
    std::vector<std::int32_t>& transformed,
    const std::vector<std::int32_t>& window,
    std::size_t overlap_size) noexcept;





// Applies the ACE inverse-transform kernel to one native 128/256/512/1024
// coefficient block. Window/history overlap is intentionally a separate
// stateful stage; the caller performs native short-block deinterleaving.
[[nodiscard]] bool ace_inverse_transform_reference(
    const std::vector<float>& coefficients,
    std::vector<float>& samples) noexcept;

// TCL Inverse_Process kernel: float invsqrt (Sony IEEE table as 1/sqrt(N)),
// Q31 conversion, DTS_ACE_MDCT_TRANSFORM on the 1024-word workspace.
[[nodiscard]] bool ace_inverse_transform_to_q31(
    const std::vector<float>& coefficients,
    std::vector<std::int32_t>& samples) noexcept;

[[nodiscard]] bool ace_inverse_transform_packed_to_q31(
    const std::vector<float>& packed,
    std::size_t block_size,
    std::vector<std::int32_t>& samples) noexcept;

// Diagnostic integer ACE path: Q15 spectral input, native 1024-word
// workspace/order, and raw integer output for the existing PCM scale stage.
[[nodiscard]] bool ace_inverse_transform_native_candidate(
    const std::vector<float>& coefficients,
    std::vector<float>& samples) noexcept;

// DTSAceMdct_Inverse_Process short-window input gather.  Native stores one
// 1024-sample spectral buffer and gathers `block_count` transforms by taking
// coefficients at [block + n*block_count].
[[nodiscard]] bool ace_mdct_deinterleave_blocks(
    const std::vector<float>& packed,
    std::size_t block_size,
    std::vector<std::vector<float>>& blocks) noexcept;

[[nodiscard]] bool ace_inverse_transform_packed_blocks(
    const std::vector<float>& packed,
    std::size_t block_size,
    std::vector<float>& samples) noexcept;

[[nodiscard]] bool ace_inverse_transform_packed_blocks_native_candidate(
    const std::vector<float>& packed,
    std::size_t block_size,
    std::vector<float>& samples) noexcept;

// ETSI TS 103 491 Table 9-112. Generates the half-window used by the
// phase-shifted overlap-add operation.
[[nodiscard]] bool ace_make_window(
    std::size_t length,
    std::size_t transition,
    std::vector<float>& window) noexcept;

// Reference implementation of TS 103 491 Tables 9-111 and 9-115/116 for
// equal-size adjacent transforms.  Mixed short/long transitions require the
// native `do_single_iwindow` history layout and are rejected until gated.
[[nodiscard]] bool ace_windowed_imdct_reference(
    std::size_t length,
    std::size_t previous_transform,
    std::size_t current_transform,
    std::vector<float>& previous,
    std::vector<float>& current) noexcept;

} // namespace dtsx
