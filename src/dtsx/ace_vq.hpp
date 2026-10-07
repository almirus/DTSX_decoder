#pragma once

#include "dtsx/ace_bit_reader.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dtsx {

// Native counterpart: DTSAceBandDequant_GetUnSignedPyramidSize().  The
// firmware stores the same values in DTS_ACE_PYRAMID_VECTOR_MATRIX.
[[nodiscard]] std::uint32_t ace_unsigned_pyramid_size(
    std::uint32_t components,
    std::uint32_t pulses) noexcept;

// Native counterpart: is_split_required() / Table 10-16.  The caller must
// pass the actual ACE recursion depth, not an inferred band index.
[[nodiscard]] bool ace_vq_split_required(
    std::uint32_t allocated_bits,
    std::uint32_t components,
    std::uint32_t depth,
    bool high_resolution) noexcept;

// Native counterpart: quantization-level prefix in
// DTSAceBandDequant_RatioDecoderBetaCodec. `log_n_q4` is the native fixed
// point value passed to that routine (for example N=16 uses 64), not a
// floating approximation of log2(N). The result is either 1, an even level,
// or 256.
[[nodiscard]] std::uint32_t ace_vq_ratio_quantization_level(
    std::uint32_t allocated_bits,
    std::uint32_t components,
    std::int32_t log_n_q4) noexcept;

// Native counterpart: DTSAceBandDequant_SignedPyramidSplitSignGetKForN().
// `partition_bits` is the locally available codeword budget; `band_bits` is
// the parent band allocation. Both values are retained separately because the
// native rejects a leaf when their remaining-bit condition is not satisfied.
[[nodiscard]] std::uint32_t ace_vq_select_pulses(
    std::uint32_t components,
    std::uint32_t partition_bits,
    std::uint32_t band_bits,
    bool high_resolution) noexcept;

// Native counterpart: the unsigned-vector portion of
// DTSAceVQ_AlgUnquant_PulseDecode().  Coefficients are enumerated with the
// same descending first-component order used by the native a3 == 2 path.
[[nodiscard]] bool ace_unsigned_pyramid_unrank(
    std::uint32_t components,
    std::uint32_t pulses,
    std::uint32_t codeword,
    std::vector<std::int32_t>& vector) noexcept;

struct AceVqDecodeResult final {
    std::uint32_t bits_consumed = 0U;
    std::uint32_t nonzero_components = 0U;
    std::vector<std::int32_t> signed_pulses;
};

struct AceVqLeaf final {
    std::uint32_t component_offset = 0U;
    std::uint32_t components = 0U;
    std::uint32_t pulses = 0U;
    float gain = 0.0F;
    std::vector<std::int32_t> signed_pulses;
    std::vector<float> normalized_pulses;
};

// Native recursive `read_split_vector` control result. The coefficients are
// intentionally not exposed here: this stage is used by the ACE payload path
// to establish exact bit consumption before fixed-point dequant/MDCT are
// connected to product PCM.
struct AceVqPayloadResult final {
    std::uint32_t bits_consumed = 0U;
    std::uint32_t leaf_count = 0U;
    std::uint32_t split_count = 0U;
    std::vector<AceVqLeaf> leaves;
};

// Reconstruct one split-tree vector from the native-order leaves.  The leaf
// gains are propagated from the BETA table; this is still pre-scalar-dequant
// spectral data, not PCM.
[[nodiscard]] bool ace_vq_assemble_vector(
    const AceVqPayloadResult& payload,
    std::uint32_t components,
    std::vector<float>& vector) noexcept;

// Native counterpart: DTSAceBandDequant_StereoDecMidSideRevert().  `mid` and
// `side` are replaced in place by L/R and each resulting vector is normalized
// to unit L2 energy, exactly as the native post-rotation loop does.  This is
// the final operation of one stereo band before scalar dequantization, not a
// PCM synthesis step.
[[nodiscard]] bool ace_stereo_mid_side_revert(
    std::vector<float>& mid,
    std::vector<float>& side,
    float mid_norm,
    float side_norm) noexcept;

struct AceBandRatio final {
    std::int32_t beta_q15 = 0;
    float left_norm = 0.0F;
    float right_norm = 0.0F;
    std::int32_t mid_side_angle_q15 = 0;
};

// Native counterpart: dtsAce_ComputeNormsFromBeta(). Stereo M/S coding
// persists the signed beta between bands, so this path consumes no ratio bits.
[[nodiscard]] bool ace_band_ratio_from_beta(
    std::int32_t beta_q15,
    std::uint32_t angle_components,
    AceBandRatio& ratio) noexcept;

struct AceVqSplit final {
    AceBandRatio ratio{};
    std::uint32_t bits_consumed = 0U;
    std::uint32_t left_bits = 0U;
    std::uint32_t right_bits = 0U;
    bool final_partition = false;
};

// Native counterparts: DTSAceBandDequant_RatioDecoderBetaCodec() and
// dtsAce_ComputeNormsFromBeta(). The input alphabet is the exact native
// ratio quantizer cardinality, not a value inferred from a band index.
[[nodiscard]] bool ace_decode_band_ratio(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t channel_count,
    bool raw_codeword,
    AceBandRatio& ratio) noexcept;

[[nodiscard]] bool ace_vq_read_split(
    AceBitReader& source,
    std::uint32_t alpha,
    std::uint32_t channel_count,
    std::uint32_t allocated_bits,
    std::uint32_t remaining_bits,
    bool raw_codeword,
    AceVqSplit& split) noexcept;

// Native counterpart: DTSAceVQ_AlgUnquant_PulseDecode(), excluding the
// subsequent in-band energy smoothing.  The codeword uses the ACE bounded
// integer decoder and signs are consumed in reverse coefficient order.
[[nodiscard]] bool ace_vq_decode_signed_pyramid(
    AceBitReader& source,
    std::uint32_t components,
    std::uint32_t pulses,
    float gain,
    std::vector<float>& vector,
    AceVqDecodeResult& result) noexcept;

[[nodiscard]] bool ace_vq_consume_split_vector(
    AceBitReader& source,
    std::uint32_t allocated_bits,
    std::uint32_t remaining_bits,
    std::uint32_t components,
    std::uint32_t num_blocks,
    std::int32_t log_n_q4,
    std::uint32_t depth,
    bool high_resolution,
    AceVqPayloadResult& result) noexcept;

} // namespace dtsx
