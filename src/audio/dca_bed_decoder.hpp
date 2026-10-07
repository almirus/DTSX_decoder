#pragma once

#include "dtsx/frame_assembler.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct dcadec_context;

namespace dtsx_decode {

struct DcaDecodedBed final {
    std::vector<std::vector<std::int32_t>> channels;
    std::vector<std::uint32_t> channel_speaker_masks;
    std::uint32_t speaker_activity_mask = 0U;
    std::uint32_t sample_rate = 0U;
    std::uint32_t samples_per_channel = 0U;
};

struct DcaDecodedObjectAsset final {
    std::uint8_t asset_ordinal = 0U;
    std::uint8_t asset_index = 0U;
    std::uint16_t coding_components = 0U;
    std::vector<std::vector<std::int32_t>> channels;
    std::uint32_t sample_rate = 0U;
    std::uint32_t samples_per_channel = 0U;
};

struct DcaCoreStreamInfo final {
    std::uint32_t channels = 0U;
    std::uint32_t sample_rate = 0U;
    std::uint32_t source_pcm_bits = 0U;
    std::uint32_t speaker_activity_mask = 0U;
    std::uint32_t samples_per_frame = 0U;
    std::int32_t bit_rate = 0;
    std::int32_t profile = 0;
    std::int32_t matrix_encoding = 0;
    bool es_matrix_surround = false;
    bool embedded_6ch = false;
    bool valid = false;
};

struct DcaExtensionStreamInfo final {
    std::uint32_t channels = 0U;
    std::uint32_t sample_rate = 0U;
    std::uint32_t source_pcm_bits = 0U;
    std::uint32_t speaker_activity_mask = 0U;
    std::int32_t profile = 0;
    std::int32_t matrix_encoding = 0;
    bool embedded_stereo = false;
    bool embedded_6ch = false;
    bool valid = false;
};

class DcaBedDecoder final {
public:
    DcaBedDecoder();
    ~DcaBedDecoder();

    DcaBedDecoder(const DcaBedDecoder&) = delete;
    DcaBedDecoder& operator=(const DcaBedDecoder&) = delete;

    void remember_core(const dtsx::ElementaryFrame& frame);

    [[nodiscard]] bool decode_extension(
        const dtsx::ElementaryFrame& frame,
        DcaDecodedBed& decoded,
        bool require_dtsx_71 = true,
        std::vector<DcaDecodedObjectAsset>* object_assets = nullptr);

    [[nodiscard]] const DcaDecodedBed& decoded_core() const noexcept {
        return decoded_core_;
    }

    [[nodiscard]] const DcaCoreStreamInfo& core_stream_info() const noexcept {
        return core_stream_info_;
    }

    [[nodiscard]] const DcaExtensionStreamInfo&
    extension_stream_info() const noexcept {
        return extension_stream_info_;
    }

    [[nodiscard]] const std::string& last_error() const noexcept {
        return last_error_;
    }

private:
    struct ContextDeleter final {
        void operator()(dcadec_context* context) const noexcept;
    };

    std::unique_ptr<dcadec_context, ContextDeleter> context_;
    std::unique_ptr<dcadec_context, ContextDeleter> core_context_;
    std::unique_ptr<dcadec_context, ContextDeleter> core_probe_context_;
    std::vector<std::unique_ptr<dcadec_context, ContextDeleter>>
        object_asset_contexts_;
    std::vector<std::uint8_t> pending_core_;
    DcaDecodedBed decoded_core_;
    DcaCoreStreamInfo core_stream_info_;
    DcaExtensionStreamInfo extension_stream_info_;
    std::string last_error_;
};

} // namespace dtsx_decode
