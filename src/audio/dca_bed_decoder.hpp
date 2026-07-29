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

class DcaBedDecoder final {
public:
    DcaBedDecoder();
    ~DcaBedDecoder();

    DcaBedDecoder(const DcaBedDecoder&) = delete;
    DcaBedDecoder& operator=(const DcaBedDecoder&) = delete;

    void remember_core(const dtsx::ElementaryFrame& frame);

    [[nodiscard]] bool decode_extension(
        const dtsx::ElementaryFrame& frame,
        DcaDecodedBed& decoded);

    [[nodiscard]] const DcaDecodedBed& decoded_core() const noexcept {
        return decoded_core_;
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
    std::vector<std::uint8_t> pending_core_;
    DcaDecodedBed decoded_core_;
    std::string last_error_;
};

} // namespace dtsx_decode
