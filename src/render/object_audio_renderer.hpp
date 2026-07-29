#pragma once

#include "app/object_frame_decoder.hpp"
#include "audio/layout.hpp"
#include "render/gain_interpolator.hpp"
#include "render/layout_panner.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <tuple>
#include <vector>

namespace dtsx_decode {

class ObjectAudioRenderer final {
public:
    explicit ObjectAudioRenderer(
        const ChannelLayout& layout,
        bool verbose = false);

    [[nodiscard]] bool render(
        const DecodedObjectAudioFrame& frame,
        std::vector<std::vector<std::int32_t>>& output);
    [[nodiscard]] bool remove_embedded_object_fold_down(
        DecodedObjectAudioFrame& frame);

private:
    using GainKey =
        std::tuple<std::uint32_t,
                   std::uint32_t,
                   std::uint32_t,
                   std::uint32_t>;

    ChannelLayout layout_;
    LayoutPanner panner_;
    bool verbose_ = false;
    bool reported_metadata_ = false;
    bool reported_reverse_metadata_ = false;
    std::int32_t reported_maximum_gain_ = 0;
    std::map<GainKey, NativeGainRamp> gain_state_;
    std::array<std::uint32_t, 2> rendered_block_counts_{};
    std::map<GainKey, NativeGainRamp> reverse_gain_state_;
    std::uint32_t reverse_block_count_ = 0U;
};

} // namespace dtsx_decode
