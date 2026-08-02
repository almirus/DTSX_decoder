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
    // visio-libdtsx.so.c: player renderer modes 0, 1 and 5 are three
    // independent native renderer instances.
    std::array<std::uint32_t, 3> rendered_block_counts_{};
    std::map<GainKey, NativeGainRamp> reverse_gain_state_;
    // visio-libdtsx.so.c: legacy reverse renderer modes 2, 3 and 4 own
    // independent smoothing/snap state.
    std::array<std::uint32_t, 3> reverse_block_counts_{};
};

} // namespace dtsx_decode
