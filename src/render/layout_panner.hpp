#pragma once

#include "audio/layout.hpp"
#include "dtsx/object_coordinates.hpp"
#include "render/vector_base_panner.hpp"

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace dtsx_decode {

class LayoutPanner final {
public:
    explicit LayoutPanner(const ChannelLayout& layout);

    [[nodiscard]] bool gains_q15(
        const dtsx::RendererCoordinates& coordinates,
        float width_degrees,
        float height_degrees,
        float rotation_degrees,
        bool snap_to_nearest_speaker,
        float snap_tolerance_degrees,
        bool preserve_spatial_separation,
        bool use_noncoherent_rendering,
        std::vector<std::int32_t>& gains) const noexcept;

private:
    bool floor_only_ = false;
    std::uint32_t real_channel_count_ = 0;
    std::uint32_t panner_channel_count_ = 0;
    std::int32_t virtual_rear_channel_ = -1;
    std::array<std::uint32_t, 2> virtual_rear_destinations_{};
    std::vector<std::pair<std::uint32_t, std::uint32_t>>
        virtual_ring_folds_;
    std::vector<float> virtual_ring_azimuths_;
    std::vector<std::vector<float>> virtual_ring_matrices_;
    std::vector<std::pair<std::uint32_t, PannerVector>>
        real_speakers_;
    std::vector<PannerTriplet> triplets_;
};

} // namespace dtsx_decode
