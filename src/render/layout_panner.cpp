#include "render/layout_panner.hpp"

#include "dtsx/speaker_mask.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dtsx_decode {
namespace {

struct SpeakerPosition final {
    std::uint32_t channel = 0;
    float azimuth = 0.0F;
    float elevation = 0.0F;
};

bool position_for_channel(
    const std::string& name,
    float& azimuth,
    float& elevation) noexcept {
    std::uint32_t speaker_mask = 0U;
    std::string_view canonical_name;
    return dtsx::standard_speaker_mask(name, speaker_mask)
        && dtsx::standard_speaker_coordinates(
            speaker_mask, azimuth, elevation, canonical_name);
}

PannerVector subtract(
    const PannerVector& left,
    const PannerVector& right) noexcept {
    return {
        left.x - right.x,
        left.y - right.y,
        left.z - right.z,
    };
}

PannerVector cross(
    const PannerVector& left,
    const PannerVector& right) noexcept {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

float dot(
    const PannerVector& left,
    const PannerVector& right) noexcept {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

bool make_native_hull_triplet(
    const std::vector<PannerVector>& points,
    const std::vector<std::uint32_t>& destinations,
    std::size_t first,
    std::size_t second,
    std::size_t third,
    PannerTriplet& triplet) noexcept {
    // libdtsx.so: dts_3d_hull_f32_t_initialize/sub_E7714.  Native does
    // not use the ordinary Cartesian convex-hull plane test.  A candidate
    // loudspeaker triplet must stay in one elevation hemisphere and every
    // relevant point, expressed in that triplet basis, must have a
    // coefficient sum no greater than 1 + the configured hull epsilon.
    constexpr float kHullEpsilon = 1.0e-7F;
    const std::array<PannerVector, 3U> speakers = {
        points[first], points[second], points[third]};
    const bool upper = std::all_of(
        speakers.begin(), speakers.end(), [=](const PannerVector& point) {
            return point.y >= -kHullEpsilon;
        });
    const bool lower = std::all_of(
        speakers.begin(), speakers.end(), [=](const PannerVector& point) {
            return point.y <= kHullEpsilon;
        });
    if (!upper && !lower) {
        return false;
    }
    const float determinant_value = dot(
        speakers[0U], cross(speakers[1U], speakers[2U]));
    if (std::fabs(determinant_value) <= kHullEpsilon
        || !make_panner_triplet(
            speakers,
            {destinations[first], destinations[second],
             destinations[third]},
            triplet)) {
        return false;
    }
    const bool strictly_upper = upper && !lower;
    const bool strictly_lower = lower && !upper;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const PannerVector& point = points[index];
        if ((strictly_upper && point.y < -kHullEpsilon)
            || (strictly_lower && point.y > kHullEpsilon)) {
            continue;
        }
        float coefficient_sum = 0.0F;
        for (std::size_t row = 0U; row < 3U; ++row) {
            coefficient_sum +=
                triplet.inverse[row][0U] * point.x
                + triplet.inverse[row][1U] * point.y
                + triplet.inverse[row][2U] * point.z;
        }
        if (coefficient_sum > 1.0F + kHullEpsilon) {
            return false;
        }
    }
    return true;
}

} // namespace

LayoutPanner::LayoutPanner(const ChannelLayout& layout)
    : real_channel_count_(
        static_cast<std::uint32_t>(layout.channels.size())) {
    std::vector<SpeakerPosition> speakers;
    for (std::size_t channel = 0;
         channel < layout.channels.size();
         ++channel) {
        float azimuth = 0.0F;
        float elevation = 0.0F;
        if (layout.channels[channel] == "LFE") {
            continue;
        }
        if (!position_for_channel(
                layout.channels[channel], azimuth, elevation)) {
            throw std::runtime_error(
                "layout contains a channel unsupported by object panner");
        }
        speakers.push_back({
            static_cast<std::uint32_t>(channel),
            azimuth,
            elevation,
        });
        real_speakers_.emplace_back(
            static_cast<std::uint32_t>(channel),
            panner_vector_from_degrees(azimuth, elevation));
        floor_only_ = floor_only_ && elevation == 0.0F;
    }
    floor_only_ = std::all_of(
        speakers.begin(),
        speakers.end(),
        [](const SpeakerPosition& speaker) {
            return speaker.elevation == 0.0F;
        });
    if (speakers.empty()) {
        throw std::runtime_error(
            "layout has no full-range channels");
    }
    if (speakers.size() == 1U) {
        panner_channel_count_ = real_channel_count_;
        return;
    }

    std::vector<PannerVector> points;
    std::vector<std::uint32_t> destinations;
    // libdtsx.so's virtual-auto initializer (sub_85F08/sub_86020 and
    // sub_86550) adds a +45/-45 degree virtual copy for each real speaker
    // when the corresponding upper/lower ring is absent.  The virtual
    // copies are mapped back to the same destination channel by the native
    // matrix; retaining that destination here gives the same accumulation
    // without exposing virtual channels in the output layout.
    const bool has_upper_ring = std::any_of(
        speakers.begin(), speakers.end(), [](const SpeakerPosition& speaker) {
            return speaker.elevation > 25.0F;
        });
    const bool has_lower_ring = std::any_of(
        speakers.begin(), speakers.end(), [](const SpeakerPosition& speaker) {
            return speaker.elevation < -25.0F;
        });
    const std::size_t virtual_ring_count =
        (!has_upper_ring ? speakers.size() : 0U)
        + (!has_lower_ring ? speakers.size() : 0U);
    points.reserve(speakers.size() + virtual_ring_count + 1U);
    destinations.reserve(speakers.size() + virtual_ring_count + 1U);
    std::uint32_t next_virtual_channel = real_channel_count_;
    const auto append_virtual_ring_point =
        [&](float azimuth, float elevation, std::uint32_t destination) {
            const std::uint32_t virtual_channel = next_virtual_channel++;
            points.push_back(panner_vector_from_degrees(
                azimuth, elevation));
            destinations.push_back(virtual_channel);
            virtual_ring_folds_.emplace_back(
                virtual_channel, destination);
            virtual_ring_azimuths_.push_back(azimuth);
        };
    if (floor_only_) {
        std::sort(
            speakers.begin(),
            speakers.end(),
            [](const SpeakerPosition& left,
               const SpeakerPosition& right) {
                return left.azimuth < right.azimuth;
            });
        for (const SpeakerPosition& speaker : speakers) {
            points.push_back(panner_vector_from_degrees(
                speaker.azimuth, speaker.elevation));
            destinations.push_back(speaker.channel);
        }
        if (!has_upper_ring) {
            for (const SpeakerPosition& speaker : speakers) {
                append_virtual_ring_point(
                    speaker.azimuth, 45.0F, speaker.channel);
            }
        }
        if (!has_lower_ring) {
            for (const SpeakerPosition& speaker : speakers) {
                append_virtual_ring_point(
                    speaker.azimuth, -45.0F, speaker.channel);
            }
        }
        if (speakers.size() == 2U
            && speakers.back().azimuth
                    - speakers.front().azimuth
                < 180.0F) {
            virtual_rear_channel_ =
                static_cast<std::int32_t>(next_virtual_channel++);
            virtual_rear_destinations_ = {
                speakers.front().channel,
                speakers.back().channel,
            };
            points.push_back(
                panner_vector_from_degrees(180.0F, 0.0F));
            destinations.push_back(real_channel_count_);
        }
        panner_channel_count_ = real_channel_count_
            + static_cast<std::uint32_t>(
                next_virtual_channel - real_channel_count_);
        for (std::size_t first = 0U; first < points.size(); ++first) {
            for (std::size_t second = first + 1U;
                 second < points.size();
                 ++second) {
                for (std::size_t third = second + 1U;
                     third < points.size();
                     ++third) {
                    PannerTriplet triplet;
                    if (make_native_hull_triplet(
                            points, destinations,
                            first, second, third, triplet)) {
                        triplets_.push_back(triplet);
                    }
                }
            }
        }
    } else {
        for (const SpeakerPosition& speaker : speakers) {
            points.push_back(panner_vector_from_degrees(
                speaker.azimuth, speaker.elevation));
            destinations.push_back(speaker.channel);
        }
        if (!has_upper_ring) {
            for (const SpeakerPosition& speaker : speakers) {
                if (std::fabs(speaker.elevation) <= 25.0F) {
                    append_virtual_ring_point(
                        speaker.azimuth, 45.0F, speaker.channel);
                }
            }
        }
        if (!has_lower_ring) {
            for (const SpeakerPosition& speaker : speakers) {
                if (std::fabs(speaker.elevation) <= 25.0F) {
                    append_virtual_ring_point(
                        speaker.azimuth, -45.0F, speaker.channel);
                }
            }
        }
        panner_channel_count_ = real_channel_count_
            + static_cast<std::uint32_t>(
                next_virtual_channel - real_channel_count_);
        for (std::size_t first = 0; first < points.size(); ++first) {
            for (std::size_t second = first + 1U;
                 second < points.size();
                 ++second) {
                for (std::size_t third = second + 1U;
                     third < points.size();
                     ++third) {
                    PannerTriplet triplet;
                    if (make_native_hull_triplet(
                            points, destinations,
                            first, second, third, triplet)) {
                        triplets_.push_back(triplet);
                    }
                }
            }
        }
    }
    if (triplets_.empty()) {
        throw std::runtime_error(
            "cannot construct object panner hull for layout");
    }
    virtual_ring_matrices_.reserve(
        virtual_ring_folds_.size());
    constexpr std::array<float, 3U> kMatrixOffsets = {
        -45.0F, 0.0F, 45.0F,
    };
    constexpr std::array<float, 3U> kMatrixWeights = {
        0.7071067811865475F, 1.0F, 0.7071067811865475F,
    };
    for (std::size_t virtual_index = 0U;
         virtual_index < virtual_ring_folds_.size();
         ++virtual_index) {
        std::vector<float> matrix(real_channel_count_, 0.0F);
        for (std::size_t source = 0U;
             source < kMatrixOffsets.size();
             ++source) {
            std::vector<float> power_gains;
            if (!pan_point_source_power(
                    panner_vector_from_degrees(
                        virtual_ring_azimuths_[virtual_index]
                            + kMatrixOffsets[source],
                        0.0F),
                    panner_channel_count_,
                    triplets_,
                    1.0e-7F,
                    power_gains)) {
                throw std::runtime_error(
                    "cannot construct native virtual-speaker fold matrix");
            }
            for (const auto& speaker : real_speakers_) {
                const std::size_t destination =
                    static_cast<std::size_t>(speaker.first);
                if (destination >= matrix.size()
                    || destination >= power_gains.size()) {
                    throw std::runtime_error(
                        "invalid virtual-speaker fold matrix");
                }
                const float contribution =
                    kMatrixWeights[source]
                    * power_gains[destination];
                matrix[destination] = std::sqrt(
                    matrix[destination] * matrix[destination]
                    + contribution * contribution);
            }
        }
        float normalizer = 0.0F;
        for (const float value : matrix) {
            normalizer += value * value;
        }
        normalizer = std::sqrt(normalizer);
        if (normalizer <= 0.0F) {
            throw std::runtime_error(
                "empty virtual-speaker fold matrix");
        }
        for (float& value : matrix) {
            value /= normalizer;
        }
        virtual_ring_matrices_.push_back(std::move(matrix));
    }
}

bool LayoutPanner::gains_q15(
    const dtsx::RendererCoordinates& coordinates,
    float width_degrees,
    float height_degrees,
    float rotation_degrees,
    bool snap_to_nearest_speaker,
    float snap_tolerance_degrees,
    bool preserve_spatial_separation,
    bool use_noncoherent_rendering,
    std::vector<std::int32_t>& gains) const noexcept {
    if (panner_channel_count_ == real_channel_count_
        && triplets_.empty()) {
        gains.assign(real_channel_count_, 0);
        gains.front() = 0x8000;
        return true;
    }
    if (snap_to_nearest_speaker
        && snap_tolerance_degrees > 0.0F) {
        const PannerVector source = panner_vector_from_degrees(
            coordinates.azimuth_degrees,
            coordinates.elevation_degrees);
        constexpr float kPi =
            3.14159265358979323846F;
        const float tolerance_radians =
            snap_tolerance_degrees * kPi / 180.0F;
        const float maximum_distance_squared =
            tolerance_radians * tolerance_radians * 0.25F;
        float closest_distance_squared = 1000000.0F;
        std::int32_t closest_channel = -1;
        for (const auto& speaker : real_speakers_) {
            const PannerVector difference =
                subtract(source, speaker.second);
            const float distance_squared =
                dot(difference, difference);
            if (distance_squared <= maximum_distance_squared
                && distance_squared < closest_distance_squared) {
                closest_distance_squared = distance_squared;
                closest_channel =
                    static_cast<std::int32_t>(speaker.first);
            }
        }
        if (closest_channel >= 0) {
            gains.assign(real_channel_count_, 0);
            gains[static_cast<std::size_t>(closest_channel)] =
                0x8000;
            return true;
        }
    }
    std::vector<float> floating;
    // libdtsx.so: sub_5F12C always sets source normalization mode 1.
    // dts_3d_virtual_auto_vector_base_panner_t_pan maps mode 1 to
    // constant-power normalization; preserve-spatial-separation is the
    // separate final argument controlling virtual-speaker folding.
    const PannerNormalization normalization =
        PannerNormalization::ConstantPower;
    const PannerNormalization hull_normalization =
        virtual_ring_folds_.empty() && virtual_rear_channel_ < 0
        ? normalization
        : PannerNormalization::Unnormalized;
    // The native auto panner keeps object elevation even for a floor-only
    // destination; its synthesized +/-45 degree rings provide the vertical
    // support.  Do not flatten the metadata coordinate before hull panning.
    const float elevation = coordinates.elevation_degrees;
    bool panned = width_degrees == 0.0F
        && height_degrees == 0.0F
        ? pan_point_source(
              panner_vector_from_degrees(
                  coordinates.azimuth_degrees, elevation),
              panner_channel_count_,
              triplets_,
              1.0e-7F,
              hull_normalization,
              floating)
        : pan_extended_source(
              coordinates.azimuth_degrees,
              elevation,
              width_degrees,
              height_degrees,
              rotation_degrees,
              panner_channel_count_,
              triplets_,
              1.0e-7F,
              hull_normalization,
              floating);
    if (!panned) {
        return false;
    }
    for (std::size_t fold_index = 0U;
         fold_index < virtual_ring_folds_.size();
         ++fold_index) {
        const auto& fold = virtual_ring_folds_[fold_index];
        const std::size_t virtual_channel =
            static_cast<std::size_t>(fold.first);
        const std::size_t destination =
            static_cast<std::size_t>(fold.second);
        if (virtual_channel >= floating.size()
            || destination >= floating.size()) {
            return false;
        }
        const float virtual_gain = floating[virtual_channel];
        if (preserve_spatial_separation) {
            floating[destination] = std::sqrt(
                floating[destination] * floating[destination]
                + virtual_gain * virtual_gain);
        } else {
            if (fold_index >= virtual_ring_matrices_.size()
                || virtual_ring_matrices_[fold_index].size()
                    != real_channel_count_) {
                return false;
            }
            for (std::size_t output = 0U;
                 output < real_channel_count_;
                 ++output) {
                const float contribution =
                    virtual_gain
                    * virtual_ring_matrices_[fold_index][output];
                floating[output] = std::sqrt(
                    floating[output] * floating[output]
                    + contribution * contribution);
            }
        }
        floating[virtual_channel] = 0.0F;
    }
    (void)use_noncoherent_rendering;
    if (virtual_rear_channel_ >= 0) {
        const float virtual_gain = floating[
            static_cast<std::size_t>(
                virtual_rear_channel_)];
        constexpr float kEqualPower = 0.7071067811865475F;
        for (const std::uint32_t destination :
             virtual_rear_destinations_) {
            const float folded = virtual_gain * kEqualPower;
            floating[destination] = std::sqrt(
                floating[destination] * floating[destination]
                + folded * folded);
        }
    }
    floating.resize(real_channel_count_);
    float normalizer = 0.0F;
    if (normalization == PannerNormalization::ConstantPower) {
        for (const float value : floating) {
            normalizer += value * value;
        }
        normalizer = std::sqrt(normalizer);
    } else {
        for (const float value : floating) {
            normalizer += std::fabs(value);
        }
    }
    if (normalizer <= 0.0F) {
        return false;
    }
    for (float& value : floating) {
        value = std::max(
            0.0F, std::min(1.0F, value / normalizer));
    }
    return quantize_panner_gains(floating, 15U, gains);
}

} // namespace dtsx_decode
