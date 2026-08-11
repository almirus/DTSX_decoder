#include "dtsx/speaker_mask.hpp"

#include <array>

namespace dtsx {
namespace {

constexpr std::array<std::array<std::uint8_t, 2>, 20>
    kSpeakerActivityChannels = {{
        {{0U, 0U}},
        {{1U, 2U}},
        {{3U, 4U}},
        {{5U, 0U}},
        {{6U, 0U}},
        {{13U, 15U}},
        {{7U, 8U}},
        {{14U, 0U}},
        {{19U, 0U}},
        {{11U, 12U}},
        {{17U, 18U}},
        {{9U, 10U}},
        {{16U, 0U}},
        {{20U, 21U}},
        {{22U, 0U}},
        {{23U, 24U}},
        {{25U, 0U}},
        {{26U, 27U}},
        {{28U, 29U}},
        {{30U, 31U}},
    }};

constexpr std::array<std::uint8_t, 20>
    kSpeakerActivityChannelCounts = {
        1U, 2U, 2U, 1U, 1U, 2U, 2U, 1U, 1U, 2U,
        2U, 2U, 1U, 2U, 1U, 2U, 1U, 2U, 2U, 2U,
    };

struct SpeakerCoordinateEntry final {
    std::uint8_t bit;
    float azimuth;
    float elevation;
    std::string_view name;
};

// libdtsx.so: sub_DAE54 standard destination coordinates.
constexpr std::array<SpeakerCoordinateEntry, 18>
    kStandardSpeakerCoordinates = {{
        {0U, 0.0F, 0.0F, "FC"},
        {1U, -30.0F, 0.0F, "FL"},
        {2U, 30.0F, 0.0F, "FR"},
        {3U, -110.0F, 0.0F, "SL"},
        {4U, 110.0F, 0.0F, "SR"},
        {6U, 180.0F, 0.0F, "BC"},
        {7U, -150.0F, 0.0F, "BL"},
        {8U, 150.0F, 0.0F, "BR"},
        {9U, -90.0F, 0.0F, "LSS"},
        {10U, 90.0F, 0.0F, "RSS"},
        {11U, -60.0F, 0.0F, "LW"},
        {12U, 60.0F, 0.0F, "RW"},
        {13U, -45.0F, 45.0F, "TFL"},
        {14U, 0.0F, 45.0F, "TFC"},
        {15U, 45.0F, 45.0F, "TFR"},
        {16U, 180.0F, 45.0F, "TBC"},
        {23U, -135.0F, 45.0F, "TBL"},
        {24U, 135.0F, 45.0F, "TBR"},
    }};

} // namespace

std::uint32_t speaker_count_from_activity_mask(std::uint32_t mask) noexcept {
    // libdtsx.so: dtsGetNumSpeakersFrmSpeakerActMask, 0x2edc0.
    constexpr std::array<std::uint8_t, 20> kSpeakerCount = {
        1U, 2U, 2U, 1U, 1U, 2U, 2U, 1U, 1U, 2U,
        2U, 2U, 1U, 2U, 1U, 2U, 1U, 2U, 2U, 2U,
    };
    if (mask == 0U) {
        return 0U;
    }
    std::uint32_t result = 0;
    for (std::uint32_t index = 0; index < kSpeakerCount.size(); ++index) {
        if ((mask & (1U << index)) != 0U) {
            result = static_cast<std::uint8_t>(result + kSpeakerCount[index]);
        }
    }
    return result;
}

bool has_height_channels(std::uint32_t mask) noexcept {
    // libdtsx.so: DTSFrameScanner_HasHeightChannels, 0x31a70, tests
    // physical speaker bits. Public callers pass a speaker-activity mask,
    // so expand it before applying the native physical-height mask.
    for (const std::uint32_t speaker :
         expand_speaker_activity_mask(mask)) {
        if ((speaker & 0x01F8E000U) != 0U) {
            return true;
        }
    }
    return false;
}

std::vector<std::uint32_t> expand_speaker_activity_mask(
    std::uint32_t mask) {
    // libdtsx.so: sub_5F810/sub_5FE60, speaker activity expansion
    // through DTSX_SPKRACTNUMCH_TABLE and unk_1518B4.
    std::vector<std::uint32_t> result;
    result.reserve(speaker_count_from_activity_mask(mask));
    for (std::size_t activity = 0U;
         activity < kSpeakerActivityChannels.size();
         ++activity) {
        if ((mask & (1U << activity)) == 0U) {
            continue;
        }
        for (std::uint8_t channel = 0U;
             channel
                 < kSpeakerActivityChannelCounts[activity];
             ++channel) {
            result.push_back(
                1U << kSpeakerActivityChannels[activity][channel]);
        }
    }
    return result;
}

std::uint32_t speaker_mask_to_activity_mask(
    std::uint32_t mask) noexcept {
    // libdtsx.so: dtsxConvertSpkrMaskToSpkrActMask, 0x9e944.
    // A speaker-activity pair is present when either physical channel
    // belonging to the pair is present.
    std::uint32_t result = 0U;
    for (std::size_t activity = 0U;
         activity < kSpeakerActivityChannels.size();
         ++activity) {
        std::uint32_t channel_mask = 0U;
        for (std::uint8_t channel = 0U;
             channel < kSpeakerActivityChannelCounts[activity];
             ++channel) {
            channel_mask |=
                1U << kSpeakerActivityChannels[activity][channel];
        }
        if ((mask & channel_mask) != 0U) {
            result |= 1U << activity;
        }
    }
    return result;
}

bool standard_speaker_coordinates(
    std::uint32_t speaker_mask,
    float& azimuth_degrees,
    float& elevation_degrees,
    std::string_view& name) noexcept {
    for (const SpeakerCoordinateEntry& entry :
         kStandardSpeakerCoordinates) {
        if (speaker_mask == (1U << entry.bit)) {
            azimuth_degrees = entry.azimuth;
            elevation_degrees = entry.elevation;
            name = entry.name;
            return true;
        }
    }
    name = {};
    return false;
}

bool standard_speaker_name(
    std::uint32_t speaker_mask,
    std::string_view& name) noexcept {
    float azimuth_degrees = 0.0F;
    float elevation_degrees = 0.0F;
    if (standard_speaker_coordinates(
            speaker_mask,
            azimuth_degrees,
            elevation_degrees,
            name)) {
        return true;
    }

    // DTS-HD Master Audio Suite
    // L_C_R_Ls_Rs_Cs_Oh_Configurator identifies physical speaker bit 19 as
    // DTS_CHCFG_TOP_CENTER_SRRD.  The encoder calls this channel "Oh".
    if (speaker_mask == (1U << 19U)) {
        name = "Oh";
        return true;
    }

    name = {};
    return false;
}

bool standard_speaker_mask(
    std::string_view name,
    std::uint32_t& speaker_mask) noexcept {
    if (name == "LFE") {
        speaker_mask = 1U << 5U;
        return true;
    }
    for (const SpeakerCoordinateEntry& entry :
         kStandardSpeakerCoordinates) {
        if (name == entry.name) {
            speaker_mask = 1U << entry.bit;
            return true;
        }
    }
    speaker_mask = 0U;
    return false;
}

} // namespace dtsx
