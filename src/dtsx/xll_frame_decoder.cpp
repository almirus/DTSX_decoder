#include "dtsx/xll_frame_decoder.hpp"

#include "bitstream/dtsx_word_buffer.hpp"
#include "dtsx/crc16.hpp"
#include "dtsx/speaker_mask.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

namespace dtsx {
namespace {

static constexpr std::array<std::int32_t, 241>
    kXllDownmixCoefficientTable = {
        0, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62,
        65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 116, 123,
        130, 138, 146, 155, 164, 174, 184, 195, 207, 219, 232, 246,
        260, 276, 292, 309, 328, 347, 368, 389, 413, 437, 463, 490,
        519, 550, 583, 617, 654, 693, 734, 777, 823, 872, 924, 978,
        1036, 1066, 1098, 1130, 1163, 1197, 1232, 1268, 1305, 1343,
        1382, 1422, 1464, 1506, 1550, 1596, 1642, 1690, 1740, 1790,
        1843, 1896, 1952, 2009, 2068, 2128, 2190, 2254, 2320, 2388,
        2457, 2529, 2603, 2679, 2757, 2838, 2920, 3006, 3093, 3184,
        3277, 3372, 3471, 3572, 3677, 3784, 3894, 4008, 4125, 4246,
        4370, 4497, 4629, 4764, 4903, 5046, 5193, 5345, 5501, 5662,
        5827, 5912, 5997, 6084, 6172, 6262, 6353, 6445, 6538, 6633,
        6729, 6827, 6925, 7026, 7128, 7231, 7336, 7442, 7550, 7659,
        7771, 7883, 7997, 8113, 8231, 8350, 8471, 8594, 8719, 8845,
        8973, 9103, 9235, 9369, 9505, 9643, 9783, 9924, 10068, 10214,
        10362, 10512, 10665, 10819, 10976, 11135, 11297, 11460, 11627,
        11795, 11966, 12139, 12315, 12494, 12675, 12859, 13045, 13234,
        13426, 13621, 13818, 14018, 14222, 14428, 14637, 14849, 15064,
        15283, 15504, 15729, 15957, 16188, 16423, 16661, 16902, 17147,
        17396, 17648, 17904, 18164, 18427, 18694, 18965, 19240, 19519,
        19802, 20089, 20380, 20675, 20975, 21279, 21587, 21900, 22218,
        22540, 22867, 23170, 23534, 23875, 24221, 24573, 24929, 25290,
        25657, 26029, 26406, 26789, 27177, 27571, 27970, 28376, 28787,
        29205, 29628, 30057, 30493, 30935, 31383, 31838, 32300, 32768,
    };

static constexpr std::array<std::int32_t, 201>
    kXllInverseDownmixCoefficientTable = {
        6553600, 6186997, 5840902, 5514167, 5205710, 4914507,
        4639593, 4380059, 4135042, 3903731, 3685360, 3479204,
        3284581, 3100844, 2927386, 2763630, 2609035, 2463088,
        2325305, 2195230, 2072430, 2013631, 1956500, 1900990,
        1847055, 1794651, 1743733, 1694260, 1646190, 1599484,
        1554103, 1510010, 1467168, 1425542, 1385096, 1345798,
        1307615, 1270515, 1234468, 1199444, 1165413, 1132348,
        1100221, 1069005, 1038676, 1009206, 980573, 952752,
        925721, 899456, 873937, 849141, 825049, 801641,
        778897, 756798, 735326, 714463, 694193, 674497,
        655360, 636766, 618700, 601146, 584090, 567518,
        551417, 535772, 520571, 505801, 491451, 477507,
        463959, 450796, 438006, 425579, 413504, 401772,
        390373, 379297, 368536, 363270, 358080, 352964,
        347920, 342949, 338049, 333219, 328458, 323765,
        319139, 314579, 310084, 305654, 301287, 296982,
        292739, 288556, 284433, 280369, 276363, 272414,
        268522, 264685, 260904, 257176, 253501, 249879,
        246309, 242790, 239321, 235901, 232531, 229208,
        225933, 222705, 219523, 216386, 213295, 210247,
        207243, 204282, 201363, 198486, 195650, 192855,
        190099, 187383, 184706, 182066, 179465, 176901,
        174373, 171882, 169426, 167005, 164619, 162267,
        159948, 157663, 155410, 153190, 151001, 148844,
        146717, 144621, 142554, 140517, 138510, 136531,
        134580, 132657, 130762, 128893, 127052, 125236,
        123447, 121683, 119944, 118231, 116541, 114876,
        113235, 111617, 110022, 108450, 106901, 105373,
        103868, 102383, 100921, 99479, 98057, 96656,
        95275, 93914, 92682, 91249, 89946, 88660,
        87394, 86145, 84914, 83701, 82505, 81326,
        80164, 79019, 77890, 76777, 75680, 74598,
        73533, 72482, 71446, 70425, 69419, 68427,
        67450, 66486, 65536,
    };

std::int32_t wrapping_subtract(
    std::int32_t left,
    std::int32_t right) noexcept {
    return static_cast<std::int32_t>(
        static_cast<std::uint32_t>(left)
        - static_cast<std::uint32_t>(right));
}

std::int32_t wrapping_add(
    std::int32_t left,
    std::int32_t right) noexcept {
    return static_cast<std::int32_t>(
        static_cast<std::uint32_t>(left)
        + static_cast<std::uint32_t>(right));
}

std::int32_t saturate_24(
    std::int64_t value) noexcept {
    return static_cast<std::int32_t>(
        std::max<std::int64_t>(
            -0x800000LL,
            std::min<std::int64_t>(
                0x7FFFFFLL, value)));
}

constexpr std::uint32_t kAlternateD0Sync = 0xF14000D0U;
constexpr std::uint32_t kAlternateD1Sync = 0xF14000D1U;
constexpr std::uint32_t kAlternateD2Sync = 0xF14000D2U;
constexpr std::uint32_t kAlternateD3Sync = 0xF14000D3U;
constexpr std::uint32_t kAlternateD4Sync = 0xF14000D4U;
constexpr std::size_t kAlternateFrameSamples = 512U;
constexpr std::size_t kAlternateMaximumSegments = 8U;
constexpr std::size_t kAlternateMaximumInterstitial = 20U;
constexpr std::array<std::uint8_t, 6> kAlternateOuterSuffix = {
    0x03U, 0x34U, 0x38U, 0x8CU, 0x4FU, 0x00U};
constexpr std::array<std::uint8_t, 6> kAlternateInnerSuffix = {
    0x02U, 0x34U, 0x38U, 0x8CU, 0x4FU, 0x00U};

enum class AlternateProfile {
    D0,
    D1,
    D2,
    D3,
    D4,
};

struct AlternateGeometry final {
    std::uint32_t segments = 0U;
    std::uint8_t navigation_size_bits = 0U;
};

struct AlternateHeader final {
    std::size_t offset = 0U;
    std::size_t size = 0U;
    std::uint8_t channels = 0U;
};

struct AlternateLayout final {
    std::array<AlternateHeader, 2> headers;
    std::array<AlternateGeometry, 2> geometries;
};

bool read_bits_at(
    const std::vector<std::uint8_t>& bytes,
    std::size_t bit_offset,
    std::uint8_t bit_count,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (bit_count > 32U
        || bit_offset > bytes.size() * 8U
        || bit_count > bytes.size() * 8U - bit_offset) {
        return false;
    }
    for (std::uint8_t bit = 0U; bit < bit_count; ++bit) {
        const std::size_t absolute = bit_offset + bit;
        value = (value << 1U)
            | ((bytes[absolute >> 3U]
                >> (7U - (absolute & 7U))) & 1U);
    }
    return true;
}

bool valid_crc_bytes(
    const std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    std::size_t size) {
    if (offset > bytes.size() || size > bytes.size() - offset) {
        return false;
    }
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t index = offset; index < offset + size; ++index) {
        crc ^= static_cast<std::uint16_t>(bytes[index]) << 8U;
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = static_cast<std::uint16_t>(
                (crc & 0x8000U) != 0U
                ? (crc << 1U) ^ 0x1021U
                : crc << 1U);
        }
    }
    return crc == 0U;
}

bool alternate_geometry_at(
    const std::vector<std::uint8_t>& control,
    std::size_t bit_offset,
    AlternateGeometry& geometry) noexcept {
    std::uint32_t segment_log2 = 0U;
    std::uint32_t segment_sample_log2 = 0U;
    std::uint32_t navigation_size_minus_one = 0U;
    if (!read_bits_at(control, bit_offset, 4U, segment_log2)
        || !read_bits_at(
            control, bit_offset + 4U, 4U, segment_sample_log2)
        || !read_bits_at(
            control,
            bit_offset + 8U,
            5U,
            navigation_size_minus_one)
        || segment_log2 >= 32U
        || segment_sample_log2 >= 32U) {
        return false;
    }
    const std::uint32_t segments = 1U << segment_log2;
    const std::uint32_t segment_samples =
        1U << segment_sample_log2;
    const std::uint32_t navigation_size_bits =
        navigation_size_minus_one + 1U;
    if (segments > kAlternateMaximumSegments
        || segments * segment_samples != kAlternateFrameSamples
        || navigation_size_bits < 4U
        || navigation_size_bits > 20U) {
        return false;
    }
    geometry.segments = segments;
    geometry.navigation_size_bits = static_cast<std::uint8_t>(
        navigation_size_bits);
    return true;
}

bool alternate_unique_geometry(
    const std::vector<std::uint8_t>& control,
    std::size_t first_bit,
    std::size_t last_bit,
    std::size_t& selected_bit,
    AlternateGeometry& selected_geometry) noexcept {
    bool found = false;
    for (std::size_t bit = first_bit; bit <= last_bit; ++bit) {
        AlternateGeometry candidate;
        if (!alternate_geometry_at(control, bit, candidate)) {
            continue;
        }
        if (found) {
            return false;
        }
        found = true;
        selected_bit = bit;
        selected_geometry = candidate;
    }
    return found;
}

bool alternate_header_at(
    const std::vector<std::uint8_t>& payload,
    std::size_t byte_offset,
    AlternateHeader& header) {
    if (byte_offset > payload.size()) {
        return false;
    }
    std::size_t bit = byte_offset * 8U;
    std::uint32_t header_size_minus_one = 0U;
    std::uint32_t channels_minus_one = 0U;
    if (!read_bits_at(payload, bit, 10U, header_size_minus_one)
        || !read_bits_at(payload, bit + 10U, 4U, channels_minus_one)) {
        return false;
    }
    const std::uint32_t channels = channels_minus_one + 1U;
    if (channels > 8U) {
        return false;
    }
    bit += 14U + channels;
    std::uint32_t pcm_minus_one = 0U;
    std::uint32_t storage_minus_one = 0U;
    std::uint32_t frequency_index = 0U;
    std::uint32_t frequency_modifier = 0U;
    std::uint32_t replacement_set = 0U;
    if (!read_bits_at(payload, bit, 5U, pcm_minus_one)
        || !read_bits_at(payload, bit + 5U, 5U, storage_minus_one)
        || !read_bits_at(payload, bit + 10U, 4U, frequency_index)
        || !read_bits_at(payload, bit + 14U, 2U, frequency_modifier)
        || !read_bits_at(payload, bit + 16U, 2U, replacement_set)) {
        return false;
    }
    const std::size_t header_size = header_size_minus_one + 1U;
    const std::uint32_t pcm = pcm_minus_one + 1U;
    const std::uint32_t storage = storage_minus_one + 1U;
    if (header_size > payload.size() - byte_offset
        || pcm > storage
        || (storage != 16U && storage != 20U && storage != 24U)
        || frequency_index != 12U
        || frequency_modifier != 0U
        || replacement_set != 0U
        || !valid_crc_bytes(payload, byte_offset, header_size)) {
        return false;
    }
    header.offset = byte_offset;
    header.size = header_size;
    header.channels = static_cast<std::uint8_t>(channels);
    return true;
}

bool parse_alternate_layout(
    const XllExtension& extension,
    std::vector<std::uint8_t>& payload,
    AlternateLayout& layout) {
    AlternateProfile profile;
    switch (extension.sync_word) {
    case kAlternateD0Sync:
        profile = AlternateProfile::D0;
        break;
    case kAlternateD1Sync:
        profile = AlternateProfile::D1;
        break;
    case kAlternateD2Sync:
        profile = AlternateProfile::D2;
        break;
    case kAlternateD3Sync:
        profile = AlternateProfile::D3;
        break;
    case kAlternateD4Sync:
        profile = AlternateProfile::D4;
        break;
    default:
        return false;
    }
    payload.clear();
    payload.reserve(extension.payload.size() + 4U);
    payload.push_back(static_cast<std::uint8_t>(
        extension.sync_word >> 24U));
    payload.push_back(static_cast<std::uint8_t>(
        extension.sync_word >> 16U));
    payload.push_back(static_cast<std::uint8_t>(
        extension.sync_word >> 8U));
    payload.push_back(static_cast<std::uint8_t>(extension.sync_word));
    payload.insert(
        payload.end(), extension.payload.begin(), extension.payload.end());

    // Arcam dts_uhd_chunk_parser (sub_84185004) bounds parsing by the
    // declared chunk length.  The leading CRC region is variable-sized;
    // it is not limited to the 48/49-byte forms seen in short samples.
    const std::size_t minimum_prefix = sizeof(std::uint32_t);
    std::size_t prefix_end = 0U;
    bool prefix_found = false;
    for (std::size_t candidate = minimum_prefix;
         candidate + kAlternateOuterSuffix.size() <= payload.size();
         ++candidate) {
        if (!std::equal(
                kAlternateOuterSuffix.begin(),
                kAlternateOuterSuffix.end(),
                payload.begin() + static_cast<std::ptrdiff_t>(candidate))
            || !valid_crc_bytes(payload, 0U, candidate)) {
            continue;
        }
        if (prefix_found) {
            return false;
        }
        prefix_found = true;
        prefix_end = candidate;
    }
    if (!prefix_found) {
        return false;
    }
    const std::size_t control_start =
        prefix_end + kAlternateOuterSuffix.size();
    if (control_start >= payload.size()) {
        return false;
    }
    const std::uint8_t tag = payload[control_start];
    const std::size_t control_size = tag == 0xB2U
        ? 7U
        : (tag >= 0xC2U && tag <= 0xC6U ? 8U : 0U);
    if (control_size == 0U
        || control_size > payload.size() - control_start) {
        return false;
    }
    const std::vector<std::uint8_t> outer_control(
        payload.begin() + static_cast<std::ptrdiff_t>(control_start),
        payload.begin() + static_cast<std::ptrdiff_t>(
            control_start + control_size));
    std::size_t common_bit = 0U;
    const std::size_t first_geometry_end =
        profile == AlternateProfile::D3
        ? 31U
        : profile == AlternateProfile::D4 ? 26U : 25U;
    if (!alternate_unique_geometry(
            outer_control,
            18U,
            first_geometry_end,
            common_bit,
            layout.geometries[0])) {
        return false;
    }
    const std::size_t first_header_offset =
        control_start + control_size;
    if (!alternate_header_at(
            payload, first_header_offset, layout.headers[0])) {
        return false;
    }
    // Arcam dts_object_decoder.c sub_84167A80 (0x84167BEC..0x84167C28)
    // takes the channel count from the associated XLL decoder.  In
    // particular, D0 is not a one-channel profile: valid D0 streams carry
    // either one point-source waveform or a three-channel multi-point source.
    // The sync suffix therefore cannot be used as a channel-count
    // discriminator.
    if (common_bit < 14U) {
        return false;
    }
    const std::uint8_t field_width = static_cast<std::uint8_t>(
        common_bit - 14U);
    std::uint32_t encoded_span = 0U;
    if (!read_bits_at(
            outer_control, 9U, field_width, encoded_span)) {
        return false;
    }
    const std::size_t nominal =
        static_cast<std::size_t>(encoded_span) * 2U
        + control_start + 12U;
    bool second_found = false;
    for (const std::size_t candidate : {
             nominal,
             nominal == 0U ? nominal : nominal - 1U}) {
        AlternateHeader header;
        if (!alternate_header_at(payload, candidate, header)
            || header.channels != 4U) {
            continue;
        }
        if (second_found) {
            return false;
        }
        second_found = true;
        layout.headers[1] = header;
    }
    if (!second_found) {
        return false;
    }
    const std::size_t search_start =
        layout.headers[1].offset > 24U
        ? layout.headers[1].offset - 24U
        : 0U;
    std::size_t inner_suffix = 0U;
    bool inner_found = false;
    for (std::size_t candidate = search_start;
         candidate + kAlternateInnerSuffix.size()
             <= layout.headers[1].offset;
         ++candidate) {
        if (std::equal(
                kAlternateInnerSuffix.begin(),
                kAlternateInnerSuffix.end(),
                payload.begin() + static_cast<std::ptrdiff_t>(candidate))) {
            inner_found = true;
            inner_suffix = candidate;
        }
    }
    if (!inner_found) {
        return false;
    }
    const std::size_t inner_start =
        inner_suffix + kAlternateInnerSuffix.size();
    const std::size_t inner_size =
        layout.headers[1].offset - inner_start;
    if (inner_size < 8U || inner_size > 9U) {
        return false;
    }
    const std::vector<std::uint8_t> inner_control(
        payload.begin() + static_cast<std::ptrdiff_t>(inner_start),
        payload.begin() + static_cast<std::ptrdiff_t>(
            layout.headers[1].offset));
    std::size_t unused_bit = 0U;
    const std::size_t second_geometry_start =
        profile == AlternateProfile::D0 ? 18U : 19U;
    return alternate_unique_geometry(
        inner_control,
        second_geometry_start,
        26U,
        unused_bit,
        layout.geometries[1]);
}

bool decode_alternate_channel_set(
    const std::vector<std::uint8_t>& payload,
    const AlternateHeader& alternate_header,
    const AlternateGeometry& geometry,
    std::size_t boundary,
    XllChannelSetDecoder& decoder,
    XllChannelParameters& parameters,
    std::vector<std::vector<std::int32_t>>& output) {
    output.clear();
    if (boundary > payload.size()
        || alternate_header.offset > boundary
        || alternate_header.size
               > boundary - alternate_header.offset
        || geometry.segments == 0U
        || kAlternateFrameSamples % geometry.segments != 0U) {
        return false;
    }
    XllCommonHeader common;
    common.channel_set_count = 1U;
    common.segments_per_frame = geometry.segments;
    common.samples_per_segment = static_cast<std::uint32_t>(
        kAlternateFrameSamples / geometry.segments);
    common.segment_size_bits = geometry.navigation_size_bits;
    common.band_crc_present = 0U;
    common.scalable_lsb = false;
    common.channel_set_header_size_bits = 1U;

    bitstream::WordBuffer words(payload, false);
    bitstream::Cursor source = words.cursor();
    source.fast_forward(static_cast<std::int32_t>(
        8U * alternate_header.offset));
    XllChannelSetHeader header;
    if (!unpack_xll_primary_channel_set_header(
            source,
            common,
            header,
            false,
            0U,
            nullptr,
            2U)
        || header.probe.channel_count != alternate_header.channels
        || header.probe.channel_mask
               != ((1U << alternate_header.channels) - 1U)
        || header.bands.size() != 1U) {
        return false;
    }
    XllNavigationTable navigation;
    if (!unpack_xll_navigation_table(
            source,
            geometry.navigation_size_bits,
            geometry.segments,
            {1U},
            navigation)
        || !source.valid()) {
        return false;
    }
    const bitstream::Cursor audio_start = source;
    std::size_t audio_bytes = 0U;
    for (const XllNavigationEntry& entry : navigation.entries) {
        audio_bytes = (std::max)(
            audio_bytes,
            static_cast<std::size_t>(entry.byte_offset)
                + entry.size_bytes);
    }
    const std::size_t navigation_end =
        alternate_header.offset + alternate_header.size
        + navigation.byte_size;
    if (navigation_end > boundary
        || audio_bytes > boundary - navigation_end
        || boundary - navigation_end - audio_bytes
               > kAlternateMaximumInterstitial) {
        return false;
    }
    output.assign(
        alternate_header.channels,
        std::vector<std::int32_t>{});
    const XllChannelSetBand& band = header.bands.front();
    std::vector<std::uint8_t> adaptive_orders;
    adaptive_orders.reserve(band.prediction.size());
    for (const XllChannelPrediction& prediction : band.prediction) {
        adaptive_orders.push_back(static_cast<std::uint8_t>(
            prediction.adaptive_reflection_coefficients.size()));
    }
    for (std::uint32_t segment = 0U;
         segment < geometry.segments;
         ++segment) {
        const XllNavigationEntry* entry = navigation.find(
            0U, segment, 0U);
        if (entry == nullptr || entry->size_bytes == 0U) {
            return false;
        }
        bitstream::Cursor segment_source = audio_start;
        segment_source.fast_forward(static_cast<std::int32_t>(
            8U * entry->byte_offset));
        segment_source = segment_source.limited(
            8U * entry->size_bytes);
        if (!unpack_xll_channel_parameters(
                segment_source,
                segment,
                header.probe.parameter_bits,
                adaptive_orders,
                parameters)) {
            return false;
        }
        XllDecodedChannelSet decoded;
        if (!decoder.decode_msb_segment(
                segment_source,
                segment,
                common.samples_per_segment,
                parameters,
                band.prediction,
                band.channel_order,
                0U,
                0U,
                band.joint_pairs,
                decoded)
            || !segment_source.valid()
            || segment_source.remaining_bits() > 32U
            || decoded.channels.size() != output.size()) {
            return false;
        }
        for (std::size_t channel = 0U;
             channel < output.size();
             ++channel) {
            output[channel].insert(
                output[channel].end(),
                decoded.channels[channel].begin(),
                decoded.channels[channel].end());
        }
    }
    const std::uint8_t shift = header.probe.bit_depth < 24U
        ? static_cast<std::uint8_t>(24U - header.probe.bit_depth)
        : 0U;
    for (auto& channel : output) {
        if (channel.size() != kAlternateFrameSamples) {
            return false;
        }
        for (std::int32_t& sample : channel) {
            sample = saturate_24(
                static_cast<std::int64_t>(sample) << shift);
        }
    }
    return true;
}

bool decode_alternate_extension(
    const XllExtension& extension,
    std::array<XllChannelSetDecoder, 2>& decoders,
    std::array<XllChannelParameters, 2>& parameters,
    std::vector<std::vector<std::int32_t>>& output) {
    output.clear();
    std::vector<std::uint8_t> payload;
    AlternateLayout layout;
    if (!parse_alternate_layout(extension, payload, layout)) {
        return false;
    }
    std::array<std::vector<std::vector<std::int32_t>>, 2>
        decoded_sets;
    if (!decode_alternate_channel_set(
            payload,
            layout.headers[0],
            layout.geometries[0],
            layout.headers[1].offset,
            decoders[0],
            parameters[0],
            decoded_sets[0])) {
        return false;
    }
    if (!decode_alternate_channel_set(
            payload,
            layout.headers[1],
            layout.geometries[1],
            payload.size(),
            decoders[1],
            parameters[1],
            decoded_sets[1])) {
        return false;
    }
    for (auto& set : decoded_sets) {
        for (auto& channel : set) {
            output.push_back(std::move(channel));
        }
    }
    return !output.empty();
}

std::int32_t rounded_multiply(
    std::int32_t left,
    std::int32_t right,
    std::uint8_t fractional_bits) noexcept {
    const std::int64_t product =
        static_cast<std::int64_t>(left) * right;
    const std::int64_t rounded =
        (product
         + (std::int64_t{1} << (fractional_bits - 1U)))
        >> fractional_bits;
    return static_cast<std::int32_t>(
        std::max<std::int64_t>(
            std::numeric_limits<std::int32_t>::min(),
            std::min<std::int64_t>(
                std::numeric_limits<std::int32_t>::max(),
                rounded)));
}

bool decode_xll_downmix_coefficients(
    const XllChannelSetHeader& header,
    std::vector<std::int32_t>& coefficients) {
    // libdtsx.so: dtsxDecoderLookUpLLESDownMixCoefArray,
    // 0xb4ddc..0xb50fc.
    coefficients.clear();
    if (!header.external_downmix_coefficients.empty()) {
        if (header.external_downmix_coefficients.size()
            != static_cast<std::size_t>(
                   header.downmix_row_count)
                   * header.probe.channel_count) {
            return false;
        }
        coefficients = header.external_downmix_coefficients;
        return true;
    }
    if (!header.downmix_coefficients_present
        || header.downmix_values_per_row
               != static_cast<std::uint32_t>(
                      header.probe.channel_count)
                      + (header.primary_channel_set ? 0U : 1U)
        || header.downmix_coefficients.size()
               != static_cast<std::size_t>(
                      header.downmix_row_count)
                      * header.downmix_values_per_row) {
        return false;
    }
    coefficients.reserve(
        static_cast<std::size_t>(header.downmix_row_count)
        * header.probe.channel_count);
    std::size_t raw_offset = 0U;
    for (std::uint32_t row = 0U;
         row < header.downmix_row_count;
         ++row) {
        if (header.primary_channel_set) {
            for (std::uint32_t channel = 0U;
                 channel < header.probe.channel_count;
                 ++channel) {
                const std::uint16_t code =
                    header.downmix_coefficients[raw_offset++];
                const std::uint8_t table_code =
                    static_cast<std::uint8_t>(code);
                if (table_code == 0U) {
                    coefficients.push_back(0);
                    continue;
                }
                const std::size_t table_index =
                    static_cast<std::size_t>(table_code - 1U);
                if (table_index
                    >= kXllDownmixCoefficientTable.size()) {
                    return false;
                }
                const std::int32_t coefficient_sign =
                    (code & 0x100U) != 0U ? 1 : -1;
                coefficients.push_back(
                    kXllDownmixCoefficientTable[table_index]
                    * coefficient_sign);
            }
            continue;
        }
        const std::uint16_t scale_code =
            header.downmix_coefficients[raw_offset++];
        const std::uint8_t scale_index =
            static_cast<std::uint8_t>(scale_code);
        if (scale_index < 41U) {
            return false;
        }
        const std::size_t inverse_index =
            static_cast<std::size_t>(scale_index - 41U);
        if (inverse_index
            >= kXllInverseDownmixCoefficientTable.size()) {
            return false;
        }
        const std::int32_t scale_sign =
            (scale_code & 0x100U) != 0U ? 1 : -1;
        const std::int32_t inverse_scale =
            scale_sign
            * kXllInverseDownmixCoefficientTable[
                  inverse_index];
        for (std::uint32_t channel = 0U;
             channel < header.probe.channel_count;
             ++channel) {
            const std::uint16_t code =
                header.downmix_coefficients[raw_offset++];
            const std::uint8_t table_code =
                static_cast<std::uint8_t>(code);
            if (table_code == 0U) {
                coefficients.push_back(0);
                continue;
            }
            const std::size_t table_index =
                static_cast<std::size_t>(table_code - 1U);
            if (table_index
                >= kXllDownmixCoefficientTable.size()) {
                return false;
            }
            const std::int32_t coefficient_sign =
                (code & 0x100U) != 0U ? 1 : -1;
            coefficients.push_back(static_cast<std::int32_t>(
                ((static_cast<std::int64_t>(inverse_scale)
                  * kXllDownmixCoefficientTable[table_index]
                  + 0x8000)
                 >> 16)
                * coefficient_sign));
        }
    }
    return true;
}

bool decode_xll_inverse_scales(
    const XllChannelSetHeader& header,
    std::vector<std::int32_t>& inverse_scales) {
    inverse_scales.clear();
    if (!header.external_downmix_coefficients.empty()) {
        inverse_scales.assign(
            header.downmix_row_count, 65536);
        return true;
    }
    if (header.primary_channel_set) {
        return true;
    }
    if (header.downmix_scale_codes.size()
        != header.downmix_row_count) {
        return false;
    }
    inverse_scales.reserve(header.downmix_row_count);
    for (const std::uint16_t code :
         header.downmix_scale_codes) {
        const std::uint8_t scale_index =
            static_cast<std::uint8_t>(code);
        if (scale_index < 41U) {
            return false;
        }
        const std::size_t inverse_index =
            static_cast<std::size_t>(scale_index - 41U);
        if (inverse_index
            >= kXllInverseDownmixCoefficientTable.size()) {
            return false;
        }
        const std::int32_t sign =
            (code & 0x100U) != 0U ? 1 : -1;
        inverse_scales.push_back(
            sign
            * kXllInverseDownmixCoefficientTable[
                  inverse_index]);
    }
    return true;
}

std::int32_t shifted_xll_sample(
    std::int32_t sample,
    std::uint8_t shift) noexcept {
    return static_cast<std::int32_t>(
        static_cast<std::uint32_t>(sample) << shift);
}

void inverse_xll_downmix_samples(
    std::vector<std::int32_t>& destination,
    const std::vector<std::int32_t>& source,
    std::int32_t current_coefficient,
    std::int32_t previous_coefficient,
    std::uint8_t source_shift) {
    // libdtsx.so: sub_B5828 and
    // dtsxDecoderInv_LLESDownMixingCore, 0xb5828 and 0x10fb08.
    std::uint8_t interpolation_bits = 0U;
    std::uint64_t interpolation_rounding = 0U;
    if (destination.size() > 1U) {
        std::uint64_t interpolation_length = 1U;
        while (interpolation_length < destination.size()) {
            interpolation_length <<= 1U;
            ++interpolation_bits;
        }
        interpolation_rounding =
            interpolation_length >> 1U;
    }
    const std::int64_t coefficient_delta =
        static_cast<std::int64_t>(current_coefficient)
        - previous_coefficient;
    std::int64_t interpolation_accumulator = 0;
    for (std::size_t sample = 0U;
         sample < destination.size();
         ++sample) {
        const std::int32_t coefficient =
            coefficient_delta == 0
            ? current_coefficient
            : static_cast<std::int32_t>(
                  previous_coefficient
                  + ((interpolation_accumulator
                      + static_cast<std::int64_t>(
                          interpolation_rounding))
                     >> interpolation_bits));
        interpolation_accumulator += coefficient_delta;
        const std::int32_t shifted =
            shifted_xll_sample(source[sample], source_shift);
        const std::int32_t correction =
            static_cast<std::int32_t>(
                (static_cast<std::int64_t>(shifted)
                     * coefficient
                 + 0x4000)
                >> 15);
        destination[sample] =
            wrapping_subtract(destination[sample], correction);
    }
}

bool undo_xll_embedded_downmix(
    const std::vector<XllChannelSetHeader>& headers,
    std::vector<std::vector<
        std::vector<std::vector<std::int32_t>>>>& set_bands,
    std::vector<XllDecimatorHistory>& histories,
    const std::vector<std::vector<std::int32_t>>&
        previous_coefficients,
    std::vector<std::vector<std::int32_t>>&
        current_coefficients) {
    current_coefficients.assign(headers.size(), {});
    for (std::size_t current_set = 0U;
         current_set < headers.size();
         ++current_set) {
        const XllChannelSetHeader& current =
            headers[current_set];
        if (!current.embedded_downmix_present) {
            continue;
        }
        std::vector<std::int32_t> coefficients;
        if (!decode_xll_downmix_coefficients(
                current, coefficients)) {
            return false;
        }
        current_coefficients[current_set] = coefficients;
        const std::vector<std::int32_t>* previous = nullptr;
        if (current_set < previous_coefficients.size()
            && previous_coefficients[current_set].size()
                   == coefficients.size()) {
            previous = &previous_coefficients[current_set];
        }
        // dtsxDecoderInv_LLESDownMixing walks only non-primary
        // hierarchical channel sets.  The primary-set coefficient matrix is
        // retained for requested output downmixing, but is not subtracted
        // from its own decoded channels when the full hierarchy is active.
        if (current.primary_channel_set) {
            continue;
        }
        if (!current.hierarchical_channel_set) {
            return false;
        }
        std::size_t row = 0U;
        for (std::size_t prior_set = 0U;
             prior_set < current_set;
             ++prior_set) {
            const XllChannelSetHeader& prior =
                headers[prior_set];
            if (!prior.hierarchical_channel_set) {
                continue;
            }
            const std::uint8_t prior_depth =
                std::min<std::uint8_t>(
                    prior.probe.storage_bit_depth, 24U);
            const std::uint8_t current_depth =
                std::min<std::uint8_t>(
                    current.probe.storage_bit_depth, 24U);
            if (prior_depth < current_depth) {
                return false;
            }
            const std::uint8_t source_shift =
                static_cast<std::uint8_t>(
                    prior_depth - current_depth);
            for (std::size_t prior_channel = 0U;
                 prior_channel < prior.probe.channel_count;
                 ++prior_channel, ++row) {
                if (row >= current.downmix_row_count) {
                    return false;
                }
                for (std::size_t band = 0U;
                     band < current.bands.size();
                     ++band) {
                    if (!current.bands[band]
                             .embedded_downmix_present
                        || band >= set_bands[prior_set].size()
                        || band >= set_bands[current_set].size()
                        || prior_channel
                               >= set_bands[prior_set][band]
                                      .size()) {
                        continue;
                    }
                    auto& destination =
                        set_bands[prior_set][band][prior_channel];
                    for (std::size_t current_channel = 0U;
                         current_channel
                         < current.probe.channel_count;
                         ++current_channel) {
                        const auto& source =
                            set_bands[current_set][band]
                                     [current_channel];
                        if (source.size()
                            != destination.size()) {
                            return false;
                        }
                        inverse_xll_downmix_samples(
                            destination,
                            source,
                            coefficients[
                                row * current.probe.channel_count
                                + current_channel],
                            previous != nullptr
                            ? (*previous)[
                                  row
                                      * current.probe.channel_count
                                  + current_channel]
                            : coefficients[
                                  row
                                      * current.probe.channel_count
                                  + current_channel],
                            source_shift);
                    }
                }
                if (current.bands.size() > 1U
                    && current.bands[1U]
                           .embedded_downmix_present) {
                    if (prior_channel
                            >= histories[prior_set]
                                   .channels.size()) {
                        return false;
                    }
                    auto& destination_history =
                        histories[prior_set]
                            .channels[prior_channel];
                    for (std::size_t current_channel = 0U;
                         current_channel
                         < current.probe.channel_count;
                         ++current_channel) {
                        if (current_channel
                            >= histories[current_set]
                                   .channels.size()) {
                            return false;
                        }
                        const auto& source_history =
                            histories[current_set]
                                .channels[current_channel];
                        const std::int32_t coefficient =
                            coefficients[
                                row
                                    * current.probe.channel_count
                                + current_channel];
                        const std::int32_t history_coefficient =
                            previous != nullptr
                            ? (*previous)[
                                  row
                                      * current.probe.channel_count
                                  + current_channel]
                            : coefficient;
                        for (std::size_t sample = 0U;
                             sample
                             < destination_history.size();
                             ++sample) {
                            const std::int32_t shifted =
                                shifted_xll_sample(
                                    source_history[sample],
                                    source_shift);
                            const std::int32_t correction =
                                static_cast<std::int32_t>(
                                    (static_cast<std::int64_t>(
                                         shifted)
                                         * history_coefficient
                                     + 0x4000)
                                    >> 15);
                            destination_history[sample] =
                                wrapping_subtract(
                                    destination_history[sample],
                                    correction);
                        }
                    }
                }
            }
        }
        if (row != current.downmix_row_count) {
            return false;
        }
    }
    return true;
}

void reverse_abk_filter(
    std::vector<std::int32_t>& destination,
    const std::vector<std::int32_t>& source,
    std::int32_t coefficient,
    std::uint8_t fractional_bits,
    std::size_t destination_offset = 0U) {
    for (std::size_t sample = 0U;
         sample < source.size();
         ++sample) {
        destination[destination_offset + sample] =
            wrapping_subtract(
                destination[destination_offset + sample],
                rounded_multiply(
                    source[sample],
                    coefficient,
                    fractional_bits));
    }
}

bool reconstruct_two_frequency_bands(
    std::vector<std::int32_t> band_zero,
    std::vector<std::int32_t> band_one,
    const std::array<std::int32_t, 7>& history,
    std::vector<std::int32_t>& output) {
    // libdtsx.so: sub_B4B90, dtsxDecoderRevABksc,
    // dtsxDecoderRevABkSpPred and dtsxDecoderReconstruct_192_copy,
    // 0xb4b90 and 0x10f764..0x10fb58.
    if (band_zero.empty()
        || band_zero.size() != band_one.size()) {
        return false;
    }
    static constexpr std::array<std::int32_t, 3>
        kInitialCoefficients = {
            868669, -5931642, -1228483};
    static constexpr std::array<std::int32_t, 8>
        kBandZeroCoefficients = {
            -20577,
            122631,
            -393647,
            904476,
            -1696305,
            2825313,
            -4430736,
            6791313,
        };
    static constexpr std::array<std::int32_t, 8>
        kBandOneCoefficients = {
            41153,
            -245210,
            785564,
            -1788164,
            3259333,
            -5074941,
            6928550,
            -8204883,
        };

    reverse_abk_filter(
        band_zero, band_one, kInitialCoefficients[0], 22U);
    reverse_abk_filter(
        band_one, band_zero, kInitialCoefficients[1], 22U);
    reverse_abk_filter(
        band_zero, band_one, kInitialCoefficients[2], 22U);
    for (std::size_t sample = 0U;
         sample < band_one.size();
         ++sample) {
        band_one[sample] = wrapping_subtract(
            band_one[sample], band_zero[sample]);
    }

    std::vector<std::int32_t> band_zero_with_history(
        history.size() + band_zero.size());
    std::copy(
        history.begin(),
        history.end(),
        band_zero_with_history.begin());
    std::copy(
        band_zero.begin(),
        band_zero.end(),
        band_zero_with_history.begin()
            + static_cast<std::ptrdiff_t>(history.size()));
    std::size_t band_zero_offset = history.size();
    for (std::size_t stage = 0U;
         stage < kBandZeroCoefficients.size();
         ++stage) {
        reverse_abk_filter(
            band_zero_with_history,
            band_one,
            kBandZeroCoefficients[stage],
            23U,
            band_zero_offset);
        std::vector<std::int32_t> shifted_band_zero(
            band_one.size());
        std::copy_n(
            band_zero_with_history.begin()
                + static_cast<std::ptrdiff_t>(
                    band_zero_offset),
            band_one.size(),
            shifted_band_zero.begin());
        reverse_abk_filter(
            band_one,
            shifted_band_zero,
            kBandOneCoefficients[stage],
            23U);
        reverse_abk_filter(
            band_zero_with_history,
            band_one,
            kBandZeroCoefficients[stage],
            23U,
            band_zero_offset);
        --band_zero_offset;
    }

    output.resize(2U * band_one.size());
    for (std::size_t sample = 0U;
         sample < band_one.size();
         ++sample) {
        output[2U * sample] = band_one[sample];
        output[2U * sample + 1U] =
            band_zero_with_history[
                band_zero_offset + 1U + sample];
    }
    return true;
}

} // namespace

std::int32_t xll_metadata_downmix_coefficient(
    std::uint8_t code) noexcept {
    // libdtsx.so: sub_9FB34 indexes dtsx_dmixCoeffTable at
    // 4 * (code - 1), with codes zero and one both selecting zero.
    if (code <= 1U) {
        return 0;
    }
    const std::size_t index =
        4U * static_cast<std::size_t>(code - 1U);
    return index < kXllDownmixCoefficientTable.size()
        ? kXllDownmixCoefficientTable[index]
        : 0;
}

bool XllFrameDecoder::decode_msb_frame(
    bitstream::Cursor source,
    XllDecodedFrame& frame,
    bool one_to_one_mapping,
    const std::vector<XllSupplementalChannelSet>&
        supplemental_channel_sets,
    const std::vector<XllLossyBaseChannel>&
        lossy_base_channels) {
    // libdtsx.so: XLL ParseFrame/dtsx_initializeNavITable and
    // dtsxXLLDecodeChannelSet, 0xbb20c, 0xb4384, and 0xb8fc4.
    frame = {};
    last_error_.clear();
    std::vector<XllSupplementalChannelSet>
        effective_supplemental_channel_sets =
            supplemental_channel_sets;
    std::unique_ptr<bitstream::WordBuffer>
        standard_xll_x_words;
    if (!unpack_xll_common_header(source, frame.common)) {
        last_error_ = "common header";
        return false;
    }
    source = source.limited(
        8U * (frame.common.frame_size
              - frame.common.header_size));
    if (!source.valid()) {
        last_error_ = "frame payload bounds";
        return false;
    }

    const std::size_t raw_channel_set_count =
        frame.common.channel_set_count;
    std::vector<XllChannelSetHeader> raw_channel_sets(
        raw_channel_set_count);
    frame.channel_sets.clear();
    frame.channel_sets.reserve(
        raw_channel_set_count
        + effective_supplemental_channel_sets.size() + 1U);
    std::vector<std::uint8_t> raw_band_counts;
    raw_band_counts.reserve(raw_channel_set_count);
    std::vector<std::uint8_t> active_raw_channel_set_indices;
    active_raw_channel_set_indices.reserve(
        raw_channel_set_count
        + effective_supplemental_channel_sets.size() + 1U);
    std::vector<std::size_t> active_navigation_indices;
    active_navigation_indices.reserve(
        raw_channel_set_count
        + effective_supplemental_channel_sets.size() + 1U);
    std::vector<std::vector<std::uint32_t>>
        lossy_reference_speaker_masks_by_set;
    lossy_reference_speaker_masks_by_set.reserve(
        raw_channel_set_count
        + effective_supplemental_channel_sets.size() + 1U);
    std::uint32_t preceding_hierarchy_channels = 0U;
    for (std::size_t channel_set_index = 0U;
         channel_set_index < raw_channel_sets.size();
         ++channel_set_index) {
        XllChannelSetHeader& channel_set =
            raw_channel_sets[channel_set_index];
        const XllChannelSetHeader* previous_header =
            channel_set_index
                    < previous_raw_channel_set_headers_.size()
                ? &previous_raw_channel_set_headers_[
                      channel_set_index]
                : nullptr;
        if (!unpack_xll_primary_channel_set_header(
                source,
                frame.common,
                channel_set,
                one_to_one_mapping,
                preceding_hierarchy_channels,
                previous_header)) {
            frame = {};
            last_error_ = "channel-set header "
                + std::to_string(channel_set_index);
            return false;
        }
        raw_band_counts.push_back(
            channel_set.probe.frequency_band_count);
        const bool selected_replacement_set =
            channel_set.probe.replacement_set_index == 0U
            || channel_set.probe.default_replacement_set;
        if (!selected_replacement_set) {
            continue;
        }
        if (channel_set.hierarchical_channel_set) {
            preceding_hierarchy_channels +=
                channel_set.probe.channel_count;
        }
        active_raw_channel_set_indices.push_back(
            static_cast<std::uint8_t>(channel_set_index));
        active_navigation_indices.push_back(0U);
        frame.channel_sets.push_back(channel_set);
        if (channel_set.channel_mask_enabled) {
            lossy_reference_speaker_masks_by_set.push_back(
                expand_speaker_activity_mask(
                    speaker_mask_to_activity_mask(
                        channel_set.speaker_channel_mask)));
        } else {
            lossy_reference_speaker_masks_by_set.emplace_back(
                channel_set.probe.channel_count, 0U);
        }
    }
    if (frame.channel_sets.empty()) {
        frame = {};
        last_error_ = "no selected channel sets";
        return false;
    }

    if (!unpack_xll_navigation_table(
            source,
            frame.common.segment_size_bits,
            frame.common.segments_per_frame,
            raw_band_counts,
            frame.navigation)
        || !source.valid()) {
        frame = {};
        last_error_ = "navigation table";
        return false;
    }
    const bitstream::Cursor audio_data_start = source;
    std::uint64_t audio_bytes = 0U;
    for (const XllNavigationEntry& entry :
         frame.navigation.entries) {
        audio_bytes = std::max<std::uint64_t>(
            audio_bytes,
            static_cast<std::uint64_t>(entry.byte_offset)
                + entry.size_bytes);
    }
    if (audio_bytes > audio_data_start.remaining_bits() / 8U) {
        frame = {};
        last_error_ = "navigation audio bounds";
        return false;
    }
    std::vector<bitstream::Cursor> audio_data_starts;
    audio_data_starts.reserve(
        2U + effective_supplemental_channel_sets.size());
    audio_data_starts.push_back(audio_data_start);

    const std::uint64_t payload_bytes =
        frame.common.frame_size - frame.common.header_size;
    const std::uint64_t bytes_before_audio =
        payload_bytes - audio_data_start.remaining_bits() / 8U;
    const std::uint64_t audio_end =
        frame.common.header_size + bytes_before_audio + audio_bytes;
    const std::uint64_t extension_offset =
        (audio_end + 3U) & ~std::uint64_t{3U};
    if (extension_offset + 4U <= frame.common.frame_size) {
        bitstream::Cursor extension_source = audio_data_start;
        extension_source.fast_forward(static_cast<std::int32_t>(
            8U * (extension_offset
                  - frame.common.header_size
                  - bytes_before_audio)));
        frame.extension.frame_offset =
            static_cast<std::uint32_t>(extension_offset);
        frame.extension.sync_word =
            extension_source.extract_unsigned(32U);
        frame.extension.present =
            frame.extension.sync_word == 0x02000850U
            || frame.extension.sync_word == 0xF14000D1U
            || frame.extension.sync_word == 0xF14000D2U
            || frame.extension.sync_word == 0xF14000D3U
            || frame.extension.sync_word == 0xF14000D4U
            || frame.extension.sync_word == 0xF14000D0U;
        const std::size_t extension_bytes =
            static_cast<std::size_t>(
                frame.common.frame_size - extension_offset - 4U);
        frame.extension.payload.reserve(extension_bytes);
        for (std::size_t byte = 0U;
             byte < extension_bytes;
             ++byte) {
            frame.extension.payload.push_back(
                static_cast<std::uint8_t>(
                    extension_source.extract_unsigned(8U)));
        }
        if (!extension_source.valid()) {
            frame = {};
            last_error_ = "DTS:X extension bounds";
            return false;
        }
    }

    // The standard 0x02000850 DTS:X envelope is followed by an 18-byte
    // wrapper and then an ordinary, independently coded four-channel XLL
    // channel set.  It inherits the main XLL segment geometry.  Prefer the
    // private type-69 navigation supplied by libdtsx metadata when present;
    // otherwise validate and decode the inline channel set directly.
    if (effective_supplemental_channel_sets.empty()
        && frame.extension.present
        && frame.extension.sync_word == 0x02000850U
        && frame.extension.payload.size() > 18U) {
        standard_xll_x_words =
            std::make_unique<bitstream::WordBuffer>(
                frame.extension.payload, false);
        bitstream::Cursor supplemental_source =
            standard_xll_x_words->cursor();
        supplemental_source.fast_forward(18 * 8);
        bitstream::Cursor probe_source = supplemental_source;
        XllChannelSetProbe probe;
        const bool valid_header =
            probe_xll_channel_set_header(
                probe_source,
                false,
                probe,
                frame.common.channel_set_count,
                frame.common.legacy_sync)
            && probe.channel_count == 4U
            && probe.channel_mask == 0x0FU
            && probe.frequency_band_count == 1U
            && !frame.channel_sets.empty()
            && probe.sample_rate
                   == frame.channel_sets.front().probe.sample_rate;
        XllNavigationTable probe_navigation;
        const bool valid_navigation =
            valid_header
            && unpack_xll_navigation_table(
                probe_source,
                frame.common.segment_size_bits,
                frame.common.segments_per_frame,
                {probe.frequency_band_count},
                probe_navigation);
        std::uint64_t supplemental_audio_bytes = 0U;
        if (valid_navigation) {
            for (const XllNavigationEntry& entry :
                 probe_navigation.entries) {
                supplemental_audio_bytes =
                    std::max<std::uint64_t>(
                        supplemental_audio_bytes,
                        static_cast<std::uint64_t>(
                            entry.byte_offset)
                            + entry.size_bytes);
            }
        }
        const std::uint64_t remaining_bytes =
            probe_source.remaining_bits() / 8U;
        if (valid_navigation
            && supplemental_audio_bytes <= remaining_bytes
            && remaining_bytes - supplemental_audio_bytes <= 5U) {
            XllSupplementalChannelSet supplemental{
                supplemental_source};
            static constexpr std::array<std::uint32_t, 4>
                kHeightSpeakers = {
                    1U << 13U, 1U << 15U,
                    1U << 23U, 1U << 24U};
            static constexpr std::array<std::uint32_t, 4>
                kFoldedBedSpeakers = {
                    1U << 1U, 1U << 2U,
                    1U << 7U, 1U << 8U};
            std::vector<std::uint32_t> hierarchy_speakers;
            hierarchy_speakers.reserve(preceding_hierarchy_channels);
            for (std::size_t set = 0U;
                 set < frame.channel_sets.size();
                 ++set) {
                if (!frame.channel_sets[set]
                         .hierarchical_channel_set) {
                    continue;
                }
                hierarchy_speakers.insert(
                    hierarchy_speakers.end(),
                    lossy_reference_speaker_masks_by_set[set]
                        .begin(),
                    lossy_reference_speaker_masks_by_set[set]
                        .end());
            }
            if (hierarchy_speakers.size()
                == preceding_hierarchy_channels) {
                std::uint32_t height_mask = 0U;
                for (const std::uint32_t speaker :
                     kHeightSpeakers) {
                    height_mask |= speaker;
                }
                supplemental.speaker_activity_mask =
                    speaker_mask_to_activity_mask(height_mask);
                supplemental.reference_speaker_masks =
                    hierarchy_speakers;
                supplemental.lossy_reference_speaker_masks.assign(
                    kHeightSpeakers.size(), 0U);
                supplemental.downmix_coefficients.assign(
                    hierarchy_speakers.size()
                        * kHeightSpeakers.size(),
                    0);
                for (std::size_t reference = 0U;
                     reference < hierarchy_speakers.size();
                     ++reference) {
                    for (std::size_t height = 0U;
                         height < kHeightSpeakers.size();
                         ++height) {
                        if (hierarchy_speakers[reference]
                            == kFoldedBedSpeakers[height]) {
                            supplemental.downmix_coefficients[
                                reference * kHeightSpeakers.size()
                                + height] = 23170;
                        }
                    }
                }
            }
            effective_supplemental_channel_sets.push_back(
                std::move(supplemental));
        } else {
            standard_xll_x_words.reset();
        }
    }

    frame.supplemental_navigation.reserve(
        effective_supplemental_channel_sets.size());
    for (std::size_t supplemental_index = 0U;
         supplemental_index
             < effective_supplemental_channel_sets.size();
         ++supplemental_index) {
        bitstream::Cursor supplemental_source =
            effective_supplemental_channel_sets[
                supplemental_index].source;
        XllChannelSetHeader supplemental_header;
        const std::size_t raw_header_index =
            raw_channel_sets.size();
        const XllChannelSetHeader* previous_header =
            raw_header_index
                    < previous_raw_channel_set_headers_.size()
                ? &previous_raw_channel_set_headers_[
                      raw_header_index]
                : nullptr;
        if (!unpack_xll_primary_channel_set_header(
                supplemental_source,
                frame.common,
                supplemental_header,
                false,
                0U,
                previous_header)) {
            bitstream::Cursor probe_source =
                effective_supplemental_channel_sets[
                    supplemental_index].source;
            XllChannelSetProbe probe;
            const bool probed =
                probe_xll_channel_set_header(
                    probe_source,
                    false,
                    probe,
                    static_cast<std::uint8_t>(
                        frame.common.channel_set_count
                        + effective_supplemental_channel_sets.size()),
                    frame.common.legacy_sync);
            frame = {};
            last_error_ = "supplemental channel-set header "
                + std::to_string(supplemental_index)
                + (probed
                       ? " full parse; probe channels="
                             + std::to_string(
                                 probe.channel_count)
                             + " rate="
                             + std::to_string(
                                 probe.sample_rate)
                             + " bytes="
                             + std::to_string(
                                 probe.header_size)
                       : " probe");
            return false;
        }
        const XllSupplementalChannelSet& supplemental =
            effective_supplemental_channel_sets[
                supplemental_index];
        const std::vector<std::uint32_t> supplemental_speakers =
            expand_speaker_activity_mask(
                supplemental.speaker_activity_mask);
        if (!supplemental_speakers.empty()) {
            if (supplemental_speakers.size()
                    != supplemental_header.probe.channel_count
                || supplemental.reference_speaker_masks.size()
                       != preceding_hierarchy_channels
                || supplemental.lossy_reference_speaker_masks.size()
                       != supplemental_speakers.size()
                || supplemental.downmix_coefficients.size()
                       != preceding_hierarchy_channels
                              * supplemental_speakers.size()) {
                frame = {};
                last_error_ = "supplemental downmix metadata dimensions";
                return false;
            }
            supplemental_header.primary_channel_set = false;
            supplemental_header.hierarchical_channel_set = true;
            supplemental_header.channel_mask_enabled = true;
            supplemental_header.speaker_channel_mask = 0U;
            for (const std::uint32_t speaker :
                 supplemental_speakers) {
                supplemental_header.speaker_channel_mask |= speaker;
            }
            supplemental_header.downmix_coefficients_present = true;
            supplemental_header.embedded_downmix_present = true;
            supplemental_header.downmix_row_count =
                preceding_hierarchy_channels;
            supplemental_header.downmix_values_per_row =
                supplemental_header.probe.channel_count;
            supplemental_header.external_downmix_coefficients.clear();
            supplemental_header.external_downmix_coefficients.reserve(
                supplemental.downmix_coefficients.size());
            std::vector<std::uint32_t> hierarchy_speakers;
            hierarchy_speakers.reserve(
                preceding_hierarchy_channels);
            for (std::size_t set = 0U;
                 set < frame.channel_sets.size();
                 ++set) {
                if (!frame.channel_sets[set]
                         .hierarchical_channel_set) {
                    continue;
                }
                hierarchy_speakers.insert(
                    hierarchy_speakers.end(),
                    lossy_reference_speaker_masks_by_set[set]
                        .begin(),
                    lossy_reference_speaker_masks_by_set[set]
                        .end());
            }
            if (hierarchy_speakers.size()
                != preceding_hierarchy_channels) {
                frame = {};
                last_error_ =
                    "supplemental hierarchy speaker mapping";
                return false;
            }
            for (const std::uint32_t speaker :
                 hierarchy_speakers) {
                const auto reference = std::find(
                    supplemental.reference_speaker_masks.begin(),
                    supplemental.reference_speaker_masks.end(),
                    speaker);
                if (reference
                    == supplemental.reference_speaker_masks.end()) {
                    frame = {};
                    last_error_ =
                        "supplemental reference speaker mapping";
                    return false;
                }
                const std::size_t reference_index =
                    static_cast<std::size_t>(
                        std::distance(
                            supplemental.reference_speaker_masks.begin(),
                            reference));
                const std::size_t row_offset =
                    reference_index
                    * supplemental_header.probe.channel_count;
                supplemental_header.external_downmix_coefficients.insert(
                    supplemental_header
                        .external_downmix_coefficients.end(),
                    supplemental.downmix_coefficients.begin()
                        + static_cast<std::ptrdiff_t>(
                            row_offset),
                    supplemental.downmix_coefficients.begin()
                        + static_cast<std::ptrdiff_t>(
                            row_offset
                            + supplemental_header
                                  .probe.channel_count));
            }
            for (XllChannelSetBand& band :
                 supplemental_header.bands) {
                band.embedded_downmix_present = true;
            }
        }
        raw_channel_sets.push_back(supplemental_header);
        XllNavigationTable supplemental_navigation;
        if (!unpack_xll_navigation_table(
                supplemental_source,
                frame.common.segment_size_bits,
                frame.common.segments_per_frame,
                {supplemental_header.probe.frequency_band_count},
                supplemental_navigation)
            || !supplemental_source.valid()) {
            const std::uint32_t remaining_bytes =
                supplemental_source.remaining_bits() / 8U;
            const std::uint32_t header_bytes =
                supplemental_header.probe.header_size;
            const std::uint32_t segments =
                frame.common.segments_per_frame;
            const std::uint32_t segment_bits =
                frame.common.segment_size_bits;
            frame = {};
            last_error_ = "supplemental navigation "
                + std::to_string(supplemental_index)
                + " (header=" + std::to_string(header_bytes)
                + ", remaining=" + std::to_string(remaining_bytes)
                + ", segments=" + std::to_string(segments)
                + ", size-bits=" + std::to_string(segment_bits)
                + ")";
            return false;
        }
        std::uint64_t supplemental_audio_bytes = 0U;
        for (const XllNavigationEntry& entry :
             supplemental_navigation.entries) {
            supplemental_audio_bytes =
                std::max<std::uint64_t>(
                    supplemental_audio_bytes,
                    static_cast<std::uint64_t>(
                        entry.byte_offset)
                        + entry.size_bytes);
        }
        if (supplemental_audio_bytes
            > supplemental_source.remaining_bits() / 8U) {
            frame = {};
            last_error_ = "supplemental audio bounds "
                + std::to_string(supplemental_index);
            return false;
        }
        frame.supplemental_navigation.push_back(
            std::move(supplemental_navigation));
        audio_data_starts.push_back(supplemental_source);

        const bool selected_replacement_set =
            supplemental_header.probe.replacement_set_index == 0U
            || supplemental_header.probe.default_replacement_set;
        if (!selected_replacement_set) {
            continue;
        }
        if (supplemental_header.hierarchical_channel_set) {
            preceding_hierarchy_channels +=
                supplemental_header.probe.channel_count;
        }
        active_raw_channel_set_indices.push_back(0U);
        active_navigation_indices.push_back(
            frame.supplemental_navigation.size());
        frame.channel_sets.push_back(
            std::move(supplemental_header));
        lossy_reference_speaker_masks_by_set.push_back(
            supplemental.lossy_reference_speaker_masks);
    }

    const std::size_t channel_set_count = frame.channel_sets.size();
    for (std::size_t channel_set = 0U;
         channel_set < channel_set_count;
         ++channel_set) {
        if (active_navigation_indices[channel_set] == 0U) {
            frame.main_planar_channel_count +=
                frame.channel_sets[channel_set]
                    .probe.channel_count;
            continue;
        }
        const XllChannelSetHeader& supplemental =
            frame.channel_sets[channel_set];
        if (supplemental.channel_mask_enabled) {
            for (std::uint32_t bit = 0U; bit < 32U; ++bit) {
                const std::uint32_t speaker = 1U << bit;
                if ((supplemental.speaker_channel_mask & speaker)
                    != 0U) {
                    frame.supplemental_speaker_masks.push_back(
                        speaker);
                }
            }
        } else {
            frame.supplemental_speaker_masks.insert(
                frame.supplemental_speaker_masks.end(),
                supplemental.probe.channel_count,
                0U);
        }
    }
    auto next_channel_decoders = channel_decoders_;
    auto next_channel_parameters = channel_parameters_;
    next_channel_decoders.resize(channel_set_count);
    next_channel_parameters.resize(channel_set_count);
    std::vector<std::vector<std::vector<std::vector<std::int32_t>>>>
        set_bands(channel_set_count);
    std::vector<XllDecimatorHistory> decimator_histories(
        channel_set_count);
    for (std::size_t channel_set = 0;
         channel_set < channel_set_count;
         ++channel_set) {
        const std::uint8_t band_count =
            frame.channel_sets[channel_set]
                .probe.frequency_band_count;
        next_channel_decoders[channel_set].resize(band_count);
        next_channel_parameters[channel_set].resize(band_count);
        set_bands[channel_set].resize(band_count);
        for (auto& band : set_bands[channel_set]) {
            band.resize(
                frame.channel_sets[channel_set]
                    .probe.channel_count);
        }
    }

    std::uint8_t decoded_band_count = 0U;
    for (const XllChannelSetHeader& channel_set :
         frame.channel_sets) {
        decoded_band_count = std::max(
            decoded_band_count,
            channel_set.probe.frequency_band_count);
    }
    for (std::uint8_t band_index = 0U;
         band_index < decoded_band_count;
         ++band_index) {
        for (std::uint32_t segment = 0;
             segment < frame.common.segments_per_frame;
             ++segment) {
            for (std::uint8_t channel_set = 0;
                 channel_set < channel_set_count;
                 ++channel_set) {
            const XllChannelSetHeader& set_header =
                frame.channel_sets[channel_set];
            if (band_index >= set_header.bands.size()) {
                continue;
            }
            const std::size_t navigation_index =
                active_navigation_indices[channel_set];
            const XllNavigationTable& navigation =
                navigation_index == 0U
                ? frame.navigation
                : frame.supplemental_navigation[
                      navigation_index - 1U];
            const XllNavigationEntry* entry =
                navigation.find(
                    band_index,
                    segment,
                    active_raw_channel_set_indices[channel_set]);
            if (entry == nullptr) {
                frame = {};
                last_error_ = "missing navigation entry";
                return false;
            }
            const XllChannelSetBand& band =
                set_header.bands[band_index];
            if (entry->size_bytes == 0U) {
                frame = {};
                last_error_ = "empty navigation entry";
                return false;
            }

            const bitstream::Cursor set_audio_data_start =
                audio_data_starts[navigation_index];
            bitstream::Cursor segment_source =
                set_audio_data_start;
            segment_source.fast_forward(static_cast<std::int32_t>(
                8U * entry->byte_offset));
            const std::uint32_t lsb_size =
                band.primary_size_present
                ? band.primary_size
                : 0U;
            if (lsb_size > entry->size_bytes) {
                frame = {};
                last_error_ = "LSB size exceeds segment";
                return false;
            }
            const std::uint32_t msb_size =
                entry->size_bytes - lsb_size;
            const bool msb_crc_present =
                frame.common.band_crc_present != 0U
                && (band_index == 0U
                    || frame.common.band_crc_present == 3U);
            if (msb_crc_present) {
                bitstream::Cursor crc_source = segment_source;
                if (msb_size < 2U
                    || !valid_crc16(crc_source, 8U * msb_size)) {
                    frame = {};
                    last_error_ = "MSB CRC set "
                        + std::to_string(channel_set)
                        + " segment " + std::to_string(segment);
                    return false;
                }
            }
            segment_source = segment_source.limited(8U * msb_size);

            std::vector<std::uint8_t> adaptive_orders;
            adaptive_orders.reserve(band.prediction.size());
            for (const XllChannelPrediction& prediction :
                 band.prediction) {
                adaptive_orders.push_back(static_cast<std::uint8_t>(
                    prediction
                        .adaptive_reflection_coefficients.size()));
            }
            if (!unpack_xll_channel_parameters(
                    segment_source,
                    segment,
                    set_header.probe.parameter_bits,
                    adaptive_orders,
                    next_channel_parameters[
                        channel_set][band_index])) {
                frame = {};
                last_error_ = "channel parameters set "
                    + std::to_string(channel_set)
                    + " segment " + std::to_string(segment);
                return false;
            }

            XllDecodedChannelSet decoded;
            if (!next_channel_decoders[channel_set][band_index]
                     .decode_msb_segment(
                    segment_source,
                    segment,
                    frame.common.samples_per_segment,
                    next_channel_parameters[
                        channel_set][band_index],
                    band.prediction,
                    band.channel_order,
                    band_index,
                    0U,
                    band.joint_pairs,
                    decoded)) {
                frame = {};
                last_error_ = "MSB decode set "
                    + std::to_string(channel_set)
                    + " segment " + std::to_string(segment)
                    + ": "
                    + next_channel_decoders[channel_set][band_index]
                          .last_error();
                return false;
            }
            if (!segment_source.valid()) {
                frame = {};
                last_error_ = "MSB bitstream overrun";
                return false;
            }
            if (band.primary_size_present
                && band.primary_size != 0U) {
                bitstream::Cursor lsb_source =
                    set_audio_data_start;
                lsb_source.fast_forward(
                    static_cast<std::int32_t>(
                        8U * (entry->byte_offset
                            + entry->size_bytes
                            - band.primary_size)));
                lsb_source = lsb_source.limited(
                    8U * band.primary_size);
                const bool lsb_crc_present =
                    frame.common.band_crc_present == 3U
                    || (band_index == 0U
                        && frame.common.band_crc_present > 1U);
                if (lsb_crc_present) {
                    bitstream::Cursor crc_source = lsb_source;
                    if (band.primary_size < 2U
                        || !valid_crc16(
                            crc_source,
                            8U * band.primary_size)) {
                        frame = {};
                        last_error_ = "LSB CRC";
                        return false;
                    }
                }
                std::vector<std::uint8_t> msb_shifts(
                    set_header.probe.channel_count, 0U);
                for (std::size_t channel = 0U;
                     channel < msb_shifts.size();
                     ++channel) {
                    const std::uint8_t primary_width =
                        frame.common.scalable_resolution != 0U
                        ? frame.common.scalable_resolution
                        : band.primary_widths[channel];
                    const std::uint8_t secondary_width =
                        band.secondary_widths[channel];
                    msb_shifts[channel] =
                        static_cast<std::uint8_t>(
                            primary_width + secondary_width
                            - (primary_width != 0U
                                   && secondary_width != 0U
                               ? 1U
                               : 0U));
                }
                if (!next_channel_decoders[
                         channel_set][band_index]
                         .combine_lsb_segment(
                             lsb_source,
                             band,
                             msb_shifts,
                             decoded)) {
                    frame = {};
                    last_error_ = "LSB decode set "
                        + std::to_string(channel_set)
                        + " segment " + std::to_string(segment)
                        + ": "
                        + next_channel_decoders[channel_set][band_index]
                              .last_error();
                    return false;
                }
                if (!lsb_source.valid()) {
                    frame = {};
                    last_error_ = "LSB bitstream overrun";
                    return false;
                }
            }
            if (band_index == 1U
                && segment == 0U) {
                decimator_histories[channel_set] =
                    decoded.decimator_history;
            }
            for (std::size_t channel = 0;
                 channel < decoded.channels.size();
                 ++channel) {
                auto& destination =
                    set_bands[channel_set][band_index][channel];
                destination.insert(
                    destination.end(),
                    decoded.channels[channel].begin(),
                    decoded.channels[channel].end());
            }
            }
        }
    }

    std::vector<std::vector<std::int32_t>>
        current_downmix_inverse_scales(channel_set_count);
    for (std::size_t channel_set = 0U;
         channel_set < channel_set_count;
         ++channel_set) {
        const XllChannelSetHeader& header =
            frame.channel_sets[channel_set];
        if (header.embedded_downmix_present
            && !decode_xll_inverse_scales(
                header,
                current_downmix_inverse_scales[channel_set])) {
            frame = {};
            last_error_ = "embedded downmix scales";
            return false;
        }
    }

    std::vector<std::size_t> hierarchy_offsets(
        channel_set_count, 0U);
    std::size_t hierarchy_channels = 0U;
    for (std::size_t channel_set = 0U;
         channel_set < channel_set_count;
         ++channel_set) {
        hierarchy_offsets[channel_set] = hierarchy_channels;
        if (frame.channel_sets[channel_set]
                .hierarchical_channel_set) {
            hierarchy_channels +=
                frame.channel_sets[channel_set]
                    .probe.channel_count;
        }
    }

    for (std::size_t channel_set = 0U;
         channel_set < channel_set_count;
         ++channel_set) {
        const XllChannelSetHeader& header =
            frame.channel_sets[channel_set];
        if (header.bands.size() != 1U
            || set_bands[channel_set].size() != 1U
            || lossy_reference_speaker_masks_by_set[
                   channel_set].size()
                   != header.probe.channel_count) {
            continue;
        }
        const std::uint8_t shift =
            header.probe.bit_depth < 24U
            ? static_cast<std::uint8_t>(
                  24U - header.probe.bit_depth)
            : 0U;
        const std::int32_t rounding =
            shift == 0U ? 0 : 1 << (shift - 1U);
        std::size_t prescale_set = channel_set_count;
        if (header.hierarchical_channel_set) {
            for (std::size_t candidate = channel_set + 1U;
                 candidate < channel_set_count;
                 ++candidate) {
                const XllChannelSetHeader& next =
                    frame.channel_sets[candidate];
                if (!next.primary_channel_set
                    && next.embedded_downmix_present
                    && next.hierarchical_channel_set) {
                    prescale_set = candidate;
                    break;
                }
            }
        }
        for (std::size_t channel = 0U;
             channel < header.probe.channel_count;
             ++channel) {
            // libdtsx.so dtsxDecoderLossLessCombine and dcadec
            // combine_residual_core_frame: a zero residual_encode bit
            // means that the decoded XLL values are residuals to be added
            // to the mapped lossy channel.
            if ((header.probe.channel_mask
                 & (1U << channel))
                != 0U) {
                continue;
            }
            const std::uint32_t reference_speaker =
                lossy_reference_speaker_masks_by_set[
                    channel_set][channel];
            auto lossy = std::find_if(
                lossy_base_channels.begin(),
                lossy_base_channels.end(),
                [reference_speaker](
                    const XllLossyBaseChannel& candidate) {
                    return candidate.speaker_mask
                        == reference_speaker;
                });
            if (lossy == lossy_base_channels.end()
                && (reference_speaker == (1U << 3U)
                    || reference_speaker == (1U << 4U)
                    || reference_speaker == (1U << 7U)
                    || reference_speaker == (1U << 8U)
                    || reference_speaker == (1U << 9U)
                    || reference_speaker == (1U << 10U))) {
                // libdtsx.so dtsGetChPosnBySpkrMask applies the native
                // Ls/Lss and Rs/Rss equivalence while combining a Core
                // residual.  The Core WAVE mask can expose this pair as
                // side, back or DTS Lss/Rss positions.
                const bool left_surround =
                    reference_speaker == (1U << 3U)
                    || reference_speaker == (1U << 7U)
                    || reference_speaker == (1U << 9U);
                lossy = std::find_if(
                    lossy_base_channels.begin(),
                    lossy_base_channels.end(),
                    [left_surround](
                        const XllLossyBaseChannel& candidate) {
                        const std::uint32_t mask =
                            candidate.speaker_mask;
                        return left_surround
                            ? mask == (1U << 3U)
                                  || mask == (1U << 7U)
                                  || mask == (1U << 9U)
                            : mask == (1U << 4U)
                                  || mask == (1U << 8U)
                                  || mask == (1U << 10U);
                    });
            }
            if (lossy == lossy_base_channels.end()
                || lossy->samples == nullptr) {
                continue;
            }
            auto& residual =
                set_bands[channel_set][0U][channel];
            if (lossy->samples->size() != residual.size()) {
                frame = {};
                last_error_ = "lossy XLL residual sample count";
                return false;
            }
            std::int32_t current_scale = 65536;
            std::int32_t previous_scale = 65536;
            if (prescale_set < channel_set_count) {
                const std::size_t scale_row =
                    hierarchy_offsets[channel_set] + channel;
                const auto& current_scales =
                    current_downmix_inverse_scales[prescale_set];
                if (scale_row >= current_scales.size()) {
                    frame = {};
                    last_error_ =
                        "lossy XLL residual downmix scale";
                    return false;
                }
                current_scale = current_scales[scale_row];
                previous_scale = current_scale;
                if (prescale_set
                        < downmix_inverse_scales_.size()
                    && downmix_inverse_scales_[prescale_set]
                           .size()
                       == current_scales.size()) {
                    previous_scale =
                        downmix_inverse_scales_[
                            prescale_set][scale_row];
                }
            }
            const std::int64_t scale_delta =
                static_cast<std::int64_t>(current_scale)
                - previous_scale;
            std::uint8_t interpolation_bits = 0U;
            for (std::size_t length = 1U;
                 length < residual.size();
                 length <<= 1U) {
                ++interpolation_bits;
            }
            std::int64_t scale_ramp =
                residual.size() > 1U
                ? std::int64_t{1}
                      << (interpolation_bits - 1U)
                : 0;
            for (std::size_t sample = 0U;
                 sample < residual.size();
                 ++sample) {
                const std::int32_t interpolated_scale =
                    scale_delta == 0
                    ? current_scale
                    : static_cast<std::int32_t>(
                          previous_scale
                          + (scale_ramp
                             >> interpolation_bits));
                scale_ramp += scale_delta;
                const std::int64_t scaled =
                    (static_cast<std::int64_t>(
                         (*lossy->samples)[sample])
                         * interpolated_scale
                     + 0x8000LL)
                    >> 16U;
                const std::int32_t base =
                    saturate_24(
                        (scaled + rounding)
                        >> shift);
                residual[sample] =
                    wrapping_add(residual[sample], base);
            }
        }
    }

    const auto folded_set_bands = set_bands;
    const auto folded_decimator_histories = decimator_histories;
    std::vector<std::vector<std::int32_t>>
        current_downmix_coefficients;
    if (!undo_xll_embedded_downmix(
            frame.channel_sets,
            set_bands,
            decimator_histories,
            downmix_coefficients_,
            current_downmix_coefficients)) {
        frame = {};
        last_error_ = "embedded downmix";
        return false;
    }
    std::size_t planar_channel_offset = 0U;
    for (std::size_t channel_set = 0U;
         channel_set < channel_set_count;
         ++channel_set) {
        const XllChannelSetHeader& header =
            frame.channel_sets[channel_set];
        if (!header.primary_channel_set
            && header.hierarchical_channel_set
            && header.embedded_downmix_present
            && !current_downmix_coefficients[channel_set].empty()) {
            XllEmbeddedDownmixOutput output;
            output.source_channel_offset =
                static_cast<std::uint32_t>(planar_channel_offset);
            output.source_channel_count =
                header.probe.channel_count;
            output.current_coefficients =
                current_downmix_coefficients[channel_set];
            if (channel_set < downmix_coefficients_.size()
                && downmix_coefficients_[channel_set].size()
                       == output.current_coefficients.size()) {
                output.previous_coefficients =
                    downmix_coefficients_[channel_set];
            } else {
                output.previous_coefficients =
                    output.current_coefficients;
            }
            for (std::uint32_t bit = 0U; bit < 32U; ++bit) {
                const std::uint32_t speaker = 1U << bit;
                if ((header.speaker_channel_mask & speaker) != 0U) {
                    output.source_speaker_masks.push_back(speaker);
                }
            }
            output.source_storage_bit_depth =
                std::min<std::uint8_t>(
                    header.probe.storage_bit_depth, 24U);
            output.single_frequency_band =
                header.bands.size() == 1U;
            for (std::size_t prior_set = 0U;
                 prior_set < channel_set;
                 ++prior_set) {
                const XllChannelSetHeader& prior =
                    frame.channel_sets[prior_set];
                if (!prior.hierarchical_channel_set) {
                    continue;
                }
                const auto& speakers =
                    lossy_reference_speaker_masks_by_set[prior_set];
                output.reference_speaker_masks.insert(
                    output.reference_speaker_masks.end(),
                    speakers.begin(), speakers.end());
                output.reference_storage_bit_depths.insert(
                    output.reference_storage_bit_depths.end(),
                    prior.probe.channel_count,
                    std::min<std::uint8_t>(
                        prior.probe.storage_bit_depth, 24U));
                output.single_frequency_band =
                    output.single_frequency_band
                    && prior.bands.size() == 1U;
            }
            if (output.reference_speaker_masks.size()
                    == header.downmix_row_count
                && output.reference_storage_bit_depths.size()
                       == header.downmix_row_count
                && output.source_speaker_masks.size()
                       == header.probe.channel_count) {
                frame.embedded_downmix_outputs.push_back(
                    std::move(output));
            }
        }
        planar_channel_offset += header.probe.channel_count;
    }
    std::size_t hierarchy_channel_count = 0U;
    for (std::size_t current_set = 0U;
         current_set < channel_set_count;
         ++current_set) {
        const XllChannelSetHeader& current =
            frame.channel_sets[current_set];
        if (!current.hierarchical_channel_set) {
            continue;
        }
        if (!current.primary_channel_set
            && current.embedded_downmix_present
            && hierarchy_channel_count != 0U
            && hierarchy_channel_count
                   < frame.main_planar_channel_count) {
            XllHierarchicalDownmixOutput output;
            output.speaker_masks.reserve(hierarchy_channel_count);
            output.planar_channels.reserve(hierarchy_channel_count);
            for (std::size_t prior_set = 0U;
                 prior_set < current_set;
                 ++prior_set) {
                const XllChannelSetHeader& prior =
                    frame.channel_sets[prior_set];
                if (!prior.hierarchical_channel_set) {
                    continue;
                }
                const auto& speakers =
                    lossy_reference_speaker_masks_by_set[prior_set];
                if (speakers.size()
                    != prior.probe.channel_count) {
                    frame = {};
                    last_error_ =
                        "hierarchical downmix speaker mapping";
                    return false;
                }
                output.speaker_masks.insert(
                    output.speaker_masks.end(),
                    speakers.begin(), speakers.end());
                std::vector<std::vector<std::int32_t>>
                    set_channels;
                if (folded_set_bands[prior_set].size() == 1U) {
                    set_channels =
                        folded_set_bands[prior_set].front();
                } else if (
                    folded_set_bands[prior_set].size() == 2U) {
                    const std::size_t channel_count =
                        folded_set_bands[prior_set][0U].size();
                    if (folded_set_bands[prior_set][1U].size()
                            != channel_count
                        || folded_decimator_histories[prior_set]
                               .channels.size()
                               != channel_count) {
                        frame = {};
                        last_error_ =
                            "hierarchical downmix frequency bands";
                        return false;
                    }
                    set_channels.resize(channel_count);
                    auto histories =
                        folded_decimator_histories[prior_set];
                    for (std::size_t channel = 0U;
                         channel < channel_count;
                         ++channel) {
                        if (!reconstruct_two_frequency_bands(
                                folded_set_bands[prior_set][0U]
                                    [channel],
                                folded_set_bands[prior_set][1U]
                                    [channel],
                                histories.channels[channel],
                                set_channels[channel])) {
                            frame = {};
                            last_error_ =
                                "hierarchical downmix reconstruction";
                            return false;
                        }
                    }
                } else {
                    frame = {};
                    last_error_ =
                        "hierarchical downmix band count";
                    return false;
                }
                const std::uint8_t pcm_bit_depth =
                    prior.probe.bit_depth;
                const std::uint8_t output_shift =
                    pcm_bit_depth < 24U
                    ? static_cast<std::uint8_t>(
                          24U - pcm_bit_depth)
                    : 0U;
                for (auto& channel : set_channels) {
                    if (output_shift != 0U) {
                        for (std::int32_t& sample : channel) {
                            sample = static_cast<std::int32_t>(
                                static_cast<std::uint32_t>(sample)
                                << output_shift);
                        }
                    }
                    output.planar_channels.push_back(
                        std::move(channel));
                }
            }
            if (output.speaker_masks.size()
                    == hierarchy_channel_count
                && output.planar_channels.size()
                       == hierarchy_channel_count) {
                frame.hierarchical_downmix_outputs.push_back(
                    std::move(output));
            }
        }
        hierarchy_channel_count += current.probe.channel_count;
    }
    frame.sample_rate = 0U;
    for (std::size_t channel_set = 0;
         channel_set < channel_set_count;
         ++channel_set) {
        const std::uint32_t set_rate =
            frame.channel_sets[channel_set].probe.sample_rate;
        frame.sample_rate = std::max(frame.sample_rate, set_rate);
        std::vector<std::vector<std::int32_t>> set_channels;
        if (set_bands[channel_set].size() == 1U) {
            set_channels = std::move(
                set_bands[channel_set].front());
        } else if (set_bands[channel_set].size() == 2U) {
            const std::size_t channel_count =
                set_bands[channel_set][0U].size();
            set_channels.resize(channel_count);
            if (decimator_histories[channel_set]
                    .channels.size()
                != channel_count) {
                frame = {};
                last_error_ = "decimator history channel count";
                return false;
            }
            for (std::size_t channel = 0U;
                 channel < channel_count;
                 ++channel) {
                if (!reconstruct_two_frequency_bands(
                        std::move(
                            set_bands[channel_set][0U][channel]),
                        std::move(
                            set_bands[channel_set][1U][channel]),
                        decimator_histories[channel_set]
                            .channels[channel],
                        set_channels[channel])) {
                    frame = {};
                    last_error_ = "frequency-band reconstruction";
                    return false;
                }
            }
        } else {
            frame = {};
            last_error_ = "unsupported frequency-band count";
            return false;
        }
        for (auto& channel : set_channels) {
            const std::uint8_t pcm_bit_depth =
                frame.channel_sets[channel_set]
                    .probe.bit_depth;
            const std::uint8_t output_shift =
                pcm_bit_depth < 24U
                ? static_cast<std::uint8_t>(
                      24U - pcm_bit_depth)
                : 0U;
            if (output_shift != 0U) {
                for (std::int32_t& sample : channel) {
                    sample = static_cast<std::int32_t>(
                        static_cast<std::uint32_t>(sample)
                        << output_shift);
                }
            }
            frame.planar_channels.push_back(std::move(channel));
        }
    }
    frame.samples_per_channel =
        frame.common.segments_per_frame
        * frame.common.samples_per_segment
        * (decoded_band_count > 1U ? 2U : 1U);
    auto next_alternate_decoders = alternate_channel_decoders_;
    auto next_alternate_parameters = alternate_channel_parameters_;
    std::vector<std::vector<std::int32_t>> alternate_channels;
    const bool alternate_decoded =
        frame.samples_per_channel == kAlternateFrameSamples
        && frame.sample_rate == 48000U
        && decode_alternate_extension(
            frame.extension,
            next_alternate_decoders,
            next_alternate_parameters,
            alternate_channels);
    if (alternate_decoded) {
        frame.supplemental_speaker_masks.insert(
            frame.supplemental_speaker_masks.end(),
            alternate_channels.size(),
            0U);
        for (auto& channel : alternate_channels) {
            frame.planar_channels.push_back(std::move(channel));
        }
    }
    frame.msb_complete = !frame.planar_channels.empty();
    if (frame.msb_complete) {
        channel_decoders_ =
            std::move(next_channel_decoders);
        channel_parameters_ =
            std::move(next_channel_parameters);
        downmix_coefficients_ =
            std::move(current_downmix_coefficients);
        downmix_inverse_scales_ =
            std::move(current_downmix_inverse_scales);
        previous_raw_channel_set_headers_ =
            std::move(raw_channel_sets);
        if (alternate_decoded) {
            alternate_channel_decoders_ =
                std::move(next_alternate_decoders);
            alternate_channel_parameters_ =
                std::move(next_alternate_parameters);
        }
    }
    return frame.msb_complete;
}

} // namespace dtsx
