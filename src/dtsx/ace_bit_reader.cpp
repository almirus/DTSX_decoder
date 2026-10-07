#include "dtsx/ace_bit_reader.hpp"

#include <limits>

namespace dtsx {

AceBitReader::AceBitReader(
    const std::uint8_t* bytes,
    std::size_t size) noexcept
    : bytes_(bytes), bit_limit_(size * 8U) {
    if (bytes == nullptr && size != 0U) {
        valid_ = false;
        bit_limit_ = 0U;
    }
}

bool AceBitReader::read(
    std::uint32_t count,
    std::uint32_t& value) noexcept {
    if (!valid_ || count > 32U || position_ > bit_limit_
        || count > bit_limit_ - position_) {
        valid_ = false;
        value = 0U;
        return false;
    }
    value = 0U;
    for (std::uint32_t index = 0U; index < count; ++index) {
        value = (value << 1U)
            | ((bytes_[position_ >> 3U]
                >> (7U - static_cast<unsigned>(position_ & 7U)))
               & 1U);
        ++position_;
    }
    return true;
}

bool AceBitReader::align() noexcept {
    const std::size_t aligned = (position_ + 7U) & ~std::size_t{7U};
    if (!valid_ || aligned > bit_limit_) {
        valid_ = false;
        return false;
    }
    position_ = aligned;
    return true;
}

bool AceBitReader::seek(std::size_t bit_position) noexcept {
    if (!valid_ || bit_position > bit_limit_) {
        valid_ = false;
        return false;
    }
    position_ = bit_position;
    return true;
}

std::size_t AceBitReader::position() const noexcept { return position_; }

std::size_t AceBitReader::bits_left() const noexcept {
    return valid_ && position_ <= bit_limit_
        ? bit_limit_ - position_ : 0U;
}

bool AceBitReader::valid() const noexcept { return valid_; }

std::uint32_t ace_num_bits(std::uint32_t value) noexcept {
    std::uint32_t result = 0U;
    while (value != 0U) {
        value >>= 1U;
        ++result;
    }
    return result;
}

bool read_ace_unary(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (alphabet <= 1U) {
        return true;
    }
    while (value + 1U < alphabet) {
        std::uint32_t bit = 0U;
        if (!source.read(1U, bit)) {
            return false;
        }
        if (bit != 0U) {
            break;
        }
        ++value;
    }
    return true;
}

bool read_ace_unary_ones(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (alphabet <= 1U) {
        return true;
    }
    while (value + 1U < alphabet) {
        std::uint32_t bit = 0U;
        if (!source.read(1U, bit)) {
            return false;
        }
        if (bit == 0U) {
            break;
        }
        ++value;
    }
    return true;
}

bool read_ace_uniform(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (alphabet <= 1U) {
        return true;
    }
    const std::uint32_t width = ace_num_bits(alphabet - 1U);
    const std::uint32_t gap = (std::uint32_t{1U} << width) - alphabet;
    if (!source.read(width - 1U, value)) {
        return false;
    }
    if (value >= gap) {
        std::uint32_t bit = 0U;
        if (!source.read(1U, bit)) {
            return false;
        }
        value = ((value << 1U) | bit) - gap;
    }
    return value < alphabet;
}

bool read_ace_golomb(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t parameter,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (alphabet <= 1U) {
        return true;
    }
    const std::uint32_t maximum = ace_num_bits(alphabet - 1U) - 1U;
    if (parameter >= maximum) {
        return read_ace_uniform(source, alphabet, value);
    }
    std::uint32_t low = 0U;
    std::uint32_t high = 0U;
    if (!source.read(parameter, low)
        || !read_ace_unary(
            source,
            1U + ((alphabet - 1U) >> parameter),
            high)) {
        return false;
    }
    value = low | (high << parameter);
    return value < alphabet;
}

bool read_ace_golomb_limited(
    AceBitReader& source,
    std::uint32_t alphabet,
    std::uint32_t parameter,
    std::uint32_t limit,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (alphabet <= 1U) {
        return true;
    }
    const std::uint32_t minimum_limit = std::uint32_t{1U} << parameter;
    if (limit < minimum_limit) {
        limit = minimum_limit;
    }
    if (limit >= alphabet) {
        limit = alphabet - 1U;
    }
    if (!read_ace_golomb(source, limit + 1U, parameter, value)) {
        return false;
    }
    if (value == limit) {
        std::uint32_t tail = 0U;
        if (!read_ace_uniform(source, alphabet - limit, tail)) {
            return false;
        }
        value += tail;
    }
    return value < alphabet;
}

bool read_ace_vlc(
    AceBitReader& source,
    const std::uint8_t* primary,
    std::size_t primary_size,
    const std::uint8_t* escape,
    std::size_t escape_size,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (primary == nullptr || primary_size == 0U
        || (escape == nullptr && escape_size != 0U)) {
        return false;
    }
    std::uint32_t primary_index = 0U;
    if (!read_ace_unary(
            source,
            static_cast<std::uint32_t>(primary_size),
            primary_index)
        || primary_index >= primary_size) {
        return false;
    }
    std::uint64_t primary_base = 0U;
    for (std::size_t index = 0U; index < primary_index; ++index) {
        primary_base += std::uint64_t{1U} << primary[index];
    }
    std::uint32_t part = 0U;
    if (!source.read(primary[primary_index], part)) {
        return false;
    }
    std::uint64_t result = primary_base + part;
    std::uint64_t primary_alpha = primary_base
        + (std::uint64_t{1U} << primary[primary_index]);
    for (std::size_t index = primary_index + 1U;
         index < primary_size;
         ++index) {
        primary_alpha += std::uint64_t{1U} << primary[index];
    }
    if (escape_size != 0U && result + escape_size >= primary_alpha) {
        const std::uint64_t escape_index =
            result + escape_size - primary_alpha;
        if (escape_index >= escape_size) {
            return false;
        }
        std::uint64_t escape_base = 0U;
        for (std::size_t index = 0U; index < escape_index; ++index) {
            escape_base += std::uint64_t{1U} << escape[index];
        }
        if (!source.read(escape[escape_index], part)) {
            return false;
        }
        result = primary_alpha + escape_base + part;
    }
    if (result > std::numeric_limits<std::uint32_t>::max()) {
        return false;
    }
    value = static_cast<std::uint32_t>(result);
    return true;
}

bool read_ace_limits_vlc(
    AceBitReader& source,
    const std::uint8_t* widths,
    std::size_t width_count,
    std::uint32_t& value) noexcept {
    value = 0U;
    if (widths == nullptr || width_count == 0U) {
        return false;
    }
    std::uint64_t total = 0U;
    for (std::size_t index = 0U; index < width_count; ++index) {
        const std::uint8_t width = widths[index];
        std::uint32_t part = 0U;
        if (width == 0U || width > 31U || !source.read(width, part)) {
            return false;
        }
        total += part;
        if (total > std::numeric_limits<std::uint32_t>::max()) {
            return false;
        }
        if (part + 1U < (std::uint32_t{1U} << width)) {
            value = static_cast<std::uint32_t>(total);
            return true;
        }
    }
    value = static_cast<std::uint32_t>(total);
    return true;
}

bool read_ace_non_uniform_five_ten(
    AceBitReader& source,
    std::uint32_t& value) noexcept {
    if (!source.read(5U, value)) {
        return false;
    }
    if (value > 23U) {
        std::uint32_t tail = 0U;
        if (!source.read(5U, tail)) {
            return false;
        }
        value = ((value - 24U) << 5U) + tail;
    }
    if (value == 279U) {
        std::uint32_t tail = 0U;
        if (!source.read(22U, tail)) {
            return false;
        }
        value += tail;
    }
    return true;
}

} // namespace dtsx
