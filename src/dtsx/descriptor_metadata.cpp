#include "dtsx/descriptor_metadata.hpp"

namespace dtsx {

bool unpack_descriptor_metadata(bitstream::Cursor& source,
                                DescriptorMetadata& metadata) noexcept {
    metadata.enabled = source.extract_unsigned(1U) != 0U ? 1U : 0U;
    const std::uint32_t descriptor_value = source.extract_unsigned(2U);
    metadata.descriptor_class =
        static_cast<std::uint8_t>(descriptor_value == 0U ? 0U : 1U);
    return true;
}

} // namespace dtsx
