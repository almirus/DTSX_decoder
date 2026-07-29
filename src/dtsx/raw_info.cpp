#include "dtsx/raw_info.hpp"

namespace dtsx {

RawInfoMasks set_raw_info(const RawInfoFields& fields) noexcept {
    // libdtsx.so: DTSFrameScanner_SetRawInfo, 0x2e3e8.
    const std::uint32_t second =
        ((static_cast<std::uint32_t>(fields.value8) << 5U) & 0x20U)
        | ((static_cast<std::uint32_t>(fields.value4) << 6U) & 0x40U)
        | (fields.flags0 & 1U)
        | ((static_cast<std::uint32_t>(fields.value28) << 7U) & 0x80U)
        | ((static_cast<std::uint32_t>(fields.value32) << 8U) & 0x100U)
        | ((static_cast<std::uint32_t>(fields.value36) << 9U) & 0x200U)
        | ((static_cast<std::uint32_t>(fields.value20) << 10U) & 0xC00U)
        | ((fields.value16 << 12U) & 0xFFF000U)
        | (static_cast<std::uint32_t>(fields.value24) << 24U)
        | ((static_cast<std::uint32_t>(fields.value12) * 4U) & 0x1CU)
        | ((static_cast<std::uint32_t>(fields.value12) * 2U) & 2U);
    const std::uint32_t first =
        ((static_cast<std::uint32_t>(fields.value56) * 2U) & 2U)
        | ((static_cast<std::uint32_t>(fields.value72) * 4U) & 0xCU)
        | (fields.value68 & 1U)
        | ((static_cast<std::uint32_t>(fields.value80) * 16U) & 0x10U);
    return RawInfoMasks{first, second};
}

} // namespace dtsx
