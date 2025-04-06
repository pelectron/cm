/**
 *  Copyright 2024-2025 Pelé Constam
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
#ifndef CM_COMMON_HPP
#define CM_COMMON_HPP

#include <cstdint>
#include <type_traits>

#define CM_INLINE __attribute__((always_inline))
#define CM_WEAK __attribute__((weak))

namespace cm {
using Word = std::uint32_t;

/**
 * Standard ARM exception numbers for armv6m and armv7m. MemoryManagementFault,
 * BusFault and UsageFault are not available with armv6m.
 *
 * This is not a strong enumeration on purpose.
 */
enum Exception : std::uint8_t {
  Reset                 = 1,
  NonMaskableInterrupt  = 2,
  HardFault             = 3,
  MemoryManagementFault = 4,
  BusFault              = 5,
  UsageFault            = 6,
  SVCall                = 11,
  PendSV                = 14,
  SysTick               = 15,
  IRQ0_Exception        = 16,
  IRQmax_Exception      = 255
};

enum IRQn : std::uint8_t {
  IRQ0,
  IRQ1,
  IRQ2,
  IRQ3,
  IRQ4,
  IRQ5,
  IRQ6,
  IRQ7,
  IRQ8,
  IRQ9,
  IRQ10,
  IRQ11,
  IRQ12,
  IRQ13,
  IRQ14,
  IRQ15,
  IRQ16,
  IRQ17,
  IRQ18,
  IRQ19,
  IRQ20,
  IRQ21,
  IRQ22,
  IRQ23,
  IRQ24,
  IRQ25,
  IRQ26,
  IRQ27,
  IRQ28,
  IRQ29,
  IRQ30,
  IRQ31,
  IRQ32,
  IRQ33,
  IRQ34,
  IRQ35,
  IRQ36,
  IRQ37,
  IRQ38,
  IRQ39,
  IRQ40,
  IRQ41,
  IRQ42,
  IRQ43,
  IRQ44,
  IRQ45,
  IRQ46,
  IRQ47,
  IRQ48,
  IRQ49,
  IRQ50,
  IRQ51,
  IRQ52,
  IRQ53,
  IRQ54,
  IRQ55,
  IRQ56,
  IRQ57,
  IRQ58,
  IRQ59,
  IRQ60,
  IRQ61,
  IRQ62,
  IRQ63,
  IRQ64,
  IRQ65,
  IRQ66,
  IRQ67,
  IRQ68,
  IRQ69,
  IRQ70,
  IRQ71,
  IRQ72,
  IRQ73,
  IRQ74,
  IRQ75,
  IRQ76,
  IRQ77,
  IRQ78,
  IRQ79,
  IRQ80,
  IRQ81,
  IRQ82,
  IRQ83,
  IRQ84,
  IRQ85,
  IRQ86,
  IRQ87,
  IRQ88,
  IRQ89,
  IRQ90,
  IRQ91,
  IRQ92,
  IRQ93,
  IRQ94,
  IRQ95,
  IRQ96,
  IRQ97,
  IRQ98,
  IRQ99,
  IRQ100,
  IRQ101,
  IRQ102,
  IRQ103,
  IRQ104,
  IRQ105,
  IRQ106,
  IRQ107,
  IRQ108,
  IRQ109,
  IRQ110,
  IRQ111,
  IRQ112,
  IRQ113,
  IRQ114,
  IRQ115,
  IRQ116,
  IRQ117,
  IRQ118,
  IRQ119,
  IRQ120,
  IRQ121,
  IRQ122,
  IRQ123,
  IRQ124,
  IRQ125,
  IRQ126,
  IRQ127,
  IRQ128,
  IRQ129,
  IRQ130,
  IRQ131,
  IRQ132,
  IRQ133,
  IRQ134,
  IRQ135,
  IRQ136,
  IRQ137,
  IRQ138,
  IRQ139,
  IRQ140,
  IRQ141,
  IRQ142,
  IRQ143,
  IRQ144,
  IRQ145,
  IRQ146,
  IRQ147,
  IRQ148,
  IRQ149,
  IRQ150,
  IRQ151,
  IRQ152,
  IRQ153,
  IRQ154,
  IRQ155,
  IRQ156,
  IRQ157,
  IRQ158,
  IRQ159,
  IRQ160,
  IRQ161,
  IRQ162,
  IRQ163,
  IRQ164,
  IRQ165,
  IRQ166,
  IRQ167,
  IRQ168,
  IRQ169,
  IRQ170,
  IRQ171,
  IRQ172,
  IRQ173,
  IRQ174,
  IRQ175,
  IRQ176,
  IRQ177,
  IRQ178,
  IRQ179,
  IRQ180,
  IRQ181,
  IRQ182,
  IRQ183,
  IRQ184,
  IRQ185,
  IRQ186,
  IRQ187,
  IRQ188,
  IRQ189,
  IRQ190,
  IRQ191,
  IRQ192,
  IRQ193,
  IRQ194,
  IRQ195,
  IRQ196,
  IRQ197,
  IRQ198,
  IRQ199,
  IRQ200,
  IRQ201,
  IRQ202,
  IRQ203,
  IRQ204,
  IRQ205,
  IRQ206,
  IRQ207,
  IRQ208,
  IRQ209,
  IRQ210,
  IRQ211,
  IRQ212,
  IRQ213,
  IRQ214,
  IRQ215,
  IRQ216,
  IRQ217,
  IRQ218,
  IRQ219,
  IRQ220,
  IRQ221,
  IRQ222,
  IRQ223,
  IRQ224,
  IRQ225,
  IRQ226,
  IRQ227,
  IRQ228,
  IRQ229,
  IRQ230,
  IRQ231,
  IRQ232,
  IRQ233,
  IRQ234,
  IRQ235,
  IRQ236,
  IRQ237,
  IRQ238,
  IRQ239,
  IRQ240,
  IRQ241,
  IRQ242,
  IRQ243,
  IRQ244,
  IRQ245,
  IRQ246,
  IRQ247,
  IRQ248,
  IRQ249,
  IRQ250,
  IRQ251,
  IRQ252,
  IRQ253,
  IRQ254,
  IRQ255,
};

enum class Endian { little, big };

using Priority = std::uint8_t;

struct armv6m {};
struct armv7m {};
struct armv7m_fp {};

struct MicroSeconds {
  std::uint32_t value;
};

constexpr MicroSeconds operator""_us(unsigned long long micro_seconds) {
  return {static_cast<std::uint32_t>(micro_seconds)};
}

constexpr MicroSeconds operator""_ms(unsigned long long milli_seconds) {
  return {static_cast<std::uint32_t>(1000u * milli_seconds)};
}

// Write the value v into the memory location address with reinterpret_cast.
// The memory location is interpreted as a volatile T*.
// Effectively equivalent to *address = value.
template<typename T>
inline void volatile_write(Word address, T value) {
  volatile T& reg = *reinterpret_cast<volatile T*>(address);
  reg             = value;
}

// Write the value v into the memory location address with reinterpret_cast
// using mask to clear bits before writing v. The memory location is interpreted
// as a volatile T*. Effectively equivalent to *address = (*address & ~mask) |
// value.
template<typename T>
inline void volatile_write(Word address, T mask, T value) {
  static_assert(std::is_unsigned<T>::value,
                "T should be unsigned for bit mask operations");
  volatile T& reg = *reinterpret_cast<volatile T*>(address);
  reg             = (reg & ~mask) | value;
}

// Set the bits at the memory location address with reinterpret_cast.
// The memory location is interpreted as a volatile T*.
// Effectively equivalent to *address |= Tits.
template<typename T>
inline void volatile_set_bits(Word address, T bits) {
  static_assert(std::is_unsigned<T>::value,
                "T should be unsigned for bit mask operations");
  volatile T& reg = *reinterpret_cast<volatile T*>(address);
  reg             = reg | bits;
}

// Reset the bits at the memory location address with reinterpret_cast.
// The memory location is interpreted as a T.
// Effectively equivalent to *address &= ~bits.
template<typename T>
inline void volatile_reset_bits(Word address, T bits) {
  static_assert(std::is_unsigned<T>::value,
                "T should be unsigned for bit mask operations");
  volatile T& reg = *reinterpret_cast<volatile T*>(address);
  reg             = reg & ~bits;
}

// Read the value at memory location address with reinterpret_cast.
// The memory location is interpreted as a volatile T*.
// Returns *address.
template<typename T>
inline T volatile_read(Word address) {
  return *reinterpret_cast<volatile T*>(address);
}

// Read the value masked by bits at memory location address with
// reinterpret_cast. The memory location is interpreted as a volatile T*.
// Returns *address & bits.
template<typename T>
inline T volatile_read_bits(Word address, T bits) {
  return *reinterpret_cast<volatile T*>(address) & bits;
}

} // namespace cm

#endif
