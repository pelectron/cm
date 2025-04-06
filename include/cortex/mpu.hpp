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
#ifndef CM_MPU_HPP
#define CM_MPU_HPP

#include "cortex/common.hpp"
#include "cortex/scs.hpp"
#include <concepts>
#include <cstdint>

namespace cm {

// in anonymous namespace on purpose. This contains implementation details.
namespace {
  struct MpuConstantsBase {
    enum Regs {
      TYPE = 0xE000ED90,
      CTRL = 0xE000ED94,
      RNR  = 0xE000ED98,
      RBAR = 0xE000ED9C,
      RASR = 0xE000EDA0
    };
    enum TYPE_Fields {
      TYPE_IREGION     = 0xFF << 16,
      TYPE_IREGION_POS = 16,
      TYPE_DREGION     = 0xFF << 8,
      TYPE_DREGION_POS = 8,
      TYPE_SEPERATE    = 1
    };
    enum CTRL_Fields {
      CTRL_PRIVDFEFENA = 1 << 2,
      CTRL_HFNMIENA    = 1 << 1,
      CTRL_ENABLE      = 1
    };
    enum RNR_Fields {
      RNR_REGION = 0xFF,
    };
    enum RASR_Fields {
      RASR_ATTRS         = 0xFFFF << 16,
      RASR_ATTRS_POS     = 16,
      RASR_ATTRS_XN      = 1 << 28,
      RASR_ATTRS_AP      = 0b111 << 24,
      RASR_ATTRS_AP_POS  = 24,
      RASR_ATTRS_TEX     = 0b111 << 19,
      RASR_ATTRS_TEX_POS = 19,
      RASR_ATTRS_S       = 1 << 18,
      RASR_ATTRS_C       = 1 << 17,
      RASR_ATTRS_B       = 1 << 16,
      RASR_ATTRS_B_POS   = 16,
      RASR_SRD           = 0xFF << 8,
      RASR_SRD_POS       = 8,
      RASR_SIZE          = 0x1F << 1,
      RASR_SIZE_POS      = 1,
      RASR_ENABLE        = 1
    };
  };

  template<typename Arch>
  struct MpuConstants;
  template<>
  struct MpuConstants<armv6m> : public MpuConstantsBase {
    enum RBAR_Fields {
      RBAR_ADDR     = 0xFFFFFF << 8,
      RBAR_ADDR_POS = 8u,
      RBAR_VALID    = 1u << 4,
      RBAR_REGION   = 0xFu,
    };
  };

  template<>
  struct MpuConstants<armv7m> : public MpuConstantsBase {
    enum RBAR_Fields {
      RBAR_ADDR     = 0xFFFFFFFE0u,
      RBAR_ADDR_POS = 5u,
      RBAR_VALID    = 1u << 4,
      RBAR_REGION   = 0xFu,
    };
  };
} // namespace

/**
 * MPU region sizes.
 */
enum class Size : std::uint32_t {
  B32 = 4, //< 32 Bytes
  B64,     //< 64 Bytes
  B128,    //< 128 Bytes
  B256,    //< 256 Bytes
  B512,    //< 512 Bytes
  KB1,     //< 1 KB
  KB2,     //< 2 KB
  KB4,     //< 4 KB
  KB8,     //< 8 KB
  KB16,    //< 16 KB
  KB32,    //< 32 KB
  KB64,    //< 64 KB
  KB128,   //< 128 KB
  KB256,   //< 256 KB
  KB512,   //< 512 KB
  MB1,     //< 1 MB
  MB2,     //< 2 MB
  MB4,     //< 4 MB
  MB8,     //< 8 MB
  MB16,    //< 16 MB
  MB32,    //< 32 MB
  MB64,    //< 64 MB
  MB128,   //< 128 MB
  MB256,   //< 256 MB
  MB512,   //< 512 MB
  GB1,     //< 1 GB
  GB2,     //< 2 GB
  GB4      //< 4 GB
};

/**
 * MPU access permission
 */
enum class Access : std::uint32_t {
  /// strongly ordered memory
  ORDERED = 0,
  /// memory mapped peripheral device, non shared
  DEVICE  = 1,
  /// normal memory
  NORMAL  = 2,
};

/**
 * MPU cache policies
 */
enum class CachePolicy : std::uint32_t {
  NoCache, //< non-cacheable policy
  WbWra,   //<  write-back, write and read allocate
  WtNwa,   //< write-through, no write allocate
  WbNwa    //< write-back, no write allocate
};

enum class AccessPermission : std::uint32_t {
  /// no access protection: any access generates a permission fault.
  None = 0,
  /// Privileged Read/Write: privileged access only; any unprivileged access
  /// generates a permission fault.
  Priv = 0x1 << 24,
  /// Privileged Read/Write; Unprivileged Read-only: any unprivileged write
  /// generates a permission fault.
  Uro  = 0x2 << 24,
  /// Privileged Read/Write. Unprivileged Read/Write: full access, permission
  /// faults are never generated.
  Full = 0x3 << 24,
  /// Privileged Read-only: any unprivileged access or privileged write
  /// generates a permission fault.
  Pro  = 0x5 << 24,
  /// Privileged and Unprivileged Read-only: any write generates a permission
  /// fault.
  Ro   = 0x6 << 24,
};

enum class Attr : std::uint32_t {
  /// execute never
  ExecuteNever = 1 << 28,
  Shareable    = 1 << 18,
};

enum class SubRegionDisable : std::uint32_t {
  Zero  = 1 << 0,
  One   = 1 << 1,
  Two   = 1 << 2,
  Three = 1 << 3,
  Four  = 1 << 4,
  Five  = 1 << 5,
  Six   = 1 << 6,
  Seven = 1 << 7,
  None  = 0,
  All   = 0xFFu
};

enum class Attrs : std::uint32_t {};

constexpr Attrs attrs(Attr a) noexcept { return static_cast<Attrs>(a); }

constexpr Attrs attrs(AccessPermission ap) noexcept {
  return static_cast<Attrs>(ap);
}

constexpr Attrs attrs(Size s) noexcept {
  return static_cast<Attrs>(static_cast<std::uint32_t>(s) << 1);
}

constexpr Attrs attrs(SubRegionDisable s) noexcept {
  return static_cast<Attrs>(static_cast<std::uint32_t>(s) << 8);
}

constexpr Attrs operator|(Attrs a1, Attrs a2) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a1) |
                            static_cast<std::uint32_t>(a2));
}

constexpr Attrs operator|(Attrs a, AccessPermission ap) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(ap));
}

constexpr Attrs operator|(Attrs a, Size s) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(s) << 1);
}

constexpr Attrs operator|(Attrs a1, Attr a2) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a1) |
                            static_cast<std::uint32_t>(a2));
}

constexpr Attrs operator|(Attrs a, SubRegionDisable s) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(s) << 8);
}

constexpr Attrs operator|(Attr a1, Attrs a2) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a1) |
                            static_cast<std::uint32_t>(a2));
}
constexpr Attrs operator|(Attr a1, Attr a2) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a1) |
                            static_cast<std::uint32_t>(a2));
}

constexpr Attrs operator|(Attr a, AccessPermission ap) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(ap));
}

constexpr Attrs operator|(Attr a, Size s) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(s) << 1);
}

constexpr Attrs operator|(Attr a, SubRegionDisable s) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(s) << 8);
}

constexpr Attrs operator|(AccessPermission ap, Attrs a) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(ap));
}

constexpr Attrs operator|(AccessPermission ap, Attr a) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(ap));
}

constexpr Attrs operator|(AccessPermission ap, Size s) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(s) << 1 |
                            static_cast<std::uint32_t>(ap));
}

constexpr Attrs operator|(AccessPermission ap, SubRegionDisable s) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(ap) |
                            static_cast<std::uint32_t>(s) << 8);
}

constexpr Attrs operator|(Size s, Attrs a) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(s) << 1);
}

constexpr Attrs operator|(Size s, Attr a) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(a) |
                            static_cast<std::uint32_t>(s) << 1);
}

constexpr Attrs operator|(Size s, AccessPermission ap) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(s) << 1 |
                            static_cast<std::uint32_t>(ap));
}

constexpr Attrs operator|(Size size, SubRegionDisable s) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(size) << 1 |
                            static_cast<std::uint32_t>(s) << 8);
}

static constexpr Attrs operator|(SubRegionDisable s, Attrs a2) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(s) << 1 |
                            static_cast<std::uint32_t>(a2));
}

constexpr Attrs operator|(SubRegionDisable s, AccessPermission ap) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(s) << 1 |
                            static_cast<std::uint32_t>(ap));
}

constexpr Attrs operator|(SubRegionDisable s, Size size) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(s) << 8 |
                            static_cast<std::uint32_t>(size) << 1);
}

constexpr Attrs operator|(SubRegionDisable s, Attr a2) {
  return static_cast<Attrs>(static_cast<std::uint32_t>(s) << 8 |
                            static_cast<std::uint32_t>(a2));
}

constexpr SubRegionDisable operator|(SubRegionDisable s1, SubRegionDisable s2) {
  return static_cast<SubRegionDisable>(static_cast<std::uint32_t>(s1) << 8 |
                                       static_cast<std::uint32_t>(s2) << 8);
}

class Shared {
public:
  constexpr explicit Shared(bool v) : value(v) {}
  constexpr operator bool() const { return value; }

private:
  bool value;
};

class ExecuteNever {
public:
  constexpr explicit ExecuteNever(bool v) : value(v) {}
  constexpr operator bool() const { return value; }

private:
  bool value;
};

struct MpuRegionBase {
  std::uint32_t rnr;
  std::uint32_t rasr;
  std::uint32_t rbar;
};

template<typename Arch>
struct MpuRegion;

template<>
struct MpuRegion<armv6m> : MpuRegionBase {
  /**
   * @brief
   *
   * @param region
   * @param base_address
   * @param execute_never
   * @return
   */
  static constexpr CM_INLINE MpuRegion ordered(std::uint8_t region,
                                               Word base_address,
                                               ExecuteNever execute_never) {
    /// tex = 0b000, CB=00
    std::uint32_t rasr = C::RASR_ENABLE;
    if (execute_never)
      rasr |= C::RASR_ATTRS_XN;

    const std::uint32_t rbar =
        (base_address & C::RBAR_ADDR) | (region & C::RBAR_REGION);
    return MpuRegionBase{region, rasr, rbar};
  }
  /**
   * @brief
   *
   * @param region
   * @param base_address
   * @param execute_never
   * @return
   */
  static constexpr CM_INLINE MpuRegion
  device(std::uint8_t region, Word base_address,
         ExecuteNever execute_never) noexcept {
    std::uint32_t rasr = C::RASR_ENABLE | C::RASR_ATTRS_B;

    if (execute_never)
      rasr |= C::RASR_ATTRS_XN;

    const std::uint32_t rbar =
        (base_address & C::RBAR_ADDR) | (region & C::RBAR_REGION);

    return MpuRegionBase{region, rasr, rbar};
  }

  /**
   * @brief
   *
   * @param region
   * @param base_address
   * @param shared
   * @param policy
   * @return
   */
  static constexpr CM_INLINE MpuRegion
  normal(std::uint8_t region, Word base_address, AccessPermission permissions,
         Shared shared, ExecuteNever execute_never,
         CachePolicy policy = CachePolicy::WtNwa) noexcept {

    std::uint32_t rasr = static_cast<std::uint32_t>(cm::attrs(permissions));
    rasr |= C::RASR_ENABLE;

    if (shared)
      rasr |= C::RASR_ATTRS_S;

    if (execute_never)
      rasr |= C::RASR_ATTRS_XN;

    switch (policy) {
    case CachePolicy::WbNwa:
      // CB = 11
      rasr |= C::RASR_ATTRS_C;
      rasr |= C::RASR_ATTRS_B;
      rasr &= ~C::RASR_ATTRS_TEX_POS;
      break;
    case CachePolicy::WtNwa:
      [[fallthrough]];
    default:
      // CB = 10
      rasr |= C::RASR_ATTRS_C;
      rasr &= ~C::RASR_ATTRS_B;
      rasr &= ~C::RASR_ATTRS_TEX_POS;
      break;
    }

    const std::uint32_t rbar =
        (base_address & C::RBAR_ADDR) | (region & C::RBAR_REGION);
    return MpuRegionBase{region, rasr, rbar};
  }

private:
  constexpr CM_INLINE MpuRegion(const MpuRegionBase& base)
      : MpuRegionBase(base) {}
  using C = MpuConstants<armv6m>;
};

template<>
struct MpuRegion<armv7m> : MpuRegionBase {

  /**
   * @brief
   *
   * @param region
   * @param base_address
   * @param execute_never
   * @return
   */
  static constexpr CM_INLINE MpuRegion ordered(std::uint8_t region,
                                               Word base_address,
                                               ExecuteNever execute_never) {
    /// tex = 0b000, CB=00
    std::uint32_t rasr = C::RASR_ENABLE;
    if (execute_never)
      rasr |= C::RASR_ATTRS_XN;

    const std::uint32_t rbar =
        (base_address & C::RBAR_ADDR) | (region & C::RBAR_REGION);
    return MpuRegionBase{region, rasr, rbar};
  }

  /**
   * @brief
   *
   * @tparam Attrs
   * @param region
   * @param base_address
   * @param shared
   * @param execute_never
   * @return
   */
  template<typename Attrs>
  static constexpr CM_INLINE MpuRegion
  device(std::uint8_t region, Word base_address, Shared shared,
         ExecuteNever execute_never) noexcept {
    std::uint32_t rasr = C::RASR_ENABLE;

    if (shared) // tex=0b000, CB=01
      rasr |= C::RASR_ATTRS_B;
    else // tex=0b010, CB=00
      rasr |= 0b010 << C::RASR_ATTRS_TEX_POS;

    if (execute_never)
      rasr |= C::RASR_ATTRS_XN;

    const std::uint32_t rbar =
        (base_address & C::RBAR_ADDR) | (region & C::RBAR_REGION);

    return MpuRegionBase{region, rasr, rbar};
  }

  /**
   * @brief
   *
   * @tparam Attrs
   * @param region
   * @param base_address
   * @param attrs
   * @param outer
   * @param inner
   * @return
   */
  template<typename Attrs>
  static constexpr CM_INLINE MpuRegion
  normal(std::uint8_t region, Word base_address, Attrs attrs,
         CachePolicy outer = CachePolicy::NoCache,
         CachePolicy inner = CachePolicy::NoCache) noexcept {

    std::uint32_t rasr = static_cast<std::uint32_t>(cm::attrs(attrs));
    rasr |= C::RASR_ENABLE;

    if (outer == inner) {
      // tex = 0b00x CB=yy
      switch (outer) {
      case CachePolicy::NoCache:
        // CB = 00
        rasr &= ~C::RASR_ATTRS_C;
        rasr &= ~C::RASR_ATTRS_B;
        rasr |= 1u << C::RASR_ATTRS_TEX_POS;
        break;
      case CachePolicy::WbNwa:
        // CB = 11
        rasr |= C::RASR_ATTRS_C;
        rasr |= C::RASR_ATTRS_B;
        rasr &= ~C::RASR_ATTRS_TEX_POS;
        break;
      case CachePolicy::WbWra:
        // CB = 11
        rasr |= C::RASR_ATTRS_C;
        rasr |= C::RASR_ATTRS_B;
        rasr |= 1u << C::RASR_ATTRS_TEX_POS;
        break;
      case CachePolicy::WtNwa:
        // CB = 10
        rasr |= C::RASR_ATTRS_C;
        rasr &= ~C::RASR_ATTRS_B;
        rasr &= ~C::RASR_ATTRS_TEX_POS;
        break;
      }
    } else {
      // tex = 0b1{outer}, CB = inner
      rasr |= static_cast<std::uint32_t>(inner) << C::RASR_ATTRS_B_POS;
      rasr |= (0b100u | static_cast<std::uint32_t>(outer))
              << C::RASR_ATTRS_TEX_POS;
    }

    const std::uint32_t rbar =
        (base_address & C::RBAR_ADDR) | (region & C::RBAR_REGION);
    return MpuRegionBase{region, rasr, rbar};
  }

private:
  constexpr CM_INLINE MpuRegion(const MpuRegionBase& base)
      : MpuRegionBase(base) {}
  using C = MpuConstants<armv7m>;
};

template<typename Arch>
struct Mpu;

template<typename Arch>
struct Mpu {

  using region_type = MpuRegion<Arch>;

  static CM_INLINE bool is_implemented() noexcept { return dregion() == 0; }

  static CM_INLINE void apply_region(const region_type& region) noexcept {
    volatile_write(C::RNR, region.rnr);
    volatile_write(C::RBAR, region.rbar);
    volatile_write(C::RASR, region.rasr);
  }

  static CM_INLINE void apply_regions(const region_type* regions,
                                      std::size_t size) noexcept {
    for (std::size_t i = 0; i < size; ++i)
      apply_region(regions[i]);
  }

  static CM_INLINE void clear_region(const region_type& region) noexcept {
    volatile_write(C::RNR, region.rnr);
    volatile_write(C::RASR, 0);
  }

  static CM_INLINE std::uint32_t type() noexcept {
    return volatile_read<std::uint32_t>(C::TYPE);
  }

  static CM_INLINE uint8_t iregion() noexcept {
    return static_cast<std::uint8_t>(
        volatile_read_bits<std::uint32_t>(C::TYPE, C::TYPE_IREGION) >>
        C::TYPE_IREGION_POS);
  }

  static CM_INLINE uint8_t dregion() noexcept {
    return static_cast<std::uint8_t>(
        volatile_read_bits<std::uint32_t>(C::TYPE, C::TYPE_DREGION) >>
        C::TYPE_DREGION_POS);
  }

  static CM_INLINE bool seperate() noexcept {
    return volatile_read_bits<std::uint32_t>(C::TYPE, C::TYPE_SEPERATE) ==
           C::TYPE_SEPERATE;
  }

  /// get MPU_CTRL
  static CM_INLINE std::uint32_t ctrl() noexcept {
    return volatile_read<std::uint32_t>(C::CTRL);
  }

  /// set MPU_CTRL
  static CM_INLINE void ctrl(std::uint32_t c) noexcept {
    volatile_write<std::uint32_t>(C::CTRL, 0b11u, c);
  }

  /// get MPU_CTRL.PRIVDEFENA
  static CM_INLINE bool privdef() noexcept {
    return volatile_read_bits<std::uint32_t>(C::CTRL, C::CTRL_PRIVDFEFENA) ==
           C::CTRL_PRIVDFEFENA;
  }

  /// set MPU_CTRL.PRIVDEFENA
  static CM_INLINE void privdef(bool enable) noexcept {
    if (enable)
      volatile_set_bits<std::uint32_t>(C::CTRL, C::CTRL_PRIVDFEFENA);
    else
      volatile_reset_bits<std::uint32_t>(C::CTRL, C::CTRL_PRIVDFEFENA);
  }

  /// get MPU_CTRL.HFNMIENA
  static CM_INLINE bool hfnmi() noexcept {
    return volatile_read_bits<std::uint32_t>(C::CTRL, C::CTRL_HFNMIENA) ==
           C::CTRL_HFNMIENA;
  }

  /// set MPU_CTRL.HFNMIENA
  static CM_INLINE void hfnmi(bool enable) noexcept {
    if (enable)
      volatile_set_bits<std::uint32_t>(C::CTRL, C::CTRL_HFNMIENA);
    else
      volatile_reset_bits<std::uint32_t>(C::CTRL, C::CTRL_HFNMIENA);
  }

  /// get MPU_CTRL.ENABLE
  static CM_INLINE bool is_enabled() noexcept {
    return volatile_read_bits<std::uint32_t>(C::CTRL, C::CTRL_ENABLE) ==
           C::CTRL_ENABLE;
  }

  /// enable the mpu
  static CM_INLINE void enable() noexcept {
    __asm volatile("  DMB  ");

    volatile_set_bits<std::uint32_t>(C::CTRL, C::CTRL_ENABLE);

    if constexpr (requires { Scs<Arch>::SHCSR_MEMFAULTENA; })
      volatile_set_bits(Scs<Arch>::SHCSR, Scs<Arch>::SHCSR_MEMFAULTENA);

    __asm volatile("  DSB                               \n"
                   "  ISB                               \n");
  }

  /// disable the mpu
  static CM_INLINE void disable() noexcept {
    __asm volatile("  DMB  ");

    if constexpr (requires { Scs<Arch>::SHCSR_MEMFAULTENA; })
      volatile_reset_bits(Scs<Arch>::SHCSR, Scs<Arch>::SHCSR_MEMFAULTENA);

    volatile_reset_bits<std::uint32_t>(C::CTRL, C::CTRL_ENABLE);
    __asm volatile("  DSB                               \n"
                   "  ISB                               \n");
  }

  static CM_INLINE std::uint8_t rnr_region() noexcept {
    return static_cast<std::uint8_t>(
        volatile_read_bits<std::uint32_t>(C::RNR, C::RNR_REGION));
  }

  static CM_INLINE void rnr_region(std::uint8_t region) noexcept {
    volatile_write<std::uint32_t>(C::RNR, C::RNR_REGION, region);
  }

  static CM_INLINE std::uint32_t rbar() noexcept {
    return volatile_read<std::uint32_t>(C::RBAR);
  }

  static CM_INLINE void rbar(std::uint32_t v) noexcept {
    volatile_write<std::uint32_t>(C::RBAR, v);
  }

  static CM_INLINE std::uint32_t rbar_addr() noexcept {
    return volatile_read_bits<std::uint32_t>(C::RBAR, C::RBAR_ADDR) >>
           C::RBAR_ADDR_POS;
  }

  static CM_INLINE void rbar_addr(std::uint32_t addr) noexcept {
    volatile_write<std::uint32_t>(C::RBAR,
                                  C::RBAR_ADDR,
                                  addr << C::RBAR_ADDR_POS);
  }

  static CM_INLINE void rbar_valid(bool valid) noexcept {
    if (valid)
      volatile_set_bits<std::uint32_t>(C::RBAR, C::RBAR_VALID);
    else
      volatile_reset_bits<std::uint32_t>(C::RBAR, C::RBAR_VALID);
  }

  static CM_INLINE std::uint8_t rbar_region() noexcept {
    return static_cast<std::uint8_t>(
        volatile_read_bits<std::uint32_t>(C::RBAR, C::RBAR_REGION));
  }

  static CM_INLINE void rbar_region(std::uint8_t region) noexcept {
    volatile_write<std::uint32_t>(C::RBAR, C::RBAR_REGION, region);
  }

  static CM_INLINE std::uint32_t rasr() noexcept {
    return volatile_read<std::uint32_t>(C::RBAR);
  }

  static CM_INLINE void rasr(std::uint32_t v) noexcept {
    volatile_write<std::uint32_t>(C::RBAR, v);
  }

  static CM_INLINE Attrs rasr_attrs() noexcept {
    return static_cast<Attrs>(
        volatile_read_bits<std::uint32_t>(C::RASR, C::RASR_ATTRS));
  }

  static CM_INLINE void rasr_attrs(Attrs attributes) noexcept {
    volatile_write<std::uint32_t>(C::RASR,
                                  C::RASR_ATTRS,
                                  static_cast<std::uint32_t>(attributes));
  }

  static CM_INLINE std::uint8_t rasr_srd() noexcept {
    return static_cast<std::uint8_t>(
        volatile_read_bits<std::uint32_t>(C::RASR, C::RASR_SRD) >>
        C::RASR_SRD_POS);
  }

  static CM_INLINE void rasr_srd(std::uint8_t srd) noexcept {
    volatile_write<std::uint32_t>(C::RBAR,
                                  C::RASR_SRD,
                                  static_cast<std::uint32_t>(srd)
                                      << C::RASR_SRD_POS);
  }

  static CM_INLINE std::uint8_t rasr_size() noexcept {
    return static_cast<std::uint8_t>(
        volatile_read_bits<std::uint32_t>(C::RASR, C::RASR_SIZE) >>
        C::RASR_SIZE_POS);
  }

  static CM_INLINE void rasr_size(std::uint8_t size) noexcept {
    volatile_write<std::uint32_t>(C::RBAR,
                                  C::RASR_SIZE,
                                  static_cast<std::uint32_t>(size)
                                      << C::RASR_SIZE_POS);
  }

  static CM_INLINE void rasr_enable() noexcept {
    volatile_set_bits<std::uint32_t>(C::RASR, C::RASR_ENABLE);
  }

  static CM_INLINE void rasr_disable() noexcept {
    volatile_reset_bits<std::uint32_t>(C::RASR, C::RASR_ENABLE);
  }

private:
  using C = MpuConstants<Arch>;
};

struct NoMpu {
  using region_type = void*;
  static constexpr bool is_implemented() noexcept { return false; }
  static constexpr void apply_region(const region_type&) noexcept {}
  static constexpr void apply_regions(const region_type*,
                                      std::size_t) noexcept {}
  static void clear_region(const region_type&) noexcept {}
};

template<typename T>
concept MpuConcept =
    requires(const typename T::region_type& r,
             const typename T::region_type* pr, std::size_t size) {
      { T::is_implemented() } -> std::convertible_to<bool>;
      { T::apply_region(r) };
      { T::apply_regions(pr, size) };
      { T::clear_region(r) };
    };

} // namespace cm

#endif
