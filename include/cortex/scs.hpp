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
#ifndef CM_SCS_HPP
#define CM_SCS_HPP

#include "cortex/common.hpp"
#include <concepts>
#include <cstdint>

namespace cm {

template<typename Arch>
struct Scs;

template<typename T>
concept ScsConcept = requires(std::uint32_t exception, std::uint8_t priority) {
  {
    T::set_system_handler_prio(exception, priority)
  } -> std::convertible_to<bool>;
};

struct CpuId {
  uint8_t implementer;
  std::uint8_t variant;
  std::uint8_t architecture;
  std::uint16_t partno;
  std::uint8_t revision;
};

/**
 * The SCS is a memory-mapped 4KB address space that provides 32-bit registers
 * for configuration, status reporting and control. The SCS registers divide
 * into the following groups:
 * - system control and identification
 * - the CPUID processor identification space
 * - system configuration and status
 * - an optional system timer, SysTick
 * - a Nested Vectored Interrupt Controller (NVIC)
 * - system debug, see ARMv6-M Debug.
 */
template<>
struct Scs<armv6m> {
  static CM_INLINE CpuId cpu_id() noexcept {
    auto reg = volatile_read<uint32_t>(CPUID);
    CpuId id{
        .implementer = static_cast<std::uint8_t>((reg & CPUID_IMPLEMENTER) >>
                                                 CPUID_IMPLEMENTER_POS),
        .variant =
            static_cast<uint8_t>((reg & CPUID_VARIANT) >> CPUID_VARIANT_POS),
        .architecture = static_cast<std::uint8_t>((reg & CPUID_ARCHITECTURE) >>
                                                  CPUID_ARCHITECTURE_POS),
        .partno =
            static_cast<uint16_t>((reg & CPUID_PARTNO) >> CPUID_PARTNO_POS),
        .revision =
            static_cast<uint8_t>((reg & CPUID_REVISION) >> CPUID_REVISION_POS)};
    return id;
  }

  static constexpr CM_INLINE Word max_irq_number() noexcept { return 32; }

  static CM_INLINE Endian endian() noexcept {
    return volatile_read_bits<std::uint32_t>(AIRCR, AIRCR_ENDIANESS) == 0
               ? Endian::little
               : Endian::big;
  }

  /**
   * Sets the SYSRESETREQ bit of AIRCR to 1.
   * Asserts a signal to request a reset by the external system. The system
   * components that are reset by this request are IMPLEMENTATION DEFINED. A
   * Local reset is required as part of a system reset request. A Local reset
   * clears this bit to 0.
   * @sa ARMv6-M TRM Section B3.2.6
   */
  static CM_INLINE void sys_reset_request() noexcept {
    volatile_write<std::uint32_t>(
        AIRCR,
        (volatile_read<std::uint32_t>(AIRCR) & 0xFFu) | AIRCR_VECTKEY |
            AIRCR_SYSRESETREQ);
  }

  /**
   * Sets the VECTCLRACTIVE bit of AIRCR to 1.
   * Clears all active state information for fixed and configurable
   * exceptions.The effect of calling this if the processor is not
   * halted in Debug state is UNPREDICTABLE.
   * @sa ARMv6-M TRM Section B3.2.6
   */
  static CM_INLINE void clear_vect_active() noexcept {
    volatile_write<std::uint32_t>(
        AIRCR,
        (volatile_read<std::uint32_t>(AIRCR) & 0xFFu) | AIRCR_VECTKEY |
            AIRCR_VECTCLRACTIVE);
  }

  /**
   * Set whether an interrupt transition from inactive state to pending state is
   * a wakeup event:
   * - false: transitions from inactive to pending are not wakeup events.
   * - true: transitions from inactive to pending are wakeup events.
   * @sa ARMv6-M TRM WFE on page A6-174
   */
  static CM_INLINE void sev_on_pend(bool enable) noexcept {
    if (enable)
      volatile_set_bits<std::uint32_t>(SCR, SCR_SEVONPEND);
    else
      volatile_reset_bits<std::uint32_t>(SCR, SCR_SEVONPEND);
  }

  /**
   * Check whether an interrupt transition from inactive state to pending state
   * is a wakeup event:
   * - false: transitions from inactive to pending are not wakeup events.
   * - true: transitions from inactive to pending are wakeup events.
   * @sa ARMv6-M TRM WFE on page A6-174
   */
  static CM_INLINE bool sev_on_pend() noexcept {
    return volatile_read_bits<std::uint32_t>(SCR, SCR_SEVONPEND) ==
           SCR_SEVONPEND;
  }

  /**
   * Sets/resets the SLEEPDEEP bit in the SCR depending on enable.
   *
   * Provides a qualifying hint indicating that waking from sleep might take
   * longer. An implementation can use this bit to select between two
   * alternative sleep states:
   * - false: selected sleep state is not deep sleep.
   * - true: selected sleep state is deep sleep.
   *
   * Details of the implemented sleep states, if any, and details of the use of
   * this bit, are IMPLEMENTATION DEFINED. If the processor does not implement a
   * deep sleep state then this bit can be RAZ/WI.
   * @sa ARMv6-M TRM Section B3.2.7
   */
  static CM_INLINE void sleep_deep(bool enable) noexcept {
    if (enable)
      volatile_set_bits<std::uint32_t>(SCR, SCR_SLEEPDEEP);
    else
      volatile_reset_bits<std::uint32_t>(SCR, SCR_SLEEPDEEP);
  }

  /**
   * Get the state of the DEEPSLEEP bit of the SCR register.
   * - false: selected sleep state is not deep sleep.
   * - true: selected sleep state is deep sleep.
   * @sa ARMv6-M TRM Section B3.2.7
   */
  static CM_INLINE bool sleep_deep() noexcept {
    return volatile_read_bits<std::uint32_t>(SCR, SCR_SLEEPDEEP) ==
           SCR_SLEEPDEEP;
  }

  /**
   * Sets/resets the SLEEPONEXIT bit in the SCR depending on enable.
   *
   * Sets whether, on an exit from an ISR that returns to the base level of
   * execution priority, the processor enters a sleep state:
   * - false: do not enter sleep state
   * - true: enter sleep state
   * @sa ARMv6-M TRM Section B3.2.7
   */
  static CM_INLINE void sleep_on_exit(bool enable) noexcept {
    if (enable)
      volatile_set_bits<std::uint32_t>(SCR, SCR_SLEEPONEXIT);
    else
      volatile_reset_bits<std::uint32_t>(SCR, SCR_SLEEPONEXIT);
  }

  /**
   * if true, then on exception entry, the SP used prior to the exception is
   * adjusted to be 8-byte aligned and the context to restore it is saved. The
   * SP is restored on the associated exception return.
   * @sa ARMv6-M TRM Section B3.2.8
   */
  static CM_INLINE bool stack_align() noexcept {
    return volatile_read_bits<std::uint32_t>(CCR, CCR_STKALIGN) == CCR_STKALIGN;
  }

  /**
   * if true, unaligned word and halfword accesses generate a HardFault
   * exception.
   * @sa ARMv6-M TRM Section B3.2.8
   */
  static CM_INLINE bool unaligned_trap() noexcept {
    return volatile_read_bits<std::uint32_t>(CCR, CCR_UNALIGN_TRP) ==
           CCR_UNALIGN_TRP;
  }

private:
  template<typename Arch, IRQn MaxIrqNum>
  friend struct Nvic;

  enum SCB_Regs : std::uint32_t {
    ACTLR = 0xE000E008,
    CPUID = 0xE000ED00,
    ICSR  = 0xE000ED04,
    VTOR  = 0xE000ED08,
    AIRCR = 0xE000ED0C,
    SCR   = 0xE000ED10,
    CCR   = 0xE000ED14,
    SHPR2 = 0xE000ED1C,
    SHPR3 = 0xE000ED20,
    SHCSR = 0xE000ED24,
    DFSR  = 0xE000ED30,
  };

  enum CPUID_Fields : std::uint32_t {
    CPUID_IMPLEMENTER      = 0xFFu << 24,
    CPUID_IMPLEMENTER_POS  = 24,
    CPUID_VARIANT          = 0xF << 20,
    CPUID_VARIANT_POS      = 20,
    CPUID_ARCHITECTURE     = 0xF << 16,
    CPUID_ARCHITECTURE_POS = 16,
    CPUID_PARTNO           = 0xFFF << 4,
    CPUID_PARTNO_POS       = 4,
    CPUID_REVISION         = 0xF,
    CPUID_REVISION_POS     = 0
  };

  enum ICSR_Fields : std::uint32_t {
    ICSR_NMIPENDSET      = 1u << 31,
    ICSR_PENDSVSET       = 1 << 28,
    ICSR_PENDSVCLR       = 1 << 27,
    ICSR_PENDSETSET      = 1 << 26,
    ICSR_PENDSTCLR       = 1 << 25,
    ICSR_ISRPREEMPT      = 1 << 23,
    ICSR_ISRPENDING      = 1 << 22,
    ICSR_VECTPENDING     = 0x1F << 12,
    ICSR_VECTPENDING_POS = 12,
    ICSR_RETTOBASE       = 1 << 11,
    ICSR_VECTACTIVE      = 0xFF,
    ICSR_VECTACTIVE_POS  = 0
  };

  enum VTOR_Fields : std::uint32_t {
    VTOR_TBLOFF  = 0xFFFFFF80,
    VTOR_TBL_POS = 7
  };

  enum AIRCR_Fields : std::uint32_t {
    AIRCR_VECTKEY       = 0x05FA << 16,
    AIRCR_VECTKEYSTAT   = 0xFA05u << 16,
    AIRCR_VECT_POS      = 16,
    AIRCR_ENDIANESS     = 1 << 15,
    AIRCR_SYSRESETREQ   = 1 << 2,
    AIRCR_VECTCLRACTIVE = 1 << 1,
  };

  enum SCR_Fields : std::uint32_t {
    SCR_SEVONPEND   = 1 << 4,
    SCR_SLEEPDEEP   = 1 << 2,
    SCR_SLEEPONEXIT = 1 << 1
  };

  enum CCR_Fields : std::uint32_t {
    CCR_STKALIGN    = 1 << 9,
    CCR_UNALIGN_TRP = 1 << 3,
  };
  enum SHPR2_Fields : std::uint32_t {
    SHPR2_PRI_11     = 0b11u << 30,
    SHPR2_PRI_11_POS = 30,
  };

  enum SHPR3_Fields : std::uint32_t {
    SHPR3_PRI_15     = 0b11u << 30,
    SHPR3_PRI_15_POS = 30,
    SHPR3_PRI_14     = 0b11 << 22,
    SHPR3_PRI_14_POS = 22,
  };

  enum DFSR_Fields : std::uint32_t {
    DFSR_EXTERNAL = 1 << 4,
    DFSR_VCATCH   = 1 << 3,
    DFSR_DWTTRAP  = 1 << 2,
    DFSR_BKPT     = 1 << 1,
    DFSR_HALTED   = 1
  };
};

/**
 * from the armv7-M reference manual:
 * The System Control Space (SCS) is a memory-mapped 4KB address space that
 * provides 32-bit registers for configuration, status reporting and control.
 * The SCS registers divide into the following groups:
 * - system control and identification
 * - the CPUID processor identification space
 * - system configuration and status
 * - fault reporting
 * - a system timer, SysTick
 * - a Nested Vectored Interrupt Controller (NVIC)
 * - a Protected Memory System Architecture (PMSA)
 * - system debug.
 */
template<>
struct Scs<armv7m> : Scs<armv6m> {

  static CM_INLINE Word max_irq_number() noexcept {
    return (volatile_read<std::uint32_t>(ICTR) + 1) * 32;
  }

  /**
   * Set the priority grouping.
   * @sa ARMv7-M TRM Section Priority Grouping on page B1-127.
   */
  static CM_INLINE void prigroup(std::uint8_t p) {
    volatile_write<uint32_t>(AIRCR,
                             (volatile_read<std::uint32_t>(AIRCR) & 0xFFu) |
                                 AIRCR_VECTKEY |
                                 (static_cast<Word>(p) << AIRCR_PRIGROUP_POS));
  }

  static CM_INLINE std::uint8_t prigroup() {
    return static_cast<std::uint8_t>(
        volatile_read_bits<std::uint32_t>(AIRCR, AIRCR_PRIGROUP) >>
        AIRCR_PRIGROUP_POS);
  }

  /**
   * Sets the VECTRESET bit of the AIRCR, i.e. causes a local system reset.
   */
  static CM_INLINE void vect_reset() {
    volatile_write<uint32_t>(AIRCR,
                             (volatile_read<std::uint32_t>(AIRCR) & 0xFFu) |
                                 AIRCR_VECTKEY | AIRCR_VECTRESET);
  }

  /// enable/disable branch prediction
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE void branch_prediction(bool enable) {
    if (enable)
      volatile_set_bits<std::uint32_t>(CCR, CCR_BP);
    else
      volatile_reset_bits<std::uint32_t>(CCR, CCR_BP);
  }

  /// query if branch prediction is enabled
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE bool branch_prediction() noexcept {
    return volatile_read_bits<std::uint32_t>(CCR, CCR_BP) == CCR_BP;
  }

  /// enable/disable the instruction cache
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE void instruction_cache(bool enable) {
    if (enable)
      volatile_set_bits<std::uint32_t>(CCR, CCR_IC);
    else
      volatile_reset_bits<std::uint32_t>(CCR, CCR_IC);
  }

  /// query if instruction cache is enabled
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE bool instruction_cache() noexcept {
    return volatile_read_bits<std::uint32_t>(CCR, CCR_IC) == CCR_IC;
  }

  /// enable/disable the data cache
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE void data_cache(bool enable) {
    if (enable)
      volatile_set_bits<std::uint32_t>(CCR, CCR_DC);
    else
      volatile_reset_bits<std::uint32_t>(CCR, CCR_DC);
  }

  /// query if data cache is enabled
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE bool data_cache() noexcept {
    return volatile_read_bits<std::uint32_t>(CCR, CCR_DC) == CCR_DC;
  }

  /// enable/disable division by 0 trapping
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE void div_by_0_trap(bool enable) {
    if (enable)
      volatile_set_bits<std::uint32_t>(CCR, CCR_DIV_0_TRP);
    else
      volatile_reset_bits<std::uint32_t>(CCR, CCR_DIV_0_TRP);
  }

  /// query if data cache is enabled
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE bool div_by_0_trap() noexcept {
    return volatile_read_bits<std::uint32_t>(CCR, CCR_DIV_0_TRP) ==
           CCR_DIV_0_TRP;
  }

  /// Controls whether unprivileged software can access the STIR
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE void user_set_m_pend(bool enable) noexcept {
    if (enable)
      volatile_set_bits<std::uint32_t>(CCR, CCR_USERSETMPEND);
    else
      volatile_reset_bits<std::uint32_t>(CCR, CCR_USERSETMPEND);
  }

  /// query if unprivileged software can access the STIR
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE bool user_set_m_pend() noexcept {
    return volatile_read_bits<std::uint32_t>(CCR, CCR_USERSETMPEND) ==
           CCR_USERSETMPEND;
  }

  /// Controls whether the processor can enter Thread mode with exceptions
  /// active
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE void no_base_thread(bool enable) noexcept {
    if (enable)
      volatile_set_bits<std::uint32_t>(CCR, CCR_NONBASETHRDENA);
    else
      volatile_reset_bits<std::uint32_t>(CCR, CCR_NONBASETHRDENA);
  }

  /// query if the processor can enter Thread mode with exceptions active
  /// @sa ARMv7-M TRM Section B3.2.8
  static CM_INLINE bool no_base_thread() noexcept {
    return volatile_read_bits<std::uint32_t>(CCR, CCR_NONBASETHRDENA) ==
           CCR_NONBASETHRDENA;
  }

private:
  template<typename Arch, IRQn MaxIrqNum>
  friend struct Nvic;
  enum SCB_Regs : std::uint32_t {
    CPUID    = 0xE000ED00,
    ICSR     = 0xE000ED04,
    VTOR     = 0xE000ED08,
    AIRCR    = 0xE000ED0C,
    SCR      = 0xE000ED10,
    CCR      = 0xE000ED14,
    SHPR1    = 0xE000ED18,
    SHPR2    = 0xE000ED1C,
    SHPR3    = 0xE000ED20,
    SHCSR    = 0xE000ED24,
    CFSR     = 0xE000ED28,
    HFSR     = 0xE000ED2C,
    DFSR     = 0xE000ED30,
    MMFAR    = 0xE000ED34,
    BFAR     = 0xE000ED38,
    AFSR     = 0xE000ED3C,
    // CPUID registers: 0xE000ED40 - 0xE000ED7C
    ID_PFR0  = 0xE000ED40,
    ID_PFR1  = 0xE000ED44,
    ID_DFR0  = 0xE000ED48,
    IDAFR0   = 0xE000ED4C,
    ID_MMFR0 = 0xE000ED50,
    ID_MMFR1 = 0xE000ED54,
    ID_MMFR2 = 0xE000ED58,
    ID_MMFR3 = 0xE000ED5C,
    ID_ISAR0 = 0xE000ED60,
    ID_ISAR1 = 0xE000ED64,
    ID_ISAR2 = 0xE000ED68,
    ID_ISAR3 = 0xE000ED6C,
    ID_ISAR4 = 0xE000ED70,
    ID_ISAR5 = 0xE000ED74,
    // reserved: 0xE000ED80 - 0xE000ED84
    CPACR    = 0xE000ED88,
    // reserved: 0xE000ED8C
  };

  enum CPUID_Fields : std::uint32_t {
    CPUID_IMPLEMENTER      = 0xFFu << 24,
    CPUID_IMPLEMENTER_POS  = 24,
    CPUID_VARIANT          = 0xF << 20,
    CPUID_VARIANT_POS      = 20,
    CPUID_ARCHITECTURE     = 0xF << 16,
    CPUID_ARCHITECTURE_POS = 16,
    CPUID_PARTNO           = 0xFFF << 4,
    CPUID_PARTNO_POS       = 4,
    CPUID_REVISION         = 0xF,
    CPUID_REVISION_POS     = 0
  };

  enum ICSR_Fields : std::uint32_t {
    ICSR_NMIPENDSET      = 1u << 31,
    ICSR_PENDSVSET       = 1 << 28,
    ICSR_PENDSVCLR       = 1 << 27,
    ICSR_PENDSETSET      = 1 << 26,
    ICSR_PENDSTCLR       = 1 << 25,
    ICSR_ISRPREEMPT      = 1 << 23,
    ICSR_ISRPENDING      = 1 << 22,
    ICSR_VECTPENDING     = 0x1F << 12,
    ICSR_VECTPENDING_POS = 12,
    ICSR_RETTOBASE       = 1 << 11,
    ICSR_VECTACTIVE      = 0xFF,
    ICSR_VECTACTIVE_POS  = 0
  };

  enum VTOR_Fields { VTOR_TBLOFF = 0xFFFFFF80, VTOR_TBL_POS = 7 };

  enum AIRCR_Fields : std::uint32_t {
    AIRCR_VECTKEY       = 0x05FAu << 16,
    AIRCR_VECTKEYSTAT   = 0xFA05u << 16,
    AIRCR_VECT_POS      = 16,
    AIRCR_ENDIANESS     = 1 << 15,
    AIRCR_PRIGROUP      = 0b111 << 8,
    AIRCR_PRIGROUP_POS  = 8,
    AIRCR_SYSRESETREQ   = 1 << 2,
    AIRCR_VECTCLRACTIVE = 1 << 1,
    AIRCR_VECTRESET     = 1
  };

  enum SCR_Fields : std::uint32_t {
    SCR_SEVONPEND   = 1 << 4,
    SCR_SLEEPDEPP   = 1 << 2,
    SCR_SLEEPONEXIT = 1 << 1
  };

  enum CCR_Fields : std::uint32_t {
    CCR_BP             = 1 << 18,
    CCR_IC             = 1 << 17,
    CCR_DC             = 1 << 16,
    CCR_STKALIGN       = 1 << 9,
    CCR_BFHFNMIGN      = 1 << 8,
    CCR_DIV_0_TRP      = 1 << 4,
    CCR_UNALIGN_TRP    = 1 << 3,
    CCR_USERSETMPEND   = 1 << 1,
    CCR_NONBASETHRDENA = 1
  };

  enum SHPR1_Fields : std::uint32_t {
    SHPR1_PRI_7     = 0xFFu << 24,
    SHPR1_PRI_7_POS = 24,
    SHPR1_PRI_6     = 0xFF << 16,
    SHPR1_PRI_6_POS = 16,
    SHPR1_PRI_5     = 0xFF << 8,
    SHPR1_PRI_5_POS = 8,
    SHPR1_PRI_4     = 0xFF,
    SHPR1_PRI_4_POS = 0,
  };

  enum SHPR2_Fields : std::uint32_t {
    SHPR2_PRI_11     = 0xFFu << 24,
    SHPR2_PRI_11_POS = 24,
    SHPR2_PRI_10     = 0xFF << 16,
    SHPR2_PRI_10_POS = 16,
    SHPR2_PRI_9      = 0xFF << 8,
    SHPR2_PRI_9_POS  = 8,
    SHPR2_PRI_8      = 0xFF,
    SHPR2_PRI_8_POS  = 0,
  };

  enum SHPR3_Fields : std::uint32_t {
    SHPR3_PRI_15     = 0xFFu << 24,
    SHPR3_PRI_15_POS = 24,
    SHPR3_PRI_14     = 0xFF << 16,
    SHPR3_PRI_14_POS = 16,
    SHPR3_PRI_13     = 0xFF << 8,
    SHPR3_PRI_13_POS = 8,
    SHPR3_PRI_12     = 0xFF,
    SHPR3_PRI_12_POS = 0,
  };

  enum SHCSR_Fields : std::uint32_t {
    SHCSR_USGFAULTENA     = 1 << 18,
    SHCSR_BUSFAULTENA     = 1 << 17,
    SHCSR_MEMFAULTENA     = 1 << 16,
    SHCSR_SVCCALLPENDED   = 1 << 15,
    SHCSR_BAUSFAULTPENDED = 1 << 14,
    SHCSR_MEMFALTPENDED   = 1 << 13,
    SHCSR_USGFAULTPENDED  = 1 << 12,
    SHCSR_SYSTIOCKACT     = 1 << 11,
    SHCSR_PENDSVACT       = 1 << 10,
    SHCSR_MONITORACT      = 1 << 8,
    SHCSR_SVCALLACT       = 1 << 7,
    SHCSR_USGFAULTACT     = 1 << 3,
    SHCSR_BUSFAULTACT     = 1 << 1,
    SHCSR_MEMFAULTACT     = 1
  };
  enum HFSR_Fields {};
  enum DFSR_Fields : std::uint32_t {
    DFSR_EXTERNAL = 1 << 4,
    DFSR_VCATCH   = 1 << 3,
    DFSR_DWTTRAP  = 1 << 2,
    DFSR_BKPT     = 1 << 1,
    DFSR_HALTED   = 1
  };

  enum CFSR_Fields : std::uint32_t {
    CFSR_UsageFault            = 0xFFFF0000,
    CFSR_UsageFault_POS        = 16,
    CFSR_UsageFault_DIVBYZERO  = 1 << 25,
    CFSR_UsageFault_UNALIGNED  = 1 < 24,
    CFSR_UsageFault_NOCP       = 1 << 19,
    CFSR_UsageFault_INVPC      = 1 << 18,
    CFSR_UsageFault_INVSTATE   = 1 << 17,
    CFSR_UsageFault_UNDEFINSTR = 1 << 16,
    CFSR_BusFault              = 0xff00,
    CFSR_BusFault_POS          = 8,
    CFSR_BusFault_VFARVALID    = 1 << 15,
    CFSR_BusFault_LSPERR       = 1 << 13,
    CFSR_BusFault_STKERR       = 1 << 12,
    CFSR_BusFault_UNSTKERR     = 1 << 11,
    CFSR_BusFault_IMPRECISERR  = 1 << 10,
    CFSR_BusFault_PRECISERR    = 1 << 9,
    CFSR_BusFault_IBUSERR      = 1 << 8,
    CFSR_MemManage             = 0xFF,
    CFSR_MemManage_POS         = 0,
    CFSR_MemManage_MMARVALID   = 1 << 7,
    CFSR_MemManage_MLSPERR     = 1 << 5,
    CFSR_MemManage_MSTKERR     = 1 < 4,
    CFSR_MemManage_MUNSTKERR   = 1 << 3,
    CFSR_MemManage_DACCVIOL    = 1 << 1,
    CFSR_MemManage_IACCVIOL    = 1 << 0
  };

  enum CPACR_Fields : std::uint32_t {
    CPACR_CP11              = 0b11 << 22,
    CPACR_CP11_POS          = 22,
    CPACR_CP10              = 0b11 << 20,
    CPACR_CP10_POS          = 20,
    CPACR_CP7               = 0b11 << 14,
    CPACR_CP7_POS           = 14,
    CPACR_CP6               = 0b11 << 12,
    CPACR_CP6_POS           = 12,
    CPACR_CP5               = 0b11 << 10,
    CPACR_CP5_POS           = 10,
    CPACR_CP4               = 0b11 << 8,
    CPACR_CP4_POS           = 8,
    CPACR_CP3               = 0b11 << 6,
    CPACR_CP3_POS           = 6,
    CPACR_CP2               = 0b11 << 4,
    CPACR_CP2_POS           = 4,
    CPACR_CP1               = 0b11 << 2,
    CPACR_CP1_POS           = 2,
    CPACR_CP0               = 0b11,
    CPACR_CP0_POS           = 0,
    CPACR_ACCESS_DENIED     = 0b00,
    CPACR_ACCESS_PRIVILIGED = 0b01,
    CPACR_ACCESS_FULL       = 0b11
  };

  enum FP_Regs : std::uint32_t {
    FPCCR  = 0xE000EF34,
    FPCAR  = 0xE000EF38,
    FPDSCR = 0xE000EF3C,
    MVFR0  = 0xE000EF40,
    MVFR1  = 0xE000EF44
  };

  enum FPCCR_Fields : std::uint32_t {
    FPCCR_ASPEN  = 1u << 31,
    FPCCR_LSPEN  = 1 << 30,
    FPCCR_MONRDY = 1 << 8,
    FPCCR_BFRDY  = 1 << 6,
    FPCCR_MMRDY  = 1 << 5,
    FPCCR_HFRDY  = 1 << 4,
    FPCCR_THREAD = 1 << 3,
    FPCCR_USER   = 1 << 1,
    FPCCR_LSPACT = 1
  };

  enum FPCAR_Fields { FPCAR_ADDRESS = 0xFFFFFFFF << 2, FPCAR_ADDRESS_POS = 2 };

  enum FPDSCR_Fields {
    FPDSCR_AHP       = 1 << 26,
    FPDSCR_DN        = 1 << 25,
    FPDSCR_FZ        = 1 << 24,
    FPDSCR_RMode     = 0b11 << 22,
    FPDSCR_RMode_POS = 22
  };

  enum MVFR0_Fields {
    FP_ROUNDING_MODE       = 0xF << 28,
    FP_ROUNDING_MODE_POS   = 28,
    SHORT_VECTORS          = 0xF << 24,
    SHORT_VECTORS_POS      = 24,
    SQUARE_ROOT            = 0xF << 20,
    SQUARE_ROOT_POS        = 20,
    DIVIDE                 = 0xF << 16,
    DIVIDE_POS             = 16,
    EXCEPTION_TRAPPING     = 0xF << 12,
    EXCEPTION_TRAPPING_POS = 12,
    DOUBLE_PRECISION       = 0xF << 8,
    DOUBLE_PRECISION_POS   = 8,
    SINGLE_PRECISION       = 0xF << 4,
    SINGLE_PRECISION_POS   = 4,
    A_SIMD                 = 0xF,
    A_SIMD_POS             = 0
  };

  enum MVFR1_Fields {
    FP_FUSED_MAC     = 0xF << 28,
    FP_FUSED_MAC_POS = 28,
    FP_HPFP          = 0xF << 24,
    FP_HPFP_POS      = 24,
    D_NaN_MODE       = 0xF << 4,
    D_NaN_MODE_POS   = 4,
    FtZ_MODE         = 0xF,
    FtZ_MODE_POS     = 0
  };

  enum Other_Regs {
    MASTER_CONTROL = 0xE000E000,
    ICTR           = 0xE000E004,
    ACTLR          = 0xE000E008,
    // debug registers
    STIR           = 0xE000EF00,
    // Peripheral Identification Registers
    PID4           = 0xE000EFD0,
    PID5           = 0xE000EFD4,
    PID6           = 0xE000EFD8,
    PID7           = 0xE000EFDC,
    PID0           = 0xE000EFE0,
    PID1           = 0xE000EFE4,
    PID2           = 0xE000EFE8,
    PDI3           = 0xE000EFEC,
    // Component Identification Registers
    CID0           = 0xE000EFF0,
    CID1           = 0xE000EFF4,
    CID2           = 0xE000EFF8,
    CID3           = 0xE000EFFC
  };
};
} // namespace cm
#endif
