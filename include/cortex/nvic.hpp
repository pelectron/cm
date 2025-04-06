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
#ifndef CM_NVIC_HPP
#define CM_NVIC_HPP

#include "cortex/common.hpp"
#include "cortex/scs.hpp"
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <utility>
namespace cm {

template<std::size_t NumIrqs>
constexpr std::size_t vec_table_align() {
  constexpr auto next_pow_2 = [](std::uint32_t num_xcpt) -> std::uint32_t {
    // minimum alignment of vector table is 32 words
    if (num_xcpt <= 32)
      return 32;
    else if (num_xcpt <= 64)
      return 64;
    else if (num_xcpt <= 128)
      return 128;
    else if (num_xcpt <= 256)
      return 256;
    else if (num_xcpt <= 512)
      return 512;
    return 0;
  };
  const std::uint32_t word_align = next_pow_2(NumIrqs + 16);
  return word_align * 4;
}

using IrqHandler = void (*)();

enum class RelocateStatus {
  Success = 0,
  VtorNotPresent,
  InvalidAddress,
  InvalidAlignment
};

/**
 * Standard ARM exception vector table.
 *
 * MaxIrqNum should never be lower than the maximum available IRQ number on the
 * system. Setting this value higher will never have adverse effects, besides
 * (possibly) taking up more space. However, setting this number too low may
 * cause a fault exception as the alignment of the vector table can be
 * incorrect. See the VTOR register description for more details.
 *
 * @tparam MaxIrqNum maximum IRQ number available on the MCU (not the number of
 * IRQs). For example, if the highest IRQ is IRQ90, then MaxIrqNum should be set
 * to 90, not 91.
 */
template<IRQn MaxIrqNum>
struct alignas(vec_table_align<MaxIrqNum + 1>()) VectorTable {
  volatile std::uint32_t initial_sp;
  volatile IrqHandler Reset;
  volatile IrqHandler NMI;
  volatile IrqHandler HardFault;
  volatile IrqHandler MemoryManagementFault;
  volatile IrqHandler BusFault;
  volatile IrqHandler UsageFault;
  volatile IrqHandler reserved7;
  volatile IrqHandler reserved8;
  volatile IrqHandler reserved9;
  volatile IrqHandler reserved10;
  volatile IrqHandler SVCall;
  volatile IrqHandler reserved12;
  volatile IrqHandler reserved13;
  volatile IrqHandler PendSV;
  volatile IrqHandler Systick;
  volatile IrqHandler IRQ[MaxIrqNum + 1];
  // needed becuase else gcc emits a memcpy in unoptimized builds
  constexpr VectorTable& operator=(const VectorTable& other) noexcept {
    initial_sp            = other.initial_sp;
    Reset                 = other.Reset;
    NMI                   = other.NMI;
    HardFault             = other.HardFault;
    MemoryManagementFault = other.MemoryManagementFault;
    BusFault              = other.BusFault;
    UsageFault            = other.UsageFault;
    reserved7             = other.reserved7;
    reserved8             = other.reserved8;
    reserved9             = other.reserved9;
    reserved10            = other.reserved10;
    SVCall                = other.SVCall;
    reserved12            = other.reserved12;
    reserved13            = other.reserved13;
    PendSV                = other.PendSV;
    Systick               = other.Systick;
    for (size_t n = 0; n <= MaxIrqNum; ++n) {
      IRQ[n] = other.IRQ[n];
    }
    return *this;
  }
  // needed becuase else gcc emits a memcpy in unoptimized builds
  volatile VectorTable&
  operator=(const volatile VectorTable& other) volatile noexcept {
    initial_sp            = other.initial_sp;
    Reset                 = other.Reset;
    NMI                   = other.NMI;
    HardFault             = other.HardFault;
    MemoryManagementFault = other.MemoryManagementFault;
    BusFault              = other.BusFault;
    UsageFault            = other.UsageFault;
    reserved7             = other.reserved7;
    reserved8             = other.reserved8;
    reserved9             = other.reserved9;
    reserved10            = other.reserved10;
    SVCall                = other.SVCall;
    reserved12            = other.reserved12;
    reserved13            = other.reserved13;
    PendSV                = other.PendSV;
    Systick               = other.Systick;
    for (size_t n = 0; n <= MaxIrqNum; ++n) {
      IRQ[n] = other.IRQ[n];
    }
    return *this;
  }
};

static_assert(alignof(VectorTable<IRQ0>) == vec_table_align<0>());
static_assert(std::is_trivially_destructible_v<VectorTable<IRQ31>>);
static_assert(alignof(VectorTable<IRQ21>) == 64 * 4);

struct NvicInfo {
  bool standard_nvic;
  uint8_t num_prio_bits;
  uint16_t max_irqn;
  uint32_t vtor_mask;
};

template<typename T>
concept NvicConcept =
    requires(IRQn irq_number, Exception xcpt_number, Priority prio,
             IrqHandler handler, typename T::vector_table_type& vector_table,
             bool copy_old_table) {
      { typename T::Scs{} };
      {
        T::init(&vector_table, copy_old_table)
      } -> std::same_as<RelocateStatus>;
      { T::info() } -> std::same_as<const NvicInfo&>;
      { T::set_base_pri(prio) };
      { T::base_pri() } -> std::same_as<Priority>;
      { T::reset_base_pri() };

      { T::irq_enable(irq_number) };
      { T::irq_disable(irq_number) };
      { T::irq_trigger(irq_number) };
      { T::irq_set_pending(irq_number) };
      { T::irq_clear_pending(irq_number) };
      { T::irq_is_active(irq_number) };
      { T::irq_set_priority(irq_number, prio) };
      { T::irq_current() } -> std::convertible_to<IRQn>;
      { T::irq_set_handler(irq_number, handler) };

      { T::exception_enable(xcpt_number) };
      { T::exception_disable(xcpt_number) };
      { T::exception_set_pending(xcpt_number) };
      { T::exception_clear_pending(xcpt_number) };
      { T::exception_is_active(xcpt_number) };
      {
        T::exception_set_priority(xcpt_number, prio)
      } -> std::convertible_to<bool>;
      {
        T::exception_set_handler(xcpt_number, handler)
      } -> std::convertible_to<bool>;
      { T::exception_current() } -> std::convertible_to<Exception>;
      { T::exceptions_enable() };
      { T::exceptions_disable() };
      { T::wait_for_exception() };

      { T::vector_table() } -> std::same_as<typename T::vector_table_type*>;
      {
        T::relocate_vector_table(vector_table, copy_old_table)
      } -> std::convertible_to<RelocateStatus>;
      { T::can_relocate_vector_table() } -> std::convertible_to<bool>;
    };

/**
 * Models the standard nvic peripheral found in ARM cortex M devices. If an
 * alternative implementation is needed, just provide the same public interface
 * as this trait.
 *
 * To make a differentiation between external interrupts, i.e. non standard arm
 * exceptions usually pended by peripherals, and standard arm exceptions such as
 * the Systick, PendSV or HardFault exception, the nvic_traits use the name
 * interrupt and irq only for external interrupts, i.e. the ARM exceptions named
 * IRQ1, IRQ2, etc. The numbering for irqs is the interrupt number defined by
 * ARM, i.e. interrupt number = exception number - 16. The name exception
 * applies to standard arm exceptions and external interrupts and uses the ARM
 * exception number scheme, i.e. the exception number for IRQ0 is 16.
 *
 * The API is split in two parts:
 *  - irq_xxx functions deal with external interrupts and interrupt numbers.
 * They are generally more efficient when dealing with IRQs only, as they
 * don't check as much stuff as the corresponding exception_xxx functions
 *    - irq_enable
 *    - irq_disable
 *    - irq_trigger
 *    - irq_set_pending
 *    - irq_clear_pending
 *    - irq_is_active
 *    - irq_set_priority
 *    - irq_current
 *    - irq_set_handler
 *  - exception(s)_xxx function can deal with arm system exceptions and external
 * interrupts. They use the exception numbering scheme.
 *    - exception_enable
 *    - exception_disable
 *    - exception_set_pending
 *    - exception_clear_pending
 *    - exception_is_active
 *    - exception_set_priority
 *    - exception_current
 *    - exception_set_handler
 *    - exceptions_enable
 *    - exceptions_disable
 *
 * @tparam arch for now just a tag type
 * @tparam MaxIrqNum the highest IRQ number available on the MCU. This is
 * inclusive, i.e. if IRQ90 is the highest available, MaxIrqNum should be set to
 * 90, not 91.
 */
template<typename Arch, IRQn MaxIrqNum>
struct Nvic {
  using Scs               = cm::Scs<Arch>;
  using vector_table_type = VectorTable<MaxIrqNum>;

  static CM_INLINE RelocateStatus init(vector_table_type* table = nullptr,
                                       bool copy_old_table = true) noexcept {
    nvic_info.standard_nvic = true;
    nvic_info.max_irqn      = Scs::max_irq_number();
    exceptions_disable();
    nvic_info.num_prio_bits = num_prio_bits();
    nvic_info.vtor_mask     = vtor_mask();
    exceptions_enable();
    if (table == nullptr)
      return RelocateStatus::Success;
    return relocate_vector_table(*table, copy_old_table);
  }

  static CM_INLINE const NvicInfo& info() noexcept { return nvic_info; }

  /// enable an interrupt
  static CM_INLINE void irq_enable(IRQn irq_number) noexcept {
    const std::uint32_t word_offset = irq_number / 32u;
    const std::uint32_t bit_pos     = irq_number % 32;
    volatile_set_bits<std::uint32_t>(NVIC_ISER_BASE_REG + word_offset,
                                     1u << bit_pos);
  }

  /// disable an interrupt
  static CM_INLINE void irq_disable(IRQn irq_number) noexcept {
    const std::uint32_t word_offset = irq_number / 32u;
    const std::uint32_t bit_pos     = irq_number % 32;
    volatile_set_bits<std::uint32_t>(NVIC_ICER_BASE_REG + word_offset,
                                     1u << bit_pos);
  }

  /// trigger an interrupt by software
  static CM_INLINE void irq_trigger(IRQn irq_number) noexcept {
    volatile_write<std::uint32_t>(STIR_REG, STIR_MASK, irq_number);
  }

  /// same as irq_trigger, but more inefficient.
  static CM_INLINE void irq_set_pending(IRQn irq_number) noexcept {
    const std::uint32_t word_offset = irq_number / 32u;
    const std::uint32_t bit_pos     = irq_number % 32;
    volatile_set_bits<std::uint32_t>(NVIC_ISPR_BASE_REG + word_offset,
                                     1u << bit_pos);
  }

  /// clear a pending interrupt
  static CM_INLINE void irq_clear_pending(IRQn irq_number) noexcept {
    const std::uint32_t word_offset = irq_number / 32u;
    const std::uint32_t bit_pos     = irq_number % 32;
    volatile_set_bits<std::uint32_t>(NVIC_ICPR_BASE_REG + word_offset,
                                     1u << bit_pos);
  }

  /// set the priority of an interrupt
  static CM_INLINE void irq_set_priority(IRQn irq_number,
                                         Priority priority) noexcept {
    volatile_write<std::uint8_t>(NVIC_IPR_BASE_REG + irq_number, priority);
  }

  /// returns true if interrupt is active
  static CM_INLINE bool irq_is_active(IRQn irq_number) noexcept {
    const std::uint32_t word_offset = irq_number / 32u;
    const std::uint32_t bit_pos     = irq_number % 32;
    return (1u << bit_pos) ==
           volatile_read_bits<std::uint32_t>(NVIC_IABR_BASE_REG + word_offset,
                                             1u << bit_pos);
  }

  /// get the number of the currently executing interrupt. Returns MaxIrqNum +1
  /// if no interrupt is active.
  static CM_INLINE IRQn irq_current() noexcept {
    std::uint32_t vect_active =
        volatile_read_bits<std::uint32_t>(Scs::ICSR, Scs::ICSR_VECTACTIVE);
    if (vect_active < 16)
      return static_cast<IRQn>(MaxIrqNum + 1);
    return static_cast<IRQn>(vect_active - 16);
  }

  static CM_INLINE bool irq_set_handler(IRQn irq, IrqHandler handler) noexcept {
    vector_table()->IRQ[irq] = handler;
    return true;
  }

  static CM_INLINE void exception_enable(Exception xcpt_number) noexcept {
    switch (xcpt_number) {
    case Reset:
      break; // always enabled
    case NonMaskableInterrupt:
      break; // always enabled
    case HardFault:
      break; // always enabled
    case MemoryManagementFault:
      if constexpr (requires { Scs::SHCSR_MEMFAULTENA; })
        volatile_set_bits<std::uint32_t>(Scs::SHCSR, Scs::SHCSR_MEMFAULTENA);
      break;
    case BusFault:
      if constexpr (requires { Scs::SHCSR_BUSFAULTENA; })
        volatile_set_bits<std::uint32_t>(Scs::SHCSR, Scs::SHCSR_BUSFAULTENA);
      break;
    case UsageFault:
      if constexpr (requires { Scs::SHCSR_USGFAULTENA; })
        volatile_set_bits<std::uint32_t>(Scs::SHCSR, Scs::SHCSR_USGFAULTENA);
      break;
    case SVCall:
      break; // always enabled
    case PendSV:
      break; // always enabled
    case SysTick:
      break; // must be enabled in systick
    default:
      if (xcpt_number >= 16 and xcpt_number <= (MaxIrqNum + 16)) {
        // xcpt_number is an external interrupt number
        irq_set_pending(static_cast<IRQn>(xcpt_number - 16u));
      }
    }
  }

  static CM_INLINE void exception_disable(Exception xcpt_number) noexcept {

    switch (xcpt_number) {
    case Reset:
      break; // invalid
    case NonMaskableInterrupt:
      break; // invalid
    case HardFault:
      break; // invalid
    case MemoryManagementFault:
      if constexpr (requires { Scs::SHCSR_MEMFAULTENA; })
        volatile_reset_bits(Scs::SHCSR, Scs::SHCSR_MEMFAULTENA);
      break;
    case BusFault:
      if constexpr (requires { Scs::SHCSR_BUSFAULTENA; })
        volatile_reset_bits(Scs::SHCSR, Scs::SHCSR_BUSFAULTENA);
      break;
    case UsageFault:
      if constexpr (requires { Scs::SHCSR_USGFAULTENA; })
        volatile_reset_bits(Scs::SHCSR, Scs::SHCSR_USGFAULTENA);
      break;
    case SVCall:
      break; // invalid
    case PendSV:
      break; // invalid
    case SysTick:
      break; // TODO: need to do it via systick for now
    case IRQ0_Exception:
      [[fallthrough]];
    case IRQmax_Exception:
      [[fallthrough]];
    default:
      if (xcpt_number >= 16 and xcpt_number <= (MaxIrqNum + 16)) {
        // xcpt_number is an external interrupt number
        irq_disable(static_cast<IRQn>(xcpt_number - 16u));
      }
    }
  }

  static CM_INLINE void exception_set_pending(Exception xcpt_number) noexcept {
    switch (xcpt_number) {
    case Reset:
      break; // invalid
    case NonMaskableInterrupt:
      volatile_set_bits<std::uint32_t>(Scs::ICSR, Scs::ICSR_NMIPENDSET);
      break;
    case HardFault:
      break; // invalid
    case MemoryManagementFault:
      break; // invalid
    case BusFault:
      break; // invalid
    case UsageFault:
      break; // invalid
    case SVCall:
      break; // TODO
    case PendSV:
      volatile_set_bits<std::uint32_t>(Scs::ICSR, Scs::ICSR_PENDSVSET);
      break;
    case SysTick:
      volatile_set_bits<std::uint32_t>(Scs::ICSR, Scs::ICSR_PENDSTSET);
      break;

    case IRQ0_Exception:
      [[fallthrough]];
    case IRQmax_Exception:
      [[fallthrough]];
    default:
      if (xcpt_number >= 16 and xcpt_number <= (MaxIrqNum + 16)) {
        // xcpt_number is an external interrupt number
        irq_set_pending(static_cast<IRQn>(xcpt_number - 16u));
      }
    }
  }

  static CM_INLINE void
  exception_clear_pending(Exception xcpt_number) noexcept {
    switch (xcpt_number) {
    case Reset:
      break; // invalid
    case NonMaskableInterrupt:
      break; // invalid
    case HardFault:
      break; // invalid
    case MemoryManagementFault:
      break; // invalid
    case BusFault:
      break; // invalid
    case UsageFault:
      break; // invalid
    case SVCall:
      break; // invalid
    case PendSV:
      volatile_set_bits<std::uint32_t>(Scs::ICSR, Scs::ICSR_PENDSVCLR);
      break;
    case SysTick:
      volatile_set_bits<std::uint32_t>(Scs::ICSR, Scs::ICSR_PENDSTCLR);
      break;

    case IRQ0_Exception:
      [[fallthrough]];
    case IRQmax_Exception:
      [[fallthrough]];
    default:
      if (xcpt_number >= 16 and xcpt_number <= (MaxIrqNum + 16)) {
        // xcpt_number is an external interrupt number
        irq_clear_pending(static_cast<IRQn>(xcpt_number - 16u));
      }
    }
  }

  static CM_INLINE bool exception_set_priority(Exception xcpt_number,
                                               Priority priority) noexcept {
    switch (xcpt_number) {
    case Reset:
      [[fallthrough]];
    case NonMaskableInterrupt:
      [[fallthrough]];
    case HardFault:
      // cannot set priorities for the exceptions above
      return false;
    case MemoryManagementFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        volatile_write<std::uint32_t>(Scs::SHPR1,
                                      Scs::SHPR1_PRI_4,
                                      priority << Scs::SHPR1_PRI_4_POS);
        return true;
      } else {
        return false;
      }
    case BusFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        volatile_write<std::uint32_t>(Scs::SHPR1,
                                      Scs::SHPR1_PRI_5,
                                      priority << Scs::SHPR1_PRI_5_POS);
        return true;
      } else {
        return false;
      }
    case UsageFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        volatile_write<std::uint32_t>(Scs::SHPR1,
                                      Scs::SHPR1_PRI_6,
                                      priority << Scs::SHPR1_PRI_6_POS);
        return true;
      } else {
        return false;
      }
    case SVCall:
      volatile_write<std::uint32_t>(Scs::SHPR2,
                                    Scs::SHPR2_PRI_11,
                                    priority << Scs::SHPR2_PRI_11_POS);
      return true;
    case PendSV:
      volatile_write<std::uint32_t>(Scs::SHPR3,
                                    Scs::SHPR3_PRI_14,
                                    (priority & 0b11) << Scs::SHPR3_PRI_14_POS);
      return true;
    case SysTick:
      volatile_write<std::uint32_t>(Scs::SHPR3,
                                    Scs::SHPR3_PRI_15,
                                    priority << Scs::SHPR3_PRI_15_POS);
      return true;
    case IRQ0_Exception:
      [[fallthrough]];
    case IRQmax_Exception:
      [[fallthrough]];
    default:
      if (xcpt_number >= 16 and xcpt_number <= (MaxIrqNum + 16)) {
        // xcpt_number is an external interrupt number
        irq_set_priority(static_cast<IRQn>(xcpt_number - 16u), priority);
        return true;
      }
      return false;
    }
  }

  static CM_INLINE bool exception_is_active(Exception xcpt_number) noexcept {
    switch (xcpt_number) {
    case Reset:
      break; // invalid
    case NonMaskableInterrupt:
      break; // invalid
    case HardFault:
      break; // invalid
    case MemoryManagementFault:
      return volatile_read_bits<std::uint32_t>(Scs::SHCSR,
                                               Scs::SHCSR_MEMFAULTACT) != 0;
    case BusFault:
      return volatile_read_bits<std::uint32_t>(Scs::SHCSR,
                                               Scs::SHCSR_BUSFAULTACT) != 0;
    case UsageFault:
      return volatile_read_bits<std::uint32_t>(Scs::SHCSR,
                                               Scs::SHCSR_USGFAULTACT) != 0;
    case SVCall:
      return volatile_read_bits<std::uint32_t>(Scs::SHCSR,
                                               Scs::SHCSR_SVCALLACT) != 0;
    case PendSV:
      return volatile_read_bits<std::uint32_t>(Scs::SHCSR,
                                               Scs::SHCSR_PENDSVACT) != 0;
    case SysTick:
      return volatile_read_bits<std::uint32_t>(Scs::SHCSR,
                                               Scs::SHCSR_SYSTICKACT) != 0;
    default:
      if (xcpt_number >= 16 and xcpt_number <= (MaxIrqNum + 16)) {
        // xcpt_number is an external interrupt number
        irq_is_active(static_cast<IRQn>(xcpt_number - 16u));
      }
    }
  }

  /// returns the current active exception number, or 0 if no exception is
  /// active.
  static CM_INLINE Exception exception_current() noexcept {
    return volatile_read_bits<std::uint32_t>(Scs::ICSR_REG,
                                             Scs::ICSR_VECTACTIVE);
  }

  static CM_INLINE bool exception_set_handler(Exception xcpt_number,
                                              IrqHandler handler) noexcept {
    switch (xcpt_number) {
    case Reset:
      vector_table()->Reset = handler;
      return true;
    case NonMaskableInterrupt:
      vector_table()->NMI = handler;
      return true;
    case HardFault:
      vector_table()->HardFault = handler;
      return true;
    case MemoryManagementFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        vector_table()->MemoryManagementFault = handler;
        return true;
      } else {
        return false;
      }
    case BusFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        vector_table()->BusFault = handler;
        return true;
      } else {
        return false;
      }
    case UsageFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        vector_table()->UsageFault = handler;
        return true;
      } else {
        return false;
      }
    case SVCall:
      vector_table()->SVCall = handler;
      return true;
    case PendSV:
      vector_table()->PendSV = handler;
      return true;
    case SysTick:
      vector_table()->Systick = handler;
      return true;
    case IRQ0_Exception:
      [[fallthrough]];
    case IRQmax_Exception:
      [[fallthrough]];
    default:
      if (xcpt_number >= 16 and xcpt_number <= (MaxIrqNum + 16)) {
        // xcpt_number is an external interrupt number
        return irq_set_handler(static_cast<IRQn>(xcpt_number - 16u), handler);
      }
      return false;
    }
  }

  /// sets a new interrupt vector table. The old vector table is first copied
  /// into new_vector_table before the VTOR register is set if copy_old_table is
  /// true. The return status indicates wether the relocation succeeded or
  /// failed.
  static CM_INLINE RelocateStatus
  relocate_vector_table(vector_table_type& new_vector_table,
                        bool copy_old_table = true) noexcept {
    const auto addr = reinterpret_cast<std::uint32_t>(&new_vector_table);

    // vtor is not present
    if (nvic_info.vtor_mask == 0)
      return RelocateStatus::VtorNotPresent;

    // table address out of range
    if ((nvic_info.vtor_mask & addr) != addr)
      return RelocateStatus::InvalidAddress;

    // cannot set new vector table because alignment is off.
    if (addr % vec_table_align<MaxIrqNum + 1>() != 0)
      return RelocateStatus::InvalidAlignment;

    // copy old table into new table
    if (copy_old_table)
      new_vector_table = *vector_table();

    volatile_write<std::uint32_t>(Scs::VTOR, addr);
    return RelocateStatus::Success;
  }

  static CM_INLINE vector_table_type* vector_table() noexcept {
    return volatile_read<vector_table_type*>(Scs::VTOR);
  }

  /// sets PRIMASK to 0. This enables exceptions and interrupts with
  /// configurable priorities to become active again, if they were disabled
  /// before.
  static CM_INLINE void exceptions_enable() noexcept {
    __asm volatile(" CPSIE i");
  }

  /// sets PRIMASK to 1. This prevents any exceptions and interrupts with
  /// configurable priority from becoming active.
  static CM_INLINE void exceptions_disable() noexcept {
    __asm volatile(" CPSID i");
  }

  static CM_INLINE void set_base_pri(Priority p) noexcept {
    __asm volatile(
        "   ldr r0, %0                                              \n"
        "   msr basepri, r0                                         \n"
        "   isb                                                     \n"
        "   dsb                                                     \n" ::"o"(p)
        : "memory");
  }

  static CM_INLINE Priority base_pri() noexcept {
    uint32_t p;
    __asm volatile("   mrs r0, basepri \n"
                   "   mov %[p], r0    \n"
                   : [p] "=r"(p));
    return static_cast<Priority>(p);
  }

  static CM_INLINE void reset_base_pri() noexcept {
    __asm volatile("  mov r0, #0 \n"
                   "  msr basepri, r0 \n"
                   "  isb \n"
                   "  dsb \n");
  }
  /// issues WFI instruction.
  static CM_INLINE void wait_for_exception() noexcept {
    __asm volatile(" WFI");
  }

  static CM_INLINE bool can_relocate_vector_table() noexcept {
    return nvic_info.vtor_mask != 0;
  }

private:
  enum Regs : std::uint32_t {
    CPUID_REG     = 0xE000ED00,
    ICSR_REG      = 0xE000ED04,
    VTOR_REG      = 0xE000ED08,
    AIRCR_REG     = 0xE000ED0C,
    SCR_REG       = 0xE000ED10,
    CCR_REG       = 0xE000ED14,
    SHPR_BASE_REG = 0xE000ED18,
    SHPR1_REG     = SHPR_BASE_REG,
    SHRP2_REG     = 0xE000ED1C,
    SHPR3_REG     = 0xE000ED20,
    SHCSR_REG     = 0xE000ED24,
    CFSR_REG      = 0xE000ED28,
    HFSR_REG      = 0xE000ED2C,
    DFSR_REG      = 0xE000ED30,

    ICTR_REG = 0xE000E004,
    STIR_REG = 0xE000EF00,

    NVIC_ISER_BASE_REG = 0xE000E100,
    NVIC_ICER_BASE_REG = 0xE000E180,
    NVIC_ISPR_BASE_REG = 0xE000E200,
    NVIC_ICPR_BASE_REG = 0xE000E280,
    NVIC_IABR_BASE_REG = 0xE000E300,
    NVIC_IPR_BASE_REG  = 0xE000E400
  };

  enum Bits : std::uint32_t {
    ICSR_NMIPENDSET  = 1u << 31,
    ICSR_PENDSVSET   = 1u << 28,
    ICSR_PENDSVCLR   = 1u << 27,
    ICSR_PENDSTSET   = 1u << 26,
    ICSR_PENDSTCLR   = 1u << 25,
    ICSR_ISRPREEMPT  = 1u << 23,
    ICSR_ISRPENDING  = 1u << 22,
    ICSR_VECTPENDING = 0x1FFu << 12, // bits 20 to 12
    ICSR_RETTOBASE   = 1u << 11,
    ICSR_VECTACTIVE  = 0x1FFu,

    SHCSR_USGFAULTENA    = 1 << 18,
    SHCSR_BUSFAULTENA    = 1 << 17,
    SHCSR_MEMFAULTENA    = 1 << 16,
    SHCSR_SVCALLPENDED   = 1 << 15,
    SHCSR_BUSFAULTPENDED = 1 << 14,
    SHCSR_MEMFAULTPENDED = 1 << 13,
    SHCSR_USGFAULTPENDED = 1 << 12,
    SHCSR_SYSTICKACT     = 1 << 11,
    SHCSR_PENDSVACT      = 1 << 10,
    SHCSR_MONITORACT     = 1 << 8,
    SHCSR_SVCALLACT      = 1 << 7,
    SHCSR_USGFAULTACT    = 1 << 3,
    SHCSR_BUSFAULTACT    = 1 << 1,
    SHCSR_MEMFAULTACT    = 1 << 0,

    STIR_MASK     = 0x1F,
    VTOR_TBL_BASE = 1 << 29
  };

  static_assert((ICSR_VECTPENDING & (1 << 21)) == 0);
  static_assert((ICSR_VECTPENDING & (1 << 20)) == (1 << 20));
  static_assert((ICSR_VECTPENDING & (1 << 12)) == (1 << 12));
  static_assert((ICSR_VECTPENDING & (1 << 11)) == 0);

  enum class VtorRange { begin = 0x00000080, end = 0x3FFFFF80 };

  static CM_INLINE std::uint32_t vtor_mask() noexcept {
    const std::uint32_t vtor = volatile_read<std::uint32_t>(Scs::VTOR);
    volatile_write<std::uint32_t>(Scs::VTOR, 0xFFFFFFFFu);
    const std::uint32_t mask = volatile_read<std::uint32_t>(Scs::VTOR);
    volatile_write<std::uint32_t>(Scs::VTOR, vtor);
    return mask;
  }

  static inline constinit NvicInfo nvic_info{};

  static CM_INLINE unsigned num_prio_bits() noexcept {
    uint32_t leading_zeros = 0;
    __asm volatile(
        // store old base_pri
        "   mrs r1, basepri           \n"
        // write 0xFF to basepri
        "   mov r0, #0xFF             \n"
        "   msr basepri, r0           \n"
        // read back base_pri and put result into max
        "   mrs r0, basepri           \n"
        // reverse bit order and count leading zeros
        "   rbit r0, r0               \n"
        "   clz r0, r0                \n"
        "   mov %[leading_zeros], r0  \n"
        // restoring base_pri
        "   msr basepri, r1           \n"
        : [leading_zeros] "=r"(leading_zeros));
    if (leading_zeros > 8)
      return 0;
    return 8 - leading_zeros;
  }
};

extern "C" {
extern unsigned _estack;
void Reset_Handler();
void NMI_Handler();
void HardFault_Handler();
void SVC_Handler();
void PendSV_Handler();
void SysTick_Handler();
void MemoryManagementFault_Handler();
void BusFault_Handler();
void UsageFault_Handler();
}

/**
 * @brief The HardTableNvic must be used with the event dispatcher in case the
 *
 * @tparam Arch
 */
template<typename Arch, IRQn MaxIrqNum>
struct HardTableNvic : public Nvic<Arch, MaxIrqNum> {
public:
  using Scs               = typename Nvic<Arch, MaxIrqNum>::Scs;
  using vector_table_type = typename Nvic<Arch, MaxIrqNum>::vector_table_type;

  static CM_INLINE RelocateStatus
  init(vector_table_type* table = nullptr,
       bool copy_old_table      = false /* ignored */) noexcept {
    Nvic<Arch, MaxIrqNum>::init(nullptr, copy_old_table);
    rt_vec_table = table;
    return RelocateStatus::Success;
  }

  static vector_table_type* vector_table() noexcept { return rt_vec_table; }

  static RelocateStatus
  relocate_vector_table(vector_table_type& new_vector_table,
                        bool copy_old_table = true) {
    if (copy_old_table) {
      new_vector_table = *rt_vec_table;
    }
    rt_vec_table = &new_vector_table;
    return RelocateStatus::Success;
  }

  static constexpr bool can_relocate_vector_table() { return true; }

  static CM_INLINE bool irq_set_handler(IRQn irq, IrqHandler handler) noexcept {
    if (rt_vec_table == nullptr)
      return false;
    rt_vec_table->IRQ[irq] = handler;
    return true;
  }

  static CM_INLINE bool exception_set_handler(Exception xcpt_number,
                                              IrqHandler handler) noexcept {
    if (rt_vec_table == nullptr)
      return false;
    switch (xcpt_number) {
    case Reset:
      rt_vec_table->Reset = handler;
      return true;
    case NonMaskableInterrupt:
      rt_vec_table->NMI = handler;
      return true;
    case HardFault:
      rt_vec_table->HardFault = handler;
      return true;
    case MemoryManagementFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        rt_vec_table->MemoryManagementFault = handler;
        return true;
      } else {
        return false;
      }
    case BusFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        rt_vec_table->BusFault = handler;
        return true;
      } else {
        return false;
      }
    case UsageFault:
      if constexpr (std::is_same_v<Arch, armv7m>) {
        rt_vec_table->UsageFault = handler;
        return true;
      } else {
        return false;
      }
    case SVCall:
      rt_vec_table->SVCall = handler;
      return true;
    case PendSV:
      rt_vec_table->PendSV = handler;
      return true;
    case SysTick:
      rt_vec_table->Systick = handler;
      return true;
    case IRQ0_Exception:
      [[fallthrough]];
    case IRQmax_Exception:
      [[fallthrough]];
    default:
      if (xcpt_number >= 16 and xcpt_number <= (MaxIrqNum + 16)) {
        // xcpt_number is an external interrupt number
        return irq_set_handler(static_cast<IRQn>(xcpt_number - 16u), handler);
      } else {
        return false;
      }
    }
  }

private:
  template<size_t I>
  static void IRQ_Handler() {
    rt_vec_table->IRQ[I]();
  }

  template<size_t... Is>
  static vector_table_type create_table(std::index_sequence<Is...>) {
    vector_table_type t{};
    t.initial_sp = _estack;
    t.Reset      = Reset_Handler;
    t.NMI        = NMI_Handler;
    t.HardFault  = HardFault_Handler;
    t.SVCall     = SVC_Handler;
    t.PendSV     = PendSV_Handler;
    if constexpr (std::is_same_v<Arch, armv7m>) {
      t.MemoryManagementFault = MemoryManagementFault_Handler;
      t.BusFault              = BusFault_Handler;
      t.UsageFault            = UsageFault_Handler;
    }
    (static_cast<void>(t.IRQ[Is] = IRQ_Handler<Is>), ...);
    return t;
  }

  /**
   * @brief the vector table located in flash at 0x0. You will need to define a
   * section called '.isr_vector' in your linker script to use this.
   */
  __attribute__((section(
      ".isr_vector"))) static const inline vector_table_type flash_vector_table{
      create_table(std::make_index_sequence<MaxIrqNum + 1>())};

  static inline vector_table_type* volatile rt_vec_table{};
};

static_assert(NvicConcept<Nvic<armv6m, IRQ31>>);
static_assert(NvicConcept<Nvic<armv7m, IRQ31>>);
static_assert(NvicConcept<HardTableNvic<armv7m, IRQ31>>);

} // namespace cm
#endif
