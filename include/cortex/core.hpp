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
#ifndef CM_CORE_HPP
#define CM_CORE_HPP

#include "cortex/common.hpp"
#include "cortex/fpu.hpp"
#include "cortex/mpu.hpp"
#include "cortex/nvic.hpp"
#include "cortex/systick.hpp"

#include <cstddef>
#include <cstdint>

namespace cm {
/**
 * os::cortex::Core is the "Core" type passed to os::Scheduler to select the
 * cortex SchedulerImplementation. The provided cortex scheduler implementation
 * is located in os/cortex/scheduler.hpp.
 *
 * Standard cores are defined below (os::cortex::M3/M4/M4F/M7), but if your
 * cortex M MCU has a non standard systick, nvic, or fpu peripheral
 * implementation, this can be injected here.
 *
 * Trait parameters:
 *  - SysTickTraits: the systick to use
 *  - NvicTraits: the nvic to use
 *  - FpuTraits: the fpu to use
 *
 * See the corresponding files in the os/cortex folder for more information
 * about the traits.
 */
// clang-format off
template<typename Arch,
         Systick Systick,
         NvicConcept Nvic,
         FpuConcept Fpu,
         MpuConcept Mpu,
         std::size_t CacheLineSize = 4>
// clang-format on
struct Core {
  /// @{ required by rest of the system (kernel, tasks, etc)
  static constexpr bool stack_grows_upwards    = true;
  static constexpr std::size_t cache_line_size = CacheLineSize;
  /// @}

  /// required by the cortex scheduler implementation
  /// @{
  using scs     = Scs<Arch>;
  using systick = Systick;
  using nvic    = Nvic;
  using fpu     = Fpu;
  /// @}
  static_assert(std::is_same_v<typename nvic::Scs, scs>);

  using tick_type         = std::uint32_t;
  using vector_table_type = typename nvic::vector_table_type;

  static void init(tick_type systick_reload_value,
                   vector_table_type* vector_table = nullptr,
                   bool copy_old_vector_table      = true) {
    fpu::init();
    nvic::init(vector_table, copy_old_vector_table);
    systick::init(systick_reload_value);
  }
};

/**
 * The section below defines standard cortex M "Core"s to use with the kernel.
 * All standard cores are templated on three parameters:
 *
 *  - MaxIrqNum: This is the maximum external interrupt number (interrupt number
 * as defined by ARM) on the cortex M device. Note that this is not the amount
 * of interrupts available, as that may be lower! It is always safe to use a
 * number too large (just some space will be wasted), but using a number too low
 * will cause unpredictable faults.
 *
 *  - PrioBits: the amount of priority bits implemented by the core. Used by the
 * rest of the system to figure out the absolut maximum os::Priority available.
 *
 *  - HasVTor: Indicates that the vector table can be relocated. Must be true
 * for now.
 *
 *  - CacheLineSize: the cache line size of the processor. For cores without a
 * cache, this can be left at 4. For system which have a cache, set this to the
 * processors cache line size, usually 32 for cortex m devices.
 * @{
 */

/// The standard cortex M0+ Core. See above for the documentation of parameters.
// clang-format off
template<IRQn MaxIrqNum,
         MpuConcept Mpu         = NoMpu, 
         Systick Systick = StdSystick,
         NvicConcept Nvic       = HardTableNvic<armv6m, MaxIrqNum>>
struct M0PLUS : Core<armv6m, StdSystick, Nvic, NoFpu, Mpu, 4> {
  static_assert(MaxIrqNum < IRQ32,"The maximum IRQ number for the cortex M0+ is 31");
};
// clang-format on
/// The standard cortex M3 Core. See above for the documentation of parameters.
// clang-format off
template<IRQn MaxIrqNum,
         MpuConcept Mpu         = NoMpu, 
         FpuConcept Fpu         = fpv4_sp,
         Systick Systick = StdSystick,
         NvicConcept Nvic       = Nvic<armv6m, MaxIrqNum>>
struct M3 : Core<armv6m, StdSystick, Nvic, NoFpu, Mpu, 4> {};
// clang-format on

/// /// The standard cortex M4 Core. See above for the documentation of
/// /// parameters.
/// template <std::size_t MaxIrqNum, std::uint8_t PrioBits, bool
/// HasVtor = true,
///           std::size_t CacheLineSize = 4>
/// using M4 =
///     Core<armv7m, systick<armv7m>, nvic<armv7m, MaxIrqNum, PrioBits,
///     HasVtor>,
///          fpu_traits<>, CacheLineSize>;
///
/// /// The standard cortex M4 Core with FPU. See above for the documentation of
/// /// parameters.
/// template <std::size_t MaxIrqNum, std::uint8_t PrioBits, bool HasVtor = true,
///           std::size_t CacheLineSize = 4>
/// using M4F = Core<armv7m_fp, systick<armv7m_fp>,
///                  nvic<armv7m_fp, MaxIrqNum, PrioBits, HasVtor>,
///                  fpu_traits<armv7m_fp>, CacheLineSize>;
///
/// template <std::size_t MaxIrqNum, std::uint8_t PrioBits, bool HasVtor = true,
///           std::size_t CacheLineSize = 32>
/// using M7 = Core<armv7m_fp, systick<armv7m_fp>,
///                 nvic<armv7m_fp, MaxIrqNum, PrioBits, HasVtor>,
///                 fpu_traits<armv7m_fp>, CacheLineSize>;
/// @}
} // namespace cm
#endif
