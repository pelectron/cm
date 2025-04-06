/**
 *          Copyright Pelé Constam 2025.
 * Distributed under the Boost Software License, Version 1.0.
 *    (See accompanying file LICENSE_1_0.txt or copy at
 *          https://www.boost.org/LICENSE_1_0.txt)
 */
#ifndef CM_FPU_HPP
#define CM_FPU_HPP
#include "cortex/common.hpp"

namespace cm {

template<typename T>
concept FpuConcept =
    requires(typename T::float_type*, typename T::double_type*) {
      { T::init() };
    };

struct NoFpu {
  using float_type  = void;
  using double_type = void;
  CM_INLINE static constexpr void init() noexcept {}
};

struct fpv4_sp {
  using float_type  = float;
  using double_type = float;
  /**
   * Initializes the standard FPU found in armv7-m devices.
   * That means enabling lazy context saving and enabling the CP10 and CP11
   * coprocessors.
   */
  CM_INLINE static void init() noexcept {
    // see the ARMv7-M Architecture Reference Manual

    // This asm block first enables lazy context saving of the fpu, and then
    // enables the fpu itself
    __asm volatile(
        // enable lazy context save
        // FPCCR is located at address 0xE000EF34
        " ldr.w r0, =0xE000EF34     \n"
        // read reg
        "	ldr r1, [r0]				      \n"
        // set ASPEN and LSPEN bits (bit 31 and 30)
        "	orr r1, r1, #( 0x3 << 30 )\n"
        // Write back the modified value to the FPCCR
        "	str r1, [r0]				      \n"

        // enable fpu coprocessors
        // CPACR is located at address 0xE000ED88
        "	ldr.w r0, =0xE000ED88		  \n"
        // Read CPACR
        "	ldr r1, [r0]				      \n"
        // Set bits 20-23 to enable CP10 and CP11 coprocessors
        "	orr r1, r1, #( 0xf << 20 )\n"
        // Write back the modified value to the CPACR
        "	str r1, [r0]				      \n"
        "	bx r14						          ");
  }
};

struct fpv5_sp_d16_m : fpv4_sp {};

struct fpv5_d16_m : fpv4_sp {
  using double_type = double;
};

static_assert(FpuConcept<NoFpu>);
static_assert(FpuConcept<fpv4_sp>);
static_assert(FpuConcept<fpv5_sp_d16_m>);
static_assert(FpuConcept<fpv5_d16_m>);

} // namespace cm
#endif
