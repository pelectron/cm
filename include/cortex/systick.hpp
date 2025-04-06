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
#ifndef CM_SYSTICK_HPP
#define CM_SYSTICK_HPP

#include "cortex/common.hpp"
#include <concepts>

namespace cm {

enum class TickRateStatus {
  Success,
  NoCalibrationValue,
  NoReferenceClock,
  InaccurateCalibrationValue,
  ReloadValueOutOfRange,
};

template<typename T>
concept Systick =
    requires(typename T::tick_type reload_value, bool use_processor_clock,
             MicroSeconds us, bool ignore_inaccurate_calib) {
      { T::max_reload_value } -> std::convertible_to<typename T::tick_type>;
      { T::init(reload_value) };
      { T::enable() };
      { T::disable() };
      { T::enable_interrupt() };
      { T::disable_interrupt() };
      { T::set_clock(use_processor_clock) } -> std::convertible_to<bool>;
      { T::set_reload_value(reload_value) };
      { T::set_current_value(reload_value) };
      { T::reference_clock_is_implemented() } -> std::convertible_to<bool>;
      { T::calibration_value_is_exact() } -> std::convertible_to<bool>;
      { T::calibration_value() } -> std::convertible_to<typename T::tick_type>;
      {
        T::set_tick_rate(us, ignore_inaccurate_calib)
      } -> std::same_as<TickRateStatus>;
    };

/**
 * The standard ARM Systick implementation
 */
struct StdSystick {
  using tick_type                             = std::uint32_t;
  static constexpr tick_type max_reload_value = 0xFFFFFFu;
  /**
   * Initializes the systick peripheral.
   * This will set the clock source to the processor clock,
   * enables the systick to pend the Systick exception, set the
   * reload value, and enable the timer.
   */
  static CM_INLINE void init(tick_type reload_value) noexcept {
    disable();
    // set the Systick to use the processor clock
    set_clock(true);
    set_reload_value(reload_value);
    // TODO: maybe not choose 0 for current value?
    set_current_value(0);
    enable_interrupt();
    enable();
  }

  /// enable the timer
  static CM_INLINE void enable() {
    volatile_set_bits<std::uint32_t>(CTRL_AND_STATUS_REG, ENABLE);
  }

  /// disable the timer
  static CM_INLINE void disable() {
    volatile_reset_bits<std::uint32_t>(CTRL_AND_STATUS_REG, ENABLE);
  }

  /**
   * Enables the Systick to pend the Systick exception when the counter
   * reaches 0.
   */
  static CM_INLINE void enable_interrupt() {
    volatile_set_bits<std::uint32_t>(CTRL_AND_STATUS_REG, TICKINT);
  }

  /**
   * Disables the Systick to pend the Systick exception.
   */
  static CM_INLINE void disable_interrupt() {
    volatile_set_bits<std::uint32_t>(CTRL_AND_STATUS_REG, TICKINT);
  }

  /**
   * Sets the Systick clock source. If use_processor_clock is true, the Systick
   * will use the processor clock. Else it will use the (optional) reference
   * clock. Returns wether the processor clock is used (true) or if the external
   * reference clock is used (false), as the reference clock may not be
   * implemented.
   */
  static CM_INLINE bool set_clock(bool use_processor_clock) {
    if (use_processor_clock) {
      volatile_set_bits<std::uint32_t>(CTRL_AND_STATUS_REG, CLKSOURCE);
    } else {
      volatile_reset_bits<std::uint32_t>(CTRL_AND_STATUS_REG, CLKSOURCE);
    }
    return volatile_read_bits<std::uint32_t>(CTRL_AND_STATUS_REG, CLKSOURCE) ==
           CLKSOURCE;
  }

  /**
   * returns true if the systick is using the processor clock
   */
  static CM_INLINE bool uses_processor_clock() {
    return volatile_read_bits<std::uint32_t>(CTRL_AND_STATUS_REG, CLKSOURCE) ==
           CLKSOURCE;
  }

  static CM_INLINE void set_reload_value(tick_type reload_value) {
    volatile_write<std::uint32_t>(RELOAD_VALUE_REG,
                                  RELOAD_VALUE_MASK,
                                  reload_value);
  }

  static CM_INLINE void set_current_value(tick_type value) {
    volatile_write<std::uint32_t>(CURRENT_VALUE_REG, RELOAD_VALUE_MASK, value);
  }

  static CM_INLINE bool reference_clock_is_implemented() noexcept {
    return volatile_read_bits<std::uint32_t>(SYST_CALIB_REG, NO_REF) == 0;
  }

  static CM_INLINE bool calibration_value_is_exact() noexcept {
    return volatile_read_bits<std::uint32_t>(SYST_CALIB_REG, SKEW) == SKEW;
  }

  static CM_INLINE bool has_calibration_value() noexcept {
    return volatile_read_bits<std::uint32_t>(SYST_CALIB_REG, TENMS) != 0;
  }

  static CM_INLINE tick_type calibration_value() noexcept {
    return static_cast<tick_type>(
        volatile_read_bits<std::uint32_t>(SYST_CALIB_REG, TENMS));
  }

  /**
   * Tries to set the tick rate using the calibration value.
   * @param interval the tick interval in micro seconds
   * @param ignore_inaccurate_calib if true, the calibration value is used even
   * if it is inacurate.
   * @return TickRateStatus::Success if the tick rate has been set, else the
   * status indicates the failure reason.
   */
  static CM_INLINE TickRateStatus
  set_tick_rate(MicroSeconds interval, bool ignore_inaccurate_calib = false) {
    const auto calib      = volatile_read<std::uint32_t>(SYST_CALIB_REG);
    const tick_type tenms = calib & TENMS;

    if (tenms == 0)
      return TickRateStatus::NoCalibrationValue;

    if ((calib & NO_REF) == NO_REF)
      return TickRateStatus::NoReferenceClock;

    if ((calib & SKEW) != SKEW and not ignore_inaccurate_calib)
      return TickRateStatus::InaccurateCalibrationValue;

    // interval/10ms = reload_value/tenms -> reload_value = interval*tenms/10ms
    const tick_type reload_value = interval.value * tenms / 10'000u;
    if ((reload_value & RELOAD_VALUE_MASK) != reload_value)
      return TickRateStatus::ReloadValueOutOfRange;

    set_reload_value(reload_value);
    return TickRateStatus::Success;
  }

private:
  enum Regs : std::uint32_t {
    CTRL_AND_STATUS_REG = 0xE000E010,
    RELOAD_VALUE_REG    = 0xE000E014,
    CURRENT_VALUE_REG   = 0xE000E018,
    SYST_CALIB_REG      = 0xE000E01C
  };

  enum Bits : std::uint32_t {
    ENABLE            = 1u,
    TICKINT           = 1u << 1,
    CLKSOURCE         = 1u << 2,
    NO_REF            = 1u << 31,
    SKEW              = 1u << 30,
    TENMS             = 0x7FFFFFu,
    RELOAD_VALUE_MASK = 0xFFFFFFu
  };
};

static_assert(Systick<StdSystick>);
} // namespace cm
#endif
