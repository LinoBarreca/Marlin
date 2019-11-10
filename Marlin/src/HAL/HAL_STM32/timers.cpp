/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "HAL.h"

#include "timers.h"

// ------------------------
// Local defines
// ------------------------

#define NUM_HARDWARE_TIMERS 2

// ------------------------
// Private Variables
// ------------------------

HardwareTimer *TimerHandle[NUM_HARDWARE_TIMERS];
bool TimerEnabled[NUM_HARDWARE_TIMERS];

// ------------------------
// Public functions
// ------------------------

bool timers_initialized[NUM_HARDWARE_TIMERS] = { false };

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {

  if (!timers_initialized[timer_num]) {
    uint32_t step_prescaler = STEPPER_TIMER_PRESCALE - 1,
             temp_prescaler = TEMP_TIMER_PRESCALE - 1;

    switch (timer_num) {
      case STEP_TIMER_NUM:
        // STEPPER TIMER - use a 32bit timer if possible
        TimerHandle[timer_num] = new HardwareTimer(STEP_TIMER_DEV);
        TimerHandle[timer_num]->setPrescaleFactor(step_prescaler);
        TimerHandle[timer_num]->attachInterrupt(Step_Handler);
        TimerHandle[timer_num]->setOverflow(frequency, HERTZ_FORMAT);
        break;
      case TEMP_TIMER_NUM:
        // TEMP TIMER - any available 16bit Timer
        TimerHandle[timer_num] = new HardwareTimer(TEMP_TIMER_DEV);
        TimerHandle[timer_num]->attachInterrupt(Temp_Handler);
        TimerHandle[timer_num]->setPrescaleFactor(temp_prescaler);
        TimerHandle[timer_num]->setOverflow(frequency, HERTZ_FORMAT);
        break;
    }
    timers_initialized[timer_num] = true;
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  TimerHandle[timer_num]->resume();
  TimerEnabled[timer_num]=true;
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  TimerHandle[timer_num]->pause();
  TimerEnabled[timer_num]=false;
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return TimerEnabled[timer_num];
}

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
