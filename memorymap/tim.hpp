/*******************************************************************************
 *
 * Copyright (C) 2012 Jorge Aparicio <jorge.aparicio.r@gmail.com>
 *
 * This file is part of libstm32pp.
 *
 * libstm32pp is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * libstm32pp is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libstm32pp. If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

#pragma once

#include "common.hpp"

namespace tim {
  enum Address {
    TIM2 = alias::APB1 + 0x0000,
    TIM3 = alias::APB1 + 0x0400,
    TIM4 = alias::APB1 + 0x0800,
    TIM5 = alias::APB1 + 0x0C00,
    TIM6 = alias::APB1 + 0x1000,
    TIM7 = alias::APB1 + 0x1400,
    TIM12 = alias::APB1 + 0x1800,
    TIM13 = alias::APB1 + 0x1C00,
    TIM14 = alias::APB1 + 0x2000,
    #ifdef STM32F1XX
    TIM1 = alias::APB2 + 0x2C00,
    TIM8 = alias::APB2 + 0x3400,
    TIM9 = alias::APB2 + 0x4C00,
    TIM10 = alias::APB2 + 0x5000,
    TIM11 = alias::APB2 + 0x5400,
#else
    TIM1 = alias::APB2 + 0x0000,
    TIM8 = alias::APB2 + 0x0400,
    TIM9 = alias::APB2 + 0x4000,
    TIM10 = alias::APB2 + 0x4400,
    TIM11 = alias::APB2 + 0x4800,
#endif
#ifdef VALUE_LINE
  TIM15 = alias::APB2 + 0x4000,
  TIM16 = alias::APB2 + 0x4400,
  TIM17 = alias::APB2 + 0x4800,
#endif
  };

  struct Registers {
      __RW
      u32 CR1;  // 0x00: Control 1
      __RW
      u32 CR2;  // 0x04: Control 2
      __RW
      u32 SMCR;  // 0x08: Slave mode control
      __RW
      u32 DIER;  // 0x0C: DMA/Interrupt enable
      __RW
      u32 SR;  // 0x10: Status
      __RW
      u32 EGR;  // 0x14: Event generation
      __RW
      u32 CCMR1;  // 0x18: Capture/Compare mode 1
      __RW
      u32 CCMR2;  // 0x1C: Capture/Compare mode 2
      __RW
      u32 CCER;  // 0x20: Capture/Compare enable
      __RW
      u32 CNT;  // 0x24: Counter
      __RW
      u32 PSC;  // 0x28: Prescaler
      __RW
      u32 ARR;  // 0x2C: Auto-reload
      __RW
      u32 RCR;  // 0x30: Repetition counter
      __RW
      u32 CCR1;  // 0x34: Capture/Compare 1
      __RW
      u32 CCR2;  // 0x38: Capture/Compare 2
      __RW
      u32 CCR3;  // 0x3C: Capture/Compare 3
      __RW
      u32 CCR4;  // 0x40: Capture/Compare 4
      __RW
      u32 BDTR;  // 0x44: Break and dead-time
      __RW
      u32 DCR;  // 0x48: DMA control
      __RW
      u32 DMAR;  // 0x4C: DMA address for full transfer
      union {
          __RW
          u32 TIM2_OR;  // 0x50: Timer 2 option
          __RW
          u32 TIM5_OR;  // 0x50: Timer 5 option
          __RW
          u32 TIM11_OR;  // 0x50: Timer 11 option
      };
  };

  namespace cr1 {
    enum {
      OFFSET = 0x00
    };

    namespace cen {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        COUNTER_DISABLED = 0 << POSITION,
        COUNTER_ENABLED = 1 << POSITION,
      };
    }  // namespace cen

    namespace udis {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        UPDATE_EVENT_ENABLED = 0 << POSITION,
        UPDATE_EVENT_DISABLED = 1 << POSITION,
      };
    }  // namespace udis

    namespace urs {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        UPDATE_REQUEST_SOURCE_ALL = 0 << POSITION,
        UPDATE_REQUEST_SOURCE_OVERFLOW_UNDERFLOW = 1 << POSITION,
      };
    }  // namespace urs

    namespace opm {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        DONT_STOP_COUNTER_AT_NEXT_UPDATE_EVENT = 0 << POSITION,
        STOP_COUNTER_AT_NEXT_UPDATE_EVENT = 1 << POSITION,
      };
    }  // namespace opm

    namespace dir {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        UPCOUNTER = 0 << POSITION,
        DOWNCOUNTER = 1 << POSITION,
      };
    }  // namespace dir

    namespace cms {
      enum {
        POSITION = 5,
        MASK = 0b11 << POSITION
      };
      enum States {
        EDGE_ALIGNED_MODE = 0 << POSITION,
        CENTER_ALIGNED_MODE_1 = 1 << POSITION,
        CENTER_ALIGNED_MODE_2 = 2 << POSITION,
        CENTER_ALIGNED_MODE_3 = 3 << POSITION,
      };
    }  // namespace cms

    namespace arpe {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        AUTO_RELOAD_UNBUFFERED = 0 << POSITION,
        AUTO_RELOAD_BUFFERED = 1 << POSITION,
      };
    }  // namespace arpe

    namespace ckd {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };
      enum States {
        x1_T_CK_INT = 0 << POSITION,
        x2_T_CK_INT = 1 << POSITION,
        x4_T_CK_INT = 2 << POSITION,
      };
    }  // namespace ckd

  }  // namespace cr1

  namespace cr2 {
    enum {
      OFFSET = 0x04
    };

    namespace ccpc {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        CC_NOT_PRELOADED = 0 << POSITION,
        CC_PRELOADED = 1 << POSITION,
      };
    }  // namespace ccpc

    namespace ccus {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        CC_UPDATE_COMG = 0 << POSITION,
        CC_UPDATE_COMG_TRGI = 1 << POSITION,
      };
    }  // namespace ccus

    namespace ccds {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_ON_CCX = 0 << POSITION,
        DMA_ON_UPDATE_EVENT = 1 << POSITION,
      };
    }  // namespace ccds

    namespace mms {
      enum {
        POSITION = 4,
        MASK = 0b111 << POSITION
      };
      enum States {
        RESET = 0 << POSITION,
        ENABLE = 1 << POSITION,
        UPDATE = 2 << POSITION,
        COMPARE_PULSE = 3 << POSITION,
        COMPARE_OC1REF = 4 << POSITION,
        COMPARE_OC2REF = 5 << POSITION,
        COMPARE_OC3REF = 6 << POSITION,
        COMPARE_OC4REF = 7 << POSITION,
      };
    }  // namespace mms

    namespace ti1s {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        TI1_INPUT_CH1 = 0 << POSITION,
        TI1_INPUT_XOR = 1 << POSITION,
      };
    }  // namespace ti1s

    namespace ois1 {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        OC1_0 = 0 << POSITION,
        OC1_1 = 1 << POSITION,
      };
    }  // namespace ois1

    namespace ois1n {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        OC1N_0 = 0 << POSITION,
        OC1N_1 = 1 << POSITION,
      };
    }  // namespace ois1n

    namespace ois2 {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        OC2_0 = 0 << POSITION,
        OC2_1 = 1 << POSITION,
      };
    }  // namespace ois2

    namespace ois2n {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        OC2N_0 = 0 << POSITION,
        OC2N_1 = 1 << POSITION,
      };
    }  // namespace ois2n

    namespace ois3 {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        OC3_0 = 0 << POSITION,
        OC3_1 = 1 << POSITION,
      };
    }  // namespace ois3

    namespace ois3n {
      enum {
        POSITION = 13,
        MASK = 1 << POSITION
      };
      enum States {
        OC3N_0 = 0 << POSITION,
        OC3N_1 = 1 << POSITION,
      };
    }  // namespace ois3n

    namespace ois4 {
      enum {
        POSITION = 14,
        MASK = 1 << POSITION
      };
      enum States {
        OC4_0 = 0 << POSITION,
        OC4_1 = 1 << POSITION,
      };
    }  // namespace ois4

    namespace ois4n {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        OC4N_0 = 0 << POSITION,
        OC4N_1 = 1 << POSITION,
      };
    }  // namespace ois4n

  }  // namespace cr2

  namespace smcr {
    enum {
      OFFSET = 0x08
    };

    namespace sms {
      enum {
        POSITION = 0,
        MASK = 0b111 << POSITION
      };
      enum States {
        SLAVE_MODE_DISABLED = 0 << POSITION,
        ENCODER_MODE_1 = 1 << POSITION,
        ENCODER_MODE_2 = 2 << POSITION,
        ENCODER_MODE_3 = 3 << POSITION,
        RESET_MODE = 4 << POSITION,
        GATED_MODE = 5 << POSITION,
        TRIGGER_MODE = 6 << POSITION,
        EXT_CLOCK_MODE = 7 << POSITION,
      };
    }  // namespace sms

    namespace ts {
      enum {
        POSITION = 4,
        MASK = 0b111 << POSITION
      };
      enum States {
        INTERNAL_TRIGGER_0 = 0 << POSITION,
        INTERNAL_TRIGGER_1 = 1 << POSITION,
        INTERNAL_TRIGGER_2 = 2 << POSITION,
        INTERNAL_TRIGGER_3 = 3 << POSITION,
        TI1_EDGE_DETECTOR = 4 << POSITION,
        FILTERED_TMR_INPUT_1 = 5 << POSITION,
        FILTERED_TMR_INPUT_2 = 6 << POSITION,
        EXTERNAL_TRIGGER_IN = 7 << POSITION,
      };
    }  // namespace ts

    namespace msm {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        TRGO_NODELAY = 0 << POSITION,
        TRGO_DELAY = 1 << POSITION,
      };
    }  // namespace msm

    namespace etf {
      enum {
        POSITION = 8,
        MASK = 0b1111 << POSITION
      };
      enum States {
        NO_FILTER = 0 << POSITION,
        F_CK_INT_N2 = 1 << POSITION,
        F_CK_INT_N4 = 2 << POSITION,
        F_CK_INT_N8 = 3 << POSITION,
        F_CK_INT_2_N6 = 4 << POSITION,
        F_CK_INT_2_N8 = 5 << POSITION,
        F_CK_INT_4_N6 = 6 << POSITION,
        F_CK_INT_4_N8 = 7 << POSITION,
        F_CK_INT_8_N6 = 8 << POSITION,
        F_CK_INT_8_N8 = 9 << POSITION,
        F_CK_INT_16_N5 = 10 << POSITION,
        F_CK_INT_16_N6 = 11 << POSITION,
        F_CK_INT_16_N8 = 12 << POSITION,
        F_CK_INT_32_N5 = 13 << POSITION,
        F_CK_INT_32_N6 = 14 << POSITION,
        F_CK_INT_32_N8 = 15 << POSITION,
      };
    }  // namespace etf

    namespace etps {
      enum {
        POSITION = 12,
        MASK = 0b11 << POSITION
      };
      enum States {
        ETPR_DIV_1 = 0 << POSITION,
        ETPR_DIV_2 = 1 << POSITION,
        ETPR_DIV_4 = 2 << POSITION,
        ETPR_DIV_8 = 3 << POSITION,
      };
    }  // namespace etps

    namespace ece {
      enum {
        POSITION = 14,
        MASK = 1 << POSITION
      };
      enum States {
        EXT_CLK_MODE_2_DISABLED = 0 << POSITION,
        EXT_CLK_MODE_2_ENABLED = 1 << POSITION,
      };
    }  // namespace ece

    namespace etp {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        ETR_NON_INVERTED = 0 << POSITION,
        ETR_INVERTED = 1 << POSITION,
      };
    }  // namespace etp

  }

  namespace dier {
    enum {
      OFFSET = 0x0C
    };

    namespace uie {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace uie

    namespace cc1ie {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace cc1ie

    namespace cc2ie {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace cc2ie

    namespace cc3ie {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace cc3ie

    namespace cc4ie {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace cc4ie

    namespace comie {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace comie

    namespace tie {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace tie

    namespace bie {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace bie

    namespace ude {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_REQUEST_DISABLED = 0 << POSITION,
        DMA_REQUEST_ENABLED = 1 << POSITION,
      };
    }  // namespace ude

    namespace cc1de {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_REQUEST_DISABLED = 0 << POSITION,
        DMA_REQUEST_ENABLED = 1 << POSITION,
      };
    }  // namespace cc1de

    namespace cc2de {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_REQUEST_DISABLED = 0 << POSITION,
        DMA_REQUEST_ENABLED = 1 << POSITION,
      };
    }  // namespace cc2de

    namespace cc3de {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_REQUEST_DISABLED = 0 << POSITION,
        DMA_REQUEST_ENABLED = 1 << POSITION,
      };
    }  // namespace cc3de

    namespace cc4de {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_REQUEST_DISABLED = 0 << POSITION,
        DMA_REQUEST_ENABLED = 1 << POSITION,
      };
    }  // namespace cc4de

    namespace comde {
      enum {
        POSITION = 13,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_REQUEST_DISABLED = 0 << POSITION,
        DMA_REQUEST_ENABLED = 1 << POSITION,
      };
    }  // namespace comde

    namespace tde {
      enum {
        POSITION = 14,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_REQUEST_DISABLED = 0 << POSITION,
        DMA_REQUEST_ENABLED = 1 << POSITION,
      };
    }  // namespace tde

  }  // namespace dier

  namespace sr {
    enum {
      OFFSET = 0x10
    };

    namespace uif {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        NO_UPDATE_OCCURRED = 0 << POSITION,
        UPDATE_INTERRUPT_PENDING = 1 << POSITION,
      };
    }  // namespace uif

    namespace cc1if {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        NO_INTERRUPT_OCCURRED = 0 << POSITION,
        INTERRUPT_PENDING = 1 << POSITION,
      };
    }  // namespace cc1if

    namespace cc2if {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        NO_INTERRUPT_OCCURRED = 0 << POSITION,
        INTERRUPT_PENDING = 1 << POSITION,
      };
    }  // namespace cc2if

    namespace cc3if {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        NO_INTERRUPT_OCCURRED = 0 << POSITION,
        INTERRUPT_PENDING = 1 << POSITION,
      };
    }  // namespace cc3if

    namespace cc4if {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        NO_INTERRUPT_OCCURRED = 0 << POSITION,
        INTERRUPT_PENDING = 1 << POSITION,
      };
    }  // namespace cc4if

    namespace comif {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        NO_INTERRUPT_OCCURRED = 0 << POSITION,
        INTERRUPT_PENDING = 1 << POSITION,
      };
    }  // namespace comif

    namespace tif {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        NO_INTERRUPT_OCCURRED = 0 << POSITION,
        INTERRUPT_PENDING = 1 << POSITION,
      };
    }  // namespace tif

    namespace bif {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        NO_INTERRUPT_OCCURRED = 0 << POSITION,
        INTERRUPT_PENDING = 1 << POSITION,
      };
    }  // namespace bif

    namespace cc1of {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        NO_OVERCAPTURE_OCCURRED = 0 << POSITION,
        OVERCAPTURE_DETECTED = 1 << POSITION,
      };
    }  // namespace cc1of

    namespace cc2of {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        NO_OVERCAPTURE_OCCURRED = 0 << POSITION,
        OVERCAPTURE_DETECTED = 1 << POSITION,
      };
    }  // namespace cc2of

    namespace cc3of {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        NO_OVERCAPTURE_OCCURRED = 0 << POSITION,
        OVERCAPTURE_DETECTED = 1 << POSITION,
      };
    }  // namespace cc3of

    namespace cc4of {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        NO_OVERCAPTURE_OCCURRED = 0 << POSITION,
        OVERCAPTURE_DETECTED = 1 << POSITION,
      };
    }  // namespace cc4of

  }  // namespace sr

  namespace egr {
    enum {
      OFFSET = 0x14
    };

    namespace ug {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACTION = 0 << POSITION,
        GENERATE_AN_UPDATE = 1 << POSITION,
      };
    }  // namespace ug

    namespace cc1g {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACTION = 0 << POSITION,
        GENERATE_EVENT = 1 << POSITION,
      };
    }  // namespace cc1g

    namespace cc2g {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACTION = 0 << POSITION,
        GENERATE_EVENT = 1 << POSITION,
      };
    }  // namespace cc2g

    namespace cc3g {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACTION = 0 << POSITION,
        GENERATE_EVENT = 1 << POSITION,
      };
    }  // namespace cc3g

    namespace cc4g {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACTION = 0 << POSITION,
        GENERATE_EVENT = 1 << POSITION,
      };
    }  // namespace cc4g

    namespace comg {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACTION = 0 << POSITION,
        GENERATE_AN_UPDATE = 1 << POSITION,
      };
    }  // namespace comg

    namespace tg {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACTION = 0 << POSITION,
        GENERATE_EVENT = 1 << POSITION,
      };
    }  // namespace tg

    namespace bg {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACTION = 0 << POSITION,
        GENERATE_BREAK = 1 << POSITION,
      };
    }  // namespace bg

  }  // namespace egr

  namespace iccmr1 {
    enum {
      OFFSET = 0x18
    };

    namespace cc1s {
      enum {
        POSITION = 0,
        MASK = 0b11 << POSITION
      };
      enum States {
        CC_OUTPUT = 0 << POSITION,
        CC_INPUT_TI1 = 1 << POSITION,
        CC_INPUT_TI2 = 2 << POSITION,
        CC_INPUT_TRC = 3 << POSITION,
      };
    }  // namespace cc1s

    namespace ic1psc {
      enum {
        POSITION = 2,
        MASK = 0b11 << POSITION
      };
      enum States {
        CAPTURE_EACH_TIME = 0 << POSITION,
        CAPTURE_EVERY_2_EV = 1 << POSITION,
        CAPTURE_EVERY_4_EV = 2 << POSITION,
        CAPTURE_EVERY_8_EV = 3 << POSITION,
      };
    }  // namespace ic1psc

    namespace ic1f {
      enum {
        POSITION = 4,
        MASK = 0b1111 << POSITION
      };
      enum States {
        F_DTS = 0 << POSITION,
        F_CK_INT_N2 = 1 << POSITION,
        F_CK_INT_N4 = 2 << POSITION,
        F_CK_INT_N8 = 3 << POSITION,
        F_DTS_2_N6 = 4 << POSITION,
        F_DTS_2_N8 = 5 << POSITION,
        F_DTS_4_N6 = 6 << POSITION,
        F_DTS_4_N8 = 7 << POSITION,
        F_DTS_8_N6 = 8 << POSITION,
        F_DTS_8_N8 = 9 << POSITION,
        F_DTS_16_N5 = 10 << POSITION,
        F_DTS_16_N6 = 11 << POSITION,
        F_DTS_16_N8 = 12 << POSITION,
        F_DTS_32_N5 = 13 << POSITION,
        F_DTS_32_N6 = 14 << POSITION,
        F_DTS_32_N8 = 15 << POSITION,
      };
    }  // namespace ic1f

    namespace cc2s {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };
      enum States {
        CC_OUTPUT = 0 << POSITION,
        CC_INPUT_TI1 = 1 << POSITION,
        CC_INPUT_TI2 = 2 << POSITION,
        CC_INPUT_TRC = 3 << POSITION,
      };
    }  // namespace cc2s

    namespace ic2psc {
      enum {
        POSITION = 10,
        MASK = 0b11 << POSITION
      };
      enum States {
        CAPTURE_EACH_TIME = 0 << POSITION,
        CAPTURE_EVERY_2_EV = 1 << POSITION,
        CAPTURE_EVERY_4_EV = 2 << POSITION,
        CAPTURE_EVERY_8_EV = 3 << POSITION,
      };
    }  // namespace ic2psc

    namespace ic2f {
      enum {
        POSITION = 12,
        MASK = 0b1111 << POSITION
      };
      enum States {
        F_DTS = 0 << POSITION,
        F_CK_INT_N2 = 1 << POSITION,
        F_CK_INT_N4 = 2 << POSITION,
        F_CK_INT_N8 = 3 << POSITION,
        F_DTS_2_N6 = 4 << POSITION,
        F_DTS_2_N8 = 5 << POSITION,
        F_DTS_4_N6 = 6 << POSITION,
        F_DTS_4_N8 = 7 << POSITION,
        F_DTS_8_N6 = 8 << POSITION,
        F_DTS_8_N8 = 9 << POSITION,
        F_DTS_16_N5 = 10 << POSITION,
        F_DTS_16_N6 = 11 << POSITION,
        F_DTS_16_N8 = 12 << POSITION,
        F_DTS_32_N5 = 13 << POSITION,
        F_DTS_32_N6 = 14 << POSITION,
        F_DTS_32_N8 = 15 << POSITION,
      };
    }  // namespace ic2f

  }  // namespace iccmr1

  namespace occmr1 {
    enum {
      OFFSET = 0x18
    };

    namespace cc1s {
      enum {
        POSITION = 0,
        MASK = 0b11 << POSITION
      };
      enum States {
        CC_OUTPUT = 0 << POSITION,
        CC_INPUT_TI1 = 1 << POSITION,
        CC_INPUT_TI2 = 2 << POSITION,
        CC_INPUT_TRC = 3 << POSITION,
      };
    }  // namespace cc1s

    namespace oc1fe {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        CC_NORMAL = 0 << POSITION,
        CMP_MATCH_CC_OUTPUT = 1 << POSITION,
      };
    }  // namespace oc1fe

    namespace oc1pe {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        PRELOAD_DISABLE = 0 << POSITION,
        PRELOAD_ENABLE = 1 << POSITION,
      };
    }  // namespace oc1pe

    namespace oc1m {
      enum {
        POSITION = 4,
        MASK = 0b111 << POSITION
      };
      enum States {
        FROZEN_MODE = 0 << POSITION,
        CH_ACTIVE_ON_MATCH = 1 << POSITION,
        CH_INACTIVE_ON_MATCH = 2 << POSITION,
        TOGGLE_MODE = 3 << POSITION,
        FORCE_INACTIVE = 4 << POSITION,
        FORCE_ACTIVE = 5 << POSITION,
        PWM_MODE_1 = 6 << POSITION,
        PWM_MODE_2 = 7 << POSITION,
      };
    }  // namespace oc1m

    namespace oc1ce {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        IGNORE = 0 << POSITION,
        CLEAR = 1 << POSITION,
      };
    }  // namespace oc1ce

    namespace cc2s {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };
      enum States {
        CC_OUTPUT = 0 << POSITION,
        CC_INPUT_TI1 = 1 << POSITION,
        CC_INPUT_TI2 = 2 << POSITION,
        CC_INPUT_TRC = 3 << POSITION,
      };
    }  // namespace cc2s

    namespace oc2fe {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        CC_NORMAL = 0 << POSITION,
        CMP_MATCH_CC_OUTPUT = 1 << POSITION,
      };
    }  // namespace oc2fe

    namespace oc2pe {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        PRELOAD_DISABLE = 0 << POSITION,
        PRELOAD_ENABLE = 1 << POSITION,
      };
    }  // namespace oc2pe

    namespace oc2m {
      enum {
        POSITION = 12,
        MASK = 0b111 << POSITION
      };
      enum States {
        FROZEN_MODE = 0 << POSITION,
        CH_ACTIVE_ON_MATCH = 1 << POSITION,
        CH_INACTIVE_ON_MATCH = 2 << POSITION,
        TOGGLE_MODE = 3 << POSITION,
        FORCE_INACTIVE = 4 << POSITION,
        FORCE_ACTIVE = 5 << POSITION,
        PWM_MODE_1 = 6 << POSITION,
        PWM_MODE_2 = 7 << POSITION,
      };
    }  // namespace oc2m

    namespace oc2ce {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        IGNORE = 0 << POSITION,
        CLEAR = 1 << POSITION,
      };
    }  // namespace oc2ce

  }  // namespace occmr1

  namespace iccmr2 {
    enum {
      OFFSET = 0x1C
    };

    namespace cc3s {
      enum {
        POSITION = 0,
        MASK = 0b11 << POSITION
      };
      enum States {
        CC_OUTPUT = 0 << POSITION,
        CC_INPUT_TI1 = 1 << POSITION,
        CC_INPUT_TI2 = 2 << POSITION,
        CC_INPUT_TRC = 3 << POSITION,
      };
    }  // namespace cc3s

    namespace ic3psc {
      enum {
        POSITION = 2,
        MASK = 0b11 << POSITION
      };
      enum States {
        CAPTURE_EACH_TIME = 0 << POSITION,
        CAPTURE_EVERY_2_EV = 1 << POSITION,
        CAPTURE_EVERY_4_EV = 2 << POSITION,
        CAPTURE_EVERY_8_EV = 3 << POSITION,
      };
    }  // namespace ic3psc

    namespace ic3f {
      enum {
        POSITION = 4,
        MASK = 0b1111 << POSITION
      };
      enum States {
        F_DTS = 0 << POSITION,
        F_CK_INT_N2 = 1 << POSITION,
        F_CK_INT_N4 = 2 << POSITION,
        F_CK_INT_N8 = 3 << POSITION,
        F_DTS_2_N6 = 4 << POSITION,
        F_DTS_2_N8 = 5 << POSITION,
        F_DTS_4_N6 = 6 << POSITION,
        F_DTS_4_N8 = 7 << POSITION,
        F_DTS_8_N6 = 8 << POSITION,
        F_DTS_8_N8 = 9 << POSITION,
        F_DTS_16_N5 = 10 << POSITION,
        F_DTS_16_N6 = 11 << POSITION,
        F_DTS_16_N8 = 12 << POSITION,
        F_DTS_32_N5 = 13 << POSITION,
        F_DTS_32_N6 = 14 << POSITION,
        F_DTS_32_N8 = 15 << POSITION,
      };
    }  // namespace ic3f

    namespace cc4s {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };
      enum States {
        CC_OUTPUT = 0 << POSITION,
        CC_INPUT_TI1 = 1 << POSITION,
        CC_INPUT_TI2 = 2 << POSITION,
        CC_INPUT_TRC = 3 << POSITION,
      };
    }  // namespace cc4s

    namespace ic4psc {
      enum {
        POSITION = 10,
        MASK = 0b11 << POSITION
      };
      enum States {
        CAPTURE_EACH_TIME = 0 << POSITION,
        CAPTURE_EVERY_2_EV = 1 << POSITION,
        CAPTURE_EVERY_4_EV = 2 << POSITION,
        CAPTURE_EVERY_8_EV = 3 << POSITION,
      };
    }  // namespace ic4psc

    namespace ic4f {
      enum {
        POSITION = 12,
        MASK = 0b1111 << POSITION
      };
      enum States {
        F_DTS = 0 << POSITION,
        F_CK_INT_N2 = 1 << POSITION,
        F_CK_INT_N4 = 2 << POSITION,
        F_CK_INT_N8 = 3 << POSITION,
        F_DTS_2_N6 = 4 << POSITION,
        F_DTS_2_N8 = 5 << POSITION,
        F_DTS_4_N6 = 6 << POSITION,
        F_DTS_4_N8 = 7 << POSITION,
        F_DTS_8_N6 = 8 << POSITION,
        F_DTS_8_N8 = 9 << POSITION,
        F_DTS_16_N5 = 10 << POSITION,
        F_DTS_16_N6 = 11 << POSITION,
        F_DTS_16_N8 = 12 << POSITION,
        F_DTS_32_N5 = 13 << POSITION,
        F_DTS_32_N6 = 14 << POSITION,
        F_DTS_32_N8 = 15 << POSITION,
      };
    }  // namespace ic4f

  }  // namespace iccmr2

  namespace occmr2 {
    enum {
      OFFSET = 0x1C
    };

    namespace cc3s {
      enum {
        POSITION = 0,
        MASK = 0b11 << POSITION
      };
      enum States {
        CC_OUTPUT = 0 << POSITION,
        CC_INPUT_TI1 = 1 << POSITION,
        CC_INPUT_TI2 = 2 << POSITION,
        CC_INPUT_TRC = 3 << POSITION,
      };
    }  // namespace cc3s

    namespace oc3fe {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        CC_NORMAL = 0 << POSITION,
        CMP_MATCH_CC_OUTPUT = 1 << POSITION,
      };
    }  // namespace oc3fe

    namespace oc3pe {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        PRELOAD_DISABLE = 0 << POSITION,
        PRELOAD_ENABLE = 1 << POSITION,
      };
    }  // namespace oc3pe

    namespace oc3m {
      enum {
        POSITION = 4,
        MASK = 0b111 << POSITION
      };
      enum States {
        FROZEN_MODE = 0 << POSITION,
        CH_ACTIVE_ON_MATCH = 1 << POSITION,
        CH_INACTIVE_ON_MATCH = 2 << POSITION,
        TOGGLE_MODE = 3 << POSITION,
        FORCE_INACTIVE = 4 << POSITION,
        FORCE_ACTIVE = 5 << POSITION,
        PWM_MODE_1 = 6 << POSITION,
        PWM_MODE_2 = 7 << POSITION,
      };
    }  // namespace oc3m

    namespace oc3ce {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        IGNORE = 0 << POSITION,
        CLEAR = 1 << POSITION,
      };
    }  // namespace oc3ce

    namespace cc4s {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };
      enum States {
        CC_OUTPUT = 0 << POSITION,
        CC_INPUT_TI1 = 1 << POSITION,
        CC_INPUT_TI2 = 2 << POSITION,
        CC_INPUT_TRC = 3 << POSITION,
      };
    }  // namespace cc4s

    namespace oc4fe {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        CC_NORMAL = 0 << POSITION,
        CMP_MATCH_CC_OUTPUT = 1 << POSITION,
      };
    }  // namespace oc4fe

    namespace oc4pe {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        PRELOAD_DISABLE = 0 << POSITION,
        PRELOAD_ENABLE = 1 << POSITION,
      };
    }  // namespace oc4pe

    namespace oc4m {
      enum {
        POSITION = 12,
        MASK = 0b111 << POSITION
      };
      enum States {
        FROZEN_MODE = 0 << POSITION,
        CH_ACTIVE_ON_MATCH = 1 << POSITION,
        CH_INACTIVE_ON_MATCH = 2 << POSITION,
        TOGGLE_MODE = 3 << POSITION,
        FORCE_INACTIVE = 4 << POSITION,
        FORCE_ACTIVE = 5 << POSITION,
        PWM_MODE_1 = 6 << POSITION,
        PWM_MODE_2 = 7 << POSITION,
      };
    }  // namespace oc4m

    namespace oc4ce {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        IGNORE = 0 << POSITION,
        CLEAR = 1 << POSITION,
      };
    }  // namespace oc4ce

  }  // namespace occmr2
/*
  namespace iccmr {
// TODO TIM ICCMR bits
  }// namespace iccmr

  namespace occmr {
// TODO TIM OCCMR bits
  }// namespace occmr
*/
  namespace ccer {
    enum {
      OFFSET = 0x20
    };

    namespace cc1e {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        DISABLED = 0 << POSITION,
        ENABLED = 1 << POSITION,
      };
    }  // namespace cc1e

    namespace cc1p {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        RISING_EDGE_ACTIVE_HIGH = 0 << POSITION,
        FALLING_EDGE_ACTIVE_LOW = 1 << POSITION,
      };
    }  // namespace cc1p

    namespace cc1ne {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        DISABLED = 0 << POSITION,
        ENABLED = 1 << POSITION,
      };
    }  // namespace cc1ne

    namespace cc1np {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        ACTIVE_HIGH = 0 << POSITION,
        ACTIVE_LOW = 1 << POSITION,
      };
    }  // namespace cc1np

    namespace cc2e {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        DISABLED = 0 << POSITION,
        ENABLED = 1 << POSITION,
      };
    }  // namespace cc2e

    namespace cc2p {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        RISING_EDGE_ACTIVE_HIGH = 0 << POSITION,
        FALLING_EDGE_ACTIVE_LOW = 1 << POSITION,
      };
    }  // namespace cc2p

    namespace cc2ne {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        DISABLED = 0 << POSITION,
        ENABLED = 1 << POSITION,
      };
    }  // namespace cc2ne

    namespace cc2np {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        ACTIVE_HIGH = 0 << POSITION,
        ACTIVE_LOW = 1 << POSITION,
      };
    }  // namespace cc2np

    namespace cc3e {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        DISABLED = 0 << POSITION,
        ENABLED = 1 << POSITION,
      };
    }  // namespace cc3e

    namespace cc3p {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        RISING_EDGE_ACTIVE_HIGH = 0 << POSITION,
        FALLING_EDGE_ACTIVE_LOW = 1 << POSITION,
      };
    }  // namespace cc3p

    namespace cc3ne {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        DISABLED = 0 << POSITION,
        ENABLED = 1 << POSITION,
      };
    }  // namespace cc3ne

    namespace cc3np {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        ACTIVE_HIGH = 0 << POSITION,
        ACTIVE_LOW = 1 << POSITION,
      };
    }  // namespace cc3np

    namespace cc4e {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        DISABLED = 0 << POSITION,
        ENABLED = 1 << POSITION,
      };
    }  // namespace cc4e

    namespace cc4p {
      enum {
        POSITION = 13,
        MASK = 1 << POSITION
      };
      enum States {
        RISING_EDGE_ACTIVE_HIGH = 0 << POSITION,
        FALLING_EDGE_ACTIVE_LOW = 1 << POSITION,
      };
    }  // namespace cc4p

    namespace cc4ne {
      enum {
        POSITION = 14,
        MASK = 1 << POSITION
      };
      enum States {
        DISABLED = 0 << POSITION,
        ENABLED = 1 << POSITION,
      };
    }  // namespace cc4ne

    namespace cc4np {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        ACTIVE_HIGH = 0 << POSITION,
        ACTIVE_LOW = 1 << POSITION,
      };
    }  // namespace cc4np

  }// namespace ccer

  namespace cnt {
    enum {
      OFFSET = 0x24
    };
  }  // namespace cnt

  namespace psc {
    enum {
      OFFSET = 0x28
    };
  }  // namespace psc

  namespace arr {
    enum {
      OFFSET = 0x2C
    };
  }  // namespace arr

  namespace rcr {
    enum {
      OFFSET = 0x30
    };
  }  // namespace rcr

  namespace ccr1 {
    enum {
      OFFSET = 0x34
    };
  }  // namespace ccr1

  namespace ccr2 {
    enum {
      OFFSET = 0x38
    };
  }  // namespace ccr2

  namespace ccr3 {
    enum {
      OFFSET = 0x3C
    };
  }  // namespace ccr3

  namespace ccr4 {
    enum {
      OFFSET = 0x40
    };
  }  // namespace ccr4

  namespace bdtr {
    enum {
      OFFSET = 0x44
    };

    namespace dtg {
      enum {
        POSITION = 0,
        MASK = 0xFF << POSITION
      };
    }  // namespace dtg

    namespace lock {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };
      enum States {
        LOCK_OFF = 0 << POSITION,
        LOCK_LEVEL_1 = 1 << POSITION,
        LOCK_LEVEL_2 = 2 << POSITION,
        LOCK_LEVEL_3 = 3 << POSITION,
      };
    }  // namespace lock

    namespace ossi {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        OC_OUTPUTS_DISABLE = 0 << POSITION,
        OC_OUTPUTS_FORCED_IDLE = 1 << POSITION,
      };
    }  // namespace ossi

    namespace ossr {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        OC_OUTPUTS_DISABLE = 0 << POSITION,
        OC_OUTPUTS_FORCED_INACTIVE = 1 << POSITION,
      };
    }  // namespace ossr

    namespace bke {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        BREAK_DISABLE = 0 << POSITION,
        BREAK_ENABLE = 1 << POSITION,
      };
    }  // namespace bke

    namespace bkp {
      enum {
        POSITION = 13,
        MASK = 1 << POSITION
      };
      enum States {
        BREAK_ACTIVE_LOW = 0 << POSITION,
        BREAK_ACTIVE_HIGH = 1 << POSITION,
      };
    }  // namespace bkp

    namespace aoe {
      enum {
        POSITION = 14,
        MASK = 1 << POSITION
      };
      enum States {
        MOE_SW_ONLY = 0 << POSITION,
        MOE_SW_AUTO = 1 << POSITION,
      };
    }  // namespace aoe

    namespace moe {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        OC_OUTPUS_DISABLED = 0 << POSITION,
        OC_OUTPUS_ENABLED = 1 << POSITION,
      };
    }  // namespace moe

  }// namespace bdtr

  namespace dcr {
    enum {
      OFFSET = 0x48
    };

    namespace dba {
      enum {
        POSITION = 0,
        MASK = 0x1F << POSITION
      };
    }  // namespace dba

    namespace dbl {
      enum {
        POSITION = 8,
        MASK = 0x1F << POSITION
      };
    }  // namespace dbl

  }// namespace dcr

  namespace dmar {
    enum {
      OFFSET = 0x4C
    };
  }  // namespace dmar

  namespace tim2_or {
    enum {
      OFFSET = 0x50
    };
// TODO TIM TIM2_OR bits
  }// namespace tim2_or

  namespace tim5_or {
    enum {
      OFFSET = 0x50
    };
// TODO TIM TIM5_OR bits
  }// namespace tim5_or

  namespace tim11_or {
    enum {
      OFFSET = 0x50
    };
// TODO TIM TIM11_OR bits
  }// namespace tim11_or
}  // namespace tim
