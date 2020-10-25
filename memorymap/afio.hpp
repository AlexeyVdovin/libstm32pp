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

namespace afio {
  enum {
    ADDRESS = alias::APB2 + 0x0000
  };

  struct Registers
  {
      __RW
      u32 EVCR;       // 0x00: Event control
      __RW
      u32 MAPR;       // 0x04: Remap and debug configuration
      __RW
      u32 EXTICR[4];  // 0x08: External interrupt configuration
      u32 _RESERVED0;
      __RW
      u32 MAPR2;      // 0x1C: Remap and debug configuration 2
  };

  namespace mapr {
    namespace spi1 {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace i2c1 {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace usart1 {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace usart2 {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace usart3 {
      enum {
        POSITION = 4,
        MASK = 0b11 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        PARTIAL_REMAP = 1,
        FULL_REMAP = 3
      };
    }
    namespace tim1 {
      enum {
        POSITION = 6,
        MASK = 0b11 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        PARTIAL_REMAP = 1,
        FULL_REMAP = 3
      };
    }
    namespace tim2 {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        PARTIAL_REMAP_PA15 = 1,
        PARTIAL_REMAP_PA0 = 2,
        FULL_REMAP = 3
      };
    }
    namespace tim3 {
      enum {
        POSITION = 10,
        MASK = 0b11 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        PARTIAL_REMAP = 2,
        FULL_REMAP = 3
      };
    }
    namespace tim4 {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace can {
      enum {
        POSITION = 13,
        MASK = 0b11 << POSITION
      };
      enum States {
        PA11_PA12 = 0,
        PB8_PB9   = 2,
        PD0_PD1   = 3
      };
    }
    namespace pd01 {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace tim5ch4 {
      enum {
        POSITION = 16,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace adc1_inj {
      enum {
        POSITION = 17,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace adc1 {
      enum {
        POSITION = 18,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace adc2_inj {
      enum {
        POSITION = 19,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace adc2 {
      enum {
        POSITION = 20,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REMAP = 0,
        REMAP = 1
      };
    }
    namespace swj {
      enum {
        POSITION = 24,
        MASK = 0b111 << POSITION
      };
      enum States {
        SWJ = 0,
        SWJ_NO_NJTRST = 1,
        JTAG_DP_DISABLED = 2,
        JTAG_SW_DP_DISABLED = 4
      };
    }
  }

  namespace exticr {
    enum {
      MASK = 0b1111,
      POSITION = 4
    };
    enum States {
      PA = 0,
      PB = 1,
      PC = 2,
      PD = 3,
      PE = 4,
      PF = 5,
      PG = 6,
    };
  }  // namespace exticr
}// namespace afio
