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

namespace flash {
  enum {
#ifdef STM32F1XX
    ADDRESS = alias::AHB + 0x2000
#else // STM32F1XX
  ADDRESS = alias::AHB1 + 0x3C00
#endif // STM32F1XX
};

struct Registers {
    __RW
    u32 ACR;      // 0x00: Access control
    u32 KEYR;     // 0x04: FPEC key register
    u32 OPTKEYR;  // 0x08: Option key register
    u32 SR;       // 0x0c: Status register
    u32 CR;       // 0x10: Control register
    u32 AR;       // 0x14: Address register
    u32 Reserved;
    u32 OBR;      // 0x1C: Option byte register
    u32 WRPR;     // 0x20: Write protection register
};

#ifdef STM32F1XX
namespace acr {
  enum {
    OFFSET = 0x00
  };
#ifndef VALUE_LINE
  namespace latency {
    enum {
      POSITION = 0,
      MASK = 0b111 << POSITION
    };
    enum States {
      ZERO_WAIT_STATE = 0 << POSITION,
      ONE_WAIT_STATE = 1 << POSITION,
      TWO_WAIT_STATES = 2 << POSITION
    };
  }  // namespace latency
#endif
  namespace hlfcya {
    enum {
      POSITION = 3,
      MASK = 1 << POSITION
    };
    enum States {
      FLASH_HALF_CYCLE_ACCESS_DISABLED = 0 << POSITION,
      FLASH_HALF_CYCLE_ACCESS_ENABLED = 1 << POSITION
    };
  }  // namespace prftbe
#ifndef VALUE_LINE
  namespace prftbe {
    enum {
      POSITION = 4,
      MASK = 1 << POSITION
    };
    enum States {
      PREFETCH_DISABLED = 0 << POSITION,
      PREFETCH_ENABLED = 1 << POSITION
    };
  }  // namespace prftbe
#endif
}  // namespace acr
namespace sr {
  enum {
    OFFSET = 0x0C
  };
  namespace bsy {
    enum {
      POSITION = 0,
      MASK = 0b1 << POSITION
    };
    enum States {
      IDLE = 0 << POSITION,
      BUSY = 1 << POSITION
    };
  }  // namespace bsy
  namespace pgerr {
    enum {
      POSITION = 2,
      MASK = 0b1 << POSITION
    };
    enum States {
      NO_ERROR = 0 << POSITION,
      ERROR = 1 << POSITION
    };
  } // namespace pgerr
  namespace wrprterr {
    enum {
      POSITION = 4,
      MASK = 0b1 << POSITION
    };
    enum States {
      NO_ERROR = 0 << POSITION,
      ERROR = 1 << POSITION
    };
  } // namespace wrprterr
  namespace eop {
    enum {
      POSITION = 5,
      MASK = 0b1 << POSITION
    };
    enum States {
      IN_PROGRESS = 0 << POSITION,
      COMPLETED = 1 << POSITION
    };
  } // namespace eop
} // namespace sr
namespace cr {
  enum {
    OFFSET = 0x10
  };
  namespace pg {
    enum {
      POSITION = 0,
      MASK = 0b1 << POSITION
    };
    enum States {
      NONE = 0 << POSITION,
      PROGRAMMING = 1 << POSITION
    };
  }  // namespace pg
  namespace per {
    enum {
      POSITION = 1,
      MASK = 0b1 << POSITION
    };
    enum States {
      NONE = 0 << POSITION,
      PAGE_ERASE = 1 << POSITION
    };
  }  // namespace per
  namespace mer {
    enum {
      POSITION = 2,
      MASK = 0b1 << POSITION
    };
    enum States {
      NONE = 0 << POSITION,
      MASS_ERASE = 1 << POSITION
    };
  }  // namespace mer
  namespace optpg {
    enum {
      POSITION = 4,
      MASK = 0b1 << POSITION
    };
    enum States {
      NONE = 0 << POSITION,
      OPT_BYTE_PGM = 1 << POSITION
    };
  }  // namespace optpg
  namespace opter {
    enum {
      POSITION = 5,
      MASK = 0b1 << POSITION
    };
    enum States {
      NONE = 0 << POSITION,
      OPT_BYTE_ERASE = 1 << POSITION
    };
  }  // namespace opter
  namespace strt {
    enum {
      POSITION = 6,
      MASK = 0b1 << POSITION
    };
    enum States {
      NONE = 0 << POSITION,
      START_ERASE = 1 << POSITION
    };
  }  // namespace per
  namespace lock {
    enum {
      POSITION = 7,
      MASK = 0b1 << POSITION
    };
    enum States {
      UNLOCKED = 0 << POSITION,
      LOCKED = 1 << POSITION
    };
  }  // namespace lock
  namespace optwre {
    enum {
      POSITION = 9,
      MASK = 0b1 << POSITION
    };
    enum States {
      OPT_LOCKED = 0 << POSITION,
      WRITE_ENABLE = 1 << POSITION
    };
  }  // namespace optwre
  namespace errie {
    enum {
      POSITION = 10,
      MASK = 0b1 << POSITION
    };
    enum States {
      DISABLED = 0 << POSITION,
      ERR_INT_ENABLED = 1 << POSITION
    };
  }  // namespace errie
  namespace eopie {
    enum {
      POSITION = 12,
      MASK = 0b1 << POSITION
    };
    enum States {
      DISABLED = 0 << POSITION,
      EOP_INT_ENABLED = 1 << POSITION
    };
  }  // namespace eopie
} // namespace cr
namespace obr {
  enum {
    OFFSET = 0x1C
  };
  namespace opterr {
    enum {
      POSITION = 0,
      MASK = 0b1 << POSITION
    };
    enum States {
      NO_ERROR = 0 << POSITION,
      ERROR = 1 << POSITION
    };
  }  // namespace opterr
  namespace rdprt {
    enum {
      POSITION = 1,
      MASK = 0b1 << POSITION
    };
    enum States {
      READ_ENABLED = 0 << POSITION,
      READ_PROTECTED = 1 << POSITION
    };
  }  // namespace rdprt
  namespace user {
    enum {
      POSITION = 2,
      MASK = 0xFF << POSITION
    };
  }  // namespace user
  namespace data0 {
    enum {
      POSITION = 10,
      MASK = 0xFF << POSITION
    };
  }  // namespace data0
  namespace data1 {
    enum {
      POSITION = 18,
      MASK = 0xFF << POSITION
    };
  }  // namespace data1
} // namespace obr
#else /* STM32F2XX || STM32F4XX */
namespace acr {
  enum {
    OFFSET = 0x00
  };
  namespace latency {
    enum {
      POSITION = 0,
      MASK = 0b111 << POSITION
    };
    enum States {
      ZERO_WAIT_STATE = 0 << POSITION,
      ONE_WAIT_STATE = 1 << POSITION,
      TWO_WAIT_STATES = 2 << POSITION,
      THREE_WAIT_STATES = 3 << POSITION,
      FOUR_WAIT_STATES = 4 << POSITION,
      FIVE_WAIT_STATES = 5 << POSITION,
      SIX_WAIT_STATES = 6 << POSITION,
      SEVEN_WAIT_STATES = 7 << POSITION,
    };
  }  // namespace latency

  namespace prften {
    enum {
      POSITION = 8,
      MASK = 1 << POSITION
    };
    enum States {
      PREFETCH_DISABLED = 0 << POSITION,
      PREFETCH_ENABLED = 1 << POSITION,
    };
  }  // namespace prften

  namespace icen {
    enum {
      POSITION = 9,
      MASK = 1 << POSITION
    };
    enum States {
      INSTRUCTION_CACHE_DISABLED = 0 << POSITION,
      INSTRUCTION_CACHE_ENABLED = 1 << POSITION,
    };
  }  // namespace icen

  namespace dcen {
    enum {
      POSITION = 10,
      MASK = 1 << POSITION
    };
    enum States {
      DATA_CACHE_DISABLED = 0 << POSITION,
      DATA_CACHE_ENABLED = 1 << POSITION,
    };
  }  // namespace dcen
}  // namespace acr
#endif /* STM32F1XX */
}  // namespace flash
