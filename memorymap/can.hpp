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

namespace can {
  struct Registers {
      __RW
      u32 MCR;      // 0x000: Master control
      __RW
      u32 MSR;      // 0x004: Master status
      __RW
      u32 TSR;      // 0x008: Transmit status
      __RW
      u32 RFR[2];      // 0x00C: Receive FIFO
      __RW
      u32 IER;      // 0x014: Interrupt enable
      __RW
      u32 ESR;      // 0x018: Error status
      __RW
      u32 BTR;      // 0x01C: Bit timing
      u32 _RESERVED0[88];
      struct {
          __RW
          u32 IR;   // 0x180, 0x190, 0x1A0: identifier
          __RW
          u32 DTR;  // 0x184, 0x194, 0x1A4: data length control and time stamp
          __RW
          u32 DLR;  // 0x188, 0x198, 0x1A8: data low
          __RW
          u32 DHR;  // 0x18C, 0x19C, 0x1AC: data high
      } TX[3];  // TX FIFO mailbox
      struct {
          __RW
          u32 IR;   // 0x1B0, 0x1C0: Identifier
          __RW
          u32 DTR;  // 0x1B4, 0x1C4: Data length control and time stamp
          __RW
          u32 DLR;  // 0x1B8, 0x1C8: Data low
          __RW
          u32 DHR;  // 0x1BC, 0x1CC: Data high
      } RX[2];  // RX FIFO mailbox
      u32 _RESERVED1[12];
      __RW
      u32 FMR;      // 0x200: Filter master
      __RW
      u32 FM1R;     // 0x204: Filter mode
      u32 _RESERVED2;
      __RW
      u32 FS1R;     // 0x20C: Filter scale
      u32 _RESERVED3;
      __RW
      u32 FFA1R;    // 0x214: Filter FIFO assignment
      u32 _RESERVED4;
      __RW
      u32 FA1R;     // 0x21C: Filter activation
      u32 _RESERVED5[8];
      struct {
          __RW
          u32 L;    // 0x240 + 8 * I: Low
          __RW
          u32 H;    // 0x244 + 8 * I: High
      } FR[
      #if not defined STM32F1XX || defined CONNECTIVITY_LINE
      27 // 0x240-0x31C: Filter bank
#else // !STM32F1XX || CONNECTIVITY_LINE
      13// 0x240-0x2AC: Filter bank
#endif // !STM32F1XX || CONNECTIVITY_LINE
      ];
  };

  enum Address {
#if defined CONNECTIVITY_LINE || not defined STM32F1XX
    CAN1 = alias::APB1 + 0x6400,
    CAN2 = alias::APB1 + 0x6800,
#else // !CONNECTIVITY_LINE || STM32F1XX
  CAN = alias::APB1 + 0x6400,
#endif // !CONNECTIVITY_LINE || STM32F1XX
};

  namespace mcr {
    enum {
      OFFSET = 0x00
    };

    namespace inrq {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        NORMAL_MODE = 0 << POSITION,
        INITIALIZATION_MODE = 1 << POSITION,
      };
    }  // namespace inrq

    namespace sleep {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        SLEEP_EXIT = 0 << POSITION,
        SLEEP_ENTER = 1 << POSITION,
      };
    }  // namespace sleep

    namespace txfp {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        TX_MESSAGE_ID_PRIORTY = 0 << POSITION,
        TX_QUEUE_PRIORITY = 1 << POSITION,
      };
    }  // namespace txfp

    namespace rflm {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        OVERRUN_NOT_LOCKED = 0 << POSITION,
        OVERRUN_LOCKED = 1 << POSITION,
      };
    }  // namespace rflm

    namespace nart {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        AUTO_RETRANSMIT = 0 << POSITION,
        NO_RETRANSMIT = 1 << POSITION,
      };
    }  // namespace nart

    namespace awum {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        AUTO_WAKEUP_OFF = 0 << POSITION,
        AUTO_WAKEUP_ON = 1 << POSITION,
      };
    }  // namespace awum

    namespace abom {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        SOFTWARE_BUSOFF = 0 << POSITION,
        AUTO_BUSOFF = 1 << POSITION,
      };
    }  // namespace abom

    namespace ttcm {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        TIME_TRIGGERED_MODE_DISABLED = 0 << POSITION,
        TIME_TRIGGERED_MODE_ENABLED = 1 << POSITION,
      };
    }  // namespace ttcm

    namespace reset {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        NORMAL_OPERATION = 0 << POSITION,
        MASTER_RESET = 1 << POSITION,
      };
    }  // namespace reset

    namespace dbf {
      enum {
        POSITION = 16,
        MASK = 1 << POSITION
      };
      enum States {
        NORMAL_RUN = 0 << POSITION,
        DEBUG_FREEZE = 1 << POSITION,
      };
    }  // namespace dbf
  }

  namespace msr {
    enum {
      OFFSET = 0x04
    };

    namespace inak {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
    }  // namespace inak

    namespace slak {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
    }  // namespace slak

    namespace erri {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        CLEAR = 0 << POSITION,
      };
    }  // namespace erri

    namespace wkui {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        CLEAR = 0 << POSITION,
      };
    }  // namespace wkui

    namespace slaki {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        CLEAR = 0 << POSITION,
      };
    }  // namespace slaki

    namespace txm {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        TRANSMITTER = 1 << POSITION,
      };
    }  // namespace txm

    namespace rxm {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        RECEIVER = 1 << POSITION,
      };
    }  // namespace rxm

    namespace samp {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
    }  // namespace samp

    namespace rx {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
    }  // namespace rx
  }

  namespace tsr {
    enum {
      OFFSET = 0x08
    };

    namespace rqcp0 {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        CLEAR = 1 << POSITION,
      };
    }  // namespace rqcp0

    namespace txok0 {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
    }  // namespace txok0

    namespace alst0 {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
    }  // namespace alst0

    namespace terr0 {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
    }  // namespace terr0

    namespace abrq0 {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        ABORT = 1 << POSITION,
      };
    }  // namespace abrq0

    namespace rqcp1 {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        CLEAR = 1 << POSITION,
      };
    }  // namespace rqcp1

    namespace txok1 {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
    }  // namespace txok1

    namespace alst1 {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
    }  // namespace alst1

    namespace terr1 {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
    }  // namespace terr1

    namespace abrq1 {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        ABORT = 1 << POSITION,
      };
    }  // namespace abrq1

    namespace rqcp2 {
      enum {
        POSITION = 16,
        MASK = 1 << POSITION
      };
      enum States {
        CLEAR = 1 << POSITION,
      };
    }  // namespace rqcp2

    namespace txok2 {
      enum {
        POSITION = 17,
        MASK = 1 << POSITION
      };
    }  // namespace txok2

    namespace alst2 {
      enum {
        POSITION = 18,
        MASK = 1 << POSITION
      };
    }  // namespace alst2

    namespace terr2 {
      enum {
        POSITION = 19,
        MASK = 1 << POSITION
      };
    }  // namespace terr2

    namespace abrq2 {
      enum {
        POSITION = 23,
        MASK = 1 << POSITION
      };
      enum States {
        ABORT = 1 << POSITION,
      };
    }  // namespace abrq2

    namespace code {
      enum {
        POSITION = 24,
        MASK = 0b11 << POSITION
      };
    }  // namespace code

    namespace tme0 {
      enum {
        POSITION = 26,
        MASK = 1 << POSITION
      };
      enum States {
        EMPTY = 1 << POSITION,
      };
    }  // namespace tme0

    namespace tme1 {
      enum {
        POSITION = 27,
        MASK = 1 << POSITION
      };
      enum States {
        EMPTY = 1 << POSITION,
      };
    }  // namespace tme1
    
    namespace tme2 {
      enum {
        POSITION = 28,
        MASK = 1 << POSITION
      };
      enum States {
        EMPTY = 1 << POSITION,
      };
    }  // namespace tme2

    namespace low0 {
      enum {
        POSITION = 29,
        MASK = 1 << POSITION
      };
    }  // namespace low0

    namespace low1 {
      enum {
        POSITION = 30,
        MASK = 1 << POSITION
      };
    }  // namespace low1

    namespace low2 {
      enum {
        POSITION = 31,
        MASK = 1 << POSITION
      };
    }  // namespace low2
  }
  
  namespace rfr { // receive FIFO register

    namespace fmp {
      enum {
        POSITION = 0,
        MASK = 3 << POSITION
      };
      enum States {
        EMPTY = 0 << POSITION,
      };
    }  // namespace fmp

    namespace full {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        FULL = 1 << POSITION,
      };
    }  // namespace full

    namespace fovr {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        CLEAR = 0 << POSITION,
        OVERRUN = 1 << POSITION,
      };
    }  // namespace fovr

    namespace rfom {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        READY = 0 << POSITION,
        RELEASE = 1 << POSITION,
      };
    }  // namespace rfom
  }

  namespace ier {
    enum {
      OFFSET = 0x14
    };

    namespace tmeie {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
    }  // namespace tmeie

    namespace fmpie0 {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
    }  // namespace fmpie0

    namespace ffie0 {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
    }  // namespace ffie0

    namespace fovie0 {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
    }  // namespace fovie0

    namespace fmpie1 {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
    }  // namespace fmpie1

    namespace ffie1 {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
    }  // namespace ffie1

    namespace fovie1 {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
    }  // namespace fovie1
    
    namespace ewgie {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
    }  // namespace ewgie

    namespace epvie {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
    }  // namespace epvie

    namespace bofie {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
    }  // namespace bofie

    namespace lecie {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
    }  // namespace lecie

    namespace errie {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
    }  // namespace errie

    namespace wkuie {
      enum {
        POSITION = 16,
        MASK = 1 << POSITION
      };
    }  // namespace wkuie

    namespace slkie {
      enum {
        POSITION = 17,
        MASK = 1 << POSITION
      };
    }  // namespace slkie
  }

  namespace esr {
    enum {
      OFFSET = 0x18
    };

    namespace ewgf {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
    }  // namespace ewgf
    
    namespace epvf {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
    }  // namespace epvf
    
    namespace boff {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
    }  // namespace boff
    
    namespace lec {
      enum {
        POSITION = 4,
        MASK = 0b111 << POSITION
      };
      enum States {
        NO_ERROR        = 0 << POSITION,
        STUFF_ERROR     = 1 << POSITION,
        FORM_ERROR      = 2 << POSITION,
        ACK_ERROR       = 3 << POSITION,
        RECESSIVE_ERROR = 4 << POSITION,
        DOMINANT_ERROR  = 5 << POSITION,
        CRC_ERROR       = 6 << POSITION,
        SW_ERROR        = 7 << POSITION,
      };
    }  // namespace lec

    namespace tec {
      enum {
        POSITION = 16,
        MASK = 0xff << POSITION
      };
    }  // namespace tec

    namespace rec {
      enum {
        POSITION = 24,
        MASK = 0xff << POSITION
      };
    }  // namespace rec
  }

  namespace btr {
    enum {
      OFFSET = 0x1c
    };

    namespace brp {
      enum {
        POSITION = 0,
        MASK = 0x1ff << POSITION
      };
    }  // namespace brp
    
    namespace ts1 {
      enum {
        POSITION = 16,
        MASK = 0x0f << POSITION
      };
    }  // namespace ts1

    namespace ts2 {
      enum {
        POSITION = 20,
        MASK = 0x07 << POSITION
      };
    }  // namespace ts2

    namespace sjw {
      enum {
        POSITION = 24,
        MASK = 0x03 << POSITION
      };
    }  // namespace sjw

    namespace lbkm {
      enum {
        POSITION = 30,
        MASK = 1 << POSITION
      };
      enum States {
        LOOPBACK_OFF = 0 << POSITION,
        LOOPBACK_ON  = 1 << POSITION,
      };
    }  // namespace lbkm

    namespace silm {
      enum {
        POSITION = 31,
        MASK = 1 << POSITION
      };
      enum States {
        NORMAL_OPERATION = 0 << POSITION,
        SILENT_MODE      = 1 << POSITION,
      };
    }  // namespace silm
  }

  namespace tx {
    namespace tixr {
      enum {
        OFFSET = 0x00
      };

      namespace txrq {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          EMPTY  = 0 << POSITION,
          TX_REQ = 1 << POSITION,
        };
      }  // namespace txrq
    
      namespace rtr {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          DATA_FR   = 0 << POSITION,
          REMOTE_FR = 1 << POSITION,
        };
      }  // namespace rtr
    
      namespace ide {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          STD_ID = 0 << POSITION,
          EXT_ID = 1 << POSITION,
        };
      }  // namespace ide

      namespace exid {
        enum {
          POSITION = 3,
          MASK = 0x1fffffff << POSITION
        };
      }  // namespace exid

      namespace stid {
        enum {
          POSITION = 21,
          MASK = 0x7ff << POSITION
        };
      }  // namespace stid
    }
    
    namespace tdtxr {
      enum {
        OFFSET = 0x04
      };

      namespace dlc {
        enum {
          POSITION = 0,
          MASK = 0x0F << POSITION
        };
      }  // namespace dlc

      namespace tgt {
        enum {
          POSITION = 8,
          MASK = 1 << POSITION
        };
      }  // namespace tgt

      namespace time {
        enum {
          POSITION = 16,
          MASK = 0xffff << POSITION
        };
      }  // namespace time
    }

    namespace tdlxr {
      enum {
        OFFSET = 0x08
      };
    }
    
    namespace tdhxr {
      enum {
        OFFSET = 0x0c
      };
    }
  }

  namespace rx {
      namespace rtr {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          DATA_FR   = 0 << POSITION,
          REMOTE_FR = 1 << POSITION,
        };
      }  // namespace rtr
    
      namespace ide {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          STD_ID = 0 << POSITION,
          EXT_ID = 1 << POSITION,
        };
      }  // namespace ide

      namespace exid {
        enum {
          POSITION = 3,
          MASK = 0x1fffffff << POSITION
        };
      }  // namespace exid

      namespace stid {
        enum {
          POSITION = 21,
          MASK = 0x7ff << POSITION
        };
      }  // namespace stid

      namespace dlc {
        enum {
          POSITION = 0,
          MASK = 0x0F << POSITION
        };
      }  // namespace dlc

      namespace fmi {
        enum {
          POSITION = 8,
          MASK = 0xff << POSITION
        };
      }  // namespace fmi

      namespace time {
        enum {
          POSITION = 16,
          MASK = 0xffff << POSITION
        };
      }  // namespace time
  }

  namespace fmr {
    enum {
      OFFSET = 0x200
    };

    namespace finit {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        ACTIVE_MODE = 0 << POSITION,
        INIT_MODE   = 1 << POSITION,
      };
    }  // namespace finit
    
    namespace can2sb {
      enum {
        POSITION = 8,
        MASK = 0x3f << POSITION
      };
    }  // namespace can2sb
  }
  
  namespace fm1r {
    enum {
      OFFSET = 0x204
    };
  }

  namespace fs1r {
    enum {
      OFFSET = 0x20C
    };
  }

  namespace ffa1r {
    enum {
      OFFSET = 0x214
    };
  }

  namespace fa1r {
    enum {
      OFFSET = 0x21C
    };
  }
  
  // TODO CAN register bits

}// namespace can
