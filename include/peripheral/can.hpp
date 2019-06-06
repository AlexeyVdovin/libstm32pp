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

/*******************************************************************************
 *
 *                         Controller Area Network
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#ifndef VALUE_LINE

#include "../defs.hpp"
#include "../../memorymap/can.hpp"

#if defined CONNECTIVITY_LINE || not defined STM32F1XX
#define CAN1_REGS reinterpret_cast<can::Registers *>(can::address::E::CAN1)
#define CAN2_REGS reinterpret_cast<can::Registers *>(can::address::E::CAN2)
#else // !CONNECTIVITY_LINE || STM32F1XX
#define CAN_REGS reinterpret_cast<can::Registers *>(can::address::E::CAN)
#endif // !CONNECTIVITY_LINE || STM32F1XX
namespace can {
  typedef struct {
    u32 addr;
    bool ide;
    bool rtr;
  } hdr;

  typedef struct {
	  hdr h;
	  u16 ts;
	  u8  fmi;
	  u8  len;
	  union {
	    u8  d[8];
	    u32 u[2];
	  };
  } msg;
  
  namespace interrupt {
    enum State {
    	DISABLE = 0,
    	ENABLE = 1
    };
  }
  namespace operation {
    enum Mode {
    	UNKNOWN = 0,
    	SLEEP   = 1,
    	NORMAL  = 2,
    	INIT    = 3,
    	ERROR   = 4,
    	BUSSOFF = 5
    };
  }
  namespace mailbox {
    enum Complete {
    	MBX0 = tsr::rqcp0::MASK,
    	MBX1 = tsr::rqcp1::MASK,
    	MBX2 = tsr::rqcp2::MASK,
    	UNKNOWN = 0xffffffff
    };
  }


  template<Address>
  class Functions {
    public:
      enum {
        FREQUENCY = clk::APB1
      };

      static inline void enableClocks();
      static inline void disableClocks();
      static inline u32 getFrequency() { return FREQUENCY; }
      static inline void masterReset();
      static inline void configEnter();
      static inline void configExit();
      static inline u32 setBaudrate(u32 bps);
      static inline u32 setBaudrate(u32 bps, u8 sjw, u8 bs1, u8 bs2);
      static inline void configure(
    		  mcr::txfp::States txfp,
    		  mcr::rflm::States rflm,
    		  mcr::nart::States nart,
    		  mcr::awum::States awum = mcr::awum::AUTO_WAKEUP_ON,
    		  mcr::abom::States abom = mcr::abom::AUTO_BUSOFF,
    		  mcr::ttcm::States ttcm = mcr::ttcm::TIME_TRIGGERED_MODE_DISABLED,
    		  mcr::dbf::States dbf = mcr::dbf::NORMAL_RUN,
    		  btr::lbkm::States lbkm = btr::lbkm::LOOPBACK_OFF,
              btr::silm::States silm = btr::silm::NORMAL_OPERATION
             );

      static inline operation::Mode getMode();
      static inline void sleepEnter();
      static inline void sleepExit();

      template<interrupt::State> static inline void txEmptyInterrupt();

      template<interrupt::State> static inline void rx0MsgInterrupt();
      template<interrupt::State> static inline void rx0FullInterrupt();
      template<interrupt::State> static inline void rx0OverrunInterrupt();

      template<interrupt::State> static inline void rx1MsgInterrupt();
      template<interrupt::State> static inline void rx1FullInterrupt();
      template<interrupt::State> static inline void rx1OverrunInterrupt();

      template<interrupt::State> static inline void errorInterrupt();
      template<interrupt::State> static inline void errorWarningInterrupt();
      template<interrupt::State> static inline void errorPassiveInterrupt();
      template<interrupt::State> static inline void busOffInterrupt();
      template<interrupt::State> static inline void lastErrorCodeInterrupt();
      template<interrupt::State> static inline void wakeupInterrupt();
      template<interrupt::State> static inline void sleepInterrupt();

      static inline void unmaskInterrupts();
      static inline void maskInterrupts();

      template<u8 BANK, u8 FIFO> static inline void setFilterMaskBank(hdr* a, hdr* m);
      template<u8 BANK, u8 FIFO> static inline void setFilterListBank(hdr* a0, hdr* a1);
      template<u8 BANK, u8 FIFO> static inline void setFilterMaskBank(hdr* a0, hdr* m0, hdr* a1, hdr* m1);
      template<u8 BANK, u8 FIFO> static inline void setFilterListBank(hdr* a0, hdr* a1, hdr* a2, hdr* a3);
      template<u8 BANK> static inline void setFilterActive(bool active);
      
      template<u8 FIFO> static inline void Receive(msg* m); // copies msg from fifo mailbox
      template<u8 T> static inline void Send(msg* m);
      static inline u8 Send(msg* m, bool force = false); // returns mailbox id
      template<u8 T> static inline bool isTxEmpty();
      static inline mailbox::Complete getTxCompleteMbx();
      template<u8 T> static inline void cleanTxComplete();

      template<u8 FIFO> static bool msgRxPending();
      template<u8 FIFO> static bool rxFifoOverrun();
      template<u8 FIFO> static bool rxFifoFull();

  };
}  // namespace can

// High-level access to the peripheral
#if defined CONNECTIVITY_LINE || not defined STM32F1XX
typedef can::Functions<can::CAN1> CAN1;
typedef can::Functions<can::CAN2> CAN2;
#else // !CONNECTIVITY_LINE || STM32F1XX
typedef can::Functions<can::CAN> CAN;
#endif // !CONNECTIVITY_LINE || STM32F1XX
#include "../../bits/can.tcc"

#endif
