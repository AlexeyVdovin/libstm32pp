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

#include "../include/peripheral/rcc.hpp"
#include "../include/core/nvic.hpp"


namespace can {
  template<Address C>
  void Functions<C>::enableClocks()
  {
    switch (C) {
#if defined CONNECTIVITY_LINE || not defined STM32F1XX
      case CAN1:
        RCC::enableClocks<rcc::apb1enr::CAN1>();
        break;
      case CAN2:
        RCC::enableClocks<rcc::apb1enr::CAN2>();
        break;
#else // !CONNECTIVTY_LINE || STM32F1XX
      case CAN:
        RCC::enableClocks<rcc::apb1enr::CAN>();
        break;
#endif // !CONNECTIVITY_LINE
    }
  }

  template<Address C>
  void Functions<C>::disableClocks()
  {
    switch (C) {
#if defined CONNECTIVITY_LINE || not defined STM32F1XX
      case CAN1:
        RCC::disableClocks<rcc::apb1enr::CAN1>();
        break;
      case CAN2:
        RCC::disableClocks<rcc::apb1enr::CAN2>();
        break;
#else // !CONNECTIVTY_LINE || STM32F1XX
      case CAN:
        RCC::disableClocks<rcc::apb1enr::CAN>();
        break;
#endif // !CONNECTIVITY_LINE
    }
  }
  
  template<Address C>
  void Functions<C>::masterReset()
  {
    reinterpret_cast<Registers*>(C)->MCR |= mcr::reset::MASK;
  }
  
  template<Address C>
  void Functions<C>::configEnter()
  {
    reinterpret_cast<Registers*>(C)->MCR |= mcr::inrq::MASK;
    reinterpret_cast<Registers*>(C)->MCR &= ~(mcr::sleep::MASK);
    while(!(reinterpret_cast<Registers*>(C)->MSR & msr::inak::MASK)) {} 
  }
  
  template<Address C>
  void Functions<C>::configExit()
  {
    reinterpret_cast<Registers*>(C)->MCR &= ~(mcr::inrq::MASK);
    while(reinterpret_cast<Registers*>(C)->MSR & msr::inak::MASK) {} 
  }
  
  template<Address C>
  u32 Functions<C>::setBaudrate(u32 bps, u8 sjw, u8 bs1, u8 bs2)
  {
    register u32 brp = FREQUENCY / bps / (3 + bs1 + bs2);
    register u32 btr = reinterpret_cast<Registers*>(C)->BTR & 
    	~(btr::brp::MASK | btr::sjw::MASK | btr::ts1::MASK | btr::ts2::MASK);
    reinterpret_cast<Registers*>(C)->BTR = btr | 
    	((brp-1) << btr::brp::POSITION) | (sjw << btr::sjw::POSITION) | (bs1 << btr::ts1::POSITION) | (bs2 << btr::ts2::POSITION); 
    bps = FREQUENCY / brp / (3 + bs1 + bs2);
    return bps;
  }

  template<Address C>
  u32 Functions<C>::setBaudrate(u32 bps)
  {
	register u32 btr = reinterpret_cast<Registers*>(C)->BTR;
    u8 bs1 = (btr & btr::ts1::MASK) >> btr::ts1::POSITION;
    u8 bs2 = (btr & btr::ts2::MASK) >> btr::ts2::POSITION;
    u8 sjw = (btr & btr::sjw::MASK) >> btr::sjw::POSITION;
    return setBaudrate(bps, sjw, bs1, bs2);
  }
  
  template<Address C>
  void Functions<C>::configure(
	  mcr::txfp::States txfp,
	  mcr::rflm::States rflm,
	  mcr::nart::States nart,
	  mcr::awum::States awum,
	  mcr::abom::States abom,
	  mcr::ttcm::States ttcm,
	  mcr::dbf::States dbf,
	  btr::lbkm::States lbkm,
      btr::silm::States silm)
  {
    u32 mcr = reinterpret_cast<Registers*>(C)->MCR &
        ~(mcr::txfp::MASK | mcr::rflm::MASK | mcr::nart::MASK | mcr::awum::MASK | mcr::abom::MASK | mcr::ttcm::MASK | mcr::dbf::MASK);
    reinterpret_cast<Registers*>(C)->MCR = mcr | txfp | rflm | nart | awum | abom | ttcm | dbf;
    
    u32 btr = reinterpret_cast<Registers*>(C)->BTR & ~(btr::lbkm::MASK | btr::silm::MASK);
    reinterpret_cast<Registers*>(C)->BTR = btr | lbkm | silm;
  }
  
  template<Address C>
  operation::Mode Functions<C>::getMode()
  {
    operation::Mode mode = operation::UNKNOWN;
    bool slak = (reinterpret_cast<Registers*>(C)->MSR & msr::slak::MASK) != 0;
    bool inak = (reinterpret_cast<Registers*>(C)->MSR & msr::inak::MASK) != 0;
     
    if(!slak && inak) mode = operation::INIT;
    else if(slak && !inak) mode = operation::SLEEP;
    else if(!slak && !inak) mode = operation::NORMAL;
    
    return mode;
  }
  
  template<Address C>
  void Functions<C>::sleepEnter()
  {
    reinterpret_cast<Registers*>(C)->MCR |= mcr::sleep::SLEEP_ENTER;
    while(!(reinterpret_cast<Registers*>(C)->MSR & msr::slak::MASK)) {}
  }

  template<Address C>
  void Functions<C>::sleepExit()
  {
    reinterpret_cast<Registers*>(C)->MCR &= ~(mcr::sleep::MASK);
    while(reinterpret_cast<Registers*>(C)->MSR & msr::slak::MASK) {}
  }

  template<Address C>
  template<u8 BANK, u8 FIFO>
  void Functions<C>::setFilterMaskBank(hdr* a, hdr* m)
  {
    static_assert(FIFO < 2, "FIFO 0 and 1 is only supported.");
    static_assert(
#if defined CONNECTIVITY_LINE       
      BANK < 28,
#else
      BANK < 14,
#endif
        "This bank is not supported by selected MCU.");

    reinterpret_cast<Registers*>(C)->FMR |= fmr::finit::INIT_MODE;
    // Deactivate filter
    reinterpret_cast<Registers*>(C)->FA1R &= ~(1 << BANK);
    // Addr - Mask mode
    reinterpret_cast<Registers*>(C)->FM1R &= ~(1 << BANK);
    // Single 32-bit scale
    reinterpret_cast<Registers*>(C)->FS1R |= 1 << BANK;
    // FIFO assignment
    u32 ffa1r = reinterpret_cast<Registers*>(C)->FFA1R;  
    reinterpret_cast<Registers*>(C)->FFA1R = (ffa1r & ~(1 << BANK)) | (FIFO << BANK);

    reinterpret_cast<Registers*>(C)->FR[BANK].L = /* Addr */
            (a->ide ? (a->addr << 3) | 4 : (a->addr << 21)) | (a->rtr ? 2 : 0);
    reinterpret_cast<Registers*>(C)->FR[BANK].H = /* Mask */   
            (a->ide ? (m->addr << 3) : (m->addr << 21)) | (m->ide ? 4 : 0) | (m->rtr ? 2 : 0);

    reinterpret_cast<Registers*>(C)->FMR &= ~fmr::finit::INIT_MODE;
  }

  template<Address C>
  template<u8 BANK, u8 FIFO>
  void Functions<C>::setFilterListBank(hdr* a0, hdr* a1)
  {
    static_assert(FIFO < 2, "FIFO 0 and 1 is only supported.");
    static_assert(
#if defined CONNECTIVITY_LINE       
      BANK < 28,
#else
      BANK < 14,
#endif
        "This bank is not supported by selected MCU.");

    reinterpret_cast<Registers*>(C)->FMR |= fmr::finit::INIT_MODE;
    // Deactivate filter
    reinterpret_cast<Registers*>(C)->FA1R &= ~(1 << BANK);
    // Addr - Addr mode
    reinterpret_cast<Registers*>(C)->FM1R |= 1 << BANK;
    // Single 32-bit scale
    reinterpret_cast<Registers*>(C)->FS1R |= 1 << BANK;
    // FIFO assignment
    u32 ffa1r = reinterpret_cast<Registers*>(C)->FFA1R;  
    reinterpret_cast<Registers*>(C)->FFA1R = (ffa1r & ~(1 << BANK)) | (FIFO << BANK);
    
    reinterpret_cast<Registers*>(C)->FR[BANK].L = /* Addr 0 */
            (a0->ide ? (a0->addr << 3) | 4 : (a0->addr << 21)) | (a0->rtr ? 2 : 0);
    reinterpret_cast<Registers*>(C)->FR[BANK].H = /* Addr 1 */
            (a1->ide ? (a1->addr << 3) | 4 : (a1->addr << 21)) | (a1->rtr ? 2 : 0);

    reinterpret_cast<Registers*>(C)->FMR &= ~fmr::finit::INIT_MODE;
  }
  
  template<Address C>
  template<u8 BANK, u8 FIFO>
  void Functions<C>::setFilterMaskBank(hdr* a0, hdr* m0, hdr* a1, hdr* m1)
  {
	enum {
		IDE = 0x08,
		RTR = 0x10
	};
    static_assert(FIFO < 2, "FIFO 0 and 1 is only supported.");
    static_assert(
#if defined CONNECTIVITY_LINE       
      BANK < 28,
#else
      BANK < 14,
#endif
        "This bank is not supported by selected MCU.");
  
    reinterpret_cast<Registers*>(C)->FMR |= fmr::finit::INIT_MODE;
    // Deactivate filter
    reinterpret_cast<Registers*>(C)->FA1R &= ~(1 << BANK);
    // Addr - Mask mode
    reinterpret_cast<Registers*>(C)->FM1R &= ~(1 << BANK);
    // Two 16-bit scale
    reinterpret_cast<Registers*>(C)->FS1R &= ~(1 << BANK);
    // FIFO assignment
    u32 ffa1r = reinterpret_cast<Registers*>(C)->FFA1R;  
    reinterpret_cast<Registers*>(C)->FFA1R = (ffa1r & ~(1 << BANK)) | (FIFO << BANK);
    
    reinterpret_cast<Registers*>(C)->FR[BANK].L = /* Addr0 - Mask0 */
    	  (a0->ide ? (((a0->addr >> 13) & 0xffe0) | ((a0->addr >> 15) & 0x07) | IDE) : (a0->addr << 5)) | (a0->rtr ? RTR : 0) |
    	(((a0->ide ? (((m0->addr >> 13) & 0xffe0) | ((m0->addr >> 15) & 0x07) | IDE) : (m0->addr << 5)) | (m0->rtr ? RTR : 0)) << 16);

    reinterpret_cast<Registers*>(C)->FR[BANK].H = /* Addr1 - Mask1 */   
      	  (a1->ide ? (((a1->addr >> 13) & 0xffe0) | ((a1->addr >> 15) & 0x07) | IDE) : (a1->addr << 5)) | (a1->rtr ? RTR : 0) |
      	(((a1->ide ? (((m1->addr >> 13) & 0xffe0) | ((m1->addr >> 15) & 0x07) | IDE) : (m1->addr << 5)) | (m1->rtr ? RTR : 0)) << 16);
    	    		
    reinterpret_cast<Registers*>(C)->FMR &= ~fmr::finit::INIT_MODE;
  }
  
  template<Address C>
  template<u8 BANK, u8 FIFO>
  void Functions<C>::setFilterListBank(hdr* a0, hdr* a1, hdr* a2, hdr* a3)
  {
	enum {
		IDE = 0x08,
		RTR = 0x10
	};
    static_assert(FIFO < 2, "FIFO 0 and 1 is only supported.");
    static_assert(
#if defined CONNECTIVITY_LINE       
      BANK < 28,
#else
      BANK < 14,
#endif
        "This bank is not supported by selected MCU.");
  
    reinterpret_cast<Registers*>(C)->FMR |= fmr::finit::INIT_MODE;
    // Deactivate filter
    reinterpret_cast<Registers*>(C)->FA1R &= ~(1 << BANK);
    // Addr - Addr mode
    reinterpret_cast<Registers*>(C)->FM1R |= 1 << BANK;
    // Four 16-bit scale
    reinterpret_cast<Registers*>(C)->FS1R |= 1 << BANK;
    // FIFO assignment
    u32 ffa1r = reinterpret_cast<Registers*>(C)->FFA1R;  
    reinterpret_cast<Registers*>(C)->FFA1R = (ffa1r & ~(1 << BANK)) | (FIFO << BANK);

    reinterpret_cast<Registers*>(C)->FR[BANK].L = /* Addr0 - Addr1 */
    	  (a0->ide ? (((a0->addr >> 13) & 0xffe0) | ((a0->addr >> 15) & 0x07) | IDE) : (a0->addr << 5)) | (a0->rtr ? RTR : 0) |
    	(((a1->ide ? (((a1->addr >> 13) & 0xffe0) | ((a1->addr >> 15) & 0x07) | IDE) : (a1->addr << 5)) | (a1->rtr ? RTR : 0)) << 16);

    reinterpret_cast<Registers*>(C)->FR[BANK].H = /* Addr2 - Addr3 */   
      	  (a2->ide ? (((a2->addr >> 13) & 0xffe0) | ((a2->addr >> 15) & 0x07) | IDE) : (a2->addr << 5)) | (a2->rtr ? RTR : 0) |
      	(((a3->ide ? (((a3->addr >> 13) & 0xffe0) | ((a3->addr >> 15) & 0x07) | IDE) : (a3->addr << 5)) | (a3->rtr ? RTR : 0)) << 16);
    
    reinterpret_cast<Registers*>(C)->FMR &= ~fmr::finit::INIT_MODE;
  }
  
  template<Address C>
  template<u8 BANK>
  void Functions<C>::setFilterActive(bool active)
  {
    static_assert(
#if defined CONNECTIVITY_LINE       
      BANK < 28,
#else
      BANK < 14,
#endif
        "This bank is not supported by selected MCU.");
        
    reinterpret_cast<Registers*>(C)->FA1R = (reinterpret_cast<Registers*>(C)->FA1R & ~(1 << BANK)) | (active ? (1 << BANK) : 0); 
  }
  
  template<Address C>
  template<u8 FIFO>
  void Functions<C>::Receive(msg* m) // copies msg from fifo mailbox
  {
    static_assert(FIFO < 2, "FIFO 0 and 1 is only supported.");
    u32 ir = reinterpret_cast<Registers*>(C)->RX[FIFO].IR;
    m->h.ide = (ir & rx::ide::MASK) >> rx::ide::POSITION; 
    m->h.rtr = (ir & rx::rtr::MASK) >> rx::rtr::POSITION;
    if((ir & rx::ide::MASK) == rx::ide::STD_ID)
      m->h.addr = (ir & rx::stid::MASK) >> rx::stid::POSITION;
    else
      m->h.addr = (ir & rx::exid::MASK) >> rx::exid::POSITION;
    u32 dtr = reinterpret_cast<Registers*>(C)->RX[FIFO].DTR;
    m->ts = (dtr & rx::time::MASK) >> rx::time::POSITION;
    m->fmi = (dtr & rx::fmi::MASK) >> rx::fmi::POSITION;
    m->len = (dtr & rx::dlc::MASK) >> rx::dlc::POSITION;
    m->u[0] = reinterpret_cast<Registers*>(C)->RX[FIFO].DLR;
    m->u[1] = reinterpret_cast<Registers*>(C)->RX[FIFO].DHR;
    reinterpret_cast<Registers*>(C)->RFR[FIFO] |= rfr::rfom::RELEASE;
  }
  
  template<Address C>
  template<u8 FIFO>
  bool Functions<C>::msgRxPending()
  {
    static_assert(FIFO < 2, "FIFO 0 and 1 is only supported.");
    return ((reinterpret_cast<Registers*>(C)->RFR[FIFO] & rfr::fmp::MASK) != rfr::fmp::EMPTY);
  }
  
  template<Address C>
  template<u8 FIFO>
  bool Functions<C>::rxFifoOverrun()
  {
    static_assert(FIFO < 2, "FIFO 0 and 1 is only supported.");
    bool res = ((reinterpret_cast<Registers*>(C)->RFR[FIFO] & rfr::fovr::MASK) == rfr::fovr::OVERRUN);
    reinterpret_cast<Registers*>(C)->RFR[FIFO] &= ~rfr::fovr::OVERRUN;
    return res;
  }
  
  template<Address C>
  template<u8 FIFO>
  bool Functions<C>::rxFifoFull()
  {
    static_assert(FIFO < 2, "FIFO 0 and 1 is only supported.");
    bool res = ((reinterpret_cast<Registers*>(C)->RFR[FIFO] & rfr::full::MASK) == rfr::full::FULL);
    reinterpret_cast<Registers*>(C)->RFR[FIFO] &= ~rfr::full::FULL;
    return res;
  }
  
  template<Address C>
  template<u8 T>
  bool Functions<C>::isTxEmpty()
  {
    static_assert(T < 3, "Mailbox 0..2 is only supported.");
    return ((reinterpret_cast<Registers*>(C)->TSR & (tsr::tme0::MASK << T)) == (tsr::tme0::EMPTY << T));
  }
  
  template<Address C>
  template<u8 T>
  void Functions<C>::Send(msg* m)
  {
	enum {
		TXRQ = 1,
		RTR = 2,
		IDE = 4
	};
    static_assert(T < 3, "Mailbox 0..2 is only supported.");
    reinterpret_cast<Registers*>(C)->TX[T].DTR =
            (m->len & 0x0f) |
            // TODO: TGT flag
            (m->ts << 16);
    reinterpret_cast<Registers*>(C)->TX[T].DLR = m->u[0];
    reinterpret_cast<Registers*>(C)->TX[T].DHR = m->u[1];
    reinterpret_cast<Registers*>(C)->TX[T].IR =
            (m->h.ide ? ((m->h.addr << 3) | IDE) : (m->h.addr << 21)) |
            (m->h.rtr ? RTR : 0) | TXRQ;
  }

  template<Address C>
  u8 Functions<C>::Send(msg* m, bool force) // returns mailbox id
  {
    u8 res = 0xff;
    do
    {
      u8 t = (reinterpret_cast<Registers*>(C)->TSR & tsr::code::MASK) >> tsr::code::POSITION;
      switch(t)
      {
        case 0:
        {
          if(!force && !isTxEmpty<0>()) break;
          Send<0>(m);
          res = t;
          break;
        }
        case 1:
        {
          if(!force && !isTxEmpty<1>()) break;
          Send<1>(m);
          res = t;
          break;
        }
        case 2:
        {
          if(!force && !isTxEmpty<2>()) break;
          Send<2>(m);
          res = t;
          break;
        }
        default:
          break;
      }
      
    } while(0);
    return res;
  }
  
  template<Address C>
  mailbox::Complete Functions<C>::getTxCompleteMbx()
  {
	return (mailbox::Complete)(reinterpret_cast<Registers*>(C)->TSR &
			(tsr::rqcp0::MASK | tsr::rqcp0::MASK | tsr::rqcp0::MASK));
  }
  
  template<Address C>
  template<u8 T>
  void Functions<C>::cleanTxComplete()
  {
    static_assert(T < 3, "Mailbox 0..2 is only supported.");
    switch(T)
    {
      case 0:
        reinterpret_cast<Registers*>(C)->TSR |= tsr::rqcp0::CLEAR;
    	break;
      case 1:
    	reinterpret_cast<Registers*>(C)->TSR |= tsr::rqcp0::CLEAR;
    	break;
      case 2:
    	reinterpret_cast<Registers*>(C)->TSR |= tsr::rqcp0::CLEAR;
    	break;
    }
  }

  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::txEmptyInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::tmeie::POSITION>() = E;
  }

  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::rx0MsgInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::fmpie0::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::rx0FullInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::ffie0::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::rx0OverrunInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::fovie0::POSITION>() = E;
  }

  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::rx1MsgInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::fmpie1::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::rx1FullInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::ffie1::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::rx1OverrunInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::fovie1::POSITION>() = E;
  }

  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::errorInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::errie::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::errorWarningInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::ewgie::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::errorPassiveInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::epvie::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::busOffInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::bofie::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::lastErrorCodeInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::lecie::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::wakeupInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::wkuie::POSITION>() = E;
  }
  
  template<Address C>  
  template<interrupt::State E>
  void Functions<C>::sleepInterrupt()
  {
    *(bool volatile*)bitband::peripheral<C + ier::OFFSET, ier::slkie::POSITION>() = E;
  }
  
  template<Address C>
  void Functions<C>::unmaskInterrupts()
  {
	switch(C)
	{
#if defined CONNECTIVITY_LINE || not defined STM32F1XX
      case CAN1:
          NVIC::enableIrq<nvic::irqn::CAN1_TX>();
          NVIC::enableIrq<nvic::irqn::CAN1_RX0>();
          NVIC::enableIrq<nvic::irqn::CAN1_RX1>();
          NVIC::enableIrq<nvic::irqn::CAN1_SCE>();
        break;
      case CAN2:
          NVIC::enableIrq<nvic::irqn::CAN2_TX>();
          NVIC::enableIrq<nvic::irqn::CAN2_RX0>();
          NVIC::enableIrq<nvic::irqn::CAN2_RX1>();
          NVIC::enableIrq<nvic::irqn::CAN2_SCE>();
        break;
#else // !CONNECTIVTY_LINE || STM32F1XX
      case CAN:
          NVIC::enableIrq<nvic::irqn::USB_HP_CAN_TX>();
          NVIC::enableIrq<nvic::irqn::USB_LP_CAN_RX0>();
          NVIC::enableIrq<nvic::irqn::CAN1_RX1>();
          NVIC::enableIrq<nvic::irqn::CAN1_SCE>();
        break;
#endif // !CONNECTIVITY_LINE
	}
  }
  
  template<Address C>
  void Functions<C>::maskInterrupts()
  {
	switch(C)
	{
#if defined CONNECTIVITY_LINE || not defined STM32F1XX
	  case CAN1:
		  NVIC::disableIrq<nvic::irqn::CAN1_TX>();
		  NVIC::disableIrq<nvic::irqn::CAN1_RX0>();
		  NVIC::disableIrq<nvic::irqn::CAN1_RX1>();
		  NVIC::disableIrq<nvic::irqn::CAN1_SCE>();
		break;
	  case CAN2:
		  NVIC::disableIrq<nvic::irqn::CAN2_TX>();
		  NVIC::disableIrq<nvic::irqn::CAN2_RX0>();
		  NVIC::disableIrq<nvic::irqn::CAN2_RX1>();
		  NVIC::disableIrq<nvic::irqn::CAN2_SCE>();
		break;
#else // !CONNECTIVTY_LINE || STM32F1XX
	  case CAN:
		  NVIC::disableIrq<nvic::irqn::USB_HP_CAN_TX>();
		  NVIC::disableIrq<nvic::irqn::USB_LP_CAN_RX0>();
		  NVIC::disableIrq<nvic::irqn::CAN1_RX1>();
		  NVIC::disableIrq<nvic::irqn::CAN1_SCE>();
		break;
#endif // !CONNECTIVITY_LINE
	}
  }


}  // namespace can
