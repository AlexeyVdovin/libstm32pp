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

#include "bitband.hpp"

namespace i2c {
  /**
   * @brief Configures the I2C in standard mode.
   * @brief Overrides the old configuration.
   */
  template<Address I>
  void Standard<I>::configureI2c(
      i2c::cr1::pe::States PE,
      i2c::cr1::engc::States ENGC,
      i2c::cr1::nostretch::States NOSTRETCH,
      i2c::cr1::ack::States ACK,
      i2c::cr2::iterren::States ITERREN,
      i2c::cr2::itevten::States ITVEN,
      i2c::cr2::itbufen::States ITBUFEN,
      i2c::cr2::dmaen::States DMAEN,
      i2c::cr2::last::States LAST)
  {
    static_assert((FREQUENCY >= 2000000) || (FREQUENCY <= 42000000),
		"I2C Frequency must be between 2 MHz and 42 MHz (inclusive)");

    reinterpret_cast<Registers*>(I)->CR1 =
		PE + ENGC + NOSTRETCH + ACK;

    reinterpret_cast<Registers*>(I)->CR2 =
		ITERREN + ITVEN + ITBUFEN + DMAEN + LAST +
			((FREQUENCY / 1000000) << cr2::freq::POSITION);
    reinterpret_cast<Registers*>(I)->OAR1 = oar1::res1::DEFAULT;
    reinterpret_cast<Registers*>(I)->OAR2 = oar2::endual::DUAL_MODE_DISABLED;
  }

  /**
   * @brief Configures the I2C in SmBus mode.
   * @brief Overrides the old configuration.
   */
  template<Address I>
  void Standard<I>::configureSmBus(
      i2c::cr1::pe::States PE,
      i2c::cr1::smbtype::States SMBTYPE,
      i2c::cr1::enarp::States ENARP,
      i2c::cr1::enpec::States ENPEC,
      i2c::cr1::engc::States ENGC,
      i2c::cr1::nostretch::States NOSTRETCH,
      i2c::cr1::ack::States ACK,
      i2c::cr2::iterren::States ITERREN,
      i2c::cr2::itevten::States ITVEN,
      i2c::cr2::itbufen::States ITBUFEN,
      i2c::cr2::dmaen::States DMAEN,
      i2c::cr2::last::States LAST)
  {
	static_assert((FREQUENCY >= 2000000) || (FREQUENCY <= 42000000),
		"I2C Frequency must be between 2 MHz and 42 MHz (inclusive)");

	reinterpret_cast<Registers*>(I)->CR1 =
		PE + cr1::smbus::SMBUS_MODE + SMBTYPE + ENARP + ENPEC + ENGC + NOSTRETCH + ACK;

	reinterpret_cast<Registers*>(I)->CR2 =
		ITERREN + ITVEN + ITBUFEN + DMAEN + LAST +
			((FREQUENCY / 1000000) << cr2::freq::POSITION);
    reinterpret_cast<Registers*>(I)->OAR1 = oar1::res1::DEFAULT;
    reinterpret_cast<Registers*>(I)->OAR2 = oar2::endual::DUAL_MODE_DISABLED;
  }


  /**
   * @brief Configures the I2C clock.
   * @note  In standard mode, CCR >= 4 and in fast mode, CCR  >= 1
   *                           APB1
   * FREQ = -----------------------------------------
   *        CCR *(NORMAL:2, FAST:2 + 1, FAST: 16 + 9)
   */
  template<Address I>
  template<
      ccr::f_s::States F_S,
      ccr::duty::States DUTY,
      u32 FREQUENCY_HZ
  >
  void Standard<I>::configureClock()
  {
    enum {
      CCR = FREQUENCY
          / (FREQUENCY_HZ
              *
              (F_S == ccr::f_s::STANDARD_MODE ?
                  2 :
                  (DUTY
                      == ccr::duty::T_LOW_16_T_HIGH_9 ?
                                                        25 :
                                                        3)))
    };

    static_assert(CCR < 2048,
        "This frequency can't be archived with this configuration.");
    static_assert((CCR >= 1) ||
        (F_S == ccr::f_s::STANDARD_MODE),
        "This frequency can't be archived with this configuration.");
    static_assert((CCR >= 4) ||
        (F_S == ccr::f_s::FAST_MODE),
        "This frequency can't be archived with this configuration.");

    reinterpret_cast<Registers*>(I)->CCR = F_S + DUTY + CCR;
  }

  /**
   * @brief Enables the I2C's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address I>
  void Standard<I>::enableClock()
  {
    switch (I) {
      case I2C1:
        RCC::enableClocks<rcc::apb1enr::I2C1>();
        break;
      case I2C2:
        RCC::enableClocks<rcc::apb1enr::I2C2>();
        break;
#ifndef STM32F1XX
      case I2C3:
        RCC::enableClocks<rcc::apb1enr::I2C3>();
        break;
#endif // !STM32F1XX
    }
  }

  /**
   * @brief Disables the I2C's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address I>
  void Standard<I>::disableClock()
  {
    switch (I) {
      case I2C1:
        RCC::disableClocks<rcc::apb1enr::I2C1>();
        break;
      case I2C2:
        RCC::disableClocks<rcc::apb1enr::I2C2>();
        break;
#ifndef STM32F1XX
      case I2C3:
        RCC::disableClocks<rcc::apb1enr::I2C3>();
        break;
#endif // !STM32F1XX
    }
  }

  /**
   * @brief Turns on the I2C peripheral.
   */
  template<Address I>
  void Standard<I>::enablePeripheral()
  {
    *(u32 volatile*) (bitband::peripheral<
        I + cr1::OFFSET,
        cr1::pe::POSITION
    >()) = 1;
  }

  /**
   * @brief Turns off the I2C peripheral.
   */
  template<Address I>
  void Standard<I>::disablePeripheral()
  {
    *(u32 volatile*) (bitband::peripheral<
        I + cr1::OFFSET,
        cr1::pe::POSITION
    >()) = 0;
  }

  /**
   * @brief Set 7bit Slave address1.
   */
  template<Address I>
  void Standard<I>::setSlave7BitAddr1(u8 addr)
  {
    reinterpret_cast<Registers*>(I)->OAR1 = oar1::mode::MODE_7BIT + oar1::res1::DEFAULT + (addr<<1);
  }

  /**
   * @brief Set 10bit Slave address1.
   */
  template<Address I>
  void Standard<I>::setSlave10BitAddr1(u8 addr)
  {
    reinterpret_cast<Registers*>(I)->OAR1 = oar1::mode::MODE_10BIT + oar1::res1::DEFAULT + addr;
  }

  /**
   * @brief Set 7bit Slave address2 and Enable dual address mode.
   */
  template<Address I>
  void Standard<I>::setSlave7BitAddr2(u8 addr)
  {
    reinterpret_cast<Registers*>(I)->OAR2 = (addr<<1) + oar2::endual::DUAL_MODE_ENABLED;
  }

  /**
   * @brief Sends Start condition.
   */
  template<Address I>
  void Standard<I>::sendStart()
  {
    *(u32 volatile*) (bitband::peripheral<
        I + cr1::OFFSET,
        cr1::start::POSITION
    >()) = 1;
  }

  /**
   * @brief Sends Stop condition.
   */
  template<Address I>
  void Standard<I>::sendStop()
  {
    *(u32 volatile*) (bitband::peripheral<
        I + cr1::OFFSET,
        cr1::stop::POSITION
    >()) = 1;
  }

  /**
   * @brief Sends data.
   */
  template<Address I>
  void Standard<I>::sendData(u8 const data)
  {
    reinterpret_cast<Registers*>(I)->DR = data;
  }

  /**
   * @brief Reads data.
   */
  template<Address I>
  u8 Standard<I>::getData()
  {
    return reinterpret_cast<Registers*>(I)->DR;
  }

  /**
   * @brief Sends the slave's address.
   */
  template<Address I>
  void Standard<I>::sendAddress(
      u8 const add,
      operation::E op)
  {
    reinterpret_cast<Registers*>(I)->DR = (add << 1) + op;
  }

  /**
   * @brief Send acknowledge after byte reception.
   */
  template<Address I>
  void Standard<I>::enableACK()
  {
    *(u32 volatile*) (bitband::peripheral<
        I + cr1::OFFSET,
        cr1::ack::POSITION
    >()) = 1;
  }

  /**
   * @brief Don't send acknowledge after byte reception.
   */
  template<Address I>
  void Standard<I>::disableACK()
  {
    *(u32 volatile*) (bitband::peripheral<
        I + cr1::OFFSET,
        cr1::ack::POSITION
    >()) = 0;
  }

  template<Address I>
  void Standard<I>::clearNAK()
  {
    *(u32 volatile*) (bitband::peripheral<
        I + sr1::OFFSET,
        sr1::af::POSITION
    >()) = 0;
  }

  template<Address I>
  void Standard<I>::reset()
  {
    *(u32 volatile*) (bitband::peripheral<
        I + cr1::OFFSET,
        cr1::swrst::POSITION
    >()) = 1;
  }

  /**
   * @brief returns true if a start condition has been sent.
   */
  template<Address I>
  bool Standard<I>::hasSentStart()
  {
    return *(bool volatile*) bitband::peripheral<
        I + sr1::OFFSET,
        sr1::sb::POSITION
    >();
  }

  /**
   * @brief returns true when a stop condition has been sent.
   */
  template<Address I>
  bool Standard<I>::hasSentStop()
  {
    return *(bool volatile*) bitband::peripheral<
        I + sr1::OFFSET,
        sr1::stopf::POSITION
    >();
  }

  /**
   * @brief returns true when the slave address has been transmitted.
   */
  template<Address I>
  bool Standard<I>::hasAddressTransmitted()
  {
    return *(bool volatile*) bitband::peripheral<
        I + sr1::OFFSET,
        sr1::addr::POSITION
    >();
  }

  /**
   * @brief returns true if data has been received.
   */
  template<Address I>
  bool Standard<I>::hasReceivedData()
  {
    return *(bool volatile*) bitband::peripheral<
        I + sr1::OFFSET,
        sr1::rxne::POSITION
    >();
  }

  /**
   * @brief 0 - Can't send data, 1 - Can send data.
   */
  template<Address I>
  bool Standard<I>::canSendData()
  {
    return *(bool volatile*) bitband::peripheral<
        I + sr1::OFFSET,
        sr1::txe::POSITION
    >();
  }

  /**
   * @brief 0 - Transfer hasn't finished, 1 - Transfer has finished.
   */
  template<Address I>
  bool Standard<I>::hasTranferFinished()
  {
    return *(bool volatile*) bitband::peripheral<
        I + sr1::OFFSET,
        sr1::btf::POSITION
    >();
  }

  /**
   * @brief  0 - No communication on the bus,
   *          1 - Communication ongoing on the bus.
   */
  template<Address I>
  bool Standard<I>::isTheBusBusy()
  {
    return *(bool volatile*) (bitband::peripheral<
        I + sr2::OFFSET,
        sr2::busy::POSITION
    >());
  }

  template<Address I>
  bool Standard<I>::isSlaveTransmitting()
  {
    return *(bool volatile*) (bitband::peripheral<
        I + sr2::OFFSET,
        sr2::tra::POSITION
    >());
  }

  template<Address I>
  bool Standard<I>::isAddrMatched()
  {
    return *(bool volatile*) bitband::peripheral<
        I + sr1::OFFSET,
        sr1::addr::POSITION
    >();
  }

  template<Address I>
  bool Standard<I>::isStopReceived()
  {
    return *(bool volatile*) bitband::peripheral<
        I + sr1::OFFSET,
        sr1::stopf::POSITION
    >();
  }
  
  template<Address I>
  bool Standard<I>::isNakReceived()
  {
	return *(bool volatile*) bitband::peripheral<
		I + sr1::OFFSET,
		sr1::af::POSITION
	>();
  }

  /**
   * @brief Writes a value to a slave device register.
   */
  template<Address I>
  void Standard<I>::writeSlaveRegister(
      u8 const slaveAddress,
      u8 const registerAddress,
      u8 const value)
  {
    sendStart();

    while (!hasSentStart()) {
    };

    Standard<I>::sendAddress(slaveAddress, operation::WRITE);

    while (!hasAddressTransmitted()) {
    };

    reinterpret_cast<Registers*>(I)->SR2;

    sendData(registerAddress);

    while (!canSendData()) {
    };

    sendData(value);

    while (!hasTranferFinished()) {
    };

    sendStop();

    while (isTheBusBusy()) {
    };
  }

  /**
   * @brief Reads a value from a slave device register.
   */
  template<Address I>
  u8 Standard<I>::readSlaveRegister(
      u8 const slaveAddress,
      u8 const registerAddress)
  {
    sendStart();

    while (!hasSentStart()) {
    };

    sendAddress(slaveAddress, operation::WRITE);

    while (!hasAddressTransmitted()) {
    };

    reinterpret_cast<Registers*>(I)->SR2;

    sendData(registerAddress);

    while (!canSendData()) {
    };

    sendStart();

    while (!hasSentStart()) {
    };

    sendAddress(slaveAddress, operation::READ);

    disableACK();

    while (!hasAddressTransmitted()) {
    };

    reinterpret_cast<Registers*>(I)->SR2;

    while (!hasReceivedData()) {
    };

    sendStop();

    while (isTheBusBusy()) {
    };

    return getData();
  }

  template<Address I>
  void Standard<I>::unmaskInterrupts()
  {
    switch(I)
    {
      case I2C1:
        NVIC::enableIrq<nvic::irqn::I2C1_EV>();
        NVIC::enableIrq<nvic::irqn::I2C1_ER>();
        break;
      case I2C2:
        NVIC::enableIrq<nvic::irqn::I2C2_EV>();
        NVIC::enableIrq<nvic::irqn::I2C2_ER>();
        break;
    }
  }

  template<Address I>
  void Standard<I>::maskInterrupts()
  {
    switch(I)
    {
      case I2C1:
        NVIC::disableIrq<nvic::irqn::I2C1_EV>();
        NVIC::disableIrq<nvic::irqn::I2C1_ER>();
        break;
      case I2C2:
        NVIC::disableIrq<nvic::irqn::I2C2_EV>();
        NVIC::disableIrq<nvic::irqn::I2C2_ER>();
        break;
    }
  }

}  // namespace i2c
