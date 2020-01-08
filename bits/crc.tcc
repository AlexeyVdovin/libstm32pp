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

namespace crc {
  void Functions::enableClock()
  {
#ifndef STM32F1XX
    RCC::enableClocks<rcc::ahb1enr::CRC>();
#else // !STM32F1XX
    RCC::enableClocks<rcc::ahbenr::CRC>();
#endif // !STM32F1XX
  }

  void Functions::disableClock()
  {
#ifndef STM32F1XX
    RCC::disableClocks<rcc::ahb1enr::CRC>();
#else // !STM32F1XX
    RCC::disableClocks<rcc::ahbenr::CRC>();
#endif // !STM32F1XX
  }
  
  void Functions::reset()
  {
	  CRC_REGS->CR = 1;
  }
  
  void Functions::calc(u32 data)
  {
	  CRC_REGS->DR = data;
  }
  
  u32 Functions::getCrc()
  {
	  return CRC_REGS->DR;
  }

}  // namespace crc
