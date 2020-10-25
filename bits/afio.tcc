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

namespace afio {
  void Functions::enableClock()
  {
    RCC::enableClocks<rcc::apb2enr::AFIO>();
  }

  void Functions::disableClock()
  {
    RCC::disableClocks<rcc::apb2enr::AFIO>();
  }

  template <exticr::States PORT, u8 PIN>
  void Functions::configureExti()
  {
    reinterpret_cast<Registers*>(ADDRESS)->EXTICR[PIN / 4] &=
        exticr::MASK << (exticr::POSITION * (PIN % 4));

    reinterpret_cast<Registers*>(ADDRESS)->EXTICR[PIN / 4] |=
            PORT << (exticr::POSITION * (PIN % 4));
  }
  
  template <mapr::usart1::States MODE>
  void Functions::configureUsart1()
  {
    reinterpret_cast<Registers*>(ADDRESS)->MAPR = 
         (reinterpret_cast<Registers*>(ADDRESS)->MAPR & ~mapr::usart1::MASK) | (MODE << mapr::usart1::POSITION);
  }
  
  template <mapr::can::States MODE>
  void Functions::configureCan()
  {
    reinterpret_cast<Registers*>(ADDRESS)->MAPR = 
      (reinterpret_cast<Registers*>(ADDRESS)->MAPR & ~mapr::can::MASK) | (MODE << mapr::can::POSITION);
  }

  template <mapr::swj::States MODE>
  void Functions::configureSwj()
  {
    reinterpret_cast<Registers*>(ADDRESS)->MAPR = 
      (reinterpret_cast<Registers*>(ADDRESS)->MAPR & ~mapr::swj::MASK) | (MODE << mapr::swj::POSITION);
  }
} // namespace afio
