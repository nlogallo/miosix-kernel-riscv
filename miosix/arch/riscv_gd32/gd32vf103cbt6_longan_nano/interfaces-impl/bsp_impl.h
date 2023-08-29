/***************************************************************************
 *   Copyright (C) 2018 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/ 

/***********************************************************************
* bsp_impl.h Part of the Miosix Embedded OS.
* Board support package, this file initializes hardware.
************************************************************************/

#ifndef BSP_IMPL_H
#define BSP_IMPL_H

#include "config/miosix_settings.h"
#include "interfaces/gpio.h"

namespace miosix {

/**
\addtogroup Hardware
\{
*/

// Input button on the board (BOOT0)
typedef miosix::Gpio<GPIOA, 8> button;

/**
 * \internal
 * used by the ledOn() and ledOff() implementation
 */
typedef Gpio<GPIOC, 13> red_led;
typedef Gpio<GPIOA,  1> green_led;
typedef Gpio<GPIOA,  2> blue_led;

inline void redLedOn()
{
    //Led is connected to VCC, so to turn it on the GPIO has to be low
    red_led::low();
}

inline void redLedOff()
{
    red_led::high();
}

inline void greenLedOn()
{
    //Led is connected to VCC, so to turn it on the GPIO has to be low
    green_led::low();
}

inline void greenLedOff()
{
    green_led::high();
}

inline void blueLedOn()
{
    //Led is connected to VCC, so to turn it on the GPIO has to be low
    blue_led::low();
}

inline void blueLedOff()
{
    blue_led::high();
}

inline void ledOn() {
	// select green as the "default" LED
	greenLedOn();
}

inline void ledOff() {
	greenLedOff();
}

/**
 * Polls the SD card sense GPIO.
 * 
 * This board has no SD card whatsoever, but a card can be connected to the
 * following GPIOs:
 * TODO: never tested
 * 
 * \return true. As there's no SD card sense switch, let's pretend that
 * the card is present.
 */
inline bool sdCardSense() { return true; }

/**
\}
*/

} //namespace miosix

#endif //BSP_IMPL_H
