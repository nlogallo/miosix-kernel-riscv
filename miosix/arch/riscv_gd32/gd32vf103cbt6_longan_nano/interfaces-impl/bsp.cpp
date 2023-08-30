/***************************************************************************
 *   Copyright (C) 2008, 2009, 2010, 2011, 2012, 2013, 2014                *
 *   by Terraneo Federico                                                  *
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
* bsp.cpp Part of the Miosix Embedded OS.
* Board support package, this file initializes hardware.
************************************************************************/

#include <sys/ioctl.h>
#include "interfaces/bsp.h"
#include "interfaces/delays.h"
#include "drivers/serial.h"
#include "kernel/logging.h"

namespace miosix {

//
// Initialization
//

void IRQbspInit()
{
    // enable all gpios
    RCU_APB2EN |= RCU_APB2EN_AFEN;
    RCU_APB2EN |= RCU_APB2EN_PAEN;
    RCU_APB2EN |= RCU_APB2EN_PBEN;
    RCU_APB2EN |= RCU_APB2EN_PCEN;
    RCU_APB2EN |= RCU_APB2EN_PDEN;
    RCU_APB2EN |= RCU_APB2EN_PEEN;

    button::mode(Mode::INPUT_PULL_UP);

    red_led::mode(Mode::OUTPUT);
    green_led::mode(Mode::OUTPUT);
    blue_led::mode(Mode::OUTPUT);
    
    redLedOff();
    greenLedOff();
    blueLedOff();

    // check that the LEDs work
    ledOn();
    delayMs(100);
    ledOff();

    // init serial port
    DefaultConsole::instance().IRQset(
			intrusive_ref_ptr<Device>(new GD32Serial(defaultSerial, defaultSerialSpeed)));
    
    // FIXME: the new GD32Serial turns on green and blue led
    greenLedOff();
    blueLedOff();
}

void bspInit2()
{
// FIXME: this code is never reached
}

//
// Shutdown and reboot
//

/**
\internal
Reboots system
*/
void shutdown()
{
    reboot();
}

void reboot()
{
    ioctl(STDOUT_FILENO, IOCTL_SYNC, 0);

    disableInterrupts();
    miosix_private::IRQsystemReboot();
}

} //namespace miosix
