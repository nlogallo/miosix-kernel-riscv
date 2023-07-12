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

#include <cstring>
#include <errno.h>
#include <termios.h>
#include "serial_gd32.h"
#include "kernel/sync.h"
#include "kernel/scheduler/scheduler.h"
#include "interfaces/portability.h"
#include "filesystem/ioctl.h"

#include <mutex>
#include "kernel/queue.h"
#include "interfaces/gpio.h"

using namespace std;

namespace miosix {

// Pointer to serial port classes to let interrupts access the classes
static GD32Serial *ports[2]={0};

static Mutex txMutex;
static Mutex rxMutex;
static Queue<char, 64> rxQueue;

//
// class GD32Serial
//
GD32Serial::GD32Serial(int id, int baudrate) : Device(Device::TTY)
{
    InterruptDisableLock dLock;
    
    using u2tx=Gpio<GPIO_BASE,2>;
    using u2rx=Gpio<GPIO_BASE,3>;
    u2tx::mode(Mode::ALTERNATE);
    u2rx::mode(Mode::ALTERNATE);
    u2tx::alternateFunction(7);
    u2rx::alternateFunction(7);
    
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    USART0->BRR=136<<4 | 12;
    
    USART0->CR1 = USART_CR1_UE
                | USART_CR1_RXNEIE
                | USART_CR1_TE
                | USART_CR1_RE;

    // NVIC_SetPriority(USART2_IRQn, 15);
    eclic_irq_enable(USART0_IRQn, 15, 15);
}

ssize_t GD32Serial::writeBlock(const void *buffer, size_t size, off_t where) {
    unique_lock<mutex> l(txMutex);
    
    for(size_t i = 0; i < size; i++)
    {
        while((USART0->SR & USART_SR_TXE)==0)  ;
        USART0->DR=*buffer++;
    }
    return size;
}

void usart2irqIMPL()
{
    unsigned int status=USART->SR;
    if(status & USART_SR_RXNE)
    {
        char c=USART0->DR;
        if((status & USART_SR_FE)==0)
        {
            bool hppw;
            rwQueue.IRQput(c, hppw);
            if(hppw) Scheduler::IRQfindNextThread();
            }
        }
}

ssize_t GD32Serial::readBlock(void *buffer, size_t size, off_t where) {
	if (size < 1) {
		return 0;
	}

	unique_lock<mutex> l(rxMutex);
	char *buf = reinterpret_cast<char*>(buffer);

	char c;
	rxQueue.get(c);
	buf[0]=c;
	return 1;
}

}
