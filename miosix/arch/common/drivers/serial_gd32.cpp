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

#include <mutex>
#include "serial_gd32.h"
#include "kernel/scheduler/scheduler.h"
#include "kernel/error.h"
#include "interfaces/gpio.h"

using namespace std;

static const int numPorts=3; //Supporting only USART0, USART1 and USART2

// TX and RX pins
typedef miosix::Gpio<GPIOA,  9>  u0tx;
typedef miosix::Gpio<GPIOA, 10> u0rx;

typedef miosix::Gpio<GPIOA, 2>  u1tx;
typedef miosix::Gpio<GPIOA, 3>  u1rx;

typedef miosix::Gpio<GPIOB, 10> u2tx;
typedef miosix::Gpio<GPIOB, 11> u2rx;

// Pointer to serial port classes to let interrupts access the classes
static miosix::GD32Serial *ports[numPorts]={0};

/**
 * \internal interrupt routine for usart0 actual implementation
 */
void __attribute__((noinline)) usart0irqImpl()
{
   if(ports[0]) ports[0]->IRQhandleInterrupt();
}

/**
 * \internal interrupt routine for usart0
 */
void __attribute__((naked)) USART0_IRQHandler()
{
    saveContext();
    asm volatile("j _Z13usart0irqImplv");
    restoreContext();
}

/**
 * \internal interrupt routine for usart1 actual implementation
 */
void __attribute__((noinline)) usart1irqImpl()
{
   if(ports[1]) ports[1]->IRQhandleInterrupt();
}

/**
 * \internal interrupt routine for usart1
 */
void __attribute__((naked)) USART1_IRQHandler()
{
    saveContext();
    asm volatile("j _Z13usart1irqImplv");
    restoreContext();
}

/**
 * \internal interrupt routine for usart2 actual implementation
 */
void __attribute__((noinline)) usart2irqImpl()
{
   if(ports[2]) ports[2]->IRQhandleInterrupt();
}

/**
 * \internal interrupt routine for usart2
 */
void __attribute__((naked)) USART2_IRQHandler()
{
    saveContext();
    asm volatile("j _Z13usart2irqImplv");
    restoreContext();
}

namespace miosix {

//
// class GD32Serial
//
GD32Serial::GD32Serial(int id, int baudrate)
	: Device(Device::TTY), portId(id)
{
    InterruptDisableLock dLock;

    ports[id]=this;

	uint32_t clock = SystemCoreClock; 
	// set the pins, enable the USART clock, enable the correct interrupt and select the correct port
	switch(id) {
		case 0:
			u0tx::mode(Mode::ALTERNATE);
			u0rx::mode(Mode::INPUT);
    		RCU_APB2EN |= RCU_APB2EN_USART0EN;
			miosix_private::doEnableInterrupt(USART0_IRQn);
			port = USART0;
			break;
		case 1:
			clock /= 2; //TODO: the bus might be further prescaled
			u1tx::mode(Mode::ALTERNATE);
			u1rx::mode(Mode::INPUT);
    		RCU_APB1EN |= RCU_APB1EN_USART1EN;
			miosix_private::doEnableInterrupt(USART1_IRQn);
			port = USART1;
			break;
		case 2:
			clock /= 2; //TODO: the bus might be further prescaled
			u2tx::mode(Mode::ALTERNATE);
			u2rx::mode(Mode::INPUT);
    		RCU_APB1EN |= RCU_APB1EN_USART2EN;
			miosix_private::doEnableInterrupt(USART2_IRQn);
			port = USART2;
			break;
		default:
			errorHandler(UNEXPECTED);
	}

	// set the baud rate
    uint32_t udiv = (clock + baudrate/2U) / baudrate;
    USART_BAUD(port) = udiv;
	USART_CTL0(port) = USART_CTL0_UEN
					   | USART_CTL0_RBNEIE
					   | USART_CTL0_TEN
					   | USART_CTL0_REN;
}

void GD32Serial::IRQhandleInterrupt()
{
    if((USART_STAT(port) & USART_STAT_RBNE) != 0)
    {
		char c = (char)GET_BITS(USART_DATA(port), 0U, 8U);
    	if((USART_STAT(port) & USART_STAT_FERR) == 0)
        {
            bool hppw;
            rxQueue.IRQput(c, hppw);
            if(hppw) Scheduler::IRQfindNextThread();
		}
	}
}

ssize_t GD32Serial::writeBlock(uint32_t *buffer, size_t size, off_t where) {
   std::unique_lock<Mutex> lock(txMutex);

    for(size_t i = 0; i < size; i++)
    {
		// wait until the tx is empty
        while((USART_STAT(port) & USART_STAT_TBE) == 0) ;
        USART_DATA(port) = USART_DATA_DATA & *buffer++;
    }
    return size;
}

ssize_t GD32Serial::readBlock(uint16_t *buffer, size_t size, off_t where) {
	if (size < 1) {
		return 0;
	}

	std::unique_lock<Mutex> lock(rxMutex);
	char *buf = reinterpret_cast<char*>(buffer);

	// get the char put in the queue by the interrupt
	char c;
	rxQueue.get(c);
	buf[0] = c;
	return 1;
}

void GD32Serial::IRQwrite(const char *str)
{
    // We can reach here also with only kernel paused, so make sure
    // interrupts are disabled. This is important for the DMA case
 /*   bool interrupts=areInterruptsEnabled();
    if(interrupts) fastDisableInterrupts();
    #ifdef SERIAL_DMA
    if(dmaTx)
    {
        #if defined(_ARCH_CORTEXM3_STM32F1) || defined(_ARCH_CORTEXM4_STM32F3) \
         || defined(_ARCH_CORTEXM4_STM32L4)
        //If no DMA transfer is in progress bit EN is zero. Otherwise wait until
        //DMA xfer ends, by waiting for the TC (or TE) interrupt flag
        static const unsigned int irqMask[]=
        {
            (DMA_ISR_TCIF4 | DMA_ISR_TEIF4),
            (DMA_ISR_TCIF7 | DMA_ISR_TEIF7),
            (DMA_ISR_TCIF2 | DMA_ISR_TEIF2)
        };
        #if defined(_ARCH_CORTEXM4_STM32F3) || defined(_ARCH_CORTEXM4_STM32L4)
        // Workaround for ST messing up with flag definitions...
        constexpr unsigned int DMA_CCR4_EN = DMA_CCR_EN;
        #endif
        while((dmaTx->CCR & DMA_CCR4_EN) && !(DMA1->ISR & irqMask[getId()-1])) ;
        #else //_ARCH_CORTEXM3_STM32F1
        //Wait until DMA xfer ends. EN bit is cleared by hardware on transfer end
        while(dmaTx->CR & DMA_SxCR_EN) ;
        #endif //_ARCH_CORTEXM3_STM32F1
    }
    #endif //SERIAL_DMA
    */
    while(*str)
    {
        while((USART_STAT(port) & USART_STAT_TBE)==0) ;
        USART_DATA(port)=*str++;
    }
    /*waitSerialTxFifoEmpty();
    if(interrupts) fastEnableInterrupts();*/
}

}
