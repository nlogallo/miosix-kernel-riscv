/***************************************************************************
 *   Copyright (C) 2010, 2011, 2012, 2013, 2014 by Terraneo Federico       *
 *   Copyright (C) 2019 by Cremonese Filippo, Picca Niccolò                *
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

#ifndef INTERRUPTS_H
#define	INTERRUPTS_H

/**
  \brief   Enable IRQ Interrupts
 */
__attribute__( ( always_inline ) ) static inline void __enable_irq(void)
{
	unsigned long tmp1, tmp2;
	asm volatile ("csrrs %0, mstatus, 0x8" : "=r"(tmp1));
	asm volatile ("csrrs %0, mie, 0xb" : "=r"(tmp2));
}


/**
  \brief   Disable IRQ Interrupts
 */
__attribute__( ( always_inline ) ) static inline void __disable_irq(void)
{
	unsigned long tmp;
	asm volatile ("csrrc %0, mstatus, 0x8" : "=r"(tmp));
}


/**
  \brief   Check if IRQS are enabled
 */
__attribute__( ( always_inline ) ) static inline bool __check_are_irqs_enabled(void)
{
	unsigned long tmp;
	asm volatile ("csrr %0, mstatus": "=r"(tmp));
	return tmp & (1 << 8);
}


/**
  \brief   Enable a single IRQ
 */
__attribute__( ( always_inline ) ) static inline void __enable_one_irq(int interrupt_number)
{
	unsigned long tmp;
	asm volatile ("csrrs %0, mie, %1" : "=r"(tmp) : "r"(interrupt_number));
}


/**
  \brief   Disable a single IRQ
 */
__attribute__( ( always_inline ) ) static inline void __disable_one_irq(int interrupt_number)
{
	unsigned long tmp;
	asm volatile ("csrrc %0, mie, %1" : "=r"(tmp) : "r"(interrupt_number));
}

/**
 * Called when an unexpected interrupt occurs.
 * It is called by stage_1_boot.cpp for all weak interrupts not defined.
 */
void unexpectedInterrupt();

#endif	//INTERRUPTS_H
