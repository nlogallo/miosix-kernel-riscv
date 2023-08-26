/***************************************************************************
 *   Copyright (C) 2016 by Fabiano Riccardi, Sasan                         *
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

#include "interfaces/os_timer.h"
#include "kernel/timeconversion.h"

using namespace miosix;

// TODO: now it's all a stub

namespace miosix {

// static HRTB *b=nullptr;
// static TimeConversion tc;
// static VHT *vht=nullptr;
// static VirtualClock *vt=nullptr;

long long getTime() noexcept
{
    // return tc.tick2ns(vt->uncorrected2corrected(vht->uncorrected2corrected(b->addBasicCorrection(b->getCurrentTick()))));
	return 0;
}

long long IRQgetTime() noexcept
{
    // return tc.tick2ns(vt->uncorrected2corrected(vht->uncorrected2corrected(b->addBasicCorrection(b->IRQgetCurrentTick()))));
	return 0;
}

namespace internal {

void IRQosTimerInit()
{
    // b=&HRTB::instance();
    // tc=TimeConversion(b->getTimerFrequency());
    // vht=&VHT::instance();
    // vt=&VirtualClock::instance();
}

void IRQosTimerSetInterrupt(long long ns) noexcept
{
    // b->IRQsetNextInterruptCS(b->removeBasicCorrection(vht->corrected2uncorrected(vt->corrected2uncorrected(tc.ns2tick(ns)))));
}

// long long ContextSwitchTimer::getNextInterrupt() const
// {
//     return tc->tick2ns(vt->uncorrected2corrected(vht->uncorrected2corrected(pImpl->b.addBasicCorrection(pImpl->b.IRQgetSetTimeCS()))));
// }

// void IRQosTimerSetTime(long long ns) noexcept
// {
//     //TODO
// }
// 
//
// IRQinitTimer(){
// 		//Enable timer from RCU
// 		rcu_periph_clock enable(RCU_TIMER1); //Enable TIMER1 from RCU
// 		rcu_periph_reset_enable(RCU_TIMER1RST); //Reset timer
// 		rcu_periph_reset_disable(RCU_TIMER1RST);
//
// 		//Setup TIMER1 base configuration
// 		//Values from driver function timer_struct_para_init()
// 		TIMER_PSC(TIMER1) = 0; //no prescaler
//      TIMER_CTL0(TIMER1) &= (~(uint32_t)(TIMER_CTL0_DIR | TIMER_CTL0_CAM));
//      TIMER_CTL0(TIMER1) |= (uint32_t)(TIMER_COUNTER_EDGE & ALIGNEDMODE_MASK); //Edge aligned
//      TIMER_CTL0(TIMER1) |= (uint32_t)(TIMER_COUNTER_UP & COUNTERDIRECTION_MASK); //Up-counter mode
// 		TIMER_CAR(TIMER1) = 65536U; //Autoreload value set to max(?) 
//      TIMER_CTL0(TIMER1) &= (~(uint32_t)TIMER_CTL0_CKDIV);
//      TIMER_CTL0(TIMER1) |= (uint32_t)(TIMER_CKDIV_DIV1 & CLOCKDIVISION_MASK); //Reset Ckdiv bit
//      TIMER_SWEVG(TIMER1) |= (uint32_t)TIMER_SWEVG_UPG //Generate update event
//
//		TIMER_CTL0(TIMER1) |= (uint32_t)TIMER_CTL0_CEN; //Enable counter, might be set automatically by hardware
//
//		//TODO: Some stuff is missing, like setting of output channels and so on
//}
//
//
//
//
// 		

unsigned int osTimerGetFrequency()
{
    // return b->getTimerFrequency();
	return 0;
}

} //namespace internal

} //namespace miosix
