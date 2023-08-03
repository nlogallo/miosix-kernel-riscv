/***************************************************************************
 *   Copyright (C) 2010, 2011, 2012 by Terraneo Federico                   *
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
 //Miosix kernel

#include "interfaces/portability.h"
#include "kernel/kernel.h"
#include "kernel/error.h"
#include "interfaces/bsp.h"
#include "kernel/scheduler/scheduler.h"
#include "core/interrupts.h"
#include "kernel/process.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cassert>

#include "kernel/logging.h"


namespace miosix_private {

void IRQsystemReboot()
{
    IRQerrorLog("rebooting\n");
    doDisableInterrupts();

    asm volatile("j _Z13Reset_Handlerv\n");
}

void initCtxsave(unsigned int *ctxsave, void *(*pc)(void *), unsigned int *sp,
        void *argv)
{

    ctxsave[0]=0;
    ctxsave[1]=(unsigned int)sp; //Initialize the thread's stack pointer
    ctxsave[2]=0;
    ctxsave[3]=0;
    ctxsave[4]=0;
    ctxsave[5]=0;
    ctxsave[6]=0;
    ctxsave[7]=0;
    ctxsave[8]=(unsigned int)pc; //a0 is the first argument to threadLauncher
    ctxsave[9]=(unsigned int)argv; //a1 is the second argument to threadLauncher
    ctxsave[10]=0;
    ctxsave[11]=0;
    ctxsave[12]=0;
    ctxsave[13]=0;
    ctxsave[14]=0;
    ctxsave[15]=0;
    ctxsave[16]=0;
    ctxsave[17]=0;
    ctxsave[18]=0;
    ctxsave[19]=0;
    ctxsave[20]=0;
    ctxsave[21]=0;
    ctxsave[22]=0;
    ctxsave[23]=0;
    ctxsave[24]=0;
    ctxsave[25]=0;
    ctxsave[26]=0;
    ctxsave[27]=0;
    ctxsave[28]=0;
    ctxsave[29]=0;
    ctxsave[30]=(unsigned int)miosix::Thread::threadLauncher; //q0 contains the IRQ return address


}


void IRQportableStartKernel()
{
    //create a temporary space to save current registers. This data is useless
    //since there's no way to stop the scheduler, but we need to save it anyway.
    unsigned int s_ctxsave[miosix::CTXSAVE_SIZE];

    ctxsave=s_ctxsave;//make global ctxsave point to it

    //Note, we can't use enableInterrupts() now since the call is not matched
    //by a call to disableInterrupts()
    __enable_irq();

    miosix::Thread::yield();
    //Never reaches here
}

void sleepCpu()
{
    return;
}

} //namespace miosix_private
