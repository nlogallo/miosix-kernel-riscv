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

#ifndef PORTABILITY_IMPL_H
#define PORTABILITY_IMPL_H

#include "interfaces/arch_registers.h"
#include "interfaces/portability.h"
#include "config/miosix_settings.h"
#include "core/interrupts.h"

/**
 * \addtogroup Drivers
 * \{
 */

/*
 * This pointer is used by the kernel, and should not be used by end users.
 * this is a pointer to a location where to store the thread's registers during
 * context switch. It requires C linkage to be used inside asm statement.
 * Registers are saved in the following order:
 *
 * *ctxsave+0   --> x1
 * *ctxsave+4   --> x2
 * *ctxsave+8   --> x4
 * *ctxsave+12  --> x5
 * ...
 * *ctxsave+120 --> x31
 * *ctxsave+120 --> q0
 * Register x0 (zero register) is not saved, since is constant
 * Register gp (x3) is not saved, since its value must be constant
 * to allow for linker relaxation)
 */
extern "C" {
extern volatile unsigned int *ctxsave;
}
const int stackPtrOffsetInCtxsave=0; ///< Allows to locate the stack pointer

/**
 * \}
 */

namespace miosix_private {
    
/**
 * \addtogroup Drivers
 * \{
 */

inline void doYield()
{
    asm volatile("ecall");

}

inline void doDisableInterrupts()
{
    __disable_irq();
}

inline void doEnableInterrupts()
{
    __enable_irq();
}



inline bool checkAreInterruptsEnabled()
{
	// TODO: find how to check if interrupts are enabled
	return false;
}

#ifdef WITH_PROCESSES

//
// class SyscallParameters
//

inline SyscallParameters::SyscallParameters(unsigned int *context) :
        registers(reinterpret_cast<unsigned int*>(context[0])) {}

inline int SyscallParameters::getSyscallId() const
{
    return registers[3];
}

inline unsigned int SyscallParameters::getFirstParameter() const
{
    return registers[0];
}

inline unsigned int SyscallParameters::getSecondParameter() const
{
    return registers[1];
}

inline unsigned int SyscallParameters::getThirdParameter() const
{
    return registers[2];
}

inline void SyscallParameters::setReturnValue(unsigned int ret)
{
    registers[0]=ret;
}

inline void portableSwitchToUserspace()
{
    asm volatile("movs r3, #1\n\t"
                 "svc  0"
                 :::"r3");
}

#endif //WITH_PROCESSES

/**
 * \}
 */

} //namespace miosix_private

#endif //PORTABILITY_IMPL_H
