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

#ifndef SERIAL_GD32_H
#define SERIAL_GD32_H

#include "filesystem/console/console_device.h"
#include "kernel/sync.h"
#include "kernel/queue.h"
#include "interfaces/delays.h"

namespace miosix {

/**
 * Serial port class for GD32 microcontrollers.
 * 
 * Classes of this type are reference counted, must be allocated on the heap
 * and managed through intrusive_ref_ptr<FileBase>
 */
class GD32Serial : public Device
{
public:
    /**
     * Constructor, initializes the serial port.
     * Calls errorHandler(UNEXPECTED) if id is not in the correct range, or when
     * attempting to construct multiple objects with the same id. That is,
     * it is possible to instantiate only one instance of this class for each
     * hardware USART.
     * \param id 0=USART0, 1=USART1, 2=USART2
     * \param baudrate serial port baudrate.
     */
    GD32Serial(int id, int baudrate);

    /**
     * Read a block of data
     * \param buffer buffer where read data will be stored
     * \param size buffer size
     * \param where where to read from
     * \return number of bytes read or a negative number on failure
     */
    ssize_t readBlock(uint16_t *buffer, size_t size, off_t where);

    /**
     * Write a block of data
     * \param buffer buffer where take data to write
     * \param size buffer size
     * \param where where to write to
     * \return number of bytes written or a negative number on failure
     */
    ssize_t writeBlock(uint32_t *buffer, size_t size, off_t where);
    
    /**
     * Write a string even if IRQs are disabled
     * \param str the string to write
     */
    void IRQwrite(const char *str);

    /**
     * \internal the serial port interrupts call this member function.
     * Never call this from user code.
     */
    void IRQhandleInterrupt();

    /**
     * \return port id, 0 for USART0, 1 for USART1, 2 for USART2, ... 
     */
    int getId() const { return portId; }
    
private:
	// locks when the serial driver is transmitting
	Mutex txMutex;
	// locks when the serial driver is receiving
	Mutex rxMutex;

	Queue<char, 64> rxQueue;

	// takes values 0, 1 or 2 depending on the port
    const unsigned char portId;
	// takes values USART0, USART1 or USART2 depending on the port
	uint32_t port;
};

} //namespace miosix

#endif //SERIAL_GD32_H
