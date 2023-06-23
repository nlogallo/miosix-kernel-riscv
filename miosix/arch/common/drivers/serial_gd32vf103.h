#pragma once

//#include "filesystem/console/console_device.h"

class GD32 : public miosix::LonganNano
{
public:
  GD32();
  
  ssize_t writeBlock(const void *buffer,
                      size_t size,
                      off_t where);
  
  ssize_t readBlock(void *buffer,
                    size_t size,
                    off_t where);          
};
                      
