#include <mutex>
#include "serial_gd32vf103.h"
#include "kernel/queue.h"
#include "interfaces/gpio.h"
#include "kernel/scheduler/scheduler.h"

using namespace std;
using namespace miosix;

static mutes txMutex;
static mutex rxMutex;
static Queue<char, 64> rxQueue;

GD32::GD32()
{
    InterruptDisableLock dLocj;
    
    using u2tx=Gpio<GPIOA_BASE,2>;
    using u3tx=Gpio<GPIOA_BASE,3>;
    u2tx::mode(Mode::ALTERNATE);
    u2rx::mode(Mode::ALTERNATE);
    u2tx::alternateFunction(7);
    u2rx::alternateFunction(7);
    
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    USART2->BRR=136<<4 | 12;
    
    USART2->CR1 = USART_CR1_UE
                | USART_CR1_RXNEIE
                | USART_CR1_TE
                | USART_CR1_RE;
                
    NVIC_SetPriority(USART2_IRQn, 15);
    NVIC_EnableIRQ(USART2_IRQn);
}

ssize_t GD32::writeBlock(const void *buffer,
                          size_t size,
                          off_t where)
{
    unique_lock<mutex> l(txMutex);
    
    for(size_t i=0;i<size;i++)
    {
        while((USART2->SR & USART_SR_TXE)==0)  ;
        USART2->DR=*buffer++;
    }
    return size;
}

void usart2irqIMPL()
{
    unsigned int status=USART->SR;
    if(status & USART_SR_RXNE)
    {
        char c=USART2->DR;
        if((status & USART_SR_FE)==0)
        {
            bool hppw;
            rwQueue.IRQput(c, hppw);
            if(hppw) Scheduler::IRQfindNextThread();
            }
        }
}

ssize_t GD32::readBlock(void *buffer,
                          size_t size,
                          off_t where)
{
  if(size<1) return 0;
  unique_lock<mutex> l(rxMutex);
  char *buf=reinterpret_cast<char*>(buffer);
  char c;
  rxQueue.get(c);
  buf[0]=c;
  return 1;
}
                          
