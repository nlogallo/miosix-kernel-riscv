
#include "interfaces/arch_registers.h"
#include "core/interrupts.h" //For the unexpected interrupt call
#include "kernel/stage_2_boot.h"
#include <string.h>

void reset_stub(void) __attribute__((section(".bootstub"), naked));
void reset_stub(void){
    /* At reset, gp is loaded with a specific address (in the middle of sdata section), in order
     * speed up accesses to frequently used memory locations (this is called linker relaxation).
     * The options are needed in order to prevent the linker from relaxing the initialization
     * of the register
     */

    asm volatile(                                \
    ".option push \n"                            \
    ".option norelax \n"                         \
    "la gp, __global_pointer$ \n"                \
    ".option pop \n"                             \
    "j _Z13Reset_Handlerv \n"                    \
    ".balign 16           \n"                    \
    "j _ZN14miosix_private13IRQEntrypointEv\n"   \
    );
}

/**
 * Called by Reset_Handler, performs initialization and calls main.
 * Never returns.
 */
void program_startup() __attribute__((noreturn));
void program_startup()
{
    __disable_irq();

    //SystemInit() is called *before* initializing .data and zeroing .bss
    SystemInit();

	//These are defined in the linker script
	extern unsigned char _etext asm("_etext");
	extern unsigned char _data asm("_data");
	extern unsigned char _edata asm("_edata");
	extern unsigned char _bss_start asm("_bss_start");
	extern unsigned char _bss_end asm("_bss_end");

    //Initialize .data section, clear .bss section
    unsigned char *etext=&_etext;
    unsigned char *data=&_data;
    unsigned char *edata=&_edata;
    unsigned char *bss_start=&_bss_start;
    unsigned char *bss_end=&_bss_end;
    memcpy(data, etext, edata-data);
    memset(bss_start, 0, bss_end-bss_start);

	//Move on to stage 2
	_init();

	//If main returns, reboot
	asm volatile("j _Z13Reset_Handlerv");

	for(;;){};
}

/**
 * Reset handler, called by hardware immediately after reset
 */
void Reset_Handler() __attribute__((noreturn, naked));
void Reset_Handler()
{
    asm volatile("la sp,  _main_stack_bottom");

    program_startup();
}

/**
 * All unused interrupts call this function.
 */
extern "C" void Default_Handler() 
{
    unexpectedInterrupt();
}

//System handlers
void /*__attribute__((weak))*/ Reset_Handler();     //These interrupts are not
void /*__attribute__((weak))*/ NMI_Handler();       //weak because they are
void /*__attribute__((weak))*/ HardFault_Handler(); //surely defined by Miosix
void /*__attribute__((weak))*/ MemManage_Handler();
void /*__attribute__((weak))*/ BusFault_Handler();
void /*__attribute__((weak))*/ UsageFault_Handler();
void /*__attribute__((weak))*/ SVC_Handler();
void /*__attribute__((weak))*/ DebugMon_Handler();
void /*__attribute__((weak))*/ PendSV_Handler();
void __attribute__((weak)) SysTick_Handler();

//Interrupt handlers
void __attribute__((weak)) CLIC_INT_SFT_IRQHandler();
void __attribute__((weak)) CLIC_INT_TMR_IRQHandler();
void __attribute__((weak)) CLIC_INT_BWEI_IRQHandler();
void __attribute__((weak)) CLIC_INT_PMOVI_IRQHandler();
void __attribute__((weak)) WWDGT_IRQHandler();
void __attribute__((weak)) LVD_From_EXTI_IRQHandler();
void __attribute__((weak)) TAMPER_IRQHandler();
void __attribute__((weak)) RTC_IRQHandler();
void __attribute__((weak)) FMC_IRQHandler();
void __attribute__((weak)) RCU_IRQHandler();
void __attribute__((weak)) EXTI0_IRQHandler();
void __attribute__((weak)) EXTI1_IRQHandler();
void __attribute__((weak)) EXTI2_IRQHandler();
void __attribute__((weak)) EXTI3_IRQHandler();
void __attribute__((weak)) EXTI4_IRQHandler();
void __attribute__((weak)) DMA0_Channel0_IRQHandler();
void __attribute__((weak)) DMA0_Channel1_IRQHandler();
void __attribute__((weak)) DMA0_Channel2_IRQHandler();
void __attribute__((weak)) DMA0_Channel3_IRQHandler();
void __attribute__((weak)) DMA0_Channel4_IRQHandler();
void __attribute__((weak)) DMA0_Channel5_IRQHandler();
void __attribute__((weak)) DMA0_Channel6_IRQHandler();
void __attribute__((weak)) ADC0_1_IRQHandler();
void __attribute__((weak)) CAN0_TX_IRQHandler();
void __attribute__((weak)) CAN0_RX0_IRQHandler();
void __attribute__((weak)) CAN0_RX1_IRQHandler();
void __attribute__((weak)) CAN0_EWMC_IRQHandler();
void __attribute__((weak)) EXTI9_5_IRQHandler();
void __attribute__((weak)) TIMER0_break_IRQHandler();
void __attribute__((weak)) TIM0_update_IRQHandler();
void __attribute__((weak)) TIM0_Trigger_Channel_Commutation_IRQHandler();
void __attribute__((weak)) TIM0_Channel_Capture_Compare_IRQHandler();
void __attribute__((weak)) TIM1_IRQHandler();
void __attribute__((weak)) TIM2_IRQHandler();
void __attribute__((weak)) TIM3_IRQHandler();
void __attribute__((weak)) I2C0_event_IRQHandler();
void __attribute__((weak)) I2C0_error_IRQHandler();
void __attribute__((weak)) I2C1_event_IRQHandler();
void __attribute__((weak)) I2C1_error_IRQHandler();
void __attribute__((weak)) SPI0_IRQHandler();
void __attribute__((weak)) SPI1_IRQHandler();
void __attribute__((weak)) USART0_IRQHandler();
void __attribute__((weak)) USART1_IRQHandler();
void __attribute__((weak)) USART2_IRQHandler();
void __attribute__((weak)) EXTI15_10_IRQHandler();
void __attribute__((weak)) RTC_Alarm_IRQHandler();
void __attribute__((weak)) USB_FS_WKUP_IRQHandler();
void __attribute__((weak)) TIM4_IRQHandler();
void __attribute__((weak)) SPI2_IRQHandler();
void __attribute__((weak)) UART3_IRQHandler();
void __attribute__((weak)) UART4_IRQHandler();
void __attribute__((weak)) TIM5_IRQHandler();
void __attribute__((weak)) TIM6_IRQHandler();
void __attribute__((weak)) DMA1_Channel0_IRQHandler();
void __attribute__((weak)) DMA1_Channel1_IRQHandler();
void __attribute__((weak)) DMA1_Channel2_IRQHandler();
void __attribute__((weak)) DMA1_Channel3_IRQHandler();
void __attribute__((weak)) DMA1_Channel4_IRQHandler();
void __attribute__((weak)) CAN1_TX_IRQHandler();
void __attribute__((weak)) CAN1_RX0_IRQHandler();
void __attribute__((weak)) CAN1_RX1_IRQHandler();
void __attribute__((weak)) CAN1_EWMC_IRQHandler();
void __attribute__((weak)) USB_FS_IRQHandler();

//Stack top, defined in the linker script
extern char _main_stack_top asm("_main_stack_top");

//Interrupt vectors, must be placed @ address 0x00000000
//The extern declaration is required otherwise g++ optimizes it out
extern void (* const __Vectors[])();
void (* const __Vectors[])() __attribute__ ((section(".isr_vector"))) =
{
    reinterpret_cast<void (*)()>(&_main_stack_top),/* Stack pointer*/
    Reset_Handler,              /* Reset Handler */
    NMI_Handler,                /* NMI Handler */
    HardFault_Handler,          /* Hard Fault Handler */
    MemManage_Handler,          /* MPU Fault Handler */
    BusFault_Handler,           /* Bus Fault Handler */
    UsageFault_Handler,         /* Usage Fault Handler */
    0,                          /* Reserved */
    0,                          /* Reserved */
    0,                          /* Reserved */
    0,                          /* Reserved */
    SVC_Handler,                /* SVCall Handler */
    DebugMon_Handler,           /* Debug Monitor Handler */
    0,                          /* Reserved */
    PendSV_Handler,             /* PendSV Handler */
    SysTick_Handler,            /* SysTick Handler */

    /* External Interrupts */
	CLIC_INT_SFT_IRQHandler,
	CLIC_INT_TMR_IRQHandler,
	CLIC_INT_BWEI_IRQHandler,
	CLIC_INT_PMOVI_IRQHandler,
	WWDGT_IRQHandler,
	LVD_From_EXTI_IRQHandler,
	TAMPER_IRQHandler,
	RTC_IRQHandler,
	FMC_IRQHandler,
	RCU_IRQHandler,
	EXTI0_IRQHandler,
	EXTI1_IRQHandler,
	EXTI2_IRQHandler,
	EXTI3_IRQHandler,
	EXTI4_IRQHandler,
	DMA0_Channel0_IRQHandler,
	DMA0_Channel1_IRQHandler,
	DMA0_Channel2_IRQHandler,
	DMA0_Channel3_IRQHandler,
	DMA0_Channel4_IRQHandler,
	DMA0_Channel5_IRQHandler,
	DMA0_Channel6_IRQHandler,
	ADC0_1_IRQHandler,
	CAN0_TX_IRQHandler,
	CAN0_RX0_IRQHandler,
	CAN0_RX1_IRQHandler,
	CAN0_EWMC_IRQHandler,
	EXTI9_5_IRQHandler,
	TIMER0_break_IRQHandler,
	TIM0_update_IRQHandler,
	TIM0_Trigger_Channel_Commutation_IRQHandler,
	TIM0_Channel_Capture_Compare_IRQHandler,
	TIM1_IRQHandler,
	TIM2_IRQHandler,
	TIM3_IRQHandler,
	I2C0_event_IRQHandler,
	I2C0_error_IRQHandler,
	I2C1_event_IRQHandler,
	I2C1_error_IRQHandler,
	SPI0_IRQHandler,
	SPI1_IRQHandler,
	USART0_IRQHandler,
	USART1_IRQHandler,
	USART2_IRQHandler,
	EXTI15_10_IRQHandler,
	RTC_Alarm_IRQHandler,
	USB_FS_WKUP_IRQHandler,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	TIM4_IRQHandler,
	SPI2_IRQHandler,
	UART3_IRQHandler,
	UART4_IRQHandler,
	TIM5_IRQHandler,
	TIM6_IRQHandler,
	DMA1_Channel0_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_IRQHandler,
	DMA1_Channel3_IRQHandler,
	DMA1_Channel4_IRQHandler,
	0,
	0,
	CAN1_TX_IRQHandler,
	CAN1_RX0_IRQHandler,
	CAN1_RX1_IRQHandler,
	CAN1_EWMC_IRQHandler,
	USB_FS_IRQHandler
};

#pragma weak CLIC_INT_SFT_IRQHandler = Default_Handler
#pragma weak CLIC_INT_TMR_IRQHandler = Default_Handler
#pragma weak CLIC_INT_BWEI_IRQHandler = Default_Handler
#pragma weak CLIC_INT_PMOVI_IRQHandler = Default_Handler
#pragma weak WWDGT_IRQHandler = Default_Handler
#pragma weak LVD_From_EXTI_IRQHandler = Default_Handler
#pragma weak TAMPER_IRQHandler = Default_Handler
#pragma weak RTC_IRQHandler = Default_Handler
#pragma weak FMC_IRQHandler = Default_Handler
#pragma weak RCU_IRQHandler = Default_Handler
#pragma weak EXTI0_IRQHandler = Default_Handler
#pragma weak EXTI1_IRQHandler = Default_Handler
#pragma weak EXTI2_IRQHandler = Default_Handler
#pragma weak EXTI3_IRQHandler = Default_Handler
#pragma weak EXTI4_IRQHandler = Default_Handler
#pragma weak DMA0_Channel0_IRQHandler = Default_Handler
#pragma weak DMA0_Channel1_IRQHandler = Default_Handler
#pragma weak DMA0_Channel2_IRQHandler = Default_Handler
#pragma weak DMA0_Channel3_IRQHandler = Default_Handler
#pragma weak DMA0_Channel4_IRQHandler = Default_Handler
#pragma weak DMA0_Channel5_IRQHandler = Default_Handler
#pragma weak DMA0_Channel6_IRQHandler = Default_Handler
#pragma weak ADC0_1_IRQHandler = Default_Handler
#pragma weak CAN0_TX_IRQHandler = Default_Handler
#pragma weak CAN0_RX0_IRQHandler = Default_Handler
#pragma weak CAN0_RX1_IRQHandler = Default_Handler
#pragma weak CAN0_EWMC_IRQHandler = Default_Handler
#pragma weak EXTI9_5_IRQHandler = Default_Handler
#pragma weak TIMER0_break_IRQHandler = Default_Handler
#pragma weak TIM0_update_IRQHandler = Default_Handler
#pragma weak TIM0_Trigger_Channel_Commutation_IRQHandler = Default_Handler
#pragma weak TIM0_Channel_Capture_Compare_IRQHandler = Default_Handler
#pragma weak TIM1_IRQHandler = Default_Handler
#pragma weak TIM2_IRQHandler = Default_Handler
#pragma weak TIM3_IRQHandler = Default_Handler
#pragma weak I2C0_event_IRQHandler = Default_Handler
#pragma weak I2C0_error_IRQHandler = Default_Handler
#pragma weak I2C1_event_IRQHandler = Default_Handler
#pragma weak I2C1_error_IRQHandler = Default_Handler
#pragma weak SPI0_IRQHandler = Default_Handler
#pragma weak SPI1_IRQHandler = Default_Handler
#pragma weak USART0_IRQHandler = Default_Handler
#pragma weak USART1_IRQHandler = Default_Handler
#pragma weak USART2_IRQHandler = Default_Handler
#pragma weak EXTI15_10_IRQHandler = Default_Handler
#pragma weak RTC_Alarm_IRQHandler = Default_Handler
#pragma weak USB_FS_WKUP_IRQHandler = Default_Handler
#pragma weak TIM4_IRQHandler = Default_Handler
#pragma weak SPI2_IRQHandler = Default_Handler
#pragma weak UART3_IRQHandler = Default_Handler
#pragma weak UART4_IRQHandler = Default_Handler
#pragma weak TIM5_IRQHandler = Default_Handler
#pragma weak TIM6_IRQHandler = Default_Handler
#pragma weak DMA1_Channel0_IRQHandler = Default_Handler
#pragma weak DMA1_Channel1_IRQHandler = Default_Handler
#pragma weak DMA1_Channel2_IRQHandler = Default_Handler
#pragma weak DMA1_Channel3_IRQHandler = Default_Handler
#pragma weak DMA1_Channel4_IRQHandler = Default_Handler
#pragma weak CAN1_TX_IRQHandler = Default_Handler
#pragma weak CAN1_RX0_IRQHandler = Default_Handler
#pragma weak CAN1_RX1_IRQHandler = Default_Handler
#pragma weak CAN1_EWMC_IRQHandler = Default_Handler
#pragma weak USB_FS_IRQHandler = Default_Handler
