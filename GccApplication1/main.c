
#include "sam.h"

/* hal_atomic.h */
#define CRITICAL_SECTION_ENTER()         \
	{                                    \
		volatile uint32_t __atomic;      \
		__atomic = __get_PRIMASK();      \
		__disable_irq();                 \
		__DMB();
#define CRITICAL_SECTION_LEAVE()         \
		__DMB();                         \
		__set_PRIMASK(__atomic);         \
	}

#define CONFIG_CPU_FREQUENCY		1000000

int main(void)
{
	SystemInit();
	
	PORT->Group[0].DIRSET.reg = PORT_PA02;
	PORT->Group[0].OUTSET.reg = PORT_PA02;
	
	/*
	 * By default, clock source OSC8M is configured to have a DIV8 prescaler (SYSCTRL->OSC8M)
	 * 
	 * So, the OSC8M clock source outputs a 1MHz clock (roughly)
	 * 
	 * Generator 0 is, by default, enabled and uses OSC8M as its source (see SAM D21 datasheet 8.3.1)
	 * 
	 * Generator 0 is the clock used by the PM (see SAM D21 datasheet Fig 14-1).
	 * 
	 * The clock is _not_ divided before reaching the CPU (PM->CPUSEL.CPUDIV is divide by 1)
	 *
	 * So, the CPU is running using a 1MHz clock on startup (per comments in system_samd21.c)
	 * 
	 * The SysTick is then configured to use the the internal core clock as its source
	 *  http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0179b/ar01s02s08.html
	 * 
	 * The delay loop contains an inner delay loop which runs multiple times
	 * This is simply because the max counter for SysTick is 24bits
	 *  http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0179b/ar01s02s08.html
	 *
	 * 
	 */
	
	/*CRITICAL_SECTION_ENTER();
	GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |
						GCLK_GENDIV_ID(0);
	GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN) |
							GCLK_GENCTRL_SRC_OSC8M |
							GCLK_GENCTRL_ID(0u);
	while(GCLK->STATUS.bit.SYNCBUSY);
	CRITICAL_SECTION_LEAVE();*/
	
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) |
						(0 << SysTick_CTRL_TICKINT_Pos) |
						(1 << SysTick_CTRL_CLKSOURCE_Pos);
	
    while (1) 
    {
		PORT->Group[0].OUTTGL.reg = PORT_PA02;
		
		uint16_t milliseconds = 1000;
		uint32_t cycles = milliseconds * (CONFIG_CPU_FREQUENCY / 1000);
		
		uint8_t  n   = cycles >> 24;
		uint32_t buf = cycles;

		while (n--) {
			SysTick->LOAD = 0xFFFFFF;
			SysTick->VAL  = 0xFFFFFF;
			while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
			buf -= 0xFFFFFF;
		}

		SysTick->LOAD = buf;
		SysTick->VAL  = buf;
		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }
}
