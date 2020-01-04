
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

void setup_led_blink(void)
{
	CRITICAL_SECTION_ENTER();
	
	/* external crystal oscillator */
	SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_XTALEN |
	SYSCTRL_XOSC32K_STARTUP(0x5) |
	SYSCTRL_XOSC32K_EN32K;
	SYSCTRL->XOSC32K.reg |= SYSCTRL_XOSC32K_ENABLE;
	while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY);
	
	/* generic clock generator */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(2u) | GCLK_GENDIV_DIV(1u);
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2u) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;
	while(GCLK->STATUS.bit.SYNCBUSY);
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_RTC | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;
	
	/* power manager */
	PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1;
	PM->APBAMASK.reg |= PM_APBAMASK_RTC;
	
	/* real-time counter */
	RTC->MODE1.CTRL.reg = RTC_MODE1_CTRL_MODE_COUNT16 | RTC_MODE1_CTRL_PRESCALER_DIV32;
	RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(1024u);
	while(RTC->MODE1.STATUS.bit.SYNCBUSY);
	RTC->MODE1.COMP[0].reg = 0xFFFF;
	while(RTC->MODE1.STATUS.bit.SYNCBUSY);
	RTC->MODE1.COMP[1].reg = 0xFFFF;
	while(RTC->MODE1.STATUS.bit.SYNCBUSY);
	RTC->MODE1.INTENSET.reg = RTC_MODE1_INTENSET_OVF;
	RTC->MODE1.CTRL.reg |= RTC_MODE1_CTRL_ENABLE;
	while(RTC->MODE1.STATUS.bit.SYNCBUSY);
	
	/* NVIC */
	NVIC_EnableIRQ(RTC_IRQn);
	
	CRITICAL_SECTION_LEAVE();
}

int main(void)
{
	SystemInit();
	
	setup_led_blink();
	
// 	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) |
// 						(0 << SysTick_CTRL_TICKINT_Pos) |
// 						(1 << SysTick_CTRL_CLKSOURCE_Pos);
	
	PORT->Group[0].DIRSET.reg = PORT_PA02;
	PORT->Group[0].OUTSET.reg = PORT_PA02;
	
    while (1) 
    {
// 		SysTick->LOAD = 0x2DC6C0; // 3 seconds
// 		SysTick->VAL = 0x2DC6C0;
// 		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
// 		
// 		PORT->Group[0].OUTTGL.reg = PORT_PA02;
    }
}

void RTC_Handler (void)
{
	if (RTC->MODE1.INTFLAG.bit.OVF)
	{
		PORT->Group[0].OUTTGL.reg = PORT_PA02;
	}
	RTC->MODE1.INTFLAG.reg = RTC_MODE1_INTFLAG_OVF; // "This flag is cleared by writing a one to the flag."
}