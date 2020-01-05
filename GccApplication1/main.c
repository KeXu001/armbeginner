
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
	
	PORT->Group[0].DIRSET.reg = PORT_PA02;
	PORT->Group[0].OUTSET.reg = PORT_PA02;
	
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
	
	/* NVIC interrupt */
	NVIC_EnableIRQ(RTC_IRQn);
	
	CRITICAL_SECTION_LEAVE();
}

void setup_sercom_usart(void)
{
	CRITICAL_SECTION_ENTER();
	
	/* port */
	PORT->Group[0].PINCFG[12].reg = PORT_PINCFG_PMUXEN; // PA12
	PORT->Group[0].PINCFG[13].reg = PORT_PINCFG_PMUXEN; // PA13
	PORT->Group[0].PINCFG[14].reg = PORT_PINCFG_PMUXEN; // PA14
	PORT->Group[0].PINCFG[15].reg = PORT_PINCFG_PMUXEN; // PA15
	
	PORT->Group[0].PMUX[6].reg = PORT_PMUX_PMUXE_C; // PA12 is in the even spot of PMUX6
	PORT->Group[0].PMUX[6].reg = PORT_PMUX_PMUXO_C; // PA13 is in the odd spot of PMUX6
	PORT->Group[0].PMUX[7].reg = PORT_PMUX_PMUXE_C; // PA14 is in the even spot of PMUX7
	PORT->Group[0].PMUX[7].reg = PORT_PMUX_PMUXO_C; // PA15 is in the odd spot of PMUX7
	
	/* generic clock generator */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM2_CORE | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	
	/* power manager */
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1;
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2;
	
	/* sercom USART */
	SERCOM2->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK |		// USART w/ internal clock
								SERCOM_USART_CTRLA_DORD |					// LSB first
								(0u << SERCOM_USART_CTRLA_CMODE_Pos) |		// asynchronous
								SERCOM_USART_CTRLA_RXPO(0x1) |				// PAD[1] = RX
								SERCOM_USART_CTRLA_TXPO(0x1) |				// PAD[2] = TX, PAD[3] = XCK
								SERCOM_USART_CTRLA_FORM(0x0);				// USART frame w/out parity
	SERCOM2->USART.BAUD.reg = SERCOM_USART_BAUD_BAUD(0xD8AE);				// 9600 f_baud = 55470 BAUD.reg
	SERCOM2->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0x0) |				// char size = 8bits
								SERCOM_USART_CTRLB_RXEN |					// enable RX
								SERCOM_USART_CTRLB_TXEN |					// enable TX
								(0x0 << SERCOM_USART_CTRLB_SBMODE_Pos);		// 1 stop bit
	SERCOM2->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	while(SERCOM2->USART.SYNCBUSY.bit.ENABLE);
	while(SERCOM2->USART.SYNCBUSY.bit.CTRLB);
	
	CRITICAL_SECTION_LEAVE();
}

int main(void)
{
	SystemInit();
	
	setup_led_blink();
	setup_sercom_usart();
	
	PORT->Group[0].OUTTGL.reg = PORT_PA02;
	
    while (1) 
    {
		
    }
}

void RTC_Handler (void)
{
	if (RTC->MODE1.INTFLAG.bit.OVF)
	{
		while(!SERCOM2->USART.INTFLAG.bit.DRE);
		SERCOM2->USART.DATA.reg = 0x46; // F
		while(!SERCOM2->USART.INTFLAG.bit.TXC);
		SERCOM2->USART.DATA.reg = 0x0A; // newline
		while(!SERCOM2->USART.INTFLAG.bit.TXC);
		
		PORT->Group[0].OUTTGL.reg = PORT_PA02;
	}
	RTC->MODE1.INTFLAG.reg = RTC_MODE1_INTFLAG_OVF; // "This flag is cleared by writing a one to the flag."
}