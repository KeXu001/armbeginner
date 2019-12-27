/*
 * GccApplication1.c
 *
 * Created: 12/19/2019 5:22:37 PM
 * Author : Kevin
 */ 


#include "sam.h"

/*
 * From hal_atomic.h
 */
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

/*
 * Configurations copied from LedBlink HPL
 */
//#define CONFIG_NVM_WAIT_STATE	0
//#define CONFIG_CPU_DIV			PM_CPUSEL_CPUDIV_DIV1_Val
//#define CONFIG_APBA_DIV			PM_APBASEL_APBADIV_DIV1_Val
//#define CONFIG_APBB_DIV			PM_APBBSEL_APBBDIV_DIV1_Val
//#define CONFIG_APBC_DIV			PM_APBCSEL_APBCDIV_DIV1_Val

#define CONFIG_CPU_FREQUENCY		1000000

int main(void)
{
	SystemInit();
	
	PORT->Group[0].DIRSET.reg = PORT_PA02;
	PORT->Group[0].OUTSET.reg = PORT_PA02;
	
	//CRITICAL_SECTION_ENTER();
	//NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS(CONFIG_NVM_WAIT_STATE);
	//CRITICAL_SECTION_LEAVE();
	
	//CRITICAL_SECTION_ENTER();
	//PM->CPUSEL.reg |= PM_CPUSEL_CPUDIV(CONFIG_CPU_DIV);
	//CRITICAL_SECTION_LEAVE();
	
	//CRITICAL_SECTION_ENTER();
	//PM->APBASEL.reg |= PM_APBASEL_APBADIV(CONFIG_APBA_DIV);
	//CRITICAL_SECTION_LEAVE();
	
	//CRITICAL_SECTION_ENTER();
	//PM->APBBSEL.reg |= PM_APBBSEL_APBBDIV(CONFIG_APBB_DIV);
	//CRITICAL_SECTION_LEAVE();
	
	//CRITICAL_SECTION_ENTER();
	//PM->APBCSEL.reg |= PM_APBCSEL_APBCDIV(CONFIG_APBC_DIV);
	//CRITICAL_SECTION_LEAVE();
	
	uint16_t calib;
	calib = (SYSCTRL->OSC8M.reg & SYSCTRL_OSC8M_CALIB_Msk) >> SYSCTRL_OSC8M_CALIB_Pos;
	uint16_t frange;
	frange = (SYSCTRL->OSC8M.reg & SYSCTRL_OSC8M_FRANGE_Msk) >> SYSCTRL_OSC8M_FRANGE_Pos;
	
	CRITICAL_SECTION_ENTER();
	SYSCTRL->OSC8M.reg = SYSCTRL_OSC8M_FRANGE(frange) |
							SYSCTRL_OSC8M_CALIB(calib) |
							SYSCTRL_OSC8M_PRESC(SYSCTRL_OSC8M_PRESC_3_Val) |
							(0u << SYSCTRL_OSC8M_RUNSTDBY_Pos) |
							(1u << SYSCTRL_OSC8M_ENABLE_Pos);
	CRITICAL_SECTION_LEAVE();
	while(!SYSCTRL->PCLKSR.bit.OSC8MRDY);
	
	CRITICAL_SECTION_ENTER();
	SYSCTRL->OSC8M.reg |= SYSCTRL_OSC8M_ONDEMAND;
	//CRITICAL_SECTION_LEAVE();
	
	//CRITICAL_SECTION_ENTER();
	SYSCTRL->OSC32K.reg &= ~SYSCTRL_OSC32K_ENABLE;
	//CRITICAL_SECTION_LEAVE();
	
	//CRITICAL_SECTION_ENTER();
	GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1u) |
						GCLK_GENDIV_ID(0u);
	//CRITICAL_SECTION_LEAVE();
	
	//CRITICAL_SECTION_ENTER();
	GCLK->GENCTRL.reg = (0u << GCLK_GENCTRL_RUNSTDBY_Pos) |
							(0u << GCLK_GENCTRL_DIVSEL_Pos) |
							(0u << GCLK_GENCTRL_OE_Pos) |
							(0u << GCLK_GENCTRL_OOV_Pos) |
							(0u << GCLK_GENCTRL_IDC_Pos) |
							(1u << GCLK_GENCTRL_GENEN_Pos) |
							(GCLK_GENCTRL_SRC_OSC8M_Val << GCLK_GENCTRL_SRC_Pos) |
							GCLK_GENCTRL_ID(0u);
	while(GCLK->STATUS.bit.SYNCBUSY);
	CRITICAL_SECTION_LEAVE();
	
	SysTick->LOAD = (0xFFFFFF << SysTick_LOAD_RELOAD_Pos);
	SysTick->CTRL = (1u << SysTick_CTRL_ENABLE_Pos) |
						(0u << SysTick_CTRL_TICKINT_Pos) |
						(1u << SysTick_CTRL_CLKSOURCE_Pos);
	
    while (1) 
    {
		PORT->Group[0].OUTTGL.reg = PORT_PA02;
		
		uint16_t milliseconds = 1000;
		uint32_t cycles = milliseconds * (CONFIG_CPU_FREQUENCY / 10000) * 10;
		
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
