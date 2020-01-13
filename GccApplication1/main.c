
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

#define CONFIG_CPU_FREQUENCY 1000000

// make NVM calibration data more accessible
// https://community.atmel.com/forum/samd10-how-readwrite-nvm-flash-without-asf
typedef struct {
    uint64_t  :27;                      /*!< bit:  0..26  Reserved                           */
    uint64_t  ADC_LINEARITY :8;         /*!< bit: 27..34  ADC Linearity Calibration          */
    uint64_t  ADC_BIASCAL :3;           /*!< bit: 35..37  ADC Bias Calibration               */
    uint64_t  OSC32K_CAL :7;            /*!< bit: 38..44  OSC32KCalibration                  */
    uint64_t  USB_TRANSN :5;            /*!< bit: 45..49  USB TRANSN calibration value       */
    uint64_t  USB_TRANSP :5;            /*!< bit: 50..54  USB TRANSP calibration value       */
    uint64_t  USB_TRIM :3;              /*!< bit: 55..57  USB TRIM calibration value         */
    uint64_t  DFLL48M_COARSE_CAL :6;    /*!< bit: 58..63  DFLL48M Coarse calibration value   */
} NVM_SW_Calib_t;
#define NVM_SW_CALIB_ADDR   NVMCTRL_OTP4
#define NVM_SW_CALIB        ((NVM_SW_Calib_t*) NVM_SW_CALIB_ADDR)


void uart_putc(char c)
{
    if (c)
    {
        while(!SERCOM2->USART.INTFLAG.bit.DRE);
        SERCOM2->USART.DATA.reg = SERCOM_USART_DATA_DATA(c);
        while(!SERCOM2->USART.INTFLAG.bit.TXC);
    }
}

void uart_puti(uint32_t val, uint32_t base)
{
    char txt[10] = {0};  // maximum value of uint32_t is 10-digits
    
    uint32_t ind = 9;  // start from end

    while(val != 0)
    {
        uint32_t rem = val % base;
        txt[ind--] = (rem <= 9) ? rem + '0' : (rem-10) + 'A';  // support both decimal and hex
        val = val / base;
    }

    if (ind==9)
    {
        txt[ind] = '0';  // edge case when val = 0
    }

    ind = 0;  // start from beginning for printing
    while(ind <= 9)
    {
        uart_putc(txt[ind++]);
    }

    return;
}

uint32_t adc_read()
{
    uint32_t res;

    ADC->SWTRIG.reg = ADC_SWTRIG_START;
    while(ADC->STATUS.bit.SYNCBUSY);
    while(!ADC->INTFLAG.bit.RESRDY);
    res = ADC->RESULT.reg;

    return res;
}

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
    PORT->Group[0].PINCFG[12].reg |= PORT_PINCFG_PMUXEN;  // PA12
    PORT->Group[0].PINCFG[13].reg |= PORT_PINCFG_PMUXEN;  // PA13
    PORT->Group[0].PINCFG[14].reg |= PORT_PINCFG_PMUXEN;  // PA14
    PORT->Group[0].PINCFG[15].reg |= PORT_PINCFG_PMUXEN;  // PA15
    
    PORT->Group[0].PMUX[6].reg |= PORT_PMUX_PMUXE_C;  // PA12 is in the even spot of PMUX6
    PORT->Group[0].PMUX[6].reg |= PORT_PMUX_PMUXO_C;  // PA13 is in the odd spot of PMUX6
    PORT->Group[0].PMUX[7].reg |= PORT_PMUX_PMUXE_C;  // PA14 is in the even spot of PMUX7
    PORT->Group[0].PMUX[7].reg |= PORT_PMUX_PMUXO_C;  // PA15 is in the odd spot of PMUX7
    
    /* generic clock generator */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM2_CORE | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
    
    /* power manager */
    PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1;
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2;
    
    /* sercom USART */
    SERCOM2->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK |       // USART w/ internal clock
                                (1u << SERCOM_USART_CTRLA_DORD_Pos) |        // LSB first
                                (0u << SERCOM_USART_CTRLA_CMODE_Pos) |       // asynchronous
                                SERCOM_USART_CTRLA_RXPO(0x1) |               // PAD[1] = RX
                                SERCOM_USART_CTRLA_TXPO(0x1) |               // PAD[2] = TX, PAD[3] = XCK
                                SERCOM_USART_CTRLA_FORM(0x0);                // USART frame w/out parity
    SERCOM2->USART.BAUD.reg = SERCOM_USART_BAUD_BAUD(0xD8AE);                // 9600 f_baud = 55470 BAUD.reg
    SERCOM2->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0x0) |              // char size = 8bits
                                SERCOM_USART_CTRLB_RXEN |                    // enable RX
                                SERCOM_USART_CTRLB_TXEN |                    // enable TX
                                (0x0 << SERCOM_USART_CTRLB_SBMODE_Pos);      // 1 stop bit
    SERCOM2->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    while(SERCOM2->USART.SYNCBUSY.bit.ENABLE);
    while(SERCOM2->USART.SYNCBUSY.bit.CTRLB);
    
    CRITICAL_SECTION_LEAVE();
}

void setup_adc(void)
{
    CRITICAL_SECTION_ENTER();

    /* port */
    PORT->Group[0].PINCFG[8].reg |= PORT_PINCFG_PMUXEN;     // PA08
    PORT->Group[0].PMUX[4].reg |= PORT_PMUX_PMUXE_B;        // PA08 is in the even spot of PMUX4
    PORT->Group[0].PINCFG[3].reg |= PORT_PINCFG_PMUXEN;     // PA03
    PORT->Group[0].PMUX[1].reg |= PORT_PMUX_PMUXO_B;        // PA03 is in the odd spot of PMUX1

    /* generic clock generator */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;

    /* power manager */
    PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1;  // redundant if this is done in setup_sercom_usart
    PM->APBCMASK.reg |= PM_APBCMASK_ADC;

    /* ADC */
    ADC->REFCTRL.reg = ADC_REFCTRL_REFCOMP |                // enable reference buffer offset
                        ADC_REFCTRL_REFSEL_AREFA;           // use VREFA = PA03 = pin4 as refernce voltage
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 |             // peripheral_clock div 4
                        ADC_CTRLB_RESSEL_12BIT |            // default 12 bit conversion
                        (0u << ADC_CTRLB_FREERUN_Pos) |     // single conversion mode
                        (0u << ADC_CTRLB_DIFFMODE_Pos);     // single-ended mode
    while(ADC->STATUS.bit.SYNCBUSY);
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_1X |            // 1x gain
                            ADC_INPUTCTRL_INPUTOFFSET(0u) | // disable pin scan (use single pin for pos(+) input)
                            ADC_INPUTCTRL_MUXNEG(0x18) |    // negative = internal ground
                            ADC_INPUTCTRL_MUXPOS(0x02);     // positive = AIN[2] = pin7
    while(ADC->STATUS.bit.SYNCBUSY);

    uint64_t linearity = NVM_SW_CALIB->ADC_LINEARITY;
    uint64_t bias = NVM_SW_CALIB->ADC_BIASCAL;
    ADC->CALIB.reg = ADC_CALIB_LINEARITY_CAL(linearity) | ADC_CALIB_BIAS_CAL(bias);

    ADC->CTRLA.reg = ADC_CTRLA_ENABLE;

    adc_read();  // "The first conversion after the reference is changed must not be used."

    CRITICAL_SECTION_LEAVE();
}

int main(void)
{
    SystemInit();
    
    setup_led_blink();
    setup_sercom_usart();
    setup_adc();
    
    PORT->Group[0].OUTTGL.reg = PORT_PA02;

    while (1)
    {
        
    }
}

uint32_t master_counter = 0;

void RTC_Handler (void)
{
    if (RTC->MODE1.INTFLAG.bit.OVF)
    {
        uart_puti(master_counter, 16);
        uart_putc(':');
        uart_putc(' ');
        master_counter++;

        uint32_t res = adc_read();
        uart_puti(res, 10);
        uart_putc('\n');
        
        PORT->Group[0].OUTTGL.reg = PORT_PA02;
    }
    RTC->MODE1.INTFLAG.reg = RTC_MODE1_INTFLAG_OVF; // "This flag is cleared by writing a one to the flag."
}