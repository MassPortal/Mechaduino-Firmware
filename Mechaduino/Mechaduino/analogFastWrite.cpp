#include "Arduino.h"
#include "wiring_private.h"
#include "analogFastWrite.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Wait for synchronization of registers between the clock domains */
static inline void syncTC_8(Tc* TCx) 
{
    while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
}

/* Wait for synchronization of registers between the clock domains */
static inline void syncTCC(Tcc* TCCx) 
{
    while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

/*
 * Internal Reference is at 1.0v
 * External Reference should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning : On Arduino Zero board the input/output voltage for SAMD21G18 is 3.3 volts maximum
 */

#if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
#error Upgade arduino variant compiance < 10603
#endif /* Compiance check*/
__attribute__((hot)) void analogFastWrite(uint32_t pin, uint32_t value)
{
    PinDescription pinDesc = g_APinDescription[pin];
    uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
    uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);

    if (tcNum >= TCC_INST_NUM) {
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        TCx->COUNT8.CC[tcChannel].reg = (uint8_t) value;
        syncTC_8(TCx);
    } else {
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        TCCx->CTRLBSET.bit.LUPD = 1;
        syncTCC(TCCx);
        TCCx->CCB[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        TCCx->CTRLBCLR.bit.LUPD = 1;
        syncTCC(TCCx);
    }
}

void analogInit(uint32_t pin, uint32_t value)
{
    
    const uint16_t GCLK_CLKCTRL_IDs[] = {
        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
        GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
        GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
      };

    static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];
    PinDescription pinDesc = g_APinDescription[pin];
    uint32_t attr = pinDesc.ulPinAttribute;
    uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
    uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
    
    if (!tcEnabled[tcNum]) {
        tcEnabled[tcNum] = true;
        pinPeripheral(pin, attr & PIN_ATTR_TIMER ? PIO_TIMER : PIO_TIMER_ALT);

        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
        while (GCLK->STATUS.bit.SYNCBUSY == 1);

        if (tcNum >= TCC_INST_NUM) {
            // -- Configure TC
            Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
            // Disable TCx
            TCx->COUNT8.CTRLA.bit.ENABLE = 0;
            syncTC_8(TCx);
            // Set Timer counter Mode to 8 bits, normal PWM
            TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM;
            syncTC_8(TCx);
            // Set the initial value
            TCx->COUNT8.CC[tcChannel].reg = (uint8_t) value;
            syncTC_8(TCx);
            // Set PER to maximum counter value (resolution : 0xFF)
            TCx->COUNT8.PER.reg = 0xFF;
            syncTC_8(TCx);
            // Enable TCx
            TCx->COUNT8.CTRLA.bit.ENABLE = 1;
            syncTC_8(TCx);
        } else {
            // -- Configure TCC
            Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
            // Disable TCCx
            TCCx->CTRLA.bit.ENABLE = 0;
            syncTCC(TCCx);

            // Set TCx as normal PWM
            TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
            syncTCC(TCCx);
            // Set the initial value
            TCCx->CC[tcChannel].reg = (uint32_t) value;
            syncTCC(TCCx);
            // Set PER to maximum counter value (resolution : 0xFF)
            TCCx->PER.reg = 0xFF; //change to 0x43FF for 10 bit... must also change mapping above
            syncTCC(TCCx);
            // Enable TCCx
            TCCx->CTRLA.bit.ENABLE = 1;
            syncTCC(TCCx);
        }
    }
}

#ifdef __cplusplus
}
#endif








