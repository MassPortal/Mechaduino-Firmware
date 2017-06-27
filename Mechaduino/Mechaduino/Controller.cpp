#include <SPI.h>

#include "State.h"
#include "Utils.h"
#include "Parameters.h"
#include "Arduino.h"

volatile uint_fast16_t oldRaw;
volatile uint_fast16_t newRaw;
static volatile uint_fast16_t realPoint;
static volatile uint_fast16_t oldPoint;

/* Finds real step position in lookuptable */
__attribute__((hot)) static inline uint16_t seekReal(uint16_t raw)
{
    uint_fast32_t i;

    if (raw < lookup[0]) {
        /* 0 edgecase */
        return (raw + MAX_RAW - lookup[SPR - 1] < lookup[0] - raw) ? SPR - 1 : 0;
    } else if (raw > lookup[SPR - 1]) {
        /* MAX_RAW egecase */
        return (lookup[0] + MAX_RAW - raw < raw - lookup[SPR - -1]) ? 0 : SPR - 1;
    } 
    /* Go to target approx */
    i = (raw - 1)*SPR / MAX_RAW;// ((MAX_RAW + 1) / SPR);
    if (lookup[i] < raw) {
        while (i<SPR - 1) {
            if (lookup[i] >= raw) {
                return (lookup[i] - raw < raw - lookup[i - 1] ) ? i : i - 1;
            }
            i++;
        }
    } else {
        while (i) {
            if (lookup[i] <= raw) {
                return (raw - lookup[i] < lookup[i + 1] - raw) ? i : i + 1;
            }
            i--;
        }
    }
    return i;
}

/* Gets called with FS frequency*/
/* Gets called frequently and it's an interrupt*/
__attribute__((hot)) __attribute__((interrupt)) void TC5_Handler(void)
{
    /* Overfolow didnt cause interrupt*/
    if (TC5->COUNT16.INTFLAG.bit.OVF != 1) return;
    /* Get new position*/
    newRaw = readEncoder();
    if (newRaw > oldRaw && oldRaw + MAX_RAW/2 < newRaw) wrapCount--;
    else if (newRaw < oldRaw && newRaw + MAX_RAW/2 < oldRaw) wrapCount++;
#if 1 
    realPoint = seekReal(newRaw);
    /*if (realPoint != oldPoint) {
        oldPoint = realPoint;
        SerialUSB.print(setpoint);
        SerialUSB.print(" * ");
        SerialUSB.print(oldPoint);
        SerialUSB.print(" * ");
        SerialUSB.println(wrapCount);
    
    }*/

    if (wrapCount<0) {
        output(realPoint - 1, 60);
        ledPin_HIGH();
    } else if (wrapCount>0) {
        output(realPoint + 1, 60);
        ledPin_HIGH();
    } else if (setpoint > realPoint) {
        output(realPoint - 1, 60);
        ledPin_HIGH();
    } else if (setpoint < realPoint) {
        output(realPoint + 1, 60);
        ledPin_HIGH();
    } else {
        ledPin_LOW();
    }

#else 
    if (wrapCount>0) {
        output(360.0*newRaw/MAX_RAW + PA, uMAX/3);
    } else if (wrapCount<0) {
        output(360.0*newRaw/MAX_RAW - PA, uMAX/3);
        /* Get real numbers and so on*/
        /* XXX this should make use of that lookuptable*/
        /* XXX This is bugged for some edge cases*/
        /* Also output thing should be adjusted to microstep-capable*/
    } else if (lookup[setpoint] + 30u < newRaw) {
        output(360.0*newRaw/MAX_RAW + PA, uMAX/3);
    } else if (lookup[setpoint] > newRaw + 30u) {
        output(360.0*newRaw/MAX_RAW - PA, uMAX/3);
    } else {
        ledPin_LOW();
    }
#endif /* 1*/
    oldRaw = newRaw;
    

    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the ovf flag
}
