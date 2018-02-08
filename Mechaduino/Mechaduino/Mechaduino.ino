
/*
  -------------------------------------------------------------
  Mechaduino 0.1 & 0.2 Firmware  v0.1.4
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  All Mechaduino related materials are released under the
  Creative Commons Attribution Share-Alike 4.0 License
  https://creativecommons.org/licenses/by-sa/4.0/

  Many thanks to all contributors!
  --------------------------------------------------------------
  
  Controlled via a SerialUSB terminal at 115200 baud.

  Implemented serial commands are:
 p  - read angle
 s  -  step
 d  -  dir
 c  -  calibration routine
*/

/* Comment for closed loop uncomment for calibreation*/
//#define CALIBRATE

#include "Arduino.h"

#include "Utils.h"
#include "Parameters.h"
#include "samd.h"

static volatile uint_fast16_t oldRaw;
static volatile uint_fast16_t newRaw;
static volatile uint_fast16_t realPoint = 0;
static volatile uint_fast8_t realMicro = 0;

/* Finds real step position in lookuptable */
__attribute__((hot)) static inline uint_fast16_t seekReal(uint16_t raw)
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
    i = (raw - 1)*SPR / MAX_RAW;
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

void setup(void)
{

    setupPins();                      // configure pins
    setupTCInterrupts();              // configure controller interrupt
    setupSPI();                       // Sets up SPI for communicating with encoder

#ifdef CALIBRATE

    SerialUSB.begin(115200);
    while (!SerialUSB);

#else /* !CALIBRATE*/

    /* Find starting setpoint*/
    newRaw = readEncoder();
    oldRaw = newRaw;

    setPoint = seekReal(newRaw);
    configureStepDir();
    enableTCInterrupts();

#endif /* CALIBRATE */

}

void loop(void)
{
#ifdef CALIBRATE

    /* Checks for serial commands*/
    serialCheck();

#else /* !CALIBRATE*/

    /* No need for whatever arduino whants to do */
    /* There is a single do-it-all interrupt*/
    while ("false");

#endif /* CALIBRATE*/

}

static volatile int_fast8_t microToDo;
static volatile int_fast16_t stepToDo;

/* Gets called with FS frequency*/
/* Gets called frequently and it's an interrupt*/
__attribute__((hot)) __attribute__((interrupt)) void TC5_Handler(void)
{
    int_fast8_t move;
    /* Overfolow didnt cause interrupt*/
    if (TC5->COUNT16.INTFLAG.bit.OVF != 1) while ("0_o");//return;
    TC5->COUNT16.INTFLAG.bit.OVF = 1; 

    /* Get new position*/
    newRaw = readEncoder();
    /* XXX TODO Wrap and real point can be inconsistent*/
    if (newRaw > oldRaw && oldRaw + MAX_RAW/2 < newRaw) wrapCount--;
    else if (newRaw < oldRaw && newRaw + MAX_RAW/2 < oldRaw) wrapCount++;
    realPoint = seekReal(newRaw);
    oldRaw = newRaw;

    if (microToDo > 0) {
        move = 1;
    } else if (microToDo < 0) {
        move = -1;
    } else if (wrapCount<0) {
        microToDo = -USTEPS;
        stepToDo = realPoint;
        move = -1;
    } else if (wrapCount>0) {
        microToDo = USTEPS;
        stepToDo = realPoint;
        move = 1;
    } else if (setPoint > realPoint && !(setPoint + 1 == realPoint && setMicro)) {
        microToDo  = -USTEPS;
        stepToDo = realPoint;
        move = -1;
        /* One is wrong and should be removed */
    } else if (setPoint < realPoint && !(realPoint + 1 == setPoint && setMicro)) {
        microToDo = USTEPS;
        stepToDo = realPoint;
        move = 1;
    } else if (setMicro > realMicro) {
        move = -1;
    } else if (setMicro < realMicro) {
        move = 1;
    } else {
        move = 0;
    }

    if (microToDo) {
        if (microToDo > 0) microToDo--;
        else microToDo++;
    }
    if (!microToDo) stepToDo = realPoint;

    /*
    SerialUSB.print(wrapCount);
    SerialUSB.print(" : ");
    SerialUSB.print(setPoint);
    SerialUSB.print(" : ");
    SerialUSB.print(stepToDo);
    SerialUSB.print(" : ");
    SerialUSB.print(microToDo);
    SerialUSB.print(" : ");
    SerialUSB.print(realMicro);
    SerialUSB.print(" : ");
    SerialUSB.print(move);
    SerialUSB.print("\n");
    */

    move ? PIN_LED_HIGH() : PIN_LED_LOW();
    output(stepToDo*USTEPS + realMicro + move, 300);
    if (!realMicro && move == -1) {
        realMicro = USTEPS - 1;
    } else if (realMicro == USTEPS - 1 && move == 1) {
        realMicro = 0;
    } else {
        realMicro += move;
    }
#if 0
    if (wrapCount<0) {
        output(realPoint - 1, 200);
        PIN_LED_HIGH();
    } else if (wrapCount>0) {
        output(realPoint + 1, 200);
        PIN_LED_HIGH();
    } else if (setPoint > realPoint) {
        output(realPoint - 1, 200);
        PIN_LED_HIGH();
    } else if (setPoint < realPoint) {
        output(realPoint + 1, 200);
        PIN_LED_HIGH();
    } else {
        PIN_LED_LOW();
    }
#endif /* 0 */
    /* Clear ovf flag */
}
