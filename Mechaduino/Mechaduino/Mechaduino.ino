
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

#include "Utils.h"
#include "Parameters.h"
#include "State.h"
#include "analogFastWrite.h"
#include "Controller.h"

/* Comment for closed loop uncomment for calibreation*/
#define CALIBRATE

/* This is a copy form same thing @ Controller.cpp*/
static inline uint16_t seekReal(uint16_t raw)
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

    setpoint = seekReal(newRaw);
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
