#include <SPI.h>
#include <Wire.h>

#include "Arduino.h"
#include "Parameters.h"
#include "Utils.h"
#include "analogFastWrite.h"
#include <math.h>
#include "samd.h"

#ifdef __cplusplus
extern "C" {
#endif


#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY)

static volatile bool dir = true;  

/* Full step target */ 
volatile uint32_t setPoint = 0;
/* Mirostep target */
volatile uint_fast8_t setMicro = 0;
/* Wraps + direction as sign*/
volatile int32_t wrapCount = 0;

void setupPins() {

    pinMode(VREF_2, OUTPUT);
    pinMode(VREF_1, OUTPUT);
    pinMode(IN_4, OUTPUT);
    pinMode(IN_3, OUTPUT);
    pinMode(IN_2, OUTPUT);
    pinMode(IN_1, OUTPUT);
    pinMode(PIN_CS, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer


#ifdef ENABLE_PROFILE_IO  
  pinMode(TEST1, OUTPUT);
#endif

  pinMode(PIN_LED, OUTPUT);

  analogInit(VREF_2, 0.33 * uMAX);
  analogInit(VREF_1, 0.33 * uMAX);

  IN_4_HIGH();   //  digitalWrite(IN_4, HIGH);
  IN_3_LOW();    //  digitalWrite(IN_3, LOW);
  IN_2_HIGH();   //  digitalWrite(IN_2, HIGH);
  IN_1_LOW();    //  digitalWrite(IN_1, LOW);
}

void setupSPI() {

  SPISettings settingsA(10e6, MSBFIRST, SPI_MODE1);             ///400000, MSBFIRST, SPI_MODE1);

  SPI.begin();    //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  SerialUSB.println("Beginning SPI communication with AS5047 encoder...");
  delay(10);
  SPI.beginTransaction(settingsA);

}

static void stepInterrupt(void) 
{
    if (dir) {
        if (setPoint < SPR - 1) {
            setPoint++;
        } else {
            wrapCount--;
            setPoint = 0;
        }
    } else {
        if (setPoint) {
            setPoint--;
        } else {
            wrapCount++;
            setPoint = SPR - 1;
        }
    }
}

static void dirInterrupt() 
{
    dir = (REG_PORT_IN0 & PORT_PA11) ? false : true;
}

void configureStepDir(void) 
{
  pinMode(PIN_STEP, INPUT);
  pinMode(PIN_DIR, INPUT);
  attachInterrupt(PIN_STEP, stepInterrupt, RISING);
  attachInterrupt(PIN_DIR, dirInterrupt, CHANGE);
}

void output(uint32_t theta, int effort) 
{
    #if USTEPS == 1
        /* NOTE: could always use largest lookup however that would result in several unused bytes :( */
        const int_fast16_t sinLook[USTEPS*4] = {0, 1024, 0, -1024};
    #elif USTEPS == 32
        /* NOTE: make it 2x smaller in case you feel like it (int16) */
        /* NOTE: make it 4x smaller in case of emergency (single quadrant || single half-wave) */
        const int_fast16_t sinLook[USTEPS*4] = {0,50,100,150,200,249,297,345,392,438,483,526,569,610,650,688,724,759,792,822,851,878,903,926,946,964,980,993,1004,1013,1019,1023,1024,1023,1019,1013,1004,993,980,964,946,926,903,878,851,822,792,759,724,688,650,610,569,526,483,438,392,345,297,249,200,150,100,50,0,-50,-100,-150,-200,-249,-297,-345,-392,-438,-483,-526,-569,-610,-650,-688,-724,-759,-792,-822,-851,-878,-903,-926,-946,-964,-980,-993,-1004,-1013,-1019,-1023,-1024,-1023,-1019,-1013,-1004,-993,-980,-964,-946,-926,-903,-878,-851,-822,-792,-759,-724,-688,-650,-610,-569,-526,-483,-438,-392,-345,-297,-249,-200,-150,-100,-50};
    #else
        #ifdef USTEPS 
            #error Unsupported number of microsteps
        #else
            #error And how many microsteps would you like (1 for none)?
        #endif /* USTEPS*/
    #endif 

    int32_t coilA, coilB;
    uint_fast16_t angle = theta%(4*USTEPS);

    coilA = (int32_t)(effort*sinLook[angle])/(1024*4);
    angle += USTEPS;
    if (angle >= 4 * USTEPS) angle -= 4 * USTEPS;
    coilB = (int32_t)(effort*sinLook[angle])/(1024*4);
    /* XXX TODO Have a good look if it's actualy fast*/
    /*How many bits does this thing has? */
    analogFastWrite(VREF_1, abs(coilA));
    analogFastWrite(VREF_2, abs(coilB));

    if (coilA >= 0) COIL_A_HIGH();
    else COIL_A_LOW();
    if (coilB >= 0) COIL_B_HIGH();
    else COIL_B_LOW();
}

static bool readEncoderDiagnostics(void)
{
  uint16_t tmp;
  CS_LOW();

  SPI.transfer(0xFF);
  SPI.transfer(0xFC);

  CS_HIGH();
  delay(1);
  CS_LOW();

  uint8_t b1 = SPI.transfer(0xC0);
  uint8_t b2 = SPI.transfer(0x00);

  tmp = (((b1 << 8) | b2));
  
  if (tmp & (1 << 14))    SerialUSB.println("Error occurred");
  if (tmp & (1 << 11))    SerialUSB.println("Magnet too much!");
  if (tmp & (1 << 10))    SerialUSB.println("Magnet much?");
  if (tmp & (1 << 9))     SerialUSB.println("CORDIC overflow");

  if ((tmp & (1 << 14)) | (tmp & (1 << 11)) | (tmp & (1 << 10)) | (tmp & (1 << 9))) return false;

  CS_HIGH();
  delay(1);
  CS_LOW();

  SPI.transfer(0x40); 
  SPI.transfer(0x01);
  CS_HIGH();

  delay(1);
  
  CS_LOW();
  b1 = SPI.transfer(0xC0);
  b2 = SPI.transfer(0x00);
  tmp = (((b1 << 8) | b2));
  
  if (tmp & (1 << 14))  SerialUSB.println("Error occurred");
  if (tmp & (1 << 2))   SerialUSB.println("Parity error");
  if (tmp & (1 << 1))   SerialUSB.println("Invalid register");
  if (tmp & (1 << 0))   SerialUSB.println("Framing error");

  if ((tmp & (1 << 14)) | (tmp & (1 << 2)) | (tmp & (1 << 1)) | (tmp & (1 << 0)))  return false;

  CS_HIGH();

  delay(1);

  return true;
}

#define AVG_COUNT   16      // Number of samples to average
#define MAX_DELTA   0xFF    // Maximum difference between two steps
#define MIN_DELTA   10      // Minumum difference between two steps
#define MAX_NOISE   10      // Maximum difference in reading for a single step
void calibrate(void)
{
    uint16_t stepReading[SPR];
    int32_t sum;
    int16_t raw0, raw1;

    uint16_t stepMin = 0;
    
    uint16_t minD = 0xFFFF ;
    uint16_t maxD = 0;
    uint16_t stepD = 0;

    dir = true;
    SerialUSB.println("::Sanity check::");

    if (readEncoderDiagnostics()) {
        SerialUSB.println("::Proceed::");
    } else {
        SerialUSB.println("::Abort::");
        return;
    }

    SerialUSB.println("::Calib begin::");
    
    raw0 = readEncoder();
    oneStep();
    delay(100);
    raw1 = readEncoder();
    
    if (raw0 > raw1 && !(raw1 + MAX_RAW/2 < raw0)) {
        SerialUSB.println("Wired backwards");
        SerialUSB.println("::Abort::");
        return;
    }

    /* Starting step must not matter - it should be adjusted when running*/
    /* Go through all possible steps*/
    for (uint_fast16_t i = 0; i<SPR; i++) {
        oneStep();
        /* 70 seems to be good enaugh */
        delay(70);

        /* Use this to lock side if neccessary*/
        raw0 = readEncoder();

        sum = 0;
        /* Get some samples and average them*/
        for (uint_fast16_t k = 0; k<AVG_COUNT; k++) {
            
            raw1 = readEncoder();
            if (raw0 > raw1 + MAX_DELTA) {
                raw1 += MAX_RAW;
            } else if (raw0 + MAX_DELTA < raw1) {
                raw1 -= MAX_RAW;
            }
            /* Save largest deviation inside any step*/
            if (stepD < abs(raw0 - raw1)) {
                stepD = abs(raw0 - raw1);
                if (stepD > MAX_NOISE) {
                    SerialUSB.println("Volatile readings");
                    SerialUSB.println("::Abort::");
                    return;
                }
            }
            sum += raw1;
        }
        /* Maybe dont drop lsb*/
        raw0 = sum / AVG_COUNT;
        if (raw0 > MAX_RAW) raw0 -= MAX_RAW;
        else if (raw0 < 0) raw0 += MAX_RAW;

        stepReading[i] = raw0;
        if (!(i%5)) {
            SerialUSB.print(100 * (float)i / SPR);
            SerialUSB.println("%");
        }
        /* Save smallest step*/
        if (stepReading[i] < stepReading[stepMin]) stepMin = i;
    }

    /* XXX TODO check continuity MIN/MAX deltas*/
    SerialUSB.print("\n\n0x");
    for (uint_fast16_t i = 0, k = stepMin; i<SPR; i++, k++) {
        if (k == SPR) k = 0;
        SerialUSB.print(stepReading[k],HEX);
        SerialUSB.print((i != SPR - 1) ? " ,0x" : "\n");
    }

    /* Bonus data to calibrate calibration process*/
    for (uint16_t i = 0, k = 1; i<SPR; i++, k++) {
        if (k == SPR) k = 0;
        raw0 = (stepReading[i] > stepReading[k]) ? stepReading[i] - MAX_RAW : stepReading[i];
        if (abs(stepReading[k] - raw0) < minD) minD = abs(stepReading[k] - raw0);
        if (abs(stepReading[k] - raw0) > maxD) maxD = abs(stepReading[k] - raw0);
    }

    SerialUSB.print("\n\n");
    SerialUSB.print("StepD = ");
    SerialUSB.println(stepD);
    SerialUSB.print("MaxD = ");
    SerialUSB.println(maxD);
    SerialUSB.print("MinD = ");
    SerialUSB.println(minD);
}

void serialCheck(void) 
{
   if (SerialUSB.available()) {

        char inChar = (char)SerialUSB.read();

        switch (inChar) {

        case 'p':           // print
            print_angle();
            break;

        case 's':           // step
            oneStep();
            print_angle();
            break;

        case 'd':           // dir
            dir = !dir;
            break;

        case 'c':
            calibrate();    // cal routine
            break;

        default:
            break;
        }
    }
}

void oneStep(void) 
{
    /* Since we have a "one step" function anyway*/
    stepInterrupt();
    output(setPoint, 200);
    delay(10);
}

uint_fast16_t readEncoder(void)
{
    uint_fast16_t tmp;
  
    CS_LOW();

    uint8_t b1 = SPI.transfer(0xFF);
    uint8_t b2 = SPI.transfer(0xFF);

    tmp = (((b1 << 8) | b2) & MAX_RAW);

    CS_HIGH();
    return tmp;
}



void print_angle(void)
{
    const int avg = 8;            //average a few readings
    uint32_t encoderReading = 0;

    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
        encoderReading += readEncoder();
        delay(10);
    }
    SerialUSB.print("stepNumber: ");
    SerialUSB.print(setPoint, DEC);
    SerialUSB.print(" , ");
    SerialUSB.print("Angle: ");
    SerialUSB.println((360.0*encoderReading)/(MAX_RAW*avg), 2);
}

void setupTCInterrupts(void) 
{
    // Enable GCLK for TC4 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY);

    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
    TC5->COUNT16.CC[0].reg = (int)( round(48000000 / FS));
    WAIT_TC16_REGS_SYNC(TC5);

    TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
    TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
    TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

    NVIC_SetPriority(TC5_IRQn, 1);              //Set interrupt priority

    // Enable InterruptVector
    NVIC_EnableIRQ(TC5_IRQn);
}

void enableTCInterrupts(void) 
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
  WAIT_TC16_REGS_SYNC(TC5);                     //wait for sync
}

#ifdef __cplusplus
}
#endif
