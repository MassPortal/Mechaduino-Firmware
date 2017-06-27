#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_RAW     0x3FFF

extern volatile uint32_t setPoint;
extern volatile uint_fast8_t setMicro;
extern volatile int32_t wrapCount;

void setupPins(void);                       // initializes pins
void setupSPI(void);                        //initializes SPI
void configureStepDir(void);                //configure step/dir interface
void output(uint32_t theta, int effort);    //calculates phase currents (commutation) and outputs to Vref pins
void calibrate(void);                       // More better
void serialCheck(void);                     //checks serial port for commands.  Must include this in loop() for serial interface to work
void oneStep(void);                         //take one step
uint_fast16_t readEncoder(void);            //read raw encoder position
void print_angle(void);                     //for debigging purposes in open loop mode:  prints [step number] , [encoder reading]
void setupTCInterrupts(void);               //configures control loop interrupt
void enableTCInterrupts(void);              //enables control loop interrupt.  Use this to enable "closed-loop" modes

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H__ */







