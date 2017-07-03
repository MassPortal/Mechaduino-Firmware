#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include <stdint.h>
#include <stdbool.h>

//----Current Parameters-----
/* Steps per reveleation */
#define SPR     400
#define USTEPS  32
/* Timer frequency*/
#define FS      25000.0 //6500.0

extern const uint16_t lookup[];

extern const int uMAX;

//Defines for pins:

#define IN_4    6
#define IN_3    5
#define VREF_2  4
#define VREF_1  9
#define IN_2    7
#define IN_1    8
#define PIN_CS      A2 
#define PIN_STEP    1
#define PIN_DIR     0
/* PIN_LED == 13 (used by bootloader & general zero compabitlity)*/


//for faster digitalWrite:
#define IN_1_HIGH() (REG_PORT_OUTSET0 = PORT_PA06)
#define IN_1_LOW() (REG_PORT_OUTCLR0 = PORT_PA06)
#define IN_2_HIGH() (REG_PORT_OUTSET0 = PORT_PA21)
#define IN_2_LOW() (REG_PORT_OUTCLR0 = PORT_PA21)
#define IN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA15)
#define IN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA15)
#define IN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA20)
#define IN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA20)
#define PIN_LED_HIGH() (REG_PORT_OUTSET0 = PORT_PA17)
#define PIN_LED_LOW() (REG_PORT_OUTCLR0 = PORT_PA17)
#define CS_HIGH() (REG_PORT_OUTSET1 = PORT_PB09)
#define CS_LOW() (REG_PORT_OUTCLR1 = PORT_PB09)

#endif
