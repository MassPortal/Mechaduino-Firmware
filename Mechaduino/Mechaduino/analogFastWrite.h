#ifndef ANALOG_FAST_WRITE_H
#define ANALOG_FAST_WRITE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void analogFastWrite( uint32_t ulPin, uint32_t ulValue);
void analogInit(uint32_t pin, uint32_t val);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_FAST_WRITE_H */







