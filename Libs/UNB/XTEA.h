
#include "em_device.h"

void Encode(unsigned long* data, unsigned char dataLength, unsigned long* key);
void Decode(unsigned long* data, unsigned char dataLength, unsigned long* key);
void XTEA_Encode(uint8_t *buf, unsigned long* key);
void XTEA_Decode(uint8_t *buf, unsigned long* key);

#define NUM_ITERATIONS 64
