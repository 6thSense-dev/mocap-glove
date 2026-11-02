#include <stdint.h>
uint16_t crc16(const uint8_t *p,unsigned n){uint16_t c=0xffff;while(n--){c^=(uint16_t)*p++<<8;for(unsigned k=0;k<8;k++)c=(c&0x8000)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}return c;}
