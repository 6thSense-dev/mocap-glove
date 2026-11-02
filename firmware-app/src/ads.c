#include "app.h"
#include <string.h>
uint8_t found_mask=0;
static uint8_t width[5]={3,3,3,3,3};

static bool frame(unsigned i,uint16_t command,uint16_t value,uint8_t *rx){
 if(i>=5||!sensor_power||usb_stop_requested)return false;
 uint8_t tx[18]={0};unsigned w=width[i];tx[0]=command>>8;tx[1]=command;tx[w]=value>>8;tx[w+1]=value;
 GPIOC->BRR=1u<<(6+i);HAL_StatusTypeDef r=HAL_SPI_TransmitReceive(&spi[i>=3],tx,rx,6*w,10);GPIOC->BSRR=1u<<(6+i);
 return r==HAL_OK && crc16(rx,5*w)==((uint16_t)rx[5*w]<<8|rx[5*w+1]);
}
static bool readreg(unsigned i,unsigned reg,uint16_t *v){uint8_t r[18];if(!frame(i,0xa000|(reg<<7),0,r))return false;if(!frame(i,0,0,r))return false;*v=(uint16_t)r[0]<<8|r[1];return true;}
static bool writereg(unsigned i,unsigned reg,uint16_t val){uint8_t r[18];uint16_t got;if(!frame(i,0x6000|(reg<<7),val,r))return false;if(reg==2)width[i]=2;return readreg(i,reg,&got)&&got==val;}
bool ads_probe(void){
 if(!sensor_power)return false;
 found_mask=0;GPIOB->BRR=GPIO_PIN_6;HAL_Delay(2);GPIOB->BSRR=GPIO_PIN_6;HAL_Delay(5); /* >2048 ADC clocks resets all devices. */
 for(unsigned i=0;i<5;i++){
  width[i]=3;uint16_t id;
  if(readreg(i,0,&id)&&(id&0xff00)==0x2400 && writereg(i,4,0x0555) && writereg(i,3,0x0f02) && writereg(i,2,0x0010))found_mask|=1u<<i;
 }
 /* Short SYNC strobe: above two and below 2048 ADC clocks at 122.88 MHz. */
 GPIOB->BRR=GPIO_PIN_6;for(volatile unsigned n=0;n<100;n++)__NOP();GPIOB->BSRR=GPIO_PIN_6;HAL_Delay(5);
 /* Drain both FIFO entries so next DRDY corresponds to fresh conversion. */
 for(unsigned i=0;i<5;i++)if(found_mask&(1u<<i)){uint8_t r[18];frame(i,0,0,r);frame(i,0,0,r);}
 return found_mask!=0;
}
bool ads_read(unsigned i,uint8_t rx[12]){return i<5 && (found_mask&(1u<<i)) && frame(i,0,0,rx);}
