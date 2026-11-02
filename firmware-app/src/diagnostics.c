#include "app.h"
#include <stdio.h>
#include <string.h>
static ADC_HandleTypeDef adc[2];
static bool adc_ready, imu_on;
static bool adc_setup(void){
 if(adc_ready)return true;
 __HAL_RCC_ADC12_CLK_ENABLE();
 GPIO_InitTypeDef g={0};g.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;g.Mode=GPIO_MODE_ANALOG;g.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOA,&g);
 for(unsigned i=0;i<2;i++){
  adc[i].Instance=i?ADC2:ADC1;adc[i].Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4;adc[i].Init.Resolution=ADC_RESOLUTION_12B;
  adc[i].Init.DataAlign=ADC_DATAALIGN_RIGHT;adc[i].Init.ScanConvMode=ADC_SCAN_DISABLE;adc[i].Init.EOCSelection=ADC_EOC_SINGLE_CONV;
  adc[i].Init.NbrOfConversion=1;adc[i].Init.ExternalTrigConv=ADC_SOFTWARE_START;adc[i].Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;
  adc[i].Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;
  if(HAL_ADC_Init(&adc[i])!=HAL_OK || HAL_ADCEx_Calibration_Start(&adc[i],ADC_SINGLE_ENDED)!=HAL_OK)return false;
 }
 ADC12_COMMON->CCR|=ADC_CCR_VREFEN;HAL_Delay(1);
 adc_ready=true;return true;
}
static bool sample(unsigned unit,uint32_t channel,uint16_t *out){
 ADC_ChannelConfTypeDef c={0};c.Channel=channel;c.Rank=ADC_REGULAR_RANK_1;c.SamplingTime=ADC_SAMPLETIME_247CYCLES_5;c.SingleDiff=ADC_SINGLE_ENDED;c.OffsetNumber=ADC_OFFSET_NONE;
 if(HAL_ADC_ConfigChannel(&adc[unit],&c)!=HAL_OK || HAL_ADC_Start(&adc[unit])!=HAL_OK)return false;
 if(HAL_ADC_PollForConversion(&adc[unit],2)!=HAL_OK){HAL_ADC_Stop(&adc[unit]);return false;}
 *out=HAL_ADC_GetValue(&adc[unit]);return HAL_ADC_Stop(&adc[unit])==HAL_OK;
}
bool currents_read(uint16_t r[3],uint32_t *mv){
 if(!sensor_power || !adc_setup())return false;
 uint16_t ref;
 if(!sample(0,ADC_CHANNEL_VREFINT,&ref)||ref==0)return false;
 *mv=__HAL_ADC_CALC_VREFANALOG_VOLTAGE(ref,ADC_RESOLUTION_12B);
 if(*mv<2900||*mv>3600)return false;
 return sample(0,ADC_CHANNEL_1,&r[0])&&sample(0,ADC_CHANNEL_2,&r[1])&&sample(1,ADC_CHANNEL_17,&r[2]);
}
void diagnostics_off(void){
 for(unsigned i=0;i<2;i++)if(adc[i].Instance){HAL_ADC_Stop(&adc[i]);HAL_ADC_DeInit(&adc[i]);adc[i].Instance=0;}
 adc_ready=false;imu_on=false;
}
/* GPIO I2C deliberately follows the board's PA9=SCL, PA8=SDA wiring.
   Open drain, external 4.7k pullups. Every SCL release checks stretch timeout. */
static void delay_us(unsigned n){uint32_t t=DWT->CYCCNT,cycles=(SystemCoreClock/1000000)*n;while((uint32_t)(DWT->CYCCNT-t)<cycles){} }
static void sda(bool h){GPIOA->BSRR=h?GPIO_PIN_8:(GPIO_PIN_8<<16);delay_us(2);}
static bool scl(bool h){GPIOA->BSRR=h?GPIO_PIN_9:(GPIO_PIN_9<<16);uint32_t t=DWT->CYCCNT;while(h&&!(GPIOA->IDR&GPIO_PIN_9)){if((uint32_t)(DWT->CYCCNT-t)>SystemCoreClock/1000)return false;}delay_us(2);return true;}
static void bus_init(void){CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;GPIOA->BSRR=GPIO_PIN_8|GPIO_PIN_9;GPIO_InitTypeDef g={0};g.Pin=GPIO_PIN_8|GPIO_PIN_9;g.Mode=GPIO_MODE_OUTPUT_OD;g.Pull=GPIO_NOPULL;g.Speed=GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(GPIOA,&g);}
static bool start(void){sda(true);if(!scl(true)||!(GPIOA->IDR&GPIO_PIN_8))return false;sda(false);return scl(false);}
static void stop(void){scl(false);sda(false);scl(true);sda(true);}
static bool put(uint8_t b){for(unsigned k=0;k<8;k++){sda(b&0x80);if(!scl(true))return false;scl(false);b<<=1;}sda(true);if(!scl(true))return false;bool ack=!(GPIOA->IDR&GPIO_PIN_8);scl(false);return ack;}
static bool get(uint8_t *b,bool ack){*b=0;sda(true);for(unsigned k=0;k<8;k++){if(!scl(true))return false;*b=(*b<<1)|!!(GPIOA->IDR&GPIO_PIN_8);scl(false);}sda(!ack);if(!scl(true))return false;scl(false);sda(true);return true;}
static bool reg_read(uint8_t reg,uint16_t *v,unsigned count){
 uint8_t bytes[24];unsigned n=2+2*count;if(n>sizeof(bytes))return false;
 bool ok=start()&&put(0x68<<1)&&put(reg)&&start()&&put((0x68<<1)|1);
 for(unsigned i=0;ok&&i<n;i++)ok=get(&bytes[i],i+1<n);
 stop();if(!ok)return false;
 for(unsigned i=0;i<count;i++)v[i]=bytes[2+2*i]|((uint16_t)bytes[3+2*i]<<8);
 return true;
}
static bool reg_write(uint8_t reg,uint16_t value){bool ok=start()&&put(0x68<<1)&&put(reg)&&put(value&255)&&put(value>>8);stop();delay_us(2);return ok;}
bool imu_command(const char *op,char *out,unsigned size){
 uint16_t id=0,error=0,conf[2],data[9],status=0;
 if(!sensor_power){snprintf(out,size,"ERR turn sensor power on first\n");return false;}
 bus_init();
 if(!reg_read(0,&id,1)||(id&255)!=0x43||!reg_read(1,&error,1)||error){snprintf(out,size,"ERR BMI323 id=%04x error=%04x; check I2C and power\n",id,error);return false;}
 if(!strcmp(op,"on")){
  if(!reg_write(0x20,0x7028)||!reg_write(0x21,0x7048))goto fail;
  HAL_Delay(80);watchdog_feed();
  if(!reg_read(0x20,conf,2)||conf[0]!=0x7028||conf[1]!=0x7048||!reg_read(1,&error,1)||error)goto fail;
  imu_on=true;snprintf(out,size,"IMU on=1 id=%04x accel_range_g=8 gyro_range_dps=2000 rate=100\n",id);return true;
 }
 if(!strcmp(op,"off")){if(!reg_write(0x20,0x0028)||!reg_write(0x21,0x0048))goto fail;imu_on=false;snprintf(out,size,"IMU on=0 id=%04x\n",id);return true;}
 if(!strcmp(op,"probe")){snprintf(out,size,"IMU on=%u id=%04x error=%04x\n",imu_on,id,error);return true;}
 if(strcmp(op,"read")||!imu_on){snprintf(out,size,"ERR use imu on, then imu read\n");return false;}
 if(!reg_read(2,&status,1)||(status&0xc0)!=0xc0){snprintf(out,size,"ERR IMU fresh sample not ready; retry\n");return false;}
 if(!reg_read(3,data,9))goto fail;
 for(unsigned i=0;i<7;i++)if(data[i]==0x8000)goto fail;
 snprintf(out,size,"IMU accel=%d,%d,%d gyro=%d,%d,%d temp_raw=%d time=%lu\n",(int16_t)data[0],(int16_t)data[1],(int16_t)data[2],(int16_t)data[3],(int16_t)data[4],(int16_t)data[5],(int16_t)data[6],(unsigned long)(data[7]|((uint32_t)data[8]<<16)));return true;
 fail:snprintf(out,size,"ERR IMU communication/configuration/sample invalid\n");return false;
}
