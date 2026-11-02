#include "app.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
GPIO_TypeDef gb,gc;TIM_TypeDef timer;DMA_Channel_TypeDef channel;DMA_TypeDef dm;
bool sensor_power=true;uint32_t SystemCoreClock=122880000;volatile uint32_t tx_deadline;volatile uint8_t usb_stop_requested;
static uint32_t now=10000;static bool adc_ok=true,advance=true;static uint16_t raw[3]={2048,2048,2048};
bool usb_ready(void){return true;}uint32_t HAL_GetTick(void){return now;}
void HAL_Delay(unsigned ms){now+=ms;if(advance)channel.CNDTR-=768;}
int HAL_DMA_Init(DMA_HandleTypeDef *d){return 0;}int HAL_DMA_Abort(DMA_HandleTypeDef*d){return 0;}
int HAL_DMA_Start(DMA_HandleTypeDef*d,uint32_t a,uint32_t b,unsigned n){channel.CNDTR=n;return 0;}
bool currents_read(uint16_t*r,uint32_t*mv){memcpy(r,raw,sizeof(raw));*mv=3300;return adc_ok;}
static void reset(void){source_stop();now+=2100;adc_ok=true;advance=true;usb_stop_requested=0;for(int i=0;i<3;i++)raw[i]=2048;gb.BSRR=gb.BRR=0;}
static void expect(const char *text){char r[256];source_report(r,sizeof(r));assert(strstr(r,text));}
int main(void){
 reset();assert(!source_config(0,10));assert(!source_config(8,10));assert(!source_config(1,101));assert(!source_config(1,0));
 reset();assert(source_config(1,10));assert(source_start());assert(gb.BSRR==GPIO_PIN_2);source_poll();now+=10;source_tick();assert(gb.BRR==GPIO_PIN_2);assert(tx_deadline==0);source_stop();assert(!source_config(1,10));expect("cooldown");
 reset();raw[0]=0;assert(source_config(1,10));assert(!source_start());expect("current_zero_out_of_range");assert(gb.BSRR==0);
 reset();adc_ok=false;assert(source_config(1,10));assert(!source_start());expect("current_adc");
 reset();advance=false;assert(source_config(1,10));assert(!source_start());expect("waveform_not_running");assert(gb.BSRR==0);
 reset();assert(source_config(1,100));assert(source_start());raw[0]=3000;source_poll();assert(!source_ok());assert(gb.BRR==GPIO_PIN_2);expect("current_limit");
 reset();assert(source_config(1,100));assert(source_start());raw[1]=2200;source_poll();assert(!source_ok());expect("current_limit");
 reset();assert(source_config(1,100));assert(source_start());now+=6;source_poll();assert(!source_ok());expect("current_monitor_late");
 reset();assert(source_config(1,100));assert(source_start());usb_stop_requested=1;source_stop();expect("usb_stop");assert(gb.BRR==GPIO_PIN_2);
 reset();assert(source_config_live());assert(source_start());now+=20;source_tick();assert(gb.BRR==GPIO_PIN_2);assert(!source_ok());expect("burst_timeout");source_stop();now+=79;assert(!source_config_live());now++;assert(source_config_live());assert(source_start());now+=10;source_stop();now+=80;assert(!source_config(4,10));now+=1920;assert(source_config(4,10));assert(source_start());source_stop();now+=80;assert(!source_config_live());expect("cooldown");
 reset();assert(source_config_live());assert(source_start());raw[2]=3000;source_poll();assert(!source_ok());expect("current_limit");
 reset();assert(source_phase_ticks()==UINT32_MAX);assert(source_config(1,50));assert(source_start());
 channel.CNDTR=4096;timer.CNT=32;assert(source_phase_ticks()==32);
 channel.CNDTR=4095;timer.CNT=40;assert(source_phase_ticks()==200);
 channel.CNDTR=1;timer.CNT=159;assert(source_phase_ticks()==655359);
 timer.CNT=0;assert(source_phase_ticks()==UINT32_MAX);
 timer.CNT=160;assert(source_phase_ticks()==UINT32_MAX);
 source_stop();assert(source_phase_ticks()==UINT32_MAX);
 reset();assert(source_config_track());assert(source_start());now+=20;source_tick();assert(!source_ok());expect("burst_timeout");source_stop();now+=299;assert(!source_config_track());now++;assert(source_config_track());assert(source_start());source_stop();now+=80;assert(!source_config_live());now+=1920;assert(source_config_live());assert(source_start());source_stop();now+=300;assert(!source_config_track());now+=1700;assert(source_config_track());
 for(unsigned axis=0;axis<3;axis++){reset();assert(source_config_track());assert(source_start());raw[axis]=3000;source_poll();assert(!source_ok());expect("current_limit");}
 reset();assert(source_config_stream());assert(source_start());now+=12;source_tick();assert(!source_ok());expect("burst_timeout");source_stop();now+=3;assert(!source_config_stream());now++;assert(source_config_stream());assert(source_start());source_stop();now+=300;assert(!source_config_track());now+=1700;assert(source_config_track());
 for(unsigned axis=0;axis<3;axis++){reset();assert(source_config_stream());assert(source_start());raw[axis]=3000;source_poll();assert(!source_ok());expect("current_limit");}
 puts("PASS: parameter bounds, timeout, cooldown, bad zero, ADC failure, DMA stall, selected/unselected overcurrent, stale monitor, USB stop");
}
