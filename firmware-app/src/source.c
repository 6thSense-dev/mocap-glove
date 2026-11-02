#include "app.h"
#include "source_waveform.h"
#include <stdio.h>
static DMA_HandleTypeDef wave;
static uint32_t table[4096];
static bool ready,configured,has_run,fast_mode;
static volatile bool running;
static unsigned axes,duration,profile,last_profile;
static uint32_t last_stop,deadline,monitor_stamp;
static uint16_t baseline[3],low[3],high[3];
static uint32_t vdda,readings;
static const char *fault="not_run";
void source_stop(void){
 if(usb_stop_requested && running && fault[0]=='o')fault="usb_stop";
 GPIOB->BRR=GPIO_PIN_2;tx_deadline=0;
 if(running){last_stop=HAL_GetTick();has_run=true;last_profile=profile;}running=false;
 if(ready){TIM6->DIER=0;TIM6->CR1=0;HAL_DMA_Abort(&wave);GPIOC->BRR=0x3f;}
 configured=false;
}
/* ISR cutoff is independent of main-loop progress. IWDG additionally resets a
   stalled MCU. Neither replaces scope verification of the real output/current. */
void source_tick(void){if(running && (int32_t)(HAL_GetTick()-deadline)>=0){GPIOB->BRR=GPIO_PIN_2;tx_deadline=0;if(fast_mode)fault="burst_timeout";}}
bool source_ok(void){return running && fault[0]=='o' && !usb_stop_requested;}
static bool configure(unsigned mask,unsigned ms,unsigned mode){
 source_stop();fault="invalid_parameters";
 if(mask<1||mask>7||ms<1||ms>100)return false;
 fault="cooldown";if(has_run && HAL_GetTick()-last_stop<(mode && mode==last_profile?(mode==1?80u:mode==2?300u:4u):2000u))return false;
 fault="power_or_clock";if(!sensor_power||SystemCoreClock!=122880000||!usb_ready()||usb_stop_requested)return false;
 axes=mask;duration=ms;profile=mode;fast_mode=mode!=0;configured=true;fault="ok";return true;
}
bool source_config(unsigned mask,unsigned ms){return configure(mask,ms,0);}
/* Z-only live acquisition: <=20 ms on, >=80 ms off; never changes manual limits. */
bool source_config_live(void){return configure(4,20,1);}
/* All-axis tracking: <=20 ms on, >=300 ms off; mode transitions use 2 s. */
bool source_config_track(void){return configure(7,20,2);}
/* Compact streaming: 12 ms hard deadline and >=4 ms off, all axes. */
bool source_config_stream(void){return configure(7,12,3);}
bool source_start(void){
 if(!ALLOW_COIL_DRIVE||!configured)return false;
 configured=false;readings=0;fault="current_adc";
 if(!currents_read(baseline,&vdda))return false;
 for(unsigned a=0;a<3;a++){
  /* INA240 REF1=sensor rail, REF2=GND: nominal zero is half rail. */
  if(baseline[a]<1700||baseline[a]>2395){fault="current_zero_out_of_range";return false;}
  low[a]=high[a]=baseline[a];
 }
 unsigned bits=0;for(unsigned a=0;a<3;a++)if(axes&(1u<<a))bits|=3u<<(2*a);
 for(unsigned i=0;i<4096;i++){unsigned on=source_bsrr[i]&bits;table[i]=on|((0x3fu^on)<<16);}
 __HAL_RCC_DMA1_CLK_ENABLE();__HAL_RCC_DMAMUX1_CLK_ENABLE();__HAL_RCC_TIM6_CLK_ENABLE();
 wave.Instance=DMA1_Channel5;wave.Init.Request=DMA_REQUEST_TIM6_UP;wave.Init.Direction=DMA_MEMORY_TO_PERIPH;wave.Init.PeriphInc=DMA_PINC_DISABLE;wave.Init.MemInc=DMA_MINC_ENABLE;wave.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;wave.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;wave.Init.Mode=DMA_CIRCULAR;wave.Init.Priority=DMA_PRIORITY_HIGH;
 fault="dma_init";if(HAL_DMA_Init(&wave)!=HAL_OK)return false;ready=true;
 TIM6->CR1=0;TIM6->PSC=0;TIM6->ARR=159;TIM6->EGR=TIM_EGR_UG;TIM6->SR=0;
 fault="dma_start";if(HAL_DMA_Start(&wave,(uint32_t)table,(uint32_t)&GPIOC->BSRR,4096)!=HAL_OK){source_stop();return false;}
 TIM6->DIER=TIM_DIER_UDE;TIM6->CR1=TIM_CR1_CEN;
 uint32_t before=wave.Instance->CNDTR;HAL_Delay(1);
 if(wave.Instance->CNDTR==before||usb_stop_requested){fault="waveform_not_running";source_stop();return false;}
 fault="ok";monitor_stamp=HAL_GetTick();deadline=monitor_stamp+duration;
 /* Short IRQ critical section prevents USB stop between check and enable. */
 __disable_irq();
 if(usb_stop_requested){__enable_irq();source_stop();return false;}
 running=true;tx_deadline=deadline;GPIOB->BSRR=GPIO_PIN_2;__enable_irq();return true;
}
void source_poll(void){
 if(!source_ok())return;
 if(HAL_GetTick()-monitor_stamp>5){fault="current_monitor_late";source_stop();return;}
 uint16_t r[3];uint32_t mv;
 if(!currents_read(r,&mv)){fault="current_adc";source_stop();return;}
 monitor_stamp=HAL_GetTick();readings++;
 if(mv<2900||mv>3600){fault="supply";source_stop();return;}
 for(unsigned a=0;a<3;a++){
  if(r[a]<low[a])low[a]=r[a];
  if(r[a]>high[a])high[a]=r[a];
  int delta=(int)r[a]-baseline[a];if(delta<0)delta=-delta;
  /* 0.2 ohm x 20 V/V = 4 V/A. Threshold 125 mA; readback is sampled,
     not a certified peak limiter. 47-ohm hardware resistors also limit current. */
  unsigned limit=(axes&(1u<<a))?125:25;
  if((uint32_t)delta*mv>limit*4u*4095u){fault="current_limit";source_stop();return;}
 }
 if(DMA1->ISR&DMA_ISR_TEIF5){fault="dma_transfer";source_stop();}
}
void source_report(char *out,unsigned size){
 snprintf(out,size,"COIL result=%s axes=%u ms=%u samples=%lu vdda_mv=%lu zero=%u,%u,%u min=%u,%u,%u max=%u,%u,%u\n",fault,axes,duration,(unsigned long)readings,(unsigned long)vdda,baseline[0],baseline[1],baseline[2],low[0],low[1],low[2],high[0],high[1],high[2]);
}

/* Waveform position in 122.88 MHz timer ticks, modulo 4096*160.
   Timestamp at DRDY ISR entry; fixed ADC/filter/IRQ delay must be calibrated.
   Reject reads across a timer rollover or DMA update. Bounded in the ISR. */
uint32_t source_phase_ticks(void){
 if(!running || !ready)return UINT32_MAX;
 for(unsigned retry=0;retry<4;retry++){
  uint32_t n1=DMA1_Channel5->CNDTR;
  uint32_t t1=TIM6->CNT;
  uint32_t n2=DMA1_Channel5->CNDTR;
  uint32_t t2=TIM6->CNT;
  if(n1==n2 && n1<=4096 && t2>=t1 && t2<160 && t1>=16)
   return (((4096-n1)&4095)*160+(t1+t2)/2);
 }
 return UINT32_MAX;
}
