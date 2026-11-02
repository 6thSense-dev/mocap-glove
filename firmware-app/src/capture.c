#include "app.h"
#include <string.h>
#include <stdio.h>
#include "lockin.h"
#define N 256
static uint8_t frames[5][N][12];
static uint8_t zero[12];
static DMA_HandleTypeDef dma_rx[2],dma_tx[2];
static uint8_t list[2][3],cnt[2];
static volatile uint16_t samples[2];
static volatile uint8_t busy[2],which[2],active,failed;
static bool dma_ready=false;
static uint8_t selected;
static unsigned sample_target=N;
static volatile unsigned capture_fault,edge_count[2];
static unsigned elapsed_ms,bad_sensor,bad_sample,drdy_levels;
static uint32_t spi_errors[2];
static volatile uint32_t phase_ticks[2];
/* Faults preserve the first failing stage, never accept partial or bad-CRC data. */
static void capture_fail(unsigned reason){if(!capture_fault)capture_fault=reason;failed=1;active=0;GPIOB->BRR=GPIO_PIN_2;}
void capture_report(char *out,unsigned size){
 const char *names[]={"none","not_ready","drive_unavailable","dma_start","spi_error","drdy_overrun","source_start","settling_read","source_guard","usb_stop","timeout","crc"};
 snprintf(out,size,"ERR capture reason=%s samples=%u,%u edges=%u,%u elapsed_ms=%u drdy=%u bad_sensor=%u bad_sample=%u spi_error=%lu,%lu\n",names[capture_fault<12?capture_fault:0],samples[0],samples[1],edge_count[0],edge_count[1],elapsed_ms,drdy_levels,bad_sensor,bad_sample,(unsigned long)spi_errors[0],(unsigned long)spi_errors[1]);
}
static GPIO_TypeDef *const port[5]={GPIOC,GPIOB,GPIOD,GPIOB,GPIOB};
static const uint16_t pin[5]={GPIO_PIN_11,GPIO_PIN_7,GPIO_PIN_2,GPIO_PIN_0,GPIO_PIN_1};
static void next(unsigned b){unsigned i=list[b][which[b]];GPIOC->BRR=1u<<(6+i);if(HAL_SPI_TransmitReceive_DMA(&spi[b],zero,frames[i][samples[b]],12)!=HAL_OK){GPIOC->BSRR=1u<<(6+i);capture_fail(3);}}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *s){unsigned b=s==&spi[1];GPIOC->BSRR=1u<<(6+list[b][which[b]]);if(!active){busy[b]=0;return;}if(++which[b]<cnt[b])next(b);else{samples[b]++;busy[b]=0;}}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *s){(void)s;capture_fail(4);}
void capture_irq(unsigned b){if(!active || !cnt[b] || samples[b]>=sample_target)return;edge_count[b]++;if(busy[b]){capture_fail(5);return;}if(samples[b]==0)phase_ticks[b]=source_phase_ticks();busy[b]=1;which[b]=0;next(b);}
void HAL_GPIO_EXTI_Callback(uint16_t p){for(unsigned b=0;b<2;b++)if(cnt[b] && pin[list[b][0]]==p)capture_irq(b);}
void EXTI0_IRQHandler(void){HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);}
void EXTI1_IRQHandler(void){HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);}
void EXTI2_IRQHandler(void){HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);}
void EXTI9_5_IRQHandler(void){HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);}
void EXTI15_10_IRQHandler(void){HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);}
void DMA1_Channel1_IRQHandler(void){HAL_DMA_IRQHandler(&dma_rx[0]);}
void DMA1_Channel2_IRQHandler(void){HAL_DMA_IRQHandler(&dma_tx[0]);}
void DMA1_Channel3_IRQHandler(void){HAL_DMA_IRQHandler(&dma_rx[1]);}
void DMA1_Channel4_IRQHandler(void){HAL_DMA_IRQHandler(&dma_tx[1]);}
static void dma_init(void){
 __HAL_RCC_DMAMUX1_CLK_ENABLE();__HAL_RCC_DMA1_CLK_ENABLE();
 for(unsigned b=0;b<2;b++)for(unsigned t=0;t<2;t++){
  DMA_HandleTypeDef *d=t?&dma_tx[b]:&dma_rx[b];d->Instance=b?(t?DMA1_Channel4:DMA1_Channel3):(t?DMA1_Channel2:DMA1_Channel1);
  d->Init.Request=b?(t?DMA_REQUEST_SPI2_TX:DMA_REQUEST_SPI2_RX):(t?DMA_REQUEST_SPI1_TX:DMA_REQUEST_SPI1_RX);d->Init.Direction=t?DMA_MEMORY_TO_PERIPH:DMA_PERIPH_TO_MEMORY;d->Init.PeriphInc=DMA_PINC_DISABLE;d->Init.MemInc=DMA_MINC_ENABLE;d->Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;d->Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;d->Init.Mode=DMA_NORMAL;d->Init.Priority=DMA_PRIORITY_VERY_HIGH;if(HAL_DMA_Init(d)!=HAL_OK)Error_Handler();
 }
 __HAL_LINKDMA(&spi[0],hdmarx,dma_rx[0]);__HAL_LINKDMA(&spi[0],hdmatx,dma_tx[0]);__HAL_LINKDMA(&spi[1],hdmarx,dma_rx[1]);__HAL_LINKDMA(&spi[1],hdmatx,dma_tx[1]);
 for(int irq=DMA1_Channel1_IRQn;irq<=DMA1_Channel4_IRQn;irq++){HAL_NVIC_SetPriority(irq,1,0);HAL_NVIC_EnableIRQ(irq);}dma_ready=true;
}
void capture_stop(void){active=0;IRQn_Type q[]={EXTI0_IRQn,EXTI1_IRQn,EXTI2_IRQn,EXTI9_5_IRQn,EXTI15_10_IRQn};for(unsigned k=0;k<5;k++)HAL_NVIC_DisableIRQ(q[k]);if(dma_ready && sensor_power){HAL_SPI_Abort(&spi[0]);HAL_SPI_Abort(&spi[1]);}GPIOC->BSRR=0x7c0;busy[0]=busy[1]=0;}
static bool capture_count(uint8_t mask,bool drive,unsigned count){
 sample_target=count;
 capture_fault=0;spi_errors[0]=spi_errors[1]=0;elapsed_ms=0;bad_sensor=0;bad_sample=0;drdy_levels=0;edge_count[0]=edge_count[1]=0;samples[0]=samples[1]=0;
 if(!sensor_power || !mask || (mask&~found_mask) || usb_stop_requested){capture_fail(1);return false;}
 if(drive && !ALLOW_COIL_DRIVE){capture_fail(2);return false;}
 phase_ticks[0]=phase_ticks[1]=UINT32_MAX;
 selected=mask;cnt[0]=cnt[1]=0;samples[0]=samples[1]=0;busy[0]=busy[1]=0;failed=0;
 for(unsigned i=0;i<5;i++)if(mask&(1u<<i))list[i>=3][cnt[i>=3]++]=i;
 dma_init();
 for(unsigned i=0;i<5;i++){GPIO_InitTypeDef g={0};g.Pin=pin[i];g.Mode=GPIO_MODE_INPUT;g.Pull=GPIO_NOPULL;HAL_GPIO_Init(port[i],&g);}
 for(unsigned b=0;b<2;b++)if(cnt[b]){unsigned i=list[b][0];GPIO_InitTypeDef g={0};g.Pin=pin[i];g.Mode=GPIO_MODE_IT_FALLING;g.Pull=GPIO_NOPULL;HAL_GPIO_Init(port[i],&g);__HAL_GPIO_EXTI_CLEAR_IT(pin[i]);}
 if(drive&&!source_start()){capture_fail(6);capture_stop();return false;}
 /* SYNC clears the ADC FIFO. Arm capture immediately, before fresh data can
    refill it. Waiting 3 ms here fills the two-entry FIFO again and can create
    a second DRDY edge during the very first DMA transfer (TI section 8.5.1.9.1). */
 GPIOB->BRR=GPIO_PIN_6;for(volatile unsigned n=0;n<100;n++)__NOP();
 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_11);
 active=1;IRQn_Type q[]={EXTI0_IRQn,EXTI1_IRQn,EXTI2_IRQn,EXTI9_5_IRQn,EXTI15_10_IRQn};for(unsigned k=0;k<5;k++){HAL_NVIC_SetPriority(q[k],0,0);HAL_NVIC_ClearPendingIRQ(q[k]);HAL_NVIC_EnableIRQ(q[k]);}
 uint32_t start=HAL_GetTick();GPIOB->BSRR=GPIO_PIN_6;
 while(active && ((cnt[0]&&samples[0]<sample_target)||(cnt[1]&&samples[1]<sample_target))){watchdog_feed();if(drive)source_poll();if(drive&&!source_ok()){capture_fail(8);break;}if(usb_stop_requested){capture_fail(9);break;}if(HAL_GetTick()-start>40){capture_fail(10);break;}}
 elapsed_ms=HAL_GetTick()-start;for(unsigned i=0;i<5;i++)if(HAL_GPIO_ReadPin(port[i],pin[i])==GPIO_PIN_SET)drdy_levels|=1u<<i;
 spi_errors[0]=spi[0].ErrorCode;spi_errors[1]=spi[1].ErrorCode;
 source_stop();capture_stop();
 if(failed || usb_stop_requested)return false;
 for(unsigned i=0;i<5;i++)if(mask&(1u<<i))for(unsigned n=0;n<sample_target;n++)if(crc16(frames[i][n],10)!=((uint16_t)frames[i][n][10]<<8|frames[i][n][11])){capture_fault=11;bad_sensor=i+1;bad_sample=n;return false;}
 return true;
}
void capture_dump(void){char msg[160];int n=snprintf(msg,sizeof(msg),"CAPTURE mask=%u samples=256 rate=24000 format=ADS16_BE12 phase_ticks=%lu,%lu\n",selected,(unsigned long)phase_ticks[0],(unsigned long)phase_ticks[1]);if(!usb_send(msg,n))return;
 for(unsigned i=0;i<5;i++)if(selected&(1u<<i)){n=snprintf(msg,sizeof(msg),"SENSOR %u bytes=3072\n",i+1);if(!usb_send(msg,n)||!usb_send(frames[i],sizeof(frames[i])))return;usb_send("\n",1);}
 usb_send("END\n",4);
}

bool capture(uint8_t mask,bool drive){return capture_count(mask,drive,256);}
bool capture_stream(uint8_t mask){return capture_count(mask,true,128);}
void capture_stream_dump(void){
 char msg[2400];int n=snprintf(msg,sizeof(msg),"TRACK mask=%u samples=128 rate=24000 skip=64 phase_ticks=%lu,%lu imu=1\n",selected,(unsigned long)phase_ticks[0],(unsigned long)phase_ticks[1]);unsigned used=n;
 for(unsigned i=0;i<5;i++)if(selected&(1u<<i)){
  int16_t data[64][3];unsigned clipped=0;
  for(unsigned k=0;k<64;k++)for(unsigned c=0;c<3;c++){data[k][c]=(int16_t)(((unsigned)frames[i][k+64][2+c*2]<<8)|frames[i][k+64][3+c*2]);if(data[k][c]>=32760||data[k][c]<=-32760)clipped=1;}
  int32_t values[3][3][2];lockin64(data,values);
  n=snprintf(msg+used,sizeof(msg)-used,"SENSOR %u clipped=%u\n",i+1,clipped);if(n<0||(unsigned)n>=sizeof(msg)-used)return;used+=n;
  for(unsigned t=0;t<3;t++){n=snprintf(msg+used,sizeof(msg)-used,"TONE %u %ld %ld %ld %ld %ld %ld\n",4500+1125*t,(long)values[t][0][0],(long)values[t][0][1],(long)values[t][1][0],(long)values[t][1][1],(long)values[t][2][0],(long)values[t][2][1]);if(n<0||(unsigned)n>=sizeof(msg)-used)return;used+=n;}
 }
 char imu[256];
 if(!imu_command("read",imu,sizeof(imu))){usb_send(imu,strlen(imu));return;}
 unsigned imu_len=strlen(imu);if(used+imu_len+4>sizeof(msg))return;memcpy(msg+used,imu,imu_len);used+=imu_len;
 if(used+4>sizeof(msg))return;
 memcpy(msg+used,"END\n",4);usb_send(msg,used+4);
}
