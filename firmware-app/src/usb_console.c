#include "app.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include <string.h>
USBD_HandleTypeDef usb;
extern PCD_HandleTypeDef hpcd_USB_FS;
static uint8_t rx[64],tx[512],linecoding[7]={0x00,0xc2,0x01,0,0,0,8};
static char ring[256];
static volatile unsigned head,tail;
static volatile bool overflow;
static int8_t init(void){USBD_CDC_SetTxBuffer(&usb,tx,0);USBD_CDC_SetRxBuffer(&usb,rx);return USBD_OK;}
static int8_t deinit(void){app_usb_stop();return USBD_OK;}
static int8_t control(uint8_t c,uint8_t *p,uint16_t n){
 if(c==CDC_SET_LINE_CODING && n==7)memcpy(linecoding,p,7);
 if(c==CDC_GET_LINE_CODING && n==7)memcpy(p,linecoding,7);
 if(c==CDC_SET_CONTROL_LINE_STATE){USBD_SetupReqTypedef *r=(void*)p;if(!(r->wValue&1))app_usb_stop();}
 return USBD_OK;
}
static int8_t receive(uint8_t *p,uint32_t *n){
 for(unsigned i=0;i<*n;i++){if(p[i]=='!'){app_usb_stop();continue;}unsigned next=(head+1)%sizeof(ring);if(next==tail){overflow=true;app_usb_stop();break;}ring[head]=p[i];__DMB();head=next;}
 USBD_CDC_SetRxBuffer(&usb,rx);USBD_CDC_ReceivePacket(&usb);return USBD_OK;
}
static int8_t sent(uint8_t *p,uint32_t *n,uint8_t ep){(void)p;(void)n;(void)ep;return USBD_OK;}
static USBD_CDC_ItfTypeDef itf={init,deinit,control,receive,sent};
void usb_init(void){if(USBD_Init(&usb,&CDC_Desc,0)!=USBD_OK || USBD_RegisterClass(&usb,&USBD_CDC)!=USBD_OK || USBD_CDC_RegisterInterface(&usb,&itf)!=USBD_OK || USBD_Start(&usb)!=USBD_OK)Error_Handler();}
bool usb_ready(void){return usb.dev_state==USBD_STATE_CONFIGURED;}
bool usb_send(const void *data,unsigned length){
 const uint8_t *p=data;
 while(length){unsigned n=length>sizeof(tx)?sizeof(tx):length;uint32_t start=HAL_GetTick();
  for(;;){if(!usb_ready())return false;USBD_CDC_HandleTypeDef *c=usb.pClassData;if(c && !c->TxState)break;if(HAL_GetTick()-start>1000)return false;}
  memcpy(tx,p,n);USBD_CDC_SetTxBuffer(&usb,tx,n);if(USBD_CDC_TransmitPacket(&usb)!=USBD_OK)return false;p+=n;length-=n;
 }
 return true;
}
bool usb_line(char *out,unsigned cap){
 static unsigned len;static char line[128];static bool discard;
 if(overflow){__disable_irq();tail=head;overflow=false;__enable_irq();len=0;discard=true;}
 while(tail!=head){char c=ring[tail];tail=(tail+1)%sizeof(ring);if(c=='\r')continue;
  if(c=='\n'){if(discard){discard=false;len=0;continue;}line[len]=0;if(len>=cap){len=0;continue;}memcpy(out,line,len+1);len=0;return true;}
  if(!discard){if(len+1<sizeof(line))line[len++]=c;else{discard=true;app_usb_stop();}}
 }return false;
}
void USB_LP_IRQHandler(void){HAL_PCD_IRQHandler(&hpcd_USB_FS);}
