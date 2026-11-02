#include "app.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
extern void board_init(void);
static void say(const char *s){usb_send(s,strlen(s));}
int main(void){
 board_init();usb_init();char line[128],reply[256];
 for(;;){
  watchdog_feed();
  if(usb_stop_requested){board_idle();usb_stop_requested=0;}
  if(!usb_line(line,sizeof(line)))continue;
  if(!strcmp(line,"status")){snprintf(reply,sizeof(reply),"OK firmware=%s core=%lu sensors=%u found=%u coil_drive_build=%u\n",VERSION,(unsigned long)SystemCoreClock,sensor_power,found_mask,ALLOW_COIL_DRIVE);say(reply);}
  else if(!strcmp(line,"currents")){uint16_t r[3];uint32_t mv;if(currents_read(r,&mv)){snprintf(reply,sizeof(reply),"CURRENT raw=%u,%u,%u vdda_mv=%lu gain=20 shunt_mohm=200\n",r[0],r[1],r[2],(unsigned long)mv);say(reply);}else say("ERR power on and check current ADC\n");}
  else if(!strncmp(line,"imu ",4)){imu_command(line+4,reply,sizeof(reply));say(reply);}
  else if(!strcmp(line,"stop")){board_idle();say("OK all outputs off\n");}
  else if(!strncmp(line,"coil_test ",10)){unsigned mask,ms;char extra;if(sscanf(line+10,"%u %u %c",&mask,&ms,&extra)!=2){say("ERR expected coil_test axes_mask duration_ms\n");}else if(!source_config(mask,ms) || !source_start()){source_report(reply,sizeof(reply));say(reply);}else{uint32_t start=HAL_GetTick();while(HAL_GetTick()-start<ms && source_ok() && !usb_stop_requested){watchdog_feed();source_poll();}source_stop();source_report(reply,sizeof(reply));say(reply);}}
  else if(!strcmp(line,"help"))say("status | sensors on | sensors off | probe | read 1..5 | capture 1..31 | capture_tx 1..31 | currents | imu probe/on/read/off | coil_test axes_mask duration_ms | capture_tx ports_mask axes_mask | stop | ! | reset\n");
  else if(!strcmp(line,"sensors on")){if(board_sensors_on())say("OK sensing rail on; HSE PLL 122880000; ADC clock 6144000\n");else{board_idle();say("ERR sensing clock or USB; rail disabled\n");}}
  else if(!strcmp(line,"sensors off")){board_idle();say("OK sensing and source off\n");}
  else if(!strcmp(line,"probe")){ads_probe();snprintf(reply,sizeof(reply),"OK found=%u; bits 0..4 are J1..J5; all found devices CRC and register readback checked\n",found_mask);say(reply);}
  else if(!strncmp(line,"read ",5)){unsigned i=atoi(line+5);uint8_t r[12];if(i<1||i>5||!ads_read(i-1,r))say("ERR ADC read or CRC\n");else{snprintf(reply,sizeof(reply),"ADC %u status=%04x ch=%d,%d,%d,%d\n",i,r[0]*256+r[1],(int16_t)(r[2]*256+r[3]),(int16_t)(r[4]*256+r[5]),(int16_t)(r[6]*256+r[7]),(int16_t)(r[8]*256+r[9]));say(reply);}}
  else if(!strncmp(line,"capture_stream ",15)){unsigned m;char extra;if(sscanf(line+15,"%u %c",&m,&extra)!=1||m<1||m>31)say("ERR stream receiver mask 1..31\n");else if(!source_config_stream()){source_report(reply,sizeof(reply));say(reply);}else if(capture_stream(m))capture_stream_dump();else{capture_report(reply,sizeof(reply));say(reply);}}
  else if(!strncmp(line,"capture_track ",14)){unsigned m;char extra;if(sscanf(line+14,"%u %c",&m,&extra)!=1 || m<1 || m>31)say("ERR tracking capture needs receiver mask 1..31\n");else if(!source_config_track()){source_report(reply,sizeof(reply));say(reply);}else if(capture(m,true))capture_dump();else {capture_report(reply,sizeof(reply));say(reply);}}
  else if(!strncmp(line,"capture_live ",13)){unsigned m;char extra;if(sscanf(line+13,"%u %c",&m,&extra)!=1 || m<1 || m>31 || (m&(m-1)))say("ERR live capture needs exactly one receiver\n");else if(!source_config_live()){source_report(reply,sizeof(reply));say(reply);}else if(capture(m,true))capture_dump();else {capture_report(reply,sizeof(reply));say(reply);}}
  else if(!strncmp(line,"capture ",8)||!strncmp(line,"capture_tx ",11)){bool drive=line[7]=='_';unsigned m=0,axes=0;char extra;int count=drive?sscanf(line+11,"%u %u %c",&m,&axes,&extra):sscanf(line+8,"%u %c",&m,&extra);if(count!=(drive?2:1)||m<1||m>31)say("ERR invalid capture arguments\n");else if(drive&&!source_config(axes,50)){source_report(reply,sizeof(reply));say(reply);}else if(capture(m,drive))capture_dump();else {capture_report(reply,sizeof(reply));say(reply);}}
  else if(!strcmp(line,"reset")){board_idle();NVIC_SystemReset();}
  else say("ERR unknown command; use help\n");
 }
}
