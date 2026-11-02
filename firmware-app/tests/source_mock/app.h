#pragma once
#include <stdint.h>
#include <stdbool.h>
#define ALLOW_COIL_DRIVE 1
#define GPIO_PIN_2 4
#define TIM_EGR_UG 1
#define TIM_DIER_UDE 1
#define TIM_CR1_CEN 1
#define DMA_REQUEST_TIM6_UP 1
#define DMA_MEMORY_TO_PERIPH 1
#define DMA_PINC_DISABLE 0
#define DMA_MINC_ENABLE 1
#define DMA_PDATAALIGN_WORD 2
#define DMA_MDATAALIGN_WORD 2
#define DMA_CIRCULAR 1
#define DMA_PRIORITY_HIGH 2
#define DMA_ISR_TEIF5 8
#define HAL_OK 0
#define __HAL_RCC_DMA1_CLK_ENABLE()
#define __HAL_RCC_DMAMUX1_CLK_ENABLE()
#define __HAL_RCC_TIM6_CLK_ENABLE()
#define __disable_irq()
#define __enable_irq()
typedef struct {volatile uint32_t BSRR,BRR;} GPIO_TypeDef;
typedef struct {volatile uint32_t DIER,CR1,PSC,ARR,EGR,SR,CNT;} TIM_TypeDef;
typedef struct {volatile uint32_t CNDTR;} DMA_Channel_TypeDef;
typedef struct {uint32_t ISR;} DMA_TypeDef;
typedef struct {unsigned Request,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority;} DMA_InitTypeDef;
typedef struct {DMA_Channel_TypeDef *Instance;DMA_InitTypeDef Init;} DMA_HandleTypeDef;
extern GPIO_TypeDef gb,gc;extern TIM_TypeDef timer;extern DMA_Channel_TypeDef channel;extern DMA_TypeDef dm;
#define GPIOB (&gb)
#define GPIOC (&gc)
#define TIM6 (&timer)
#define DMA1_Channel5 (&channel)
#define DMA1 (&dm)
extern bool sensor_power;extern uint32_t SystemCoreClock;extern volatile uint32_t tx_deadline;extern volatile uint8_t usb_stop_requested;
bool usb_ready(void);uint32_t HAL_GetTick(void);void HAL_Delay(unsigned ms);
int HAL_DMA_Init(DMA_HandleTypeDef*);int HAL_DMA_Abort(DMA_HandleTypeDef*);int HAL_DMA_Start(DMA_HandleTypeDef*,uint32_t,uint32_t,unsigned);
bool currents_read(uint16_t*,uint32_t*);void source_stop(void);void source_tick(void);bool source_config(unsigned,unsigned);bool source_start(void);bool source_ok(void);void source_poll(void);void source_report(char*,unsigned);

bool source_config_live(void);
bool source_config_track(void);
bool source_config_stream(void);
bool capture_stream(uint8_t mask);
void capture_stream_dump(void);

uint32_t source_phase_ticks(void);
