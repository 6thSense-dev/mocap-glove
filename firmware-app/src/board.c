#include "app.h"
SPI_HandleTypeDef spi[2];
bool sensor_power=false;
volatile uint8_t usb_stop_requested=0;
volatile uint32_t tx_deadline=0;
static TIM_HandleTypeDef adc_clock;
static IWDG_HandleTypeDef watchdog;
void watchdog_feed(void){if(watchdog.Instance)HAL_IWDG_Refresh(&watchdog);}
static void pins(GPIO_TypeDef *p,uint32_t mask,uint32_t mode,uint32_t af){GPIO_InitTypeDef g={0};g.Pin=mask;g.Mode=mode;g.Pull=GPIO_NOPULL;g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;g.Alternate=af;HAL_GPIO_Init(p,&g);}
void app_usb_stop(void){GPIOB->BRR=GPIO_PIN_2;tx_deadline=0;usb_stop_requested=1;}
void Error_Handler(void){__HAL_RCC_GPIOB_CLK_ENABLE();GPIOB->BRR=GPIO_PIN_2;__disable_irq();while(1){} }
void SysTick_Handler(void){HAL_IncTick();source_tick();if(tx_deadline && (int32_t)(HAL_GetTick()-tx_deadline)>=0)app_usb_stop();}
void HardFault_Handler(void){Error_Handler();}
void MemManage_Handler(void){Error_Handler();}
void BusFault_Handler(void){Error_Handler();}
void UsageFault_Handler(void){Error_Handler();}
static bool use_hsi(void){
 RCC_ClkInitTypeDef c={0};c.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;c.SYSCLKSource=RCC_SYSCLKSOURCE_HSI;c.AHBCLKDivider=RCC_SYSCLK_DIV1;c.APB1CLKDivider=RCC_HCLK_DIV1;c.APB2CLKDivider=RCC_HCLK_DIV1;
 return HAL_RCC_ClockConfig(&c,FLASH_LATENCY_0)==HAL_OK;
}
void board_idle(void){
 source_stop();capture_stop();diagnostics_off();
 if(!use_hsi())Error_Handler();
 RCC_OscInitTypeDef o={0};o.OscillatorType=RCC_OSCILLATORTYPE_HSE;o.HSEState=RCC_HSE_OFF;o.PLL.PLLState=RCC_PLL_OFF;if(HAL_RCC_OscConfig(&o)!=HAL_OK)Error_Handler();
 __HAL_RCC_TIM4_CLK_DISABLE();__HAL_RCC_SPI1_CLK_DISABLE();__HAL_RCC_SPI2_CLK_DISABLE();
 pins(GPIOA,GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_MODE_ANALOG,0);
 pins(GPIOB,GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,GPIO_MODE_ANALOG,0);
 pins(GPIOC,0x7ff,GPIO_MODE_ANALOG,0);
 GPIOB->BRR=GPIO_PIN_5;sensor_power=false;found_mask=0;
}
void board_init(void){
 HAL_Init();__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_GPIOF_CLK_ENABLE();__HAL_RCC_PWR_CLK_ENABLE();
 GPIOB->BRR=GPIO_PIN_2|GPIO_PIN_5;pins(GPIOB,GPIO_PIN_2|GPIO_PIN_5,GPIO_MODE_OUTPUT_PP,0);
 if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)!=HAL_OK)Error_Handler();
 RCC_OscInitTypeDef o={0};o.OscillatorType=RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;o.HSIState=RCC_HSI_ON;o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;o.HSI48State=RCC_HSI48_ON;o.PLL.PLLState=RCC_PLL_OFF;if(HAL_RCC_OscConfig(&o)!=HAL_OK)Error_Handler();
 if(!use_hsi())Error_Handler();
 RCC_PeriphCLKInitTypeDef pc={0};pc.PeriphClockSelection=RCC_PERIPHCLK_USB;pc.UsbClockSelection=RCC_USBCLKSOURCE_HSI48;if(HAL_RCCEx_PeriphCLKConfig(&pc)!=HAL_OK)Error_Handler();
 __HAL_RCC_CRS_CLK_ENABLE();RCC_CRSInitTypeDef cr={0};cr.Prescaler=RCC_CRS_SYNC_DIV1;cr.Source=RCC_CRS_SYNC_SOURCE_USB;cr.Polarity=RCC_CRS_SYNC_POLARITY_RISING;cr.ReloadValue=__HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);cr.ErrorLimitValue=RCC_CRS_ERRORLIMIT_DEFAULT;cr.HSI48CalibrationValue=RCC_CRS_HSI48CALIBRATION_DEFAULT;HAL_RCCEx_CRSConfig(&cr);
 /* USB owns dedicated PA11/PA12 pads; no GPIO alternate function. */
 board_idle();
 watchdog.Instance=IWDG;watchdog.Init.Prescaler=IWDG_PRESCALER_32;watchdog.Init.Reload=999;watchdog.Init.Window=IWDG_WINDOW_DISABLE;
 if(HAL_IWDG_Init(&watchdog)!=HAL_OK)Error_Handler();
}
bool board_sensors_on(void){
 if(!usb_ready())return false;
 if(sensor_power)return true;
 usb_stop_requested=0;GPIOB->BSRR=GPIO_PIN_5;HAL_Delay(25);
 if(usb_stop_requested)return false;
 RCC_OscInitTypeDef o={0};o.OscillatorType=RCC_OSCILLATORTYPE_HSE;o.HSEState=RCC_HSE_BYPASS;o.PLL.PLLState=RCC_PLL_ON;o.PLL.PLLSource=RCC_PLLSOURCE_HSE;o.PLL.PLLM=RCC_PLLM_DIV2;o.PLL.PLLN=60;o.PLL.PLLR=RCC_PLLR_DIV2;o.PLL.PLLQ=RCC_PLLQ_DIV2;o.PLL.PLLP=RCC_PLLP_DIV2;
 if(HAL_RCC_OscConfig(&o)!=HAL_OK){board_idle();return false;}
 RCC_ClkInitTypeDef c={0};c.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;c.AHBCLKDivider=RCC_SYSCLK_DIV1;c.APB1CLKDivider=RCC_HCLK_DIV1;c.APB2CLKDivider=RCC_HCLK_DIV1;
 if(HAL_RCC_ClockConfig(&c,FLASH_LATENCY_4)!=HAL_OK){board_idle();return false;}
 GPIOC->BSRR=0x7c0;GPIOC->BRR=0x3f;pins(GPIOC,0x7ff,GPIO_MODE_OUTPUT_PP,0);
 GPIOB->BSRR=GPIO_PIN_6;pins(GPIOB,GPIO_PIN_6,GPIO_MODE_OUTPUT_PP,0);
 pins(GPIOC,GPIO_PIN_11,GPIO_MODE_INPUT,0);pins(GPIOB,GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1,GPIO_MODE_INPUT,0);pins(GPIOD,GPIO_PIN_2,GPIO_MODE_INPUT,0);
 __HAL_RCC_TIM4_CLK_ENABLE();pins(GPIOB,GPIO_PIN_9,GPIO_MODE_AF_PP,GPIO_AF2_TIM4);
 adc_clock.Instance=TIM4;adc_clock.Init.Prescaler=0;adc_clock.Init.CounterMode=TIM_COUNTERMODE_UP;adc_clock.Init.Period=19;adc_clock.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
 if(HAL_TIM_PWM_Init(&adc_clock)!=HAL_OK)return false;
 TIM_OC_InitTypeDef oc={0};oc.OCMode=TIM_OCMODE_PWM1;oc.Pulse=10;oc.OCPolarity=TIM_OCPOLARITY_HIGH;oc.OCFastMode=TIM_OCFAST_DISABLE;
 if(HAL_TIM_PWM_ConfigChannel(&adc_clock,&oc,TIM_CHANNEL_4)!=HAL_OK || HAL_TIM_PWM_Start(&adc_clock,TIM_CHANNEL_4)!=HAL_OK)return false;
 __HAL_RCC_SPI1_CLK_ENABLE();__HAL_RCC_SPI2_CLK_ENABLE();
 pins(GPIOB,GPIO_PIN_3,GPIO_MODE_AF_PP,GPIO_AF5_SPI1);pins(GPIOA,GPIO_PIN_6|GPIO_PIN_7,GPIO_MODE_AF_PP,GPIO_AF5_SPI1);pins(GPIOB,GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,GPIO_MODE_AF_PP,GPIO_AF5_SPI2);
 for(int i=0;i<2;i++){spi[i].Instance=i?SPI2:SPI1;spi[i].Init.Mode=SPI_MODE_MASTER;spi[i].Init.Direction=SPI_DIRECTION_2LINES;spi[i].Init.DataSize=SPI_DATASIZE_8BIT;spi[i].Init.CLKPolarity=SPI_POLARITY_LOW;spi[i].Init.CLKPhase=SPI_PHASE_2EDGE;spi[i].Init.NSS=SPI_NSS_SOFT;spi[i].Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;spi[i].Init.FirstBit=SPI_FIRSTBIT_MSB;spi[i].Init.TIMode=SPI_TIMODE_DISABLE;spi[i].Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;spi[i].Init.NSSPMode=SPI_NSS_PULSE_DISABLE;if(HAL_SPI_Init(&spi[i])!=HAL_OK)return false;}
 sensor_power=true;HAL_Delay(5);return !usb_stop_requested;
}
