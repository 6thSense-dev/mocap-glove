#pragma once
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
extern volatile uint8_t usb_stop_requested;
extern volatile uint32_t tx_deadline;
void app_usb_stop(void);
void Error_Handler(void);
void board_idle(void);
bool board_sensors_on(void);
bool usb_ready(void);
bool usb_send(const void *data, unsigned length);
bool usb_line(char *out, unsigned capacity);
void usb_init(void);
uint16_t crc16(const uint8_t *data, unsigned length);
extern SPI_HandleTypeDef spi[2];
extern bool sensor_power;
extern uint8_t found_mask;
bool ads_probe(void);
bool ads_read(unsigned index, uint8_t rx[12]);
bool capture(uint8_t mask, bool drive);
void capture_dump(void);
void capture_report(char *out,unsigned size);
void capture_irq(unsigned bus);
void capture_stop(void);
void source_stop(void);
bool source_start(void);
#define VERSION "0.2.7-diagnostics"
void watchdog_feed(void);
bool currents_read(uint16_t raw[3], uint32_t *vdda);
void diagnostics_off(void);
bool imu_command(const char *op, char *out, unsigned size);
bool source_config(unsigned mask, unsigned ms);
bool source_config_live(void);
bool source_config_track(void);
bool source_config_stream(void);
bool capture_stream(uint8_t mask);
void capture_stream_dump(void);
void source_poll(void);
bool source_ok(void);
void source_report(char *out,unsigned size);
void source_tick(void);
#ifndef ALLOW_COIL_DRIVE
#define ALLOW_COIL_DRIVE 0
#endif

uint32_t source_phase_ticks(void);
