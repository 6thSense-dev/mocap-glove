PREFIX ?= arm-none-eabi-
CC = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE = $(PREFIX)size
CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
INC = -Iinclude -Ivendor/hal/Inc -Ivendor/device/Include -Ivendor/cmsis -Ivendor/usb/Core/Inc -Ivendor/usb/Class/CDC/Inc
CFLAGS = $(CPU) $(INC) -DHSE_VALUE=8192000UL -DSTM32G474xx -DUSE_HAL_DRIVER -DALLOW_COIL_DRIVE=1 -O2 -g3 -ffunction-sections -fdata-sections -Wall -Wextra -Wno-unused-parameter
HAL = hal hal_rcc hal_rcc_ex hal_flash hal_flash_ex hal_flash_ramfunc hal_gpio hal_exti hal_cortex hal_pwr hal_pwr_ex hal_adc hal_adc_ex hal_iwdg hal_tim hal_tim_ex hal_spi hal_dma hal_dma_ex hal_pcd hal_pcd_ex ll_usb
SRC = $(wildcard src/*.c) $(addprefix vendor/hal/Src/stm32g4xx_,$(addsuffix .c,$(HAL))) vendor/usb/Core/Src/usbd_core.c vendor/usb/Core/Src/usbd_ctlreq.c vendor/usb/Core/Src/usbd_ioreq.c vendor/usb/Class/CDC/Src/usbd_cdc.c
OBJ = $(addprefix build/,$(SRC:.c=.o)) build/startup.o
all: build/glove-v6-diagnostics.bin build/glove-v6-diagnostics.hex
build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
build/startup.o: src/startup.s
	@mkdir -p build
	$(CC) $(CPU) -c $< -o $@
build/glove-v6-diagnostics.elf: $(OBJ) stm32g474re.ld
	$(CC) $(CPU) --specs=nano.specs --specs=nosys.specs -Tstm32g474re.ld -Wl,--gc-sections,-Map=build/glove-v6-diagnostics.map $(OBJ) -lc -lm -o $@
	$(SIZE) $@
build/%.bin: build/%.elf
	$(OBJCOPY) -O binary $< $@
build/%.hex: build/%.elf
	$(OBJCOPY) -O ihex $< $@
clean:
	rm -rf build
-include $(OBJ:.o=.d)
.PHONY: all clean
