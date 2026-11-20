######################################################################
# Makefile — SVM_baremetal (STM32F407VGTx, GCC ARM)
#
# Drop-in replacement for the CubeIDE managed build.
# Build:  make            (Debug, -g3 -O0)
#         make RELEASE=1  (Release, -g0 -Os)
#         make clean
#         make flash      (openocd / stlink)
######################################################################

# ---------- toolchain ----------
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
CP      = $(PREFIX)objcopy
SZ      = $(PREFIX)size
HEX     = $(CP) -O ihex
BIN     = $(CP) -O binary -S

# ---------- target ----------
TARGET  = SVM_baremetal
LDSCRIPT = STM32F407VGTX_FLASH.ld

# ---------- build dir ----------
BUILD_DIR = build

# ---------- sources ----------
# Core (CubeMX-generated)
C_SOURCES = \
  Core/Src/main.c \
  Core/Src/stm32f4xx_it.c \
  Core/Src/stm32f4xx_hal_msp.c \
  Core/Src/stm32f4xx_hal_timebase_tim.c \
  Core/Src/freertos.c \
  Core/Src/gpio.c \
  Core/Src/adc.c \
  Core/Src/tim.c \
  Core/Src/system_stm32f4xx.c \
  Core/Src/sysmem.c \
  Core/Src/syscalls.c

# App
C_SOURCES += \
  App/Src/svpwm.c \
  App/Src/vf_control.c \
  App/Src/inverter.c \
  App/Src/cli.c \
  App/Src/cli_usb.c

# BSP
C_SOURCES += \
  BSP/Src/bsp_pwm.c \
  BSP/Src/bsp_adc.c \
  BSP/Src/bsp_button.c \
  BSP/Src/bsp_led.c \
  BSP/Src/bsp_clock.c \
  BSP/Src/bsp_gpio.c

# HAL driver
C_SOURCES += \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c

# FreeRTOS
C_SOURCES += \
  Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
  Middlewares/Third_Party/FreeRTOS/Source/queue.c \
  Middlewares/Third_Party/FreeRTOS/Source/list.c \
  Middlewares/Third_Party/FreeRTOS/Source/timers.c \
  Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
  Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
  Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c

# USB Device library
C_SOURCES += \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
  Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

# USB_DEVICE app
C_SOURCES += \
  USB_DEVICE/App/usb_device.c \
  USB_DEVICE/App/usbd_desc.c \
  USB_DEVICE/App/usbd_cdc_if.c \
  USB_DEVICE/Target/usbd_conf.c

# Startup (assembly)
ASM_SOURCES = \
  Core/Startup/startup_stm32f407vgtx.s

# ---------- includes ----------
C_INCLUDES = \
  -ICore/Inc \
  -IApp/Inc \
  -IBSP/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/CMSIS/Include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
  -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
  -IUSB_DEVICE/App \
  -IUSB_DEVICE/Target \
  -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
  -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

# ---------- defines ----------
C_DEFS = -DUSE_HAL_DRIVER -DSTM32F407xx

ifdef RELEASE
  C_DEFS  += -DNDEBUG
  OPT      = -Os
  DBG      = -g0
else
  C_DEFS  += -DDEBUG
  OPT      = -O0
  DBG      = -g3
endif

# ---------- MCU flags ----------
CPU     = -mcpu=cortex-m4
FPU     = -mfpu=fpv4-sp-d16
FLOAT   = -mfloat-abi=hard
MCU     = $(CPU) -mthumb $(FPU) $(FLOAT)

# ---------- compiler / assembler flags ----------
CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) $(DBG)
CFLAGS += -Wall -fdata-sections -ffunction-sections
CFLAGS += -std=gnu11

ASFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) $(DBG)
ASFLAGS += -Wall -fdata-sections -ffunction-sections

# ---------- linker flags ----------
LDLIBS  = -Wl,--gc-sections -Wl,--print-memory-usage
LDLIBS += -lm -lnosys -lc -lgcc -lstdc++
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LDLIBS)

# ---------- object list ----------
OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# ---------- rules ----------
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "  CC    $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "  AS    $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "  LD    $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	@$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	@$(BIN) $< $@

$(BUILD_DIR):
	@mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	  -c "program $< verify reset exit 0x08000000"

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) --format=berkeley $<

.PHONY: all clean flash size
