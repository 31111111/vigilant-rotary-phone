# =============================================================================
# 交叉编译 Makefile（为 STM32CubeIDE 工程手写，便于命令行构建）
# 目标芯片：STM32F407VETx（Cortex-M4F）
# 工具链：xPack GNU Arm Embedded GCC 13.2.1（免安装，解压即用）
# 用法：
#   make            # 构建 .elf / .bin / .hex
#   make clean      # 清理
# =============================================================================

# ---- 工具链路径（按实际解压位置修改；已在 PATH 中则可直接用 arm-none-eabi-）----
GCC_DIR := /tmp/xpack-arm-none-eabi-gcc-13.2.1-1.1
PREFIX  := $(GCC_DIR)/bin/arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
CP      := $(PREFIX)objcopy
SZ      := $(PREFIX)size

# ---- 目标 ----
TARGET  := robot
BUILD   := build

# ---- MCU 选项 ----
MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# ---- 宏定义 ----
C_DEFS := -DUSE_HAL_DRIVER -DSTM32F407xx
AS_DEFS := -DSTM32F407xx

# ---- 头文件路径 ----
INCLUDES := \
  -ICore/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/CMSIS/Include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
  -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

# ---- 编译选项 ----
CFLAGS := $(MCU) $(C_DEFS) $(INCLUDES) -Os -Wall -fdata-sections -ffunction-sections \
          -g3 -std=gnu11 -MMD -MP
ASFLAGS := $(MCU) $(AS_DEFS) $(INCLUDES) -g3
LDSCRIPT := STM32F407VETX_FLASH.ld
LIBS := -lc -lm -lnosys
LDFLAGS := $(MCU) -specs=nano.specs -T$(LDSCRIPT) -Wl,--gc-sections \
           -Wl,-Map=$(BUILD)/$(TARGET).map,--cref $(LIBS)

# ---- 源文件（自动收集）----
C_SOURCES := $(shell find Core Drivers Middlewares -name "*.c" 2>/dev/null)
ASM_SOURCES := Core/Startup/startup_stm32f407vetx.s

OBJECTS := $(addprefix $(BUILD)/,$(C_SOURCES:.c=.o)) $(BUILD)/startup_stm32f407vetx.o
vpath %.c $(sort $(dir $(C_SOURCES)))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex
	@echo "================ 固件尺寸 ================"
	@$(SZ) $(BUILD)/$(TARGET).elf

$(BUILD)/%.o: %.c Makefile | $(BUILD)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/startup_stm32f407vetx.o: $(ASM_SOURCES) Makefile | $(BUILD)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(CP) -O binary $< $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(CP) -O ihex $< $@

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	@rm -rf $(BUILD)

.PHONY: all clean
-include $(OBJECTS:.o=.d)
