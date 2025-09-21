################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32f4xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32f4xx.o: C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c Drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../../Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/BSP/Components/vl53l1x_uld -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/X-NUCLEO-53L1A1 -Og -ffunction-sections -Wall -fno-builtin-fputc -fstack-usage -MMD -MP -MF"Drivers/CMSIS/system_stm32f4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS

clean-Drivers-2f-CMSIS:
	-$(RM) ./Drivers/CMSIS/system_stm32f4xx.d ./Drivers/CMSIS/system_stm32f4xx.o ./Drivers/CMSIS/system_stm32f4xx.su

.PHONY: clean-Drivers-2f-CMSIS

