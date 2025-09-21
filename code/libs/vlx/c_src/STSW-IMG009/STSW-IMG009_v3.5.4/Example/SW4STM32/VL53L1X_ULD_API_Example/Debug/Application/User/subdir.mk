################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/main.c \
C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/stm32f4xx_hal_msp.c \
C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/stm32f4xx_it.c \
C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/syscalls.c \
C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/vl53l1_platform.c 

OBJS += \
./Application/User/main.o \
./Application/User/stm32f4xx_hal_msp.o \
./Application/User/stm32f4xx_it.o \
./Application/User/syscalls.o \
./Application/User/vl53l1_platform.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/stm32f4xx_hal_msp.d \
./Application/User/stm32f4xx_it.d \
./Application/User/syscalls.d \
./Application/User/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../../Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/BSP/Components/vl53l1x_uld -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/X-NUCLEO-53L1A1 -Og -ffunction-sections -Wall -fno-builtin-fputc -fstack-usage -MMD -MP -MF"Application/User/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f4xx_hal_msp.o: C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/stm32f4xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../../Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/BSP/Components/vl53l1x_uld -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/X-NUCLEO-53L1A1 -Og -ffunction-sections -Wall -fno-builtin-fputc -fstack-usage -MMD -MP -MF"Application/User/stm32f4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f4xx_it.o: C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/stm32f4xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../../Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/BSP/Components/vl53l1x_uld -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/X-NUCLEO-53L1A1 -Og -ffunction-sections -Wall -fno-builtin-fputc -fstack-usage -MMD -MP -MF"Application/User/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/syscalls.o: C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/syscalls.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../../Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/BSP/Components/vl53l1x_uld -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/X-NUCLEO-53L1A1 -Og -ffunction-sections -Wall -fno-builtin-fputc -fstack-usage -MMD -MP -MF"Application/User/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/vl53l1_platform.o: C:/ST/VL53L1X/ULD/To\ ST\ com/en.STSW-IMG009_v3.5.4/Example/Src/vl53l1_platform.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../../Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/BSP/Components/vl53l1x_uld -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/X-NUCLEO-53L1A1 -Og -ffunction-sections -Wall -fno-builtin-fputc -fstack-usage -MMD -MP -MF"Application/User/vl53l1_platform.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/stm32f4xx_hal_msp.d ./Application/User/stm32f4xx_hal_msp.o ./Application/User/stm32f4xx_hal_msp.su ./Application/User/stm32f4xx_it.d ./Application/User/stm32f4xx_it.o ./Application/User/stm32f4xx_it.su ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/syscalls.su ./Application/User/vl53l1_platform.d ./Application/User/vl53l1_platform.o ./Application/User/vl53l1_platform.su

.PHONY: clean-Application-2f-User

