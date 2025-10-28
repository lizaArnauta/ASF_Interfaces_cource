################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/VL53L0X/platform/src/vl53l0x_platform.c \
../Core/Src/VL53L0X/platform/src/vl53l0x_platform_log.c 

OBJS += \
./Core/Src/VL53L0X/platform/src/vl53l0x_platform.o \
./Core/Src/VL53L0X/platform/src/vl53l0x_platform_log.o 

C_DEPS += \
./Core/Src/VL53L0X/platform/src/vl53l0x_platform.d \
./Core/Src/VL53L0X/platform/src/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/VL53L0X/platform/src/%.o Core/Src/VL53L0X/platform/src/%.su Core/Src/VL53L0X/platform/src/%.cyclo: ../Core/Src/VL53L0X/platform/src/%.c Core/Src/VL53L0X/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-VL53L0X-2f-platform-2f-src

clean-Core-2f-Src-2f-VL53L0X-2f-platform-2f-src:
	-$(RM) ./Core/Src/VL53L0X/platform/src/vl53l0x_platform.cyclo ./Core/Src/VL53L0X/platform/src/vl53l0x_platform.d ./Core/Src/VL53L0X/platform/src/vl53l0x_platform.o ./Core/Src/VL53L0X/platform/src/vl53l0x_platform.su ./Core/Src/VL53L0X/platform/src/vl53l0x_platform_log.cyclo ./Core/Src/VL53L0X/platform/src/vl53l0x_platform_log.d ./Core/Src/VL53L0X/platform/src/vl53l0x_platform_log.o ./Core/Src/VL53L0X/platform/src/vl53l0x_platform_log.su

.PHONY: clean-Core-2f-Src-2f-VL53L0X-2f-platform-2f-src

