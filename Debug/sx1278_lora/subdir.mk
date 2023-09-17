################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sx1278_lora/lora_sx1278.c 

OBJS += \
./sx1278_lora/lora_sx1278.o 

C_DEPS += \
./sx1278_lora/lora_sx1278.d 


# Each subdirectory must supply rules for building sources it contributes
sx1278_lora/%.o sx1278_lora/%.su sx1278_lora/%.cyclo: ../sx1278_lora/%.c sx1278_lora/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/pro/Desktop/stm32projectshal/drone_swarn_project/sx1278_lora" -I"C:/Users/pro/Desktop/stm32projectshal/drone_swarn_project/mpu6050_driver" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-sx1278_lora

clean-sx1278_lora:
	-$(RM) ./sx1278_lora/lora_sx1278.cyclo ./sx1278_lora/lora_sx1278.d ./sx1278_lora/lora_sx1278.o ./sx1278_lora/lora_sx1278.su

.PHONY: clean-sx1278_lora

