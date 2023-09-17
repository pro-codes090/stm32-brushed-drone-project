################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sx1268_lora/lora_sx1268.c 

OBJS += \
./sx1268_lora/lora_sx1268.o 

C_DEPS += \
./sx1268_lora/lora_sx1268.d 


# Each subdirectory must supply rules for building sources it contributes
sx1268_lora/%.o sx1268_lora/%.su sx1268_lora/%.cyclo: ../sx1268_lora/%.c sx1268_lora/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/pro/Desktop/stm32projectshal/drone_swarn_project/mpu6050_driver" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-sx1268_lora

clean-sx1268_lora:
	-$(RM) ./sx1268_lora/lora_sx1268.cyclo ./sx1268_lora/lora_sx1268.d ./sx1268_lora/lora_sx1268.o ./sx1268_lora/lora_sx1268.su

.PHONY: clean-sx1268_lora

