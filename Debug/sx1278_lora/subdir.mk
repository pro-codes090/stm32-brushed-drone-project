################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sx1278_lora/lora.c 

OBJS += \
./sx1278_lora/lora.o 

C_DEPS += \
./sx1278_lora/lora.d 


# Each subdirectory must supply rules for building sources it contributes
sx1278_lora/%.o sx1278_lora/%.su sx1278_lora/%.cyclo: ../sx1278_lora/%.c sx1278_lora/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/Filters" -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/pid" -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/sx1278_lora" -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/mpu6050_driver" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-sx1278_lora

clean-sx1278_lora:
	-$(RM) ./sx1278_lora/lora.cyclo ./sx1278_lora/lora.d ./sx1278_lora/lora.o ./sx1278_lora/lora.su

.PHONY: clean-sx1278_lora

