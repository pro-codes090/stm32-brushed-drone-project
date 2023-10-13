################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Filters/RC_Filters.c 

OBJS += \
./Filters/RC_Filters.o 

C_DEPS += \
./Filters/RC_Filters.d 


# Each subdirectory must supply rules for building sources it contributes
Filters/%.o Filters/%.su Filters/%.cyclo: ../Filters/%.c Filters/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/Filters" -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/pid" -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/sx1278_lora" -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/mpu6050_driver" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Filters

clean-Filters:
	-$(RM) ./Filters/RC_Filters.cyclo ./Filters/RC_Filters.d ./Filters/RC_Filters.o ./Filters/RC_Filters.su

.PHONY: clean-Filters

