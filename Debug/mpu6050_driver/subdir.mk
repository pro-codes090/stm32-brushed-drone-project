################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../mpu6050_driver/mpu6050_driver.c 

OBJS += \
./mpu6050_driver/mpu6050_driver.o 

C_DEPS += \
./mpu6050_driver/mpu6050_driver.d 


# Each subdirectory must supply rules for building sources it contributes
mpu6050_driver/%.o mpu6050_driver/%.su mpu6050_driver/%.cyclo: ../mpu6050_driver/%.c mpu6050_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/pid" -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/sx1278_lora" -I"C:/Users/pratham/Documents/stm32projectshal/drone_swarn_project/mpu6050_driver" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-mpu6050_driver

clean-mpu6050_driver:
	-$(RM) ./mpu6050_driver/mpu6050_driver.cyclo ./mpu6050_driver/mpu6050_driver.d ./mpu6050_driver/mpu6050_driver.o ./mpu6050_driver/mpu6050_driver.su

.PHONY: clean-mpu6050_driver

