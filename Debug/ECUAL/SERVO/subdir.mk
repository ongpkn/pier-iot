################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/SERVO/SERVO.c \
../ECUAL/SERVO/SERVO_cfg.c 

OBJS += \
./ECUAL/SERVO/SERVO.o \
./ECUAL/SERVO/SERVO_cfg.o 

C_DEPS += \
./ECUAL/SERVO/SERVO.d \
./ECUAL/SERVO/SERVO_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/SERVO/%.o ECUAL/SERVO/%.su: ../ECUAL/SERVO/%.c ECUAL/SERVO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ECUAL-2f-SERVO

clean-ECUAL-2f-SERVO:
	-$(RM) ./ECUAL/SERVO/SERVO.d ./ECUAL/SERVO/SERVO.o ./ECUAL/SERVO/SERVO.su ./ECUAL/SERVO/SERVO_cfg.d ./ECUAL/SERVO/SERVO_cfg.o ./ECUAL/SERVO/SERVO_cfg.su

.PHONY: clean-ECUAL-2f-SERVO

