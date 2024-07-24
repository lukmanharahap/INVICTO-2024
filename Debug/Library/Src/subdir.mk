################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/Src/invicto_motor.c \
../Library/Src/lcd_i2c.c \
../Library/Src/odometry.c \
../Library/Src/pid.c \
../Library/Src/robot_control.c 

OBJS += \
./Library/Src/invicto_motor.o \
./Library/Src/lcd_i2c.o \
./Library/Src/odometry.o \
./Library/Src/pid.o \
./Library/Src/robot_control.o 

C_DEPS += \
./Library/Src/invicto_motor.d \
./Library/Src/lcd_i2c.d \
./Library/Src/odometry.d \
./Library/Src/pid.d \
./Library/Src/robot_control.d 


# Each subdirectory must supply rules for building sources it contributes
Library/Src/%.o Library/Src/%.su Library/Src/%.cyclo: ../Library/Src/%.c Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I.././ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Library-2f-Src

clean-Library-2f-Src:
	-$(RM) ./Library/Src/invicto_motor.cyclo ./Library/Src/invicto_motor.d ./Library/Src/invicto_motor.o ./Library/Src/invicto_motor.su ./Library/Src/lcd_i2c.cyclo ./Library/Src/lcd_i2c.d ./Library/Src/lcd_i2c.o ./Library/Src/lcd_i2c.su ./Library/Src/odometry.cyclo ./Library/Src/odometry.d ./Library/Src/odometry.o ./Library/Src/odometry.su ./Library/Src/pid.cyclo ./Library/Src/pid.d ./Library/Src/pid.o ./Library/Src/pid.su ./Library/Src/robot_control.cyclo ./Library/Src/robot_control.d ./Library/Src/robot_control.o ./Library/Src/robot_control.su

.PHONY: clean-Library-2f-Src

