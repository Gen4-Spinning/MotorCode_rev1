################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Temperature/temperatureLUT.c 

OBJS += \
./Drivers/Temperature/temperatureLUT.o 

C_DEPS += \
./Drivers/Temperature/temperatureLUT.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Temperature/%.o Drivers/Temperature/%.su: ../Drivers/Temperature/%.c Drivers/Temperature/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/Eeprom" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/Console" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/PID" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/EncoderSpeed" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/FDCAN/Motor" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/Temperature" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/FDCAN" -I../Core/Inc -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Core/Inc" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/Ramp" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/sixSector" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/EncoderCalibration" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev7/Drivers/AS5x47PS" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Temperature

clean-Drivers-2f-Temperature:
	-$(RM) ./Drivers/Temperature/temperatureLUT.d ./Drivers/Temperature/temperatureLUT.o ./Drivers/Temperature/temperatureLUT.su

.PHONY: clean-Drivers-2f-Temperature

