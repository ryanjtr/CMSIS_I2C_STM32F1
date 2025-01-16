################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../i2c_emulator/i2c_bitbang.c 

OBJS += \
./i2c_emulator/i2c_bitbang.o 

C_DEPS += \
./i2c_emulator/i2c_bitbang.d 


# Each subdirectory must supply rules for building sources it contributes
i2c_emulator/%.o i2c_emulator/%.su i2c_emulator/%.cyclo: ../i2c_emulator/%.c i2c_emulator/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103xB -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DHSI_VALUE=8000000 -DLSI_VALUE=40000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"F:/STM32project/STM32F401CC/CODE/CMSIS_I2C_STM32F1/i2c_cmsis_lib" -I"F:/STM32project/STM32F401CC/CODE/CMSIS_I2C_STM32F1/my_uart" -I"F:/STM32project/STM32F401CC/CODE/CMSIS_I2C_STM32F1/DFEMO" -I"F:/STM32project/STM32F401CC/CODE/CMSIS_I2C_STM32F1/i2c_emulator" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-i2c_emulator

clean-i2c_emulator:
	-$(RM) ./i2c_emulator/i2c_bitbang.cyclo ./i2c_emulator/i2c_bitbang.d ./i2c_emulator/i2c_bitbang.o ./i2c_emulator/i2c_bitbang.su

.PHONY: clean-i2c_emulator

