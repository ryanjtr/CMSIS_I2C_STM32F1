################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../my_uart/myuart.c 

OBJS += \
./my_uart/myuart.o 

C_DEPS += \
./my_uart/myuart.d 


# Each subdirectory must supply rules for building sources it contributes
my_uart/%.o my_uart/%.su my_uart/%.cyclo: ../my_uart/%.c my_uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103xB -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DHSI_VALUE=8000000 -DLSI_VALUE=40000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"F:/STM32project/STM32F401CC/CODE/CMSIS_I2C_STM32F1/i2c_cmsis_lib" -I"F:/STM32project/STM32F401CC/CODE/CMSIS_I2C_STM32F1/my_uart" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-my_uart

clean-my_uart:
	-$(RM) ./my_uart/myuart.cyclo ./my_uart/myuart.d ./my_uart/myuart.o ./my_uart/myuart.su

.PHONY: clean-my_uart

