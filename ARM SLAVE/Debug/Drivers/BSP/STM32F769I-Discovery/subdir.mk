################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.c \
../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_eeprom.c \
../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c \
../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.c \
../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.c 

OBJS += \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.o \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_eeprom.o \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.o \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.o \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.o 

C_DEPS += \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.d \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_eeprom.d \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.d \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.d \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.o: ../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F769xx -DDEBUG -c -I"../Drivers/BSP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/stm32f769i-Discovery -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_eeprom.o: ../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_eeprom.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F769xx -DDEBUG -c -I"../Drivers/BSP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/stm32f769i-Discovery -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_eeprom.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.o: ../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F769xx -DDEBUG -c -I"../Drivers/BSP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/stm32f769i-Discovery -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_lcd.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.o: ../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F769xx -DDEBUG -c -I"../Drivers/BSP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/stm32f769i-Discovery -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.o: ../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F769xx -DDEBUG -c -I"../Drivers/BSP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/stm32f769i-Discovery -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

