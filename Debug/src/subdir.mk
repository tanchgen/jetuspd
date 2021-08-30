################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/MQTTConnectClient.c \
../src/MQTTDeserializePublish.c \
../src/MQTTFormat.c \
../src/MQTTPacket.c \
../src/MQTTSerializePublish.c \
../src/MQTTSim800.c \
../src/MQTTSubscribeClient.c \
../src/MQTTUnsubscribeClient.c \
../src/gpio.c \
../src/initialize-hardware.c \
../src/main.c \
../src/stm32f4xx_hal_msp.c \
../src/stm32f4xx_it.c \
../src/usart.c \
../src/write.c 

OBJS += \
./src/MQTTConnectClient.o \
./src/MQTTDeserializePublish.o \
./src/MQTTFormat.o \
./src/MQTTPacket.o \
./src/MQTTSerializePublish.o \
./src/MQTTSim800.o \
./src/MQTTSubscribeClient.o \
./src/MQTTUnsubscribeClient.o \
./src/gpio.o \
./src/initialize-hardware.o \
./src/main.o \
./src/stm32f4xx_hal_msp.o \
./src/stm32f4xx_it.o \
./src/usart.o \
./src/write.o 

C_DEPS += \
./src/MQTTConnectClient.d \
./src/MQTTDeserializePublish.d \
./src/MQTTFormat.d \
./src/MQTTPacket.d \
./src/MQTTSerializePublish.d \
./src/MQTTSim800.d \
./src/MQTTSubscribeClient.d \
./src/MQTTUnsubscribeClient.d \
./src/gpio.d \
./src/initialize-hardware.d \
./src/main.d \
./src/stm32f4xx_hal_msp.d \
./src/stm32f4xx_it.d \
./src/usart.d \
./src/write.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F429xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -DMQTT_CLIENT=1 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/stm32f4xx_hal_msp.o: ../src/stm32f4xx_hal_msp.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F429xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -DMQTT_CLIENT=1 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -std=gnu11 -Wno-padded -Wno-missing-prototypes -Wno-missing-declarations -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


