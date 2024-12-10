################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Applications/WAV_Recorder/wav_recorder.c 

OBJS += \
./Applications/WAV_Recorder/wav_recorder.o 

C_DEPS += \
./Applications/WAV_Recorder/wav_recorder.d 


# Each subdirectory must supply rules for building sources it contributes
Applications/WAV_Recorder/%.o Applications/WAV_Recorder/%.su Applications/WAV_Recorder/%.cyclo: ../Applications/WAV_Recorder/%.c Applications/WAV_Recorder/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Cuba/Documents/uni/STM32/STM32_HAL_SD_PROBE/Peripherals/Inc" -I"C:/Users/Cuba/Documents/uni/STM32/STM32_HAL_SD_PROBE/Applications/WAV_Recorder" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Applications-2f-WAV_Recorder

clean-Applications-2f-WAV_Recorder:
	-$(RM) ./Applications/WAV_Recorder/wav_recorder.cyclo ./Applications/WAV_Recorder/wav_recorder.d ./Applications/WAV_Recorder/wav_recorder.o ./Applications/WAV_Recorder/wav_recorder.su

.PHONY: clean-Applications-2f-WAV_Recorder

