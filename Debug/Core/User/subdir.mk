################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/User/ad9834.c \
../Core/User/adc_dual.c \
../Core/User/dac_output.c \
../Core/User/dds_control.c \
../Core/User/fft_f32_65536.c \
../Core/User/frequency_measure.c \
../Core/User/hmi_tjc.c \
../Core/User/measurement_fft.c \
../Core/User/measurement_result.c \
../Core/User/system.c 

OBJS += \
./Core/User/ad9834.o \
./Core/User/adc_dual.o \
./Core/User/dac_output.o \
./Core/User/dds_control.o \
./Core/User/fft_f32_65536.o \
./Core/User/frequency_measure.o \
./Core/User/hmi_tjc.o \
./Core/User/measurement_fft.o \
./Core/User/measurement_result.o \
./Core/User/system.o 

C_DEPS += \
./Core/User/ad9834.d \
./Core/User/adc_dual.d \
./Core/User/dac_output.d \
./Core/User/dds_control.d \
./Core/User/fft_f32_65536.d \
./Core/User/frequency_measure.d \
./Core/User/hmi_tjc.d \
./Core/User/measurement_fft.d \
./Core/User/measurement_result.d \
./Core/User/system.d 


# Each subdirectory must supply rules for building sources it contributes
Core/User/%.o Core/User/%.su Core/User/%.cyclo: ../Core/User/%.c Core/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"D:/CubeIDE/h743_pre1/Drivers/CMSIS/DSP/Include" -I"D:/CubeIDE/h743_pre1/Core/User" -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-User

clean-Core-2f-User:
	-$(RM) ./Core/User/ad9834.cyclo ./Core/User/ad9834.d ./Core/User/ad9834.o ./Core/User/ad9834.su ./Core/User/adc_dual.cyclo ./Core/User/adc_dual.d ./Core/User/adc_dual.o ./Core/User/adc_dual.su ./Core/User/dac_output.cyclo ./Core/User/dac_output.d ./Core/User/dac_output.o ./Core/User/dac_output.su ./Core/User/dds_control.cyclo ./Core/User/dds_control.d ./Core/User/dds_control.o ./Core/User/dds_control.su ./Core/User/fft_f32_65536.cyclo ./Core/User/fft_f32_65536.d ./Core/User/fft_f32_65536.o ./Core/User/fft_f32_65536.su ./Core/User/frequency_measure.cyclo ./Core/User/frequency_measure.d ./Core/User/frequency_measure.o ./Core/User/frequency_measure.su ./Core/User/hmi_tjc.cyclo ./Core/User/hmi_tjc.d ./Core/User/hmi_tjc.o ./Core/User/hmi_tjc.su ./Core/User/measurement_fft.cyclo ./Core/User/measurement_fft.d ./Core/User/measurement_fft.o ./Core/User/measurement_fft.su ./Core/User/measurement_result.cyclo ./Core/User/measurement_result.d ./Core/User/measurement_result.o ./Core/User/measurement_result.su ./Core/User/system.cyclo ./Core/User/system.d ./Core/User/system.o ./Core/User/system.su

.PHONY: clean-Core-2f-User

