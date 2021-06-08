################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f3xx.c 

CPP_SRCS += \
../Src/Futaba.cpp \
../Src/main.cpp \
../Src/sequence.cpp 

OBJS += \
./Src/Futaba.o \
./Src/main.o \
./Src/sequence.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f3xx.o 

C_DEPS += \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f3xx.d 

CPP_DEPS += \
./Src/Futaba.d \
./Src/main.d \
./Src/sequence.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16  -I"C:/stm32/kousen_robokon2019/2019k_mr/Inc" -I"C:/stm32/kousen_robokon2019/2019k_mr/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/stm32/kousen_robokon2019/2019k_mr/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/stm32/kousen_robokon2019/2019k_mr/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/stm32/kousen_robokon2019/2019k_mr/Drivers/CMSIS/Include"  -Og -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/stm32/kousen_robokon2019/2019k_mr/Inc" -I"C:/stm32/kousen_robokon2019/2019k_mr/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/stm32/kousen_robokon2019/2019k_mr/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/stm32/kousen_robokon2019/2019k_mr/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/stm32/kousen_robokon2019/2019k_mr/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


