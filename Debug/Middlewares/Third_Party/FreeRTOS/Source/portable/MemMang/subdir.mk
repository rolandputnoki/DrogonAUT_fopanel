################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -DUSE_RTOS_SYSTICK -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/HAL_Driver/Inc/Legacy" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/Utilities/STM32F4xx-Nucleo" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/inc" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/CMSIS/device" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/CMSIS/core" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/DrogonAUT_fopanel/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


