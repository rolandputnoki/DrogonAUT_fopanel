################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f446xx.s 

OBJS += \
./startup/startup_stm32f446xx.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/HAL_Driver/Inc/Legacy" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/Utilities/STM32F4xx-Nucleo" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/inc" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/CMSIS/device" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/CMSIS/core" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/HAL_Driver/Inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


