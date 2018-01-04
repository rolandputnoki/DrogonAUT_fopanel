################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/BT_modul.c \
../src/LSM6DS3_ACC_GYRO_driver.c \
../src/LSM6DS3_ACC_GYRO_driver_HL.c \
../src/adc.c \
../src/bsp.c \
../src/bsp_i2c.c \
../src/bsp_spi.c \
../src/gyari_radio_vevo_jel_meres.c \
../src/lsm6ds3.c \
../src/main.c \
../src/motor_pwm.c \
../src/stm32f4xx_it.c \
../src/syscalls.c \
../src/system_stm32f4xx.c \
../src/szervo_pwm.c \
../src/uart_communication.c 

OBJS += \
./src/BT_modul.o \
./src/LSM6DS3_ACC_GYRO_driver.o \
./src/LSM6DS3_ACC_GYRO_driver_HL.o \
./src/adc.o \
./src/bsp.o \
./src/bsp_i2c.o \
./src/bsp_spi.o \
./src/gyari_radio_vevo_jel_meres.o \
./src/lsm6ds3.o \
./src/main.o \
./src/motor_pwm.o \
./src/stm32f4xx_it.o \
./src/syscalls.o \
./src/system_stm32f4xx.o \
./src/szervo_pwm.o \
./src/uart_communication.o 

C_DEPS += \
./src/BT_modul.d \
./src/LSM6DS3_ACC_GYRO_driver.d \
./src/LSM6DS3_ACC_GYRO_driver_HL.d \
./src/adc.d \
./src/bsp.d \
./src/bsp_i2c.d \
./src/bsp_spi.d \
./src/gyari_radio_vevo_jel_meres.d \
./src/lsm6ds3.d \
./src/main.d \
./src/motor_pwm.d \
./src/stm32f4xx_it.d \
./src/syscalls.d \
./src/system_stm32f4xx.d \
./src/szervo_pwm.d \
./src/uart_communication.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -DUSE_RTOS_SYSTICK -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/HAL_Driver/Inc/Legacy" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/Utilities/STM32F4xx-Nucleo" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/inc" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/CMSIS/device" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/CMSIS/core" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Roland/Documents/Egyetemi projektek/RobonAUT/Openstm_projektek/fopanel/DrogonAUT_fo_panel/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


