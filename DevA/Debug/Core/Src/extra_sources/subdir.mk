################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/extra_sources/custom_memory_manager.c \
../Core/Src/extra_sources/microros_allocators.c \
../Core/Src/extra_sources/microros_time.c 

C_DEPS += \
./Core/Src/extra_sources/custom_memory_manager.d \
./Core/Src/extra_sources/microros_allocators.d \
./Core/Src/extra_sources/microros_time.d 

OBJS += \
./Core/Src/extra_sources/custom_memory_manager.o \
./Core/Src/extra_sources/microros_allocators.o \
./Core/Src/extra_sources/microros_time.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/extra_sources/%.o Core/Src/extra_sources/%.su Core/Src/extra_sources/%.cyclo: ../Core/Src/extra_sources/%.c Core/Src/extra_sources/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F427xx -c -I../Core/Inc -Imicro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-extra_sources

clean-Core-2f-Src-2f-extra_sources:
	-$(RM) ./Core/Src/extra_sources/custom_memory_manager.cyclo ./Core/Src/extra_sources/custom_memory_manager.d ./Core/Src/extra_sources/custom_memory_manager.o ./Core/Src/extra_sources/custom_memory_manager.su ./Core/Src/extra_sources/microros_allocators.cyclo ./Core/Src/extra_sources/microros_allocators.d ./Core/Src/extra_sources/microros_allocators.o ./Core/Src/extra_sources/microros_allocators.su ./Core/Src/extra_sources/microros_time.cyclo ./Core/Src/extra_sources/microros_time.d ./Core/Src/extra_sources/microros_time.o ./Core/Src/extra_sources/microros_time.su

.PHONY: clean-Core-2f-Src-2f-extra_sources

