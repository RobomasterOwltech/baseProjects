################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/extra_sources/microros_transports/dma_transport.c \
../Core/Src/extra_sources/microros_transports/it_transport.c \
../Core/Src/extra_sources/microros_transports/usb_cdc_transport.c 

C_DEPS += \
./Core/Src/extra_sources/microros_transports/dma_transport.d \
./Core/Src/extra_sources/microros_transports/it_transport.d \
./Core/Src/extra_sources/microros_transports/usb_cdc_transport.d 

OBJS += \
./Core/Src/extra_sources/microros_transports/dma_transport.o \
./Core/Src/extra_sources/microros_transports/it_transport.o \
./Core/Src/extra_sources/microros_transports/usb_cdc_transport.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/extra_sources/microros_transports/%.o Core/Src/extra_sources/microros_transports/%.su Core/Src/extra_sources/microros_transports/%.cyclo: ../Core/Src/extra_sources/microros_transports/%.c Core/Src/extra_sources/microros_transports/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F427xx -c -I../Core/Inc -Imicro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-extra_sources-2f-microros_transports

clean-Core-2f-Src-2f-extra_sources-2f-microros_transports:
	-$(RM) ./Core/Src/extra_sources/microros_transports/dma_transport.cyclo ./Core/Src/extra_sources/microros_transports/dma_transport.d ./Core/Src/extra_sources/microros_transports/dma_transport.o ./Core/Src/extra_sources/microros_transports/dma_transport.su ./Core/Src/extra_sources/microros_transports/it_transport.cyclo ./Core/Src/extra_sources/microros_transports/it_transport.d ./Core/Src/extra_sources/microros_transports/it_transport.o ./Core/Src/extra_sources/microros_transports/it_transport.su ./Core/Src/extra_sources/microros_transports/usb_cdc_transport.cyclo ./Core/Src/extra_sources/microros_transports/usb_cdc_transport.d ./Core/Src/extra_sources/microros_transports/usb_cdc_transport.o ./Core/Src/extra_sources/microros_transports/usb_cdc_transport.su

.PHONY: clean-Core-2f-Src-2f-extra_sources-2f-microros_transports

