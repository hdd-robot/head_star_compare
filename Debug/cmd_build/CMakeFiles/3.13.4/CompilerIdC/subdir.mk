################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cmd_build/CMakeFiles/3.13.4/CompilerIdC/CMakeCCompilerId.c 

OBJS += \
./cmd_build/CMakeFiles/3.13.4/CompilerIdC/CMakeCCompilerId.o 

C_DEPS += \
./cmd_build/CMakeFiles/3.13.4/CompilerIdC/CMakeCCompilerId.d 


# Each subdirectory must supply rules for building sources it contributes
cmd_build/CMakeFiles/3.13.4/CompilerIdC/%.o: ../cmd_build/CMakeFiles/3.13.4/CompilerIdC/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


