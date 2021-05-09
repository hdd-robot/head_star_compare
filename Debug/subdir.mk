################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../AStar.cpp \
../DLightStar.cpp \
../Grid.cpp \
../HeadStar.cpp \
../nav_platform.cpp 

OBJS += \
./AStar.o \
./DLightStar.o \
./Grid.o \
./HeadStar.o \
./nav_platform.o 

CPP_DEPS += \
./AStar.d \
./DLightStar.d \
./Grid.d \
./HeadStar.d \
./nav_platform.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


