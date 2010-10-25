################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Manipulator.cpp \
../Test.cpp \
../YouBotArm.cpp \
../armDriverYouBot3.cpp 

OBJS += \
./Manipulator.o \
./Test.o \
./YouBotArm.o \
./armDriverYouBot3.o 

CPP_DEPS += \
./Manipulator.d \
./Test.d \
./YouBotArm.d \
./armDriverYouBot3.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


