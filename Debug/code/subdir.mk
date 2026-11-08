################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/ADRC.c \
../code/LED.c \
../code/MahonyAHRS.c \
../code/battery_detect.c \
../code/buzzer.c \
../code/element_line.c \
../code/encoder.c \
../code/image.c \
../code/inertial_navigation.c \
../code/initialize.c \
../code/line.c \
../code/maze.c \
../code/menu.c \
../code/motor.c \
../code/state_control.c \
../code/tool.c 

OBJS += \
./code/ADRC.o \
./code/LED.o \
./code/MahonyAHRS.o \
./code/battery_detect.o \
./code/buzzer.o \
./code/element_line.o \
./code/encoder.o \
./code/image.o \
./code/inertial_navigation.o \
./code/initialize.o \
./code/line.o \
./code/maze.o \
./code/menu.o \
./code/motor.o \
./code/state_control.o \
./code/tool.o 

COMPILED_SRCS += \
./code/ADRC.src \
./code/LED.src \
./code/MahonyAHRS.src \
./code/battery_detect.src \
./code/buzzer.src \
./code/element_line.src \
./code/encoder.src \
./code/image.src \
./code/inertial_navigation.src \
./code/initialize.src \
./code/line.src \
./code/maze.src \
./code/menu.src \
./code/motor.src \
./code/state_control.src \
./code/tool.src 

C_DEPS += \
./code/ADRC.d \
./code/LED.d \
./code/MahonyAHRS.d \
./code/battery_detect.d \
./code/buzzer.d \
./code/element_line.d \
./code/encoder.d \
./code/image.d \
./code/inertial_navigation.d \
./code/initialize.d \
./code/line.d \
./code/maze.d \
./code/menu.d \
./code/motor.d \
./code/state_control.d \
./code/tool.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/ADSone/Soberup_2026/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


