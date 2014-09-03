TARGET = ../out 

COMPILER_ROOT = ./compiler
CXX = $(COMPILER_ROOT)/bin/arm-linux-gnueabihf-g++
CXXFLAGS = -Wall -O2 
OBJS = utils.o task.o motor.o sensor.o actuator.o serial_command.o sequence.o image_proc.o main.o subsidiary_sequence.o

OPENCV_LINKER_OPTION = -L$(COMPILER_ROOT)/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann
OPENCV_COMPILER_OPTION = -I$(COMPILER_ROOT)/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/include/opencv -I$(COMPILER_ROOT)/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/include

all:$(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) -lpthread -lwiringPi -lrt $(OPENCV_LINKER_OPTION)

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(OPENCV_COMPILER_OPTION)

.PHONY : clean
clean: 
	@rm -rf *.o *~ $(TARGET)

.PHONY : install
install:
	@scp $(TARGET) pi@192.168.100.1:~/arliss/

