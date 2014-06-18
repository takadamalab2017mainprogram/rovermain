TARGET = out 

LD_LIBRARY_PATH=./compiler/arm-linux-gnueabihf/lib
CXX = ./compiler/bin/arm-linux-gnueabihf-g++
CXXFLAGS = -Wall -O2 -L./compiler/arm-linux-gnueabihf/lib -I./compiler/arm-linux/gnueabihf 
OBJS = utils.o task.o motor.o sensor.o actuator.o serial_command.o sequence.o image_proc.o main.o 

all:$(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) -lpthread -lwiringPi -lrt `pkg-config --libs opencv`

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $< `pkg-config --cflags opencv`


.PHONY : clean
clean: 
	@rm -rf *.o *~ $(TARGET)

.PHONY : install
install:
	@scp $(TARGET) pi@192.168.100.1:~/arliss/
