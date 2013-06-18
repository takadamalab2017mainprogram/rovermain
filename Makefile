TARGET = out 

CXX = g++
CXXFLAGS = -Wall -O2 
OBJS = debug.o task.o motor.o sensor.o actuator.o serial_command.o main.o 

all:$(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) -lwiringPi -lpthread -lrt

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $< 

.PHONY : clean
clean: 
	@rm -rf *.o *~ $(TARGET)