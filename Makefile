TARGET = out 

CXX = g++
CXXFLAGS = -Wall -O2 
OBJS = utils.o task.o motor.o sensor.o actuator.o serial_command.o sequence.o main.o 

all:$(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) -lwiringPi -lpthread -lrt

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $< 

.PHONY : clean
clean: 
	@rm -rf *.o *~ $(TARGET)