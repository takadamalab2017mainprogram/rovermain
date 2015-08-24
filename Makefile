TARGET = out 

CXX = g++
CXXFLAGS = -Wall -O2  -std=c++0x 
OBJS = utils.o task.o motor.o sensor.o actuator.o serial_command.o sequence.o subsidiary_sequence.o alias.o image_proc.o pose_detector.o main.o 

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
