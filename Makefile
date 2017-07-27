include Makefile.common

CXX = g++
CXXFLAGS = -Wall -O2 -std=c++0x -llibgps 

all:$(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) -lpthread -lwiringPi -lrt `pkg-config --libs libgps` 

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY : clean
clean: 
	@rm -rf *.o *~ $(TARGET)

.PHONY : install
install:
	@scp $(TARGET) pi@192.168.100.1:~/arliss/

