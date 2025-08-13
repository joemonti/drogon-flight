SRC = ./src
INCLUDE = $(SRC)/include
DEPS = $(INCLUDE)/*.h
OBJS = $(SRC)/DrogonPid.o $(SRC)/DrogonPidTuner.o $(SRC)/DrogonPosition.o \
	  $(SRC)/DrogonController.o $(SRC)/I2C.o $(SRC)/I2CIMU.o $(SRC)/I2CServo.o \
	  $(SRC)/I2CLidar.o $(SRC)/DrogonFlight.o $(SRC)/RCoreClient.o $(SRC)/drogon-flight.o
CC = g++
DEBUG = -g
CFLAGS = -std=c++11 -I$(INCLUDE) $(DEBUG) -fPIC -Wall
LDFLAGS = -lzmq -li2c
TARGET = drogon-flight

all: $(TARGET)

libdrogonflight.so: $(OBJS) DrogonTest.o
	$(CC) -shared -Wl,-soname,libdrogonflight.so -o $@ $(OBJS) DrogonTest.o /usr/lib/x86_64-linux-gnu/libzmq.so.5

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

clean:
	rm -f src/*.o drogon-flight
