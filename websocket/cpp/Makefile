# Makefile for C++ Example

CXX = g++
ARCH := $(shell uname -m)
CXXFLAGS = -std=c++11 -Wall -O2 -Iinclude
LDFLAGS = -Llib/$(ARCH) -lwebsocket_client

.PHONY: all clean run

all:
	mkdir -p bin/$(ARCH)
	$(CXX) $(CXXFLAGS) -o bin/$(ARCH)/example example/example.cpp $(LDFLAGS)

run: all
	LD_LIBRARY_PATH=lib/$(ARCH) ./bin/$(ARCH)/example

clean:
	rm -rf bin
