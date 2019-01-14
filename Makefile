CC = g++
CFLAGS = -g -Wall -std=c++14
SRCS = main.cpp
PROG = main

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

all: 
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
clean:
	-rm -rf $(PROG)
