# Makefile for Micromouse Simulator

CC = g++
files = BitVector256.h Dir.h Maze.cpp MazeDefinitions.h Maze.h PathFinder.h 

floodfill: $(files) FloodFill.cpp
	$(CC) -o run $(files) FloodFill.cpp

leftfollower: $(files) main.cpp
	$(CC) -o LfRun $(files) main.cpp

all: $(files) FloodFill.cpp main.cpp
	$(CC) -o LfRun $(files) main.cpp
	$(CC) -o run $(files) FloodFill.cpp

clean:
	rm -f run LfRun

