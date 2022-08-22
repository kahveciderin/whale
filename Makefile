# build all the cpp files into a single executable

# define the C++ compiler to use
CC = g++

# define any compile-time flags
CFLAGS = -Wall -g

# get all the cpp files in the current directory
SRCS = $(wildcard *.cpp)

main: $(SRCS:.cpp=.o)
	$(CC) $(CFLAGS) -o $@ $^

run: main
	./main

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o main