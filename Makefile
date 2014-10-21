CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

simple.o: simple.cpp
	g++ $(CFLAGS) -c -o simple.o simple.cpp
all: simple.o
	g++ $(LIBS) -g -o binary  simple.o
clean:
	rm -rf *o binary
