CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

all: simple.o
	g++ -o binary  simple.o $(LIBS)
simple: simple.cpp
	g++ $(CFLAGS) -c -o simple.o simple.cpp
clean:
	rm -rf *o binary
