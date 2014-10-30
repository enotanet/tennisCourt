CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

all: simple.o
	g++ -o binary simple.o $(LIBS)  
simple.o: simple.cpp
	g++ $(CFLAGS) -c simple.cpp
clean:
	rm -rf *o binary
