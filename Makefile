CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
SOURCE_DIR = TennisCourt/TennisCourt

all: tennis ballFinder
	g++ -g -o binary tennis.o ballFinder.o $(LIBS)
tennis:
	g++ $(CFLAGS) -g -c -o tennis.o $(SOURCE_DIR)/TennisCourt.cpp
ballFinder:
	g++ $(CFLAGS) -g -c -o ballFinder.o $(SOURCE_DIR)/ballFinder.cpp
simple: simple.cpp
	g++ $(CFLAGS) -g -c -o simple.o simple.cpp
clean:
	rm -rf *o binary
