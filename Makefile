CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
CC = g++ -std=c++11 -g
SOURCE_DIR = TennisCourt/TennisCourt

all: tennis.o ballFinder.o filegrabber.o videowrite.o
	$(CC) -o binary tennis.o ballFinder.o filegrabber.o videowrite.o $(LIBS)
videowrite.o: $(SOURCE_DIR)/video_writer.cpp
	$(CC) $(CFLAGS) -c -o videowrite.o $(SOURCE_DIR)/video_writer.cpp
filegrabber.o: $(SOURCE_DIR)/file_frame_grabber.cpp
	$(CC) $(CFLAGS) -c -o filegrabber.o $(SOURCE_DIR)/file_frame_grabber.cpp
tennis.o: $(SOURCE_DIR)/TennisCourt.cpp
	$(CC) $(CFLAGS) -c -o tennis.o $(SOURCE_DIR)/TennisCourt.cpp
ballFinder.o: $(SOURCE_DIR)/ballFinder.cpp
	$(CC) $(CFLAGS) -c -o ballFinder.o $(SOURCE_DIR)/ballFinder.cpp
simple: simple.cpp
	$(CC) $(CFLAGS) -c -o simple.o simple.cpp
clean:
	rm -rf *o binary
