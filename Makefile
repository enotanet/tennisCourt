CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
CC = g++ -std=c++11 -g
SOURCE_DIR = TennisCourt/TennisCourt

all: tennis.o ballFinder.o analysissystem.o sysfilegrabber.o syscameragrabber.o \
     filegrabber.o videowrite.o sysvideowrite.o utils.o testing.o cameralocation.o
	$(CC) -o binary tennis.o ballFinder.o analysissystem.o sysfilegrabber.o syscameragrabber.o \
	  filegrabber.o videowrite.o sysvideowrite.o utils.o testing.o cameralocation.o $(LIBS)
videowrite.o: $(SOURCE_DIR)/video_writer.cpp
	$(CC) $(CFLAGS) -c -o videowrite.o $(SOURCE_DIR)/video_writer.cpp
sysvideowrite.o: $(SOURCE_DIR)/sys_video_writer.cpp $(SOURCE_DIR)/utils.h
	$(CC) $(CFLAGS) -c -o sysvideowrite.o $(SOURCE_DIR)/sys_video_writer.cpp
filegrabber.o: $(SOURCE_DIR)/file_frame_grabber.cpp
	$(CC) $(CFLAGS) -c -o filegrabber.o $(SOURCE_DIR)/file_frame_grabber.cpp
tennis.o: $(SOURCE_DIR)/TennisCourt.cpp $(SOURCE_DIR)/utils.h $(SOURCE_DIR)/sys_file_frame_grabber.h \
  	  $(SOURCE_DIR)/analysis_system.h $(SOURCE_DIR)/sys_camera_grabber.h \
          $(SOURCE_DIR)/ToolsForTesting/testing.h $(SOURCE_DIR)/ballFinder.h		
	$(CC) $(CFLAGS) -c -o tennis.o $(SOURCE_DIR)/TennisCourt.cpp 
sysfilegrabber.o: $(SOURCE_DIR)/sys_file_frame_grabber.cpp $(SOURCE_DIR)/file_frame_grabber.h \
  		  $(SOURCE_DIR)/utils.h
	$(CC) $(CFLAGS) -c -o sysfilegrabber.o $(SOURCE_DIR)/sys_file_frame_grabber.cpp
syscameragrabber.o: $(SOURCE_DIR)/sys_camera_grabber.cpp $(SOURCE_DIR)/utils.h
	$(CC) $(CFLAGS) -c -o syscameragrabber.o $(SOURCE_DIR)/sys_camera_grabber.cpp
analysissystem.o: $(SOURCE_DIR)/analysis_system.cpp $(SOURCE_DIR)/camera_location.h \
  		  $(SOURCE_DIR)/sys_frame_grabber.h $(SOURCE_DIR)/utils.h
	$(CC) $(CFLAGS) -c -o analysissystem.o $(SOURCE_DIR)/analysis_system.cpp
cameralocation.o: $(SOURCE_DIR)/camera_location.cpp
	$(CC) $(CFLAGS) -c -o cameralocation.o $(SOURCE_DIR)/camera_location.cpp
utils.o: $(SOURCE_DIR)/utils.cpp
	$(CC) $(CFLAGS) -c -o utils.o $(SOURCE_DIR)/utils.cpp 
ballFinder.o: $(SOURCE_DIR)/ballFinder.cpp
	$(CC) $(CFLAGS) -c -o ballFinder.o $(SOURCE_DIR)/ballFinder.cpp
testing.o: $(SOURCE_DIR)/ToolsForTesting/testing.cpp  $(SOURCE_DIR)/sys_frame_grabber.h \
  	   $(SOURCE_DIR)/sys_camera_grabber.h $(SOURCE_DIR)/sys_file_frame_grabber.h \
	   $(SOURCE_DIR)/sys_video_writer.h $(SOURCE_DIR)/utils.h
	$(CC) $(CFLAGS) -c -o testing.o $(SOURCE_DIR)/ToolsForTesting/testing.cpp
simple: simple.cpp
	$(CC) $(CFLAGS) -c -o simple.o simple.cpp
clean:
	rm -rf *o binary
