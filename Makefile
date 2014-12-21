CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
CC = g++ -std=c++11 -g
SOURCE_DIR = TennisCourt/TennisCourt
OBJ_DIR = obj
CPP_FILES = $(wildcard TennisCourt/TennisCourt/*.cpp)
CPP_TEST = $(wildcard TennisCourt/TennisCourt/ToolsForTesting/*.cpp)
OBJ_FILES = $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
OBJ_TEST_FILES = $(addprefix obj/test/,$(notdir $(CPP_TEST:.cpp=.o)))

# all: tennis.o ballFinder.o filegrabber.o videowrite.o
# 	$(CC) -o binary tennis.o ballFinder.o filegrabber.o videowrite.o $(LIBS)
# videowrite.o: $(SOURCE_DIR)/video_writer.cpp
# 	$(CC) $(CFLAGS) -c -o videowrite.o $(SOURCE_DIR)/video_writer.cpp
# filegrabber.o: $(SOURCE_DIR)/file_frame_grabber.cpp
# 	$(CC) $(CFLAGS) -c -o filegrabber.o $(SOURCE_DIR)/file_frame_grabber.cpp
# tennis.o: $(SOURCE_DIR)/TennisCourt.cpp
# 	$(CC) $(CFLAGS) -c -o tennis.o $(SOURCE_DIR)/TennisCourt.cpp
# ballFinder.o: $(SOURCE_DIR)/ballFinder.cpp
# 	$(CC) $(CFLAGS) -c -o ballFinder.o $(SOURCE_DIR)/ballFinder.cpp
# simple: simple.cpp
# 	$(CC) $(CFLAGS) -c -o simple.o simple.cpp

all: $(OBJ_FILES) $(OBJ_TEST_FILES)
	$(CC) -o binary $^ $(LIBS)
$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp
	$(CC) $(CFLAGS) -c -o $@ $<
obj/test/%.o: $(SOURCE_DIR)/ToolsForTesting/%.cpp
	$(CC) $(CFLAGS) -c -o $@ $<
clean:
	rm -rf $(OBJ_DIR)/*.o binary $(OBJ_DIR)/test/*.o
