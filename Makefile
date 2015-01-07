CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
CC = g++ -std=c++11 -g -Wall
SOURCE_DIR = TennisCourt/TennisCourt
OBJ_DIR = obj
CPP_FILES = $(wildcard TennisCourt/TennisCourt/*.cpp)
CPP_TEST = $(wildcard TennisCourt/TennisCourt/ToolsForTesting/*.cpp)
CPP_CVUTILS = $(wildcard TennisCourt/TennisCourt/CV_Utils/*.cpp)
OBJ_FILES = $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
OBJ_TEST_FILES = $(addprefix obj/test/,$(notdir $(CPP_TEST:.cpp=.o)))
OBJ_TESTUTILS = $(addprefix obj/utils/,$(notdir $(CPP_CVUTILS:.cpp=.o)))

MKDIR_P = mkdir -p

all: $(OBJ_FILES) $(OBJ_TEST_FILES) $(OBJ_TESTUTILS)
	$(CC) -o binary $^ $(LIBS)
$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp
	$(MKDIR_P) $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<
obj/test/%.o: $(SOURCE_DIR)/ToolsForTesting/%.cpp
	$(MKDIR_P) $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<
obj/utils/%.o: $(SOURCE_DIR)/CV_Utils/%.cpp
	$(MKDIR_P) $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<
clean:
	rm -rf $(OBJ_DIR)/*.o binary $(OBJ_DIR)/test/*.o $(OBJ_DIR)/utils/*.o
