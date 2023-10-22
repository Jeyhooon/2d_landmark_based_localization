# Localization Challenge
# Makefile

INCLUDE = -I./include/ -I/usr/include/
LIBRARIES = ./lib/controller.a -L/usr/lib/x86_64-linux-gnu/ -lGL -lGLU -lglut -lm -lXi -lXmu

COMPILER = g++ --std=c++11
COMPILERFLAGS = -no-pie $(INCLUDE)


PROGRAM = localization_test

SRC_DIR = src
OBJ_DIR = obj

SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
HEADERS = $(wildcard ./include/*.cpp)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

all: directories $(PROGRAM)

directories: create-obj-dir

create-obj-dir: 
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(COMPILER) -c $(COMPILERFLAGS) $< -o $@

$(PROGRAM): $(OBJECTS) $(HEADERS)
	$(COMPILER) $(COMPILERFLAGS) -o $@ $^ $(LIBRARIES)

clean: 
	-rm -rf $(OBJ_DIR) $(PROGRAM)
