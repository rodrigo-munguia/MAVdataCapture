CXX		  := g++
CXX_FLAGS := -Wall -Wextra -std=c++17 -ggdb

BIN		:= bin
SRC		:= src
INCLUDE	:= -Iinclude -IMavLink/c_library -I/usr/local/include/opencv4/
INCLUDE2 := 
LIB		:= -Llib -L/usr/local/lib
OPENCV = `pkg-config opencv --cflags --libs`


LIBRARIES	:= -lpthread -lgps
OPENCVLIBRARIES = $(OPENCV) 
EXECUTABLE	:= DataCapture


all: $(BIN)/$(EXECUTABLE)

run: clean all
	clear
	./$(BIN)/$(EXECUTABLE)

$(BIN)/$(EXECUTABLE): $(SRC)/*.cpp
	$(CXX) $(CXX_FLAGS) $(INCLUDE) $(LIB)  $^ -o $@ $(LIBRARIES) $(OPENCVLIBRARIES)

clean:
	-rm $(BIN)/*
