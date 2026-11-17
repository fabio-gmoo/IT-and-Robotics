CXX = g++
CXXFLAGS = -O2 -Wall -Iinclude -I/usr/local/include/dynamixel_sdk
LDFLAGS = -ldxl_x64_cpp -lpthread -lrt

BIN_DIR = bin
SRC_DIR = src

TARGETS = $(BIN_DIR)/main_robot_test $(BIN_DIR)/interactive_test

all: $(TARGETS)

$(BIN_DIR)/main_robot_test: $(SRC_DIR)/main_robot_test.cpp
	mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

$(BIN_DIR)/interactive_test: $(SRC_DIR)/interactive_test.cpp
	mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -rf $(BIN_DIR)/*
