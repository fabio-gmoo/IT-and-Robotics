CXX = g++
CXXFLAGS = -O2 -Wall -Iinclude -I/usr/local/include/dynamixel_sdk $(shell pkg-config --cflags opencv4)
LDFLAGS = -ldxl_x64_cpp -lpthread -lrt $(shell pkg-config --libs opencv4)

BIN_DIR = bin
SRC_DIR = src
INC_DIR = include

TARGETS = $(BIN_DIR)/joystick_teleop \
          $(BIN_DIR)/main_robot_test \
          $(BIN_DIR)/RobotServoing

all: $(TARGETS)

$(BIN_DIR)/main_robot_test: $(SRC_DIR)/main_robot_test.cpp $(INC_DIR)/pong_robot.hpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

$(BIN_DIR)/joystick_teleop: $(SRC_DIR)/joystick_teleop.cpp $(INC_DIR)/pong_robot.hpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

$(BIN_DIR)/RobotServoing: $(SRC_DIR)/RobotServoing.cpp $(INC_DIR)/pong_robot.hpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(SRC_DIR)/RobotServoing.cpp -o $@ $(LDFLAGS)

clean:
	rm -rf $(BIN_DIR)/*
