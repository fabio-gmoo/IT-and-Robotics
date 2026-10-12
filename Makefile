CXX = g++
CXXFLAGS = -O2 -Wall -Iinclude -I/usr/local/include/dynamixel_sdk $(shell pkg-config --cflags opencv4)
LDFLAGS_CV = $(shell pkg-config --libs opencv4)
LDFLAGS_DXL = -ldxl_x64_cpp -lpthread -lrt

BIN_DIR = bin
SRC_DIR = src
INC_DIR = include

TARGETS_VISION = $(BIN_DIR)/CameraCalibration \
                 $(BIN_DIR)/MonoCameraCalibration \
                 $(BIN_DIR)/RedBallDetection \
                 $(BIN_DIR)/RedBallTracking \
                 $(BIN_DIR)/ChessboardOrientationTracking

TARGETS_ROBOT = $(BIN_DIR)/interactive_test \
                $(BIN_DIR)/main_robot_test \
                $(BIN_DIR)/joystick_teleop

TARGETS_SERVOING = $(BIN_DIR)/RobotServoing

all: $(TARGETS_VISION) $(TARGETS_SERVOING)

vision: $(TARGETS_VISION)
robot: $(TARGETS_ROBOT)

$(BIN_DIR)/CameraCalibration: $(SRC_DIR)/CameraCalibration.cpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_CV)

$(BIN_DIR)/MonoCameraCalibration: $(SRC_DIR)/MonoCameraCalibration.cpp $(SRC_DIR)/ReadWriteFunctions.cpp $(INC_DIR)/ReadWriteFunctions.h
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(SRC_DIR)/MonoCameraCalibration.cpp $(SRC_DIR)/ReadWriteFunctions.cpp -o $@ $(LDFLAGS_CV)

$(BIN_DIR)/RedBallDetection: $(SRC_DIR)/RedBallDetection.cpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_CV)

$(BIN_DIR)/RedBallTracking: $(SRC_DIR)/RedBallTracking.cpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_CV)

$(BIN_DIR)/ChessboardOrientationTracking: $(SRC_DIR)/ChessboardOrientationTracking.cpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_CV)

$(BIN_DIR)/interactive_test: $(SRC_DIR)/interactive_test.cpp $(INC_DIR)/pong_robot.hpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_DXL)

$(BIN_DIR)/main_robot_test: $(SRC_DIR)/main_robot_test.cpp $(INC_DIR)/pong_robot.hpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_DXL)

$(BIN_DIR)/joystick_teleop: $(SRC_DIR)/joystick_teleop.cpp $(INC_DIR)/pong_robot.hpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS_DXL)

$(BIN_DIR)/RobotServoing: $(SRC_DIR)/RobotServoing.cpp $(SRC_DIR)/Kinematics.cpp $(SRC_DIR)/DynamixelHandler.cpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(SRC_DIR)/RobotServoing.cpp $(SRC_DIR)/Kinematics.cpp $(SRC_DIR)/DynamixelHandler.cpp -o $@ $(LDFLAGS_DXL) $(LDFLAGS_CV)

clean:
	rm -rf $(BIN_DIR)/* $(SRC_DIR)/*.o lib/*.o
	@touch $(BIN_DIR)/.gitkeep lib/.gitkeep
