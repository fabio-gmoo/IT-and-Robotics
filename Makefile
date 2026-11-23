# =============================================================================
#  PongBot - Makefile
#    make            -> pongbot + herramientas (arm_test, joystick_test, hsv_tuner, calibración)
#    make pongbot    -> solo el programa principal
#    make legacy     -> versiones antiguas que ya funcionaron (plan B)
#    make clean
# =============================================================================
CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Iinclude $(shell pkg-config --cflags opencv4)
LIBS_CV  = $(shell pkg-config --libs opencv4)
LIBS_DXL = -L/usr/local/lib -ldxl_x64_cpp -lpthread -lrt

BIN = bin
OBJ = build

CORE_SRC = src/pongbot/config.cpp src/pongbot/arm.cpp src/pongbot/vision.cpp src/pongbot/fans.cpp \
           src/pongbot/joystick.cpp src/pongbot/controllers.cpp src/pongbot/sim.cpp \
           src/pongbot/game_gui.cpp
CORE_OBJ = $(patsubst src/pongbot/%.cpp,$(OBJ)/%.o,$(CORE_SRC))
HEADERS  = $(wildcard include/pongbot/*.hpp)

TOOLS = $(BIN)/arm_test $(BIN)/joystick_test $(BIN)/fan_test $(BIN)/hsv_tuner \
        $(BIN)/CameraCalibration $(BIN)/MonoCameraCalibration

all: $(BIN)/pongbot $(TOOLS)

pongbot: $(BIN)/pongbot
tools: $(TOOLS)

$(OBJ)/%.o: src/pongbot/%.cpp $(HEADERS)
	@mkdir -p $(OBJ)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN)/pongbot: src/pongbot/main.cpp $(CORE_OBJ) $(HEADERS)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) src/pongbot/main.cpp $(CORE_OBJ) -o $@ $(LIBS_CV) $(LIBS_DXL)

$(BIN)/arm_test: tools/arm_test.cpp $(OBJ)/arm.o $(OBJ)/config.o $(HEADERS)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) $< $(OBJ)/arm.o $(OBJ)/config.o -o $@ $(LIBS_CV) $(LIBS_DXL)

$(BIN)/joystick_test: tools/joystick_test.cpp $(OBJ)/joystick.o $(OBJ)/config.o $(HEADERS)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) $< $(OBJ)/joystick.o $(OBJ)/config.o -o $@ $(LIBS_CV)

$(BIN)/fan_test: tools/fan_test.cpp $(OBJ)/fans.o $(OBJ)/joystick.o $(OBJ)/config.o $(HEADERS)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) $< $(OBJ)/fans.o $(OBJ)/joystick.o $(OBJ)/config.o -o $@ $(LIBS_CV)

$(BIN)/hsv_tuner: tools/hsv_tuner.cpp
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LIBS_CV)

$(BIN)/CameraCalibration: tools/CameraCalibration.cpp
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -w $< -o $@ $(LIBS_CV)

$(BIN)/MonoCameraCalibration: tools/MonoCameraCalibration.cpp tools/ReadWriteFunctions.cpp
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -w -Itools $^ -o $@ $(LIBS_CV)

# ---- Plan B: código antiguo tal cual estaba (ver legacy/README.md) ----
LEG = legacy/pong_robot_valeria
legacy: $(BIN)/legacy_autonomous $(BIN)/legacy_joystick_teleop $(BIN)/legacy_interactive_test

$(BIN)/legacy_autonomous: $(LEG)/RobotServoing_autonomous.cpp $(LEG)/pong_robot.hpp
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -w -I$(LEG) $< -o $@ $(LIBS_CV) $(LIBS_DXL)

$(BIN)/legacy_joystick_teleop: $(LEG)/joystick_teleop.cpp $(LEG)/pong_robot.hpp
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -w -I$(LEG) $< -o $@ $(LIBS_DXL)

$(BIN)/legacy_interactive_test: $(LEG)/interactive_test.cpp $(LEG)/pong_robot.hpp
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -w -I$(LEG) $< -o $@ $(LIBS_DXL)

clean:
	rm -rf $(OBJ) $(BIN)/*
	@touch $(BIN)/.gitkeep

.PHONY: all pongbot tools legacy clean

# ---- Autotest sin hardware ----
$(BIN)/selftest: tests/selftest.cpp $(CORE_OBJ) $(HEADERS)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -Ilegacy/pong_robot_valeria $< $(CORE_OBJ) -o $@ $(LIBS_CV) $(LIBS_DXL)

test: $(BIN)/selftest
	./$(BIN)/selftest

.PHONY: test
