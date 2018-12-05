CXX := clang++
CXXFLAGS := \
	-std=c++17 \
	$(shell pkg-config --libs --cflags eigen3) \
	-Wall -Wextra -Wpedantic -Werror \
	-Wno-unused-function
all: example
