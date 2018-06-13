CXX := clang++
CXXFLAGS := -O3 -std=c++17 -lm $(shell pkg-config --libs --cflags eigen3)
all: example
