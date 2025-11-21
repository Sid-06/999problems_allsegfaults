CXX = g++
CXXFLAGS := -std=c++17 -I. -IPhase-1 -Wall -Wextra
RELEASE_FLAGS := -O2
DEBUG_FLAGS := -g -O0
# Address Sanitizer (detects memory leaks and out-of-bounds access)
ASAN_FLAGS := -fsanitize=address -g -O1 

# --- PHASE 1 CONFIGURATION ---
P1_SOURCES = Phase-1/SampleDriver.cpp \
             Phase-1/graph.cpp \
             Phase-1/kdt.cpp \
             Phase-1/knn.cpp \
             Phase-1/srtp.cpp

# --- TARGETS ---
.PHONY: all phase1 debug asan clean

# Default target (Release build)
all: CXXFLAGS += $(RELEASE_FLAGS)
all: phase1

# Target to build the executable
# We use $^ to refer to all source files and $@ for the output name
phase1: $(P1_SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Debug build: Forces a clean first to ensure flags are applied
debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: clean phase1

# ASAN build: Forces a clean first to ensure flags are applied
asan: CXXFLAGS += $(ASAN_FLAGS)
asan: clean phase1

# Clean up build files
clean:
	rm -f phase1 output.json