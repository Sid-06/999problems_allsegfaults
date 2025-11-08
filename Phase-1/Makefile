CXX = g++
CXXFLAGS := -std=c++17 -I. -Wall -Wextra
RELEASE_FLAGS := -O2
DEBUG_FLAGS := -g -O0

GRAPH_SRC := graph.cpp
SRTP_SRC := srtp.cpp
KNN_SRC := knn.cpp
TEST_SRC := test_parser.cpp
SAMPLE_SRC := SampleDriver.cpp

.PHONY:  parser parser debug asan clean

parser : $(TEST_SRC) $(GRAPH_SRC) $(SRTP_SRC) $(KNN_SRC)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) $^ -o $@
debug : CXXFLAGS += $(DEBUG_FLAGS)
asan  : CXXFLAGS += $(ASAN_FLAGS)
clean : -rm -f *.o *.exe 
