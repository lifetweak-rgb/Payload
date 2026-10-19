CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -Iinclude -O2
SRCS = src/cattle_counter.cpp src/cattle_config.cpp src/cattle_report.cpp

.PHONY: test
test:
	$(CXX) $(CXXFLAGS) $(SRCS) test/test_cattle_counter.cpp -o /tmp/test_cattle_counter -lm
	/tmp/test_cattle_counter
	python3 test/test_send_report.py
