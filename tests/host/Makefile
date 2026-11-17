# Register-level host tests for the ICM-42688-P driver.
#
#   make test       build and run (from the repository root: make -C tests/host test)
#
# The driver is compiled for the host against the stubs in stub/ and talks to
# icm_stub.cpp, a model of the chip's register banks that logs every access.
CXX      ?= g++
SRC      ?= ../../src
CXXFLAGS ?= -O1 -g -std=gnu++11 -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation
CXXFLAGS += $(CXXFLAGS_EXTRA)

test_icm_filters: test_icm_filters.cpp icm_stub.cpp icm_stub.h stub/*.h $(SRC)/ICM42688P_voltino.cpp $(SRC)/ICM42688P_voltino.h
	$(CXX) $(CXXFLAGS) -Istub -I$(SRC) -o $@ test_icm_filters.cpp icm_stub.cpp $(SRC)/ICM42688P_voltino.cpp -lm

test: test_icm_filters
	./test_icm_filters

clean:
	rm -f test_icm_filters
.PHONY: test clean
