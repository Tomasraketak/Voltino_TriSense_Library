# Host build of the library against a simulated TriSense module.
#
#   make            build ./trace from ../../src and run it into trace.txt
#   make compare REF=<git ref>
#                   build the same harness against the library at REF and
#                   require a byte-identical trace - the check for refactors
#                   that must not change behaviour
#   make accuracy && ./accuracy
#                   attitude error against the simulated ground truth, for
#                   builds that are allowed to differ (feature switches)
#
# Extra flags: make CXXFLAGS_EXTRA=-DTRISENSE_LITE=1, -D__ARM_FP=1 (the
# hardware-FPU invSqrt path), ...
CXX      ?= g++
SRC      ?= ../../src
CXXFLAGS ?= -O2 -std=gnu++11 -Wall -Wno-unused-parameter -Wno-misleading-indentation
CXXFLAGS += $(CXXFLAGS_EXTRA)

LIBSRC = TriSense.cpp ICM42688P_voltino.cpp AK09918C.cpp BMP580.cpp

trace: trace.cpp sim.cpp stub/*.h $(addprefix $(SRC)/,$(LIBSRC))
	$(CXX) $(CXXFLAGS) -Istub -I$(SRC) -o $@ trace.cpp sim.cpp $(addprefix $(SRC)/,$(LIBSRC)) -lm

accuracy: accuracy.cpp sim.cpp stub/*.h $(addprefix $(SRC)/,$(LIBSRC))
	$(CXX) $(CXXFLAGS) -Istub -I$(SRC) -o $@ accuracy.cpp sim.cpp $(addprefix $(SRC)/,$(LIBSRC)) -lm

trace.txt: trace
	./trace > $@

all: trace.txt

compare: trace.txt
	@test -n "$(REF)" || (echo "usage: make compare REF=<git ref>"; exit 2)
	rm -rf .ref && mkdir .ref && git -C ../.. archive $(REF) src | tar -x -C .ref
	$(MAKE) --no-print-directory SRC=.ref/src trace && mv trace trace.ref && ./trace.ref > trace.ref.txt
	$(MAKE) --no-print-directory -B trace
	cmp trace.ref.txt trace.txt && echo "IDENTICAL to $(REF)"

clean:
	rm -rf trace accuracy trace.ref trace.txt trace.ref.txt .ref
.PHONY: all compare clean
