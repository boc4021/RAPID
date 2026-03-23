# =============================================================================
# Makefile - RAPID PC-side build
#
# Builds RAPID.exe: the PC utility that reads a GDS input file,
# converts coordinates to polar, and streams them over UART to the FPGA.
#
# Requirements:
#   MinGW-w64 / MSYS2 UCRT64  (gcc must be on PATH)
#   Windows (uses Win32 serial API)
#
# Usage:
#   make                       - build build/RAPID.exe
#   make clean                 - remove build artefacts
#   make run                   - build and run (uses default PORT and FILE below)
#   make run PORT=COM3 FILE=my.gds
#   make gui                   - launch the Python GUI (requires venv)
#
# Project layout:
#   src/        - all source files (C and Python)
#   build/      - compiled output (generated, not committed)
#   input.gds   - default input file
#
# Note:
#   vitis_workspace/systemControl/main.c targets the Zynq PS bare-metal
#   environment and must be built inside Xilinx Vitis, not here.
# =============================================================================

# ---- Toolchain ---------------------------------------------------------------
CC      := gcc
CFLAGS  := -O2 -Wall -Wextra -std=c11
LDFLAGS := -lm

# ---- Directories -------------------------------------------------------------
SRCDIR   := src
BUILDDIR := build

# ---- Target ------------------------------------------------------------------
TARGET  := $(BUILDDIR)/RAPID.exe

# ---- Sources -----------------------------------------------------------------
SRCS    := $(SRCDIR)/main.c $(SRCDIR)/framing.c $(SRCDIR)/inputParser.c
OBJS    := $(SRCS:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
DEPS    := $(OBJS:.o=.d)

# ---- Runtime defaults (override on command line) -----------------------------
PORT    := COM25
FILE    := input.gds

# ==============================================================================

.PHONY: all clean run gui e2e check-proto sync-proto \
        test-framing test-inputparser test-unit-c test-unit-py test-unit

all: $(TARGET)

# Create build directory if it doesn't exist
$(BUILDDIR):
	mkdir -p $(BUILDDIR)

# Link
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Built $@"

# Compile into build/ with automatic dependency tracking
$(BUILDDIR)/%.o: $(SRCDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Pull in generated header dependency files (silently ignored on first build)
-include $(DEPS)

# Run RAPID.exe directly
run: $(TARGET)
	./$(TARGET) $(PORT) $(FILE)

gui:
	@if [ -f venv/Scripts/python.exe ]; then \
		venv/Scripts/python src/gui.py; \
	else \
		python src/gui.py; \
	fi

# End-to-end test: requires com0com + pyserial (see tests/e2e_test.py)
SIM_PORT ?= COM4
PC_PORT  ?= COM6
e2e: $(TARGET)
	$(MAKE) -C tests e2e SIM_PORT=$(SIM_PORT) PC_PORT=$(PC_PORT)

# Verify all protocol copies are in sync.
# Checks: (1) src/protocol.h vs vitis_workspace/systemControl/protocol.h  (#define lines)
#         (2) src/protocol.h vs tests/protocol.py  (constant values)
check-proto:
	@diff src/protocol.h vitis_workspace/systemControl/protocol.h \
	  && echo "check-proto [1/2]: C copies are identical." \
	  || { echo "ERROR: protocol.h C copies have diverged! Run 'make sync-proto'."; exit 1; }
	@python3 tests/check_proto_py.py
	@echo "check-proto [2/2]: Python constants match src/protocol.h."

# Copy the canonical src/protocol.h into the Vitis build tree.
# Run this after editing src/protocol.h to keep the FPGA copy in sync.
sync-proto:
	cp src/protocol.h vitis_workspace/systemControl/protocol.h
	@echo "Synced src/protocol.h -> vitis_workspace/systemControl/protocol.h"

# ==============================================================================
# Unit tests (no hardware required)
# ==============================================================================

# C unit tests — framing (CRC, pack/unpack, FSM)
test-framing: $(BUILDDIR)/test_framing
	./$(BUILDDIR)/test_framing

$(BUILDDIR)/test_framing: tests/test_framing.c $(BUILDDIR)/framing.o | $(BUILDDIR)
	$(CC) $(CFLAGS) -I $(SRCDIR) -o $@ tests/test_framing.c $(BUILDDIR)/framing.o -lm

# C unit tests — inputParser (getCoordinates, convertToPolar)
test-inputparser: $(BUILDDIR)/test_inputparser
	./$(BUILDDIR)/test_inputparser

$(BUILDDIR)/test_inputparser: tests/test_inputparser.c $(BUILDDIR)/inputParser.o | $(BUILDDIR)
	$(CC) $(CFLAGS) -I $(SRCDIR) -o $@ tests/test_inputparser.c $(BUILDDIR)/inputParser.o -lm

# All C unit tests
test-unit-c: test-framing test-inputparser

# Python unit tests (no hardware, no COM port)
test-unit-py:
	python -m pytest tests/test_protocol.py tests/test_protocol_parser.py -v

# All unit tests (C + Python)
test-unit: test-unit-c test-unit-py

# Remove all build artefacts
clean:
	rm -rf $(BUILDDIR)
