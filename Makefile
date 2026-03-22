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
#   vitis_workspace/systemControl/systemControl.c targets the Zynq PS bare-metal
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
SRCS    := $(SRCDIR)/pcCommunication.c $(SRCDIR)/framing.c $(SRCDIR)/inputParser.c
OBJS    := $(SRCS:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
DEPS    := $(OBJS:.o=.d)

# ---- Runtime defaults (override on command line) -----------------------------
PORT    := COM25
FILE    := input.gds

# ==============================================================================

.PHONY: all clean run gui e2e

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
SIM_PORT ?= COM10
PC_PORT  ?= COM11
e2e: $(TARGET)
	$(MAKE) -C tests e2e SIM_PORT=$(SIM_PORT) PC_PORT=$(PC_PORT)

# Remove all build artefacts
clean:
	rm -rf $(BUILDDIR)
