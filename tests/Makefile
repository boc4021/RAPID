# =============================================================================
# tests/Makefile - RAPID end-to-end test
#
# Requires:
#   - com0com virtual COM port pair (https://com0com.sourceforge.net/)
#   - pyserial: pip install pyserial
#   - RAPID.exe built: make (from project root)
#
# Usage:
#   make e2e                              - run with default ports (COM4/COM6)
#   make e2e SIM_PORT=COM4 PC_PORT=COM5  - custom port pair
# =============================================================================

SIM_PORT ?= COM4
PC_PORT  ?= COM6

.PHONY: e2e e2e-single e2e-large timeout-test

# Standard E2E test: 3-point fixture
e2e: ../build/RAPID.exe
	python e2e_test.py \
	    --sim-port $(SIM_PORT) \
	    --pc-port  $(PC_PORT) \
	    --gds      fixtures/e2e_small.gds \
	    --rapid    ../build/RAPID.exe

# Single-point edge-case E2E
e2e-single: ../build/RAPID.exe
	python e2e_test.py \
	    --sim-port $(SIM_PORT) \
	    --pc-port  $(PC_PORT) \
	    --gds      fixtures/e2e_single.gds \
	    --rapid    ../build/RAPID.exe

# 20-point stress E2E
e2e-large: ../build/RAPID.exe
	python e2e_test.py \
	    --sim-port $(SIM_PORT) \
	    --pc-port  $(PC_PORT) \
	    --gds      fixtures/e2e_large.gds \
	    --rapid    ../build/RAPID.exe

# ACK timeout path test (RAPID.exe must exit non-zero within ~12 s)
timeout-test: ../build/RAPID.exe
	python test_ack_timeout.py \
	    --sim-port $(SIM_PORT) \
	    --pc-port  $(PC_PORT) \
	    --gds      fixtures/e2e_small.gds \
	    --rapid    ../build/RAPID.exe
