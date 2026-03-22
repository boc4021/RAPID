# =============================================================================
# tests/Makefile - RAPID end-to-end test
#
# Requires:
#   - com0com virtual COM port pair (https://com0com.sourceforge.net/)
#   - pyserial: pip install pyserial
#   - RAPID.exe built: make (from project root)
#
# Usage:
#   make e2e                              - run with default ports (COM10/COM11)
#   make e2e SIM_PORT=COM4 PC_PORT=COM5  - custom port pair
# =============================================================================

SIM_PORT ?= COM10
PC_PORT  ?= COM11

.PHONY: e2e

e2e: ../build/RAPID.exe
	python e2e_test.py \
	    --sim-port $(SIM_PORT) \
	    --pc-port  $(PC_PORT) \
	    --gds      fixtures/e2e_small.gds \
	    --rapid    ../build/RAPID.exe
