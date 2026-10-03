# Host unit tests for the hardware-independent modules.
# Runs on any machine with a C compiler: `make -C firmware/stm32/test`

CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -g -Wall -Wextra -Werror -Wshadow -fsanitize=address,undefined
INC      = -I../App/Inc

HOST_SRC = test_host.c ../App/Src/bme280.c ../App/Src/protocol.c
DIAG_SRC = test_diag.c ../App/Src/isotp.c ../App/Src/uds.c

test_host: $(HOST_SRC) ../App/Inc/bme280.h ../App/Inc/protocol.h
	$(CC) $(CFLAGS) $(INC) -o $@ $(HOST_SRC) -lm

test_diag: $(DIAG_SRC) ../App/Inc/isotp.h ../App/Inc/uds.h
	$(CC) $(CFLAGS) $(INC) -o $@ $(DIAG_SRC)

# Shared library used by tools/test_tools.py to run the Python UDS client
# against this exact C implementation.
libdiag.so: uds_host_shim.c ../App/Src/isotp.c ../App/Src/uds.c ../App/Inc/isotp.h ../App/Inc/uds.h
	$(CC) -std=c11 -O2 -Wall -Wextra -Werror -shared -fPIC $(INC) -o $@ uds_host_shim.c ../App/Src/isotp.c ../App/Src/uds.c

.PHONY: run clean
run: test_host test_diag
	./test_host
	./test_diag

clean:
	rm -f test_host test_diag libdiag.so

.DEFAULT_GOAL := run
