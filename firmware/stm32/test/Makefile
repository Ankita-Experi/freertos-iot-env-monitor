# Host unit tests for the hardware-independent modules (BME280 math, wire protocol).
# Runs on any machine with a C compiler: `make -C firmware/stm32/test`

CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -g -Wall -Wextra -Werror -Wshadow -fsanitize=address,undefined
SRC      = test_host.c ../App/Src/bme280.c ../App/Src/protocol.c

test_host: $(SRC) ../App/Inc/bme280.h ../App/Inc/protocol.h
	$(CC) $(CFLAGS) -I../App/Inc -o $@ $(SRC) -lm

.PHONY: run clean
run: test_host
	./test_host

clean:
	rm -f test_host

.DEFAULT_GOAL := run
