CC     ?= gcc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Iinclude
BUILD  := build

.PHONY: all test clean

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/mavtest: src/mavlink.c test/test_mavlink.c include/mavlink.h | $(BUILD)
	$(CC) $(CFLAGS) src/mavlink.c test/test_mavlink.c -o $@

test: $(BUILD)/mavtest
	./$(BUILD)/mavtest

clean:
	rm -rf $(BUILD)
