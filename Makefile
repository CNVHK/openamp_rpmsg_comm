CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -I./src
PYTHON ?= python
BUILD_DIR := build

.PHONY: all test clean

all: test

$(BUILD_DIR):
	$(PYTHON) -c "import os; os.makedirs('$(BUILD_DIR)', exist_ok=True)"

test: $(BUILD_DIR) $(BUILD_DIR)/test_protocol $(BUILD_DIR)/test_jc4010_can

$(BUILD_DIR)/test_protocol: src/rpmsg_protocol.c tests/test_protocol.c
	$(CC) $(CFLAGS) $^ -o $@

$(BUILD_DIR)/test_jc4010_can: remote_firmware/jc4010_can.c tests/test_jc4010_can.c
	$(CC) $(CFLAGS) -I./remote_firmware $^ -o $@

clean:
	$(PYTHON) -c "import shutil; shutil.rmtree('$(BUILD_DIR)', ignore_errors=True)"
