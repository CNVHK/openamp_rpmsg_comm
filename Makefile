CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -I./src
PYTHON ?= python3
BUILD_DIR := build

.PHONY: all test gimbal clean

all: test gimbal

$(BUILD_DIR):
	$(PYTHON) -c "import os; os.makedirs('$(BUILD_DIR)', exist_ok=True)"

test: $(BUILD_DIR) $(BUILD_DIR)/test_protocol $(BUILD_DIR)/test_motor_can

gimbal: $(BUILD_DIR) $(BUILD_DIR)/gimbal_test

$(BUILD_DIR)/test_protocol: src/rpmsg_protocol.c tests/test_protocol.c
	$(CC) $(CFLAGS) $^ -o $@

$(BUILD_DIR)/test_motor_can: remote_firmware/motor_can.c tests/test_motor_can.c
	$(CC) $(CFLAGS) -I./remote_firmware $^ -o $@

$(BUILD_DIR)/gimbal_test: src/rpmsg_protocol.c linux_user/gimbal_test.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	$(PYTHON) -c "import shutil; shutil.rmtree('$(BUILD_DIR)', ignore_errors=True)"
