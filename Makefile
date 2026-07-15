CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -I./src
PYTHON ?= python3
BUILD_DIR := build

.PHONY: all test client gimbal clean

all: test client gimbal

$(BUILD_DIR):
	$(PYTHON) -c "import os; os.makedirs('$(BUILD_DIR)', exist_ok=True)"

test: $(BUILD_DIR) $(BUILD_DIR)/test_protocol $(BUILD_DIR)/test_motor_can \
	$(BUILD_DIR)/test_motor_balance_protocol $(BUILD_DIR)/test_lqr_controller

client: $(BUILD_DIR) $(BUILD_DIR)/rpmsg_client

gimbal: $(BUILD_DIR) $(BUILD_DIR)/gimbal_test

$(BUILD_DIR)/test_protocol: src/rpmsg_protocol.c src/rpmsg_protocol.h tests/test_protocol.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_motor_can: remote_firmware/motor_can.c remote_firmware/motor_can.h tests/test_motor_can.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_motor_balance_protocol: remote_firmware/motor_can.c remote_firmware/motor_can.h tests/motor_balance_protocol_test.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_lqr_controller: remote_firmware/lqr_controller.c remote_firmware/lqr_controller.h tests/lqr_controller_test.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -lm -o $@

$(BUILD_DIR)/gimbal_test: src/rpmsg_protocol.c src/rpmsg_protocol.h linux_user/gimbal_test.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/rpmsg_client: src/rpmsg_protocol.c src/rpmsg_protocol.h linux_user/rpmsg_client.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

clean:
	$(PYTHON) -c "import shutil; shutil.rmtree('$(BUILD_DIR)', ignore_errors=True)"
