CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -I./src
PYTHON ?= python3
BUILD_DIR := build

.PHONY: all test test-python client logger gimbal broker install-rpmsg-broker install-gimbal-daemon clean

all: test client logger gimbal broker

$(BUILD_DIR):
	$(PYTHON) -c "import os; os.makedirs('$(BUILD_DIR)', exist_ok=True)"

test: $(BUILD_DIR) $(BUILD_DIR)/test_protocol $(BUILD_DIR)/test_motor_can \
	$(BUILD_DIR)/test_motor_balance_protocol $(BUILD_DIR)/test_lqr_controller \
	$(BUILD_DIR)/test_gimbal_controller $(BUILD_DIR)/test_servo_motion_controller \
	$(BUILD_DIR)/rpmsg-broker test-python

test-python: $(BUILD_DIR)/rpmsg-broker
	$(PYTHON) -m unittest tests/test_gimbal_daemon.py tests/test_rpmsg_broker.py

client: $(BUILD_DIR) $(BUILD_DIR)/rpmsg_client

logger: $(BUILD_DIR) $(BUILD_DIR)/balance_logger

gimbal: $(BUILD_DIR) $(BUILD_DIR)/gimbal_test

broker: $(BUILD_DIR) $(BUILD_DIR)/rpmsg-broker

install-rpmsg-broker: $(BUILD_DIR)/rpmsg-broker
	install -d -m 0755 /usr/local/sbin
	install -m 0755 $(BUILD_DIR)/rpmsg-broker /usr/local/sbin/rpmsg-broker
	install -m 0644 integration/systemd/rpmsg-broker.service /etc/systemd/system/rpmsg-broker.service

install-gimbal-daemon:
	install -d -m 0755 /usr/local/libexec
	install -m 0755 linux_user/gimbal_daemon.py /usr/local/libexec/gimbal-daemon
	install -m 0755 linux_user/gimbalctl.py /usr/local/bin/gimbalctl
	install -m 0644 integration/systemd/gimbal-daemon.service /etc/systemd/system/gimbal-daemon.service

$(BUILD_DIR)/test_protocol: src/rpmsg_protocol.c src/rpmsg_protocol.h tests/test_protocol.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_motor_can: remote_firmware/motor_can.c remote_firmware/motor_can.h tests/test_motor_can.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_motor_balance_protocol: remote_firmware/motor_can.c remote_firmware/motor_can.h tests/motor_balance_protocol_test.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_lqr_controller: remote_firmware/lqr_controller.c remote_firmware/lqr_controller.h tests/lqr_controller_test.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -lm -o $@

$(BUILD_DIR)/test_gimbal_controller: remote_firmware/gimbal_controller.c remote_firmware/gimbal_controller.h remote_firmware/motor_can.c remote_firmware/motor_can.h tests/gimbal_controller_test.c tests/stubs/fgeneric_timer.h tests/stubs/fparameters.h
	$(CC) $(CFLAGS) -I./tests/stubs -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_servo_motion_controller: remote_firmware/servo_motion_controller.c remote_firmware/servo_motion_controller.h remote_firmware/phytium_servo_port.h tests/servo_motion_controller_test.c tests/stubs/fgeneric_timer.h
	$(CC) $(CFLAGS) -I./tests/stubs -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/gimbal_test: src/rpmsg_protocol.c src/rpmsg_protocol.h src/rpmsg_transport.c src/rpmsg_transport.h linux_user/gimbal_test.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/rpmsg_client: src/rpmsg_protocol.c src/rpmsg_protocol.h src/rpmsg_transport.c src/rpmsg_transport.h linux_user/rpmsg_client.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/balance_logger: src/rpmsg_protocol.c src/rpmsg_protocol.h src/rpmsg_transport.c src/rpmsg_transport.h linux_user/balance_logger.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/rpmsg-broker: src/rpmsg_protocol.c src/rpmsg_protocol.h src/rpmsg_transport.h linux_user/rpmsg_broker.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -pthread -o $@

clean:
	$(PYTHON) -c "import shutil; shutil.rmtree('$(BUILD_DIR)', ignore_errors=True)"
