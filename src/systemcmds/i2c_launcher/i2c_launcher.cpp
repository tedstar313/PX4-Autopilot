/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdlib.h>
#include "i2c_launcher.hpp"
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <px4_arch/i2c_hw_description.h>

constexpr I2CLauncher::I2CDevice I2CLauncher::_devices[];

I2CLauncher::I2CLauncher() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

I2CLauncher::~I2CLauncher()
{
}

bool I2CLauncher::init()
{
	ScheduleOnInterval(1_s);

	return true;
}

void I2CLauncher::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}


	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	if (_armed) {
		// Don't try to configure devices when armed.
		return;
	}

	I2CBusIterator i2c_bus_iterator {I2CBusIterator::FilterType::ExternalBus};

	while (i2c_bus_iterator.next()) {
		scan_i2c_bus(i2c_bus_iterator.bus().bus);
	}
}

void I2CLauncher::scan_i2c_bus(int bus)
{
	struct i2c_master_s *i2c_dev = px4_i2cbus_initialize(bus);

	if (i2c_dev == nullptr) {
		PX4_ERR("invalid bus %d", bus);
		return;
	}

	for (unsigned i = 0; i < sizeof(_devices) / sizeof(_devices[0]); ++i) {

		if (_started[bus][i]) {
			continue;
		}

		const unsigned retries = 1;

		bool found = false;

		for (unsigned retry_count = 0; retry_count < retries; ++retry_count) {

			uint8_t send_data = 0;
			uint8_t recv_data = 0;
			i2c_msg_s msgv[2] {};

			// Send
			msgv[0].frequency = 100000;
			msgv[0].addr = _devices[i].i2c_addr;
			msgv[0].flags = 0;
			msgv[0].buffer = &send_data;
			msgv[0].length = sizeof(send_data);

			// Receive
			msgv[1].frequency = 100000;
			msgv[1].addr = _devices[i].i2c_addr;
			msgv[1].flags = I2C_M_READ;
			msgv[1].buffer = &recv_data;;
			msgv[1].length = sizeof(recv_data);

			if (I2C_TRANSFER(i2c_dev, &msgv[0], 2) == PX4_OK) {
				found = true;
				break;
			}
		}

		if (found) {
			char buf[32];
			snprintf(buf, sizeof(buf), "%s -X -b %d -t %d start", _devices[i].cmd, bus, bus);

			PX4_INFO("Found address 0x%x, running '%s'\n", _devices[i].i2c_addr, buf);

			// Try starting, if it succeeds we assume it's started and we no longer have to
			// check this device.
			const int ret = system(buf);

			if (ret == 0) {
				// Mark all devices with that address as started, at least for this bus
				// because even if there were more, we could not address them.
				for (unsigned j = 0; j < num_devices; ++j) {
					if (_devices[j].i2c_addr == _devices[i].i2c_addr) {
						_started[bus][j] = true;
					}
				}

				PX4_INFO("Started 0x%x successfully", _devices[i].i2c_addr);

			} else {
				PX4_INFO("Could not start 0x%x, returned %d", _devices[i].i2c_addr, ret);
			}
		}
	}

	px4_i2cbus_uninitialize(i2c_dev);
}

int I2CLauncher::task_spawn(int argc, char *argv[])
{
	I2CLauncher *instance = new I2CLauncher();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int I2CLauncher::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int I2CLauncher::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Daemon that starts drivers based on found I2C devices.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("i2c_launcher", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int i2c_launcher_main(int argc, char *argv[])
{
	return I2CLauncher::main(argc, argv);
}
