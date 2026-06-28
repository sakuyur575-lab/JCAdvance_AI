#include "JoyShockLibrary.h"
#include "JoyShock.cpp"

#include <cmath>

bool handle_input(JoyShock *jc, uint8_t *packet, int len, bool &hasIMU) {
	hasIMU = true;
	if (packet[0] == 0) return false; // ignore non-responses

	// Создаем локальные буферы на стеке для безопасного разбора без блокировок
	JOY_SHOCK_STATE local_simple_state = jc->simple_state;
	local_simple_state.buttons = 0;

	TOUCH_STATE local_touch_state = jc->touch_state;

	IMU_STATE imu_state;

	// delta time
	auto time_now = std::chrono::steady_clock::now();
	jc->delta_time = (float)(std::chrono::duration_cast<std::chrono::microseconds>(time_now - jc->last_polled).count() / 1000000.0);
	jc->last_polled = time_now;

	if (jc->cue_motion_reset)
	{
		jc->cue_motion_reset = false;
		jc->motion.Reset();
	}

	if (jc->motion.GetCalibrationMode() == GamepadMotionHelpers::CalibrationMode::Manual)
	{
		if (jc->use_continuous_calibration)
		{
			jc->motion.StartContinuousCalibration();
		}
		else
		{
			jc->motion.PauseContinuousCalibration();
		}
	}

	// ds4
	if (jc->controller_type == ControllerType::s_ds4) {
		int indexOffset = 0;
		bool isValid = true;
		if (!jc->is_usb) {
			isValid = packet[0] == 0x11;
			indexOffset = 2;
		}
		else {
			isValid = packet[0] == 0x01;
			if (isValid && (packet[31] & 0x04) == 0x04)
				return false; // ignore packets from Dongle with no connected controller
		}
		if (isValid) {
			// Gyroscope:
			int16_t gyroSampleX = uint16_to_int16(packet[indexOffset + 13] | (packet[indexOffset + 14] << 8) & 0xFF00);
			int16_t gyroSampleY = uint16_to_int16(packet[indexOffset + 15] | (packet[indexOffset + 16] << 8) & 0xFF00);
			int16_t gyroSampleZ = uint16_to_int16(packet[indexOffset + 17] | (packet[indexOffset + 18] << 8) & 0xFF00);
			int16_t accelSampleX = uint16_to_int16(packet[indexOffset + 19] | (packet[indexOffset + 20] << 8) & 0xFF00);
			int16_t accelSampleY = uint16_to_int16(packet[indexOffset + 21] | (packet[indexOffset + 22] << 8) & 0xFF00);
			int16_t accelSampleZ = uint16_to_int16(packet[indexOffset + 23] | (packet[indexOffset + 24] << 8) & 0xFF00);

			if ((gyroSampleX | gyroSampleY | gyroSampleZ | accelSampleX | accelSampleY | accelSampleZ) == 0)
			{
				hasIMU = false;
			}

			// convert to real units
			imu_state.gyroX = (float)(gyroSampleX) * (2000.0f / 32767.0f);
			imu_state.gyroY = (float)(gyroSampleY) * (2000.0f / 32767.0f);
			imu_state.gyroZ = (float)(gyroSampleZ) * (2000.0f / 32767.0f);

			imu_state.accelX = (float)(accelSampleX) / 8192.0f;
			imu_state.accelY = (float)(accelSampleY) / 8192.0f;
			imu_state.accelZ = (float)(accelSampleZ) / 8192.0f;

			// Touchpad (запись в локальный стек):
			local_touch_state.t0Id = (int)(packet[indexOffset + 35] & 0x7F);
			local_touch_state.t1Id = (int)(packet[indexOffset + 39] & 0x7F);
			local_touch_state.t0Down = (packet[indexOffset + 35] & 0x80) == 0;
			local_touch_state.t1Down = (packet[indexOffset + 39] & 0x80) == 0;

			local_touch_state.t0X = (packet[indexOffset + 36] | (packet[indexOffset + 37] & 0x0F) << 8) / 1920.0f;
			local_touch_state.t0Y = ((packet[indexOffset + 37] & 0xF0) >> 4 | packet[indexOffset + 38] << 4) / 943.0f;
			local_touch_state.t1X = (packet[indexOffset + 40] | (packet[indexOffset + 41] & 0x0F) << 8) / 1920.0f;
			local_touch_state.t1Y = ((packet[indexOffset + 41] & 0xF0) >> 4 | packet[indexOffset + 42] << 4) / 943.0f;

			// DS4 dpad
			uint8_t hat = packet[indexOffset + 5] & 0x0f;

			if ((hat > 2) & (hat < 6)) local_simple_state.buttons |= JSMASK_DOWN;
			if ((hat == 7) | (hat < 2)) local_simple_state.buttons |= JSMASK_UP;
			if ((hat > 0) & (hat < 4)) local_simple_state.buttons |= JSMASK_RIGHT;
			if ((hat > 4) & (hat < 8)) local_simple_state.buttons |= JSMASK_LEFT;

			local_simple_state.buttons |= ((int)(packet[indexOffset + 5] >> 4) << JSOFFSET_W) & JSMASK_W;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 5] >> 7) << JSOFFSET_N) & JSMASK_N;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 5] >> 5) << JSOFFSET_S) & JSMASK_S;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 5] >> 6) << JSOFFSET_E) & JSMASK_E;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 6] >> 6) << JSOFFSET_LCLICK) & JSMASK_LCLICK;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 6] >> 7) << JSOFFSET_RCLICK) & JSMASK_RCLICK;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 6] >> 5) << JSOFFSET_OPTIONS) & JSMASK_OPTIONS;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 6] >> 4) << JSOFFSET_SHARE) & JSMASK_SHARE;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 6] >> 1) << JSOFFSET_R) & JSMASK_R;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 6]) << JSOFFSET_L) & JSMASK_L;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 7]) << JSOFFSET_PS) & JSMASK_PS;
			local_simple_state.buttons |= ((int)(packet[indexOffset + 7] >> 1) << JSOFFSET_TOUCHPAD_CLICK) & JSMASK_TOUCHPAD_CLICK;

			local_simple_state.rTrigger = packet[indexOffset + 9] / 255.0f;
			local_simple_state.lTrigger = packet[indexOffset + 8] / 255.0f;

			if (local_simple_state.rTrigger > 0.0) local_simple_state.buttons |= JSMASK_ZR;
			if (local_simple_state.lTrigger > 0.0) local_simple_state.buttons |= JSMASK_ZL;

			uint16_t stick_x = packet[indexOffset + 1];
			uint16_t stick_y = packet[indexOffset + 2];
			stick_y = 255 - stick_y;

			uint16_t stick2_x = packet[indexOffset + 3];
			uint16_t stick2_y = packet[indexOffset + 4];
			stick2_y = 255 - stick2_y;

			local_simple_state.stickLX = (std::fmin)(1.0f, (stick_x - 127.0f) / 127.0f);
			local_simple_state.stickLY = (std::fmin)(1.0f, (stick_y - 127.0f) / 127.0f);
			local_simple_state.stickRX = (std::fmin)(1.0f, (stick2_x - 127.0f) / 127.0f);
			local_simple_state.stickRY = (std::fmin)(1.0f, (stick2_y - 127.0f) / 127.0f);

			jc->modifying_lock.lock();
			jc->push_sensor_samples(imu_state.gyroX, imu_state.gyroY, imu_state.gyroZ,
				imu_state.accelX, imu_state.accelY, imu_state.accelZ, jc->delta_time);

			jc->get_calibrated_gyro(imu_state.gyroX, imu_state.gyroY, imu_state.gyroZ);

			// Быстрое атомарное обновление под мьютексом
			jc->last_simple_state = jc->simple_state;
			jc->simple_state = local_simple_state;

			jc->last_touch_state = jc->touch_state;
			jc->touch_state = local_touch_state;

			jc->last_imu_state = jc->imu_state;
			jc->imu_state = imu_state;
			jc->modifying_lock.unlock();
		}

		return true;
	}

	// DualSense
	if (jc->controller_type == ControllerType::s_ds) {
		int indexOffset = 1;
		if (!jc->is_usb) {
			indexOffset = 2;
		}

		// Gyroscope:
		int16_t gyroSampleX = uint16_to_int16(packet[indexOffset + 15] | (packet[indexOffset + 16] << 8) & 0xFF00);
		int16_t gyroSampleY = uint16_to_int16(packet[indexOffset + 17] | (packet[indexOffset + 18] << 8) & 0xFF00);
		int16_t gyroSampleZ = uint16_to_int16(packet[indexOffset + 19] | (packet[indexOffset + 20] << 8) & 0xFF00);
		int16_t accelSampleX = uint16_to_int16(packet[indexOffset + 21] | (packet[indexOffset + 22] << 8) & 0xFF00);
		int16_t accelSampleY = uint16_to_int16(packet[indexOffset + 23] | (packet[indexOffset + 24] << 8) & 0xFF00);
		int16_t accelSampleZ = uint16_to_int16(packet[indexOffset + 25] | (packet[indexOffset + 26] << 8) & 0xFF00);

		if ((gyroSampleX | gyroSampleY | gyroSampleZ | accelSampleX | accelSampleY | accelSampleZ) == 0) {
			hasIMU = false;
		}

		// convert to real units
		imu_state.gyroX = (float)(gyroSampleX) * (2000.0f / 32767.0f);
		imu_state.gyroY = (float)(gyroSampleY) * (2000.0f / 32767.0f);
		imu_state.gyroZ = (float)(gyroSampleZ) * (2000.0f / 32767.0f);

		imu_state.accelX = (float)(accelSampleX) / 8192.0f;
		imu_state.accelY = (float)(accelSampleY) / 8192.0f;
		imu_state.accelZ = (float)(accelSampleZ) / 8192.0f;

		// Touchpad:
		local_touch_state.t0Id = (int)(packet[indexOffset + 32] & 0x7F);
		local_touch_state.t1Id = (int)(packet[indexOffset + 36] & 0x7F);
		local_touch_state.t0Down = (packet[indexOffset + 32] & 0x80) == 0;
		local_touch_state.t1Down = (packet[indexOffset + 36] & 0x80) == 0;

		local_touch_state.t0X = (packet[indexOffset + 33] | (packet[indexOffset + 34] & 0x0F) << 8) / 1920.0f;
		local_touch_state.t0Y = ((packet[indexOffset + 34] & 0xF0) >> 4 | packet[indexOffset + 35] << 4) / 943.0f;
		local_touch_state.t1X = (packet[indexOffset + 37] | (packet[indexOffset + 38] & 0x0F) << 8) / 1920.0f;
		local_touch_state.t1Y = ((packet[indexOffset + 38] & 0xF0) >> 4 | packet[indexOffset + 39] << 4) / 943.0f;

		// DS dpad hat
		uint8_t hat = packet[indexOffset + 7] & 0x0f;

		if ((hat > 2) & (hat < 6)) local_simple_state.buttons |= JSMASK_DOWN;
		if ((hat == 7) | (hat < 2)) local_simple_state.buttons |= JSMASK_UP;
		if ((hat > 0) & (hat < 4)) local_simple_state.buttons |= JSMASK_RIGHT;
		if ((hat > 4) & (hat < 8)) local_simple_state.buttons |= JSMASK_LEFT;

		local_simple_state.buttons |= ((int)(packet[indexOffset + 7] >> 4) << JSOFFSET_W) & JSMASK_W;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 7] >> 5) << JSOFFSET_S) & JSMASK_S;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 7] >> 6) << JSOFFSET_E) & JSMASK_E;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 7] >> 7) << JSOFFSET_N) & JSMASK_N;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 8] >> 6) << JSOFFSET_LCLICK) & JSMASK_LCLICK;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 8] >> 7) << JSOFFSET_RCLICK) & JSMASK_RCLICK;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 8] >> 5) << JSOFFSET_OPTIONS) & JSMASK_OPTIONS;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 8] >> 4) << JSOFFSET_SHARE) & JSMASK_SHARE;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 8] >> 1) << JSOFFSET_R) & JSMASK_R;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 8]) << JSOFFSET_L) & JSMASK_L;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 9]) << JSOFFSET_PS) & JSMASK_PS;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 9] >> 1) << JSOFFSET_TOUCHPAD_CLICK) & JSMASK_TOUCHPAD_CLICK;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 9] >> 2) << JSOFFSET_MIC) & JSMASK_MIC;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 9] >> 4) << JSOFFSET_FNL) & JSMASK_FNL;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 9] >> 5) << JSOFFSET_FNR) & JSMASK_FNR;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 9] >> 6) << JSOFFSET_SL) & JSMASK_SL;
		local_simple_state.buttons |= ((int)(packet[indexOffset + 9] >> 7) << JSOFFSET_SR) & JSMASK_SR;

		local_simple_state.rTrigger = packet[indexOffset + 5] / 255.0f;
		local_simple_state.lTrigger = packet[indexOffset + 4] / 255.0f;

		if (local_simple_state.rTrigger > 0.0) local_simple_state.buttons |= JSMASK_ZR;
		if (local_simple_state.lTrigger > 0.0) local_simple_state.buttons |= JSMASK_ZL;

		uint16_t stick_x = packet[indexOffset + 0];
		uint16_t stick_y = packet[indexOffset + 1];
		stick_y = 255 - stick_y;

		uint16_t stick2_x = packet[indexOffset + 2];
		uint16_t stick2_y = packet[indexOffset + 3];
		stick2_y = 255 - stick2_y;

		local_simple_state.stickLX = (std::fmin)(1.0f, (stick_x - 127.0f) / 127.0f);
		local_simple_state.stickLY = (std::fmin)(1.0f, (stick_y - 127.0f) / 127.0f);
		local_simple_state.stickRX = (std::fmin)(1.0f, (stick2_x - 127.0f) / 127.0f);
		local_simple_state.stickRY = (std::fmin)(1.0f, (stick2_y - 127.0f) / 127.0f);

		jc->modifying_lock.lock();
		jc->push_sensor_samples(imu_state.gyroX, imu_state.gyroY, imu_state.gyroZ,
			imu_state.accelX, imu_state.accelY, imu_state.accelZ, jc->delta_time);

		jc->get_calibrated_gyro(imu_state.gyroX, imu_state.gyroY, imu_state.gyroZ);

		// Быстрое атомарное обновление под мьютексом
		jc->last_simple_state = jc->simple_state;
		jc->simple_state = local_simple_state;

		jc->last_touch_state = jc->touch_state;
		jc->touch_state = local_touch_state;

		jc->last_imu_state = jc->imu_state;
		jc->imu_state = imu_state;
		jc->modifying_lock.unlock();

		return true;
	}

	// JoyCon / Pro Controller
	if (packet[0] == 0x3F) {
		jc->dstick = packet[3];
	}

	int buttons_pressed = 0;

	if (packet[0] == 0x21 || packet[0] == 0x30 || packet[0] == 0x31) {
		int offset = 0;
		uint8_t *btn_data = packet + offset + 3;

		// get button states:
		{
			uint16_t states = 0;
			uint16_t states2 = 0;

			if (jc->left_right == 1) {
				states = (btn_data[1] << 8) | (btn_data[2] & 0xFF);
			}
			else if (jc->left_right == 2) {
				states = (btn_data[1] << 8) | (btn_data[0] & 0xFF);
			}
			else if (jc->left_right == 3) {
				states = (btn_data[1] << 8) | (btn_data[2] & 0xFF);
				states2 = (btn_data[1] << 8) | (btn_data[0] & 0xFF);
			}

			buttons_pressed = states;
			if (jc->left_right == 3) {
				buttons_pressed |= states2 << 16;
				buttons_pressed &= ~(1L << 9);
				buttons_pressed &= ~(1L << 12);
				buttons_pressed &= ~(1L << 14);

				buttons_pressed &= ~(1UL << (8 + 16));
				buttons_pressed &= ~(1UL << (11 + 16));
				buttons_pressed &= ~(1UL << (13 + 16));
			}
			else if (jc->left_right == 2) {
				buttons_pressed = buttons_pressed << 16;
			}
		}

		// get stick data:
		uint8_t *stick_data = packet + offset;
		if (jc->left_right == 1) {
			stick_data += 6;
		}
		else if (jc->left_right == 2) {
			stick_data += 9;
		}

		uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
		uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);

		// use calibration data:
		if (jc->left_right == 1) {
			jc->CalcAnalogStick2(local_simple_state.stickLX, local_simple_state.stickLY,
				stick_x,
				stick_y,
				jc->stick_cal_x_l,
				jc->stick_cal_y_l);
		}
		else if (jc->left_right == 2) {
			jc->CalcAnalogStick2(local_simple_state.stickRX, local_simple_state.stickRY,
				stick_x,
				stick_y,
				jc->stick_cal_x_r,
				jc->stick_cal_y_r);
		}
		else if (jc->left_right == 3) {
			stick_data += 6;
			uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
			uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);
			jc->CalcAnalogStick2(local_simple_state.stickLX, local_simple_state.stickLY,
				stick_x,
				stick_y,
				jc->stick_cal_x_l,
				jc->stick_cal_y_l);
			stick_data += 3;
			uint16_t stick_x2 = stick_data[0] | ((stick_data[1] & 0xF) << 8);
			uint16_t stick_y2 = (stick_data[1] >> 4) | (stick_data[2] << 4);
			jc->CalcAnalogStick2(local_simple_state.stickRX, local_simple_state.stickRY,
				stick_x2,
				stick_y2,
				jc->stick_cal_x_r,
				jc->stick_cal_y_r);
		}

		jc->battery = (stick_data[1] & 0xF0) >> 4;

		// Accelerometer:
		{
			float accelSampleZ = (float)uint16_to_int16(packet[13] | (packet[14] << 8) & 0xFF00) * jc->acc_cal_coeff[0];
			float accelSampleX = (float)uint16_to_int16(packet[15] | (packet[16] << 8) & 0xFF00) * jc->acc_cal_coeff[1];
			float accelSampleY = (float)uint16_to_int16(packet[17] | (packet[18] << 8) & 0xFF00) * jc->acc_cal_coeff[2];
			float gyroSampleX = (float)uint16_to_int16(packet[19] | (packet[20] << 8) & 0xFF00) * jc->gyro_cal_coeff[0];
			float gyroSampleY = (float)uint16_to_int16(packet[21] | (packet[22] << 8) & 0xFF00) * jc->gyro_cal_coeff[1];
			float gyroSampleZ = (float)uint16_to_int16(packet[23] | (packet[24] << 8) & 0xFF00) * jc->gyro_cal_coeff[2];

			if (gyroSampleX == 0.f && gyroSampleY == 0.f && gyroSampleZ == 0.f && accelSampleX == 0.f && accelSampleY == 0.f && accelSampleZ == 0.f)
			{
				hasIMU = false;
			}

			float accelX = accelSampleX;
			float accelY = accelSampleY;
			float accelZ = accelSampleZ;
			float totalGyroX = gyroSampleX - jc->sensor_cal[1][0];
			float totalGyroY = gyroSampleY - jc->sensor_cal[1][1];
			float totalGyroZ = gyroSampleZ - jc->sensor_cal[1][2];

			// sample 2
			accelSampleZ = (float)uint16_to_int16(packet[25] | (packet[26] << 8) & 0xFF00) * jc->acc_cal_coeff[0];
			accelSampleX = (float)uint16_to_int16(packet[27] | (packet[28] << 8) & 0xFF00) * jc->acc_cal_coeff[1];
			accelSampleY = (float)uint16_to_int16(packet[29] | (packet[30] << 8) & 0xFF00) * jc->acc_cal_coeff[2];
			gyroSampleX = (float)uint16_to_int16(packet[31] | (packet[32] << 8) & 0xFF00) * jc->gyro_cal_coeff[0];
			gyroSampleY = (float)uint16_to_int16(packet[33] | (packet[34] << 8) & 0xFF00) * jc->gyro_cal_coeff[1];
			gyroSampleZ = (float)uint16_to_int16(packet[35] | (packet[36] << 8) & 0xFF00) * jc->gyro_cal_coeff[2];

			accelX += accelSampleX;
			accelY += accelSampleY;
			accelZ += accelSampleZ;
			totalGyroX += gyroSampleX - jc->sensor_cal[1][0];
			totalGyroY += gyroSampleY - jc->sensor_cal[1][1];
			totalGyroZ += gyroSampleZ - jc->sensor_cal[1][2];

			// sample 3
			accelSampleZ = (float)uint16_to_int16(packet[37] | (packet[38] << 8) & 0xFF00) * jc->acc_cal_coeff[0];
			accelSampleX = (float)uint16_to_int16(packet[39] | (packet[40] << 8) & 0xFF00) * jc->acc_cal_coeff[1];
			accelSampleY = (float)uint16_to_int16(packet[41] | (packet[42] << 8) & 0xFF00) * jc->acc_cal_coeff[2];
			gyroSampleX = (float)uint16_to_int16(packet[43] | (packet[44] << 8) & 0xFF00) * jc->gyro_cal_coeff[0];
			gyroSampleY = (float)uint16_to_int16(packet[45] | (packet[46] << 8) & 0xFF00) * jc->gyro_cal_coeff[1];
			gyroSampleZ = (float)uint16_to_int16(packet[47] | (packet[48] << 8) & 0xFF00) * jc->gyro_cal_coeff[2];

			accelX += accelSampleX;
			accelY += accelSampleY;
			accelZ += accelSampleZ;
			totalGyroX += gyroSampleX - jc->sensor_cal[1][0];
			totalGyroY += gyroSampleY - jc->sensor_cal[1][1];
			totalGyroZ += gyroSampleZ - jc->sensor_cal[1][2];

			// average the 3 samples
			accelX /= 3;
			accelY /= 3;
			accelZ /= 3;
			totalGyroX /= 3;
			totalGyroY /= 3;
			totalGyroZ /= 3;

			imu_state.accelX = -accelX;
			imu_state.accelY = accelY;
			imu_state.accelZ = -accelZ;
			imu_state.gyroX = -totalGyroY;
			imu_state.gyroY = totalGyroZ;
			imu_state.gyroZ = totalGyroX;
		}
	}

	// handle buttons
	{
		// left:
		if (jc->left_right == 1) {
			local_simple_state.buttons |= ((buttons_pressed >> 1) << JSOFFSET_UP) & JSMASK_UP;
			local_simple_state.buttons |= ((buttons_pressed) << JSOFFSET_DOWN) & JSMASK_DOWN;
			local_simple_state.buttons |= ((buttons_pressed >> 3) << JSOFFSET_LEFT) & JSMASK_LEFT;
			local_simple_state.buttons |= ((buttons_pressed >> 2) << JSOFFSET_RIGHT) & JSMASK_RIGHT;
			local_simple_state.buttons |= ((buttons_pressed >> 11) << JSOFFSET_LCLICK) & JSMASK_LCLICK;
			local_simple_state.buttons |= ((buttons_pressed >> 8) << JSOFFSET_MINUS) & JSMASK_MINUS;
			local_simple_state.buttons |= ((buttons_pressed >> 6) << JSOFFSET_L) & JSMASK_L;
			local_simple_state.buttons |= ((buttons_pressed >> 13) << JSOFFSET_CAPTURE) & JSMASK_CAPTURE;
			local_simple_state.lTrigger = (float)((buttons_pressed >> 7) & 1);
			local_simple_state.buttons |= ((int)(local_simple_state.lTrigger) << JSOFFSET_ZL) & JSMASK_ZL;
			local_simple_state.buttons |= ((buttons_pressed >> 5) << JSOFFSET_SL) & JSMASK_SL;
			local_simple_state.buttons |= ((buttons_pressed >> 4) << JSOFFSET_SR) & JSMASK_SR;

			imu_state.gyroZ = -imu_state.gyroZ;
		}

		// right:
		if (jc->left_right == 2) {
			local_simple_state.buttons |= ((buttons_pressed >> 16) << JSOFFSET_W) & JSMASK_W;
			local_simple_state.buttons |= ((buttons_pressed >> 17) << JSOFFSET_N) & JSMASK_N;
			local_simple_state.buttons |= ((buttons_pressed >> 18) << JSOFFSET_S) & JSMASK_S;
			local_simple_state.buttons |= ((buttons_pressed >> 19) << JSOFFSET_E) & JSMASK_E;
			local_simple_state.buttons |= ((buttons_pressed >> 26) << JSOFFSET_RCLICK) & JSMASK_RCLICK;
			local_simple_state.buttons |= ((buttons_pressed >> 25) << JSOFFSET_PLUS) & JSMASK_PLUS;
			local_simple_state.buttons |= ((buttons_pressed >> 22) << JSOFFSET_R) & JSMASK_R;
			local_simple_state.buttons |= ((buttons_pressed >> 28) << JSOFFSET_HOME) & JSMASK_HOME;
			local_simple_state.rTrigger = (float)((buttons_pressed >> 23) & 1);
			local_simple_state.buttons |= ((int)(local_simple_state.rTrigger) << JSOFFSET_ZR) & JSMASK_ZR;
			local_simple_state.buttons |= ((buttons_pressed >> 21) << JSOFFSET_SL) & JSMASK_SL;
			local_simple_state.buttons |= ((buttons_pressed >> 20) << JSOFFSET_SR) & JSMASK_SR;

			imu_state.gyroX = -imu_state.gyroX;
			imu_state.gyroY = -imu_state.gyroY;
			imu_state.gyroZ = -imu_state.gyroZ;

			imu_state.accelX = -imu_state.accelX;
			imu_state.accelY = -imu_state.accelY;
		}

		// pro controller:
		if (jc->left_right == 3) {
			local_simple_state.buttons |= ((buttons_pressed >> 1) << JSOFFSET_UP) & JSMASK_UP;
			local_simple_state.buttons |= ((buttons_pressed) << JSOFFSET_DOWN) & JSMASK_DOWN;
			local_simple_state.buttons |= ((buttons_pressed >> 3) << JSOFFSET_LEFT) & JSMASK_LEFT;
			local_simple_state.buttons |= ((buttons_pressed >> 2) << JSOFFSET_RIGHT) & JSMASK_RIGHT;
			local_simple_state.buttons |= ((buttons_pressed >> 16) << JSOFFSET_W) & JSMASK_W;
			local_simple_state.buttons |= ((buttons_pressed >> 17) << JSOFFSET_N) & JSMASK_N;
			local_simple_state.buttons |= ((buttons_pressed >> 18) << JSOFFSET_S) & JSMASK_S;
			local_simple_state.buttons |= ((buttons_pressed >> 19) << JSOFFSET_E) & JSMASK_E;
			local_simple_state.buttons |= ((buttons_pressed >> 11) << JSOFFSET_LCLICK) & JSMASK_LCLICK;
			local_simple_state.buttons |= ((buttons_pressed >> 26) << JSOFFSET_RCLICK) & JSMASK_RCLICK;
			local_simple_state.buttons |= ((buttons_pressed >> 25) << JSOFFSET_PLUS) & JSMASK_PLUS;
			local_simple_state.buttons |= ((buttons_pressed >> 8) << JSOFFSET_MINUS) & JSMASK_MINUS;
			local_simple_state.buttons |= ((buttons_pressed >> 22) << JSOFFSET_R) & JSMASK_R;
			local_simple_state.buttons |= ((buttons_pressed >> 6) << JSOFFSET_L) & JSMASK_L;
			local_simple_state.buttons |= ((buttons_pressed >> 28) << JSOFFSET_HOME) & JSMASK_HOME;
			local_simple_state.buttons |= ((buttons_pressed >> 13) << JSOFFSET_CAPTURE) & JSMASK_CAPTURE;
			local_simple_state.rTrigger = (float)((buttons_pressed >> 23) & 1);
			local_simple_state.lTrigger = (float)((buttons_pressed >> 7) & 1);
			local_simple_state.buttons |= ((int)(local_simple_state.lTrigger) << JSOFFSET_ZL) & JSMASK_ZL;
			local_simple_state.buttons |= ((int)(local_simple_state.rTrigger) << JSOFFSET_ZR) & JSMASK_ZR;

			imu_state.gyroZ = -imu_state.gyroZ;
		}
	}

	jc->modifying_lock.lock();
	jc->push_sensor_samples(imu_state.gyroX, imu_state.gyroY, imu_state.gyroZ,
		imu_state.accelX, imu_state.accelY, imu_state.accelZ, jc->delta_time);

	jc->get_calibrated_gyro(imu_state.gyroX, imu_state.gyroY, imu_state.gyroZ);

	// Атомарно обновляем состояния под мьютексом
	jc->last_simple_state = jc->simple_state;
	jc->simple_state = local_simple_state;

	jc->last_touch_state = jc->touch_state;
	jc->touch_state = local_touch_state;

	jc->last_imu_state = jc->imu_state;
	jc->imu_state = imu_state;
	jc->modifying_lock.unlock();

	return true;
}