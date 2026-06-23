		// Internal scaling and unit conversions as per https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/imu_sensor_notes.md
		// Use SPI calibration and convert them to Gs
		acc_cal_coeff[0] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][0]))) * 4.0f;
		acc_cal_coeff[1] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][1]))) * 4.0f;
		acc_cal_coeff[2] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][2]))) * 4.0f;

		// Use SPI calibration and convert them to degrees per second
		gyro_cal_coeff[0] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][0])));
		gyro_cal_coeff[1] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][1])));
		gyro_cal_coeff[2] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][2])));

		// Device colours
		body_colour =
			(((int)device_colours[0]) << 16) +
			(((int)device_colours[1]) << 8) +
			(((int)device_colours[2]));
		button_colour =
			(((int)device_colours[3]) << 16) +
			(((int)device_colours[4]) << 8) +
			(((int)device_colours[5]));
		left_grip_colour =
			(((int)device_colours[6]) << 16) +
			(((int)device_colours[7]) << 8) +
			(((int)device_colours[8]));
		right_grip_colour =
			(((int)device_colours[9]) << 16) +
			(((int)device_colours[10]) << 8) +
			(((int)device_colours[11]));

		printf("Body: %#08x; Buttons: %#08x; Left Grip: %#08x; Right Grip: %#08x;\n",
			body_colour,
			button_colour,
			left_grip_colour,
			right_grip_colour);

		//hex_dump(reinterpret_cast<unsigned char*>(sensor_cal[0]), 6);
		//hex_dump(reinterpret_cast<unsigned char*>(sensor_cal[1]), 6);

		return true;
	}

	void enable_IMU(unsigned char *buf, int bufLength) {
		memset(buf, 0, bufLength);

		// Enable IMU data
		printf("Enabling IMU data...\n");
		if (controller_type == ControllerType::s_ds4)
		{
			if (is_usb)
			{
				init_ds4_bt();
				enable_gyro_ds4_bt(buf, bufLength);
			}
			else
			{
				init_ds4_usb();
			}
		}
		else
		{
			buf[0] = 0x01; // Enabled
			send_subcommand(0x1, 0x40, buf, 1);
		}
	}

	bool init_usb() {
		unsigned char buf[0x400];
		memset(buf, 0, 0x400);

		// set blocking:
		// this insures we get the MAC Address
		hid_set_nonblocking(this->handle, 0);

		//Get MAC Left
		printf("Getting MAC...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x01;
		hid_exchange(this->handle, buf, 0x2);

		//if (buf[2] == 0x3) {
		//	printf("%s disconnected!\n", this->name.c_str());
		//}
		//else {
		//	printf("Found %s, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", this->name.c_str(), buf[9], buf[8], buf[7], buf[6], buf[5], buf[4]);
		//}

		// set non-blocking:
		//hid_set_nonblocking(jc->handle, 1);

		// Do handshaking
		printf("Doing handshake...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x02;
		hid_exchange(this->handle, buf, 0x2);

		// Switch baudrate to 3Mbit
		printf("Switching baudrate...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x03;
		hid_exchange(this->handle, buf, 0x2);

		//Do handshaking again at new baudrate so the firmware pulls pin 3 low?
		printf("Doing handshake...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x02;
		hid_exchange(this->handle, buf, 0x2);

		//Only talk HID from now on
		printf("Only talk HID...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x04;
		hid_exchange(this->handle, buf, 0x2);

		// Enable vibration
		printf("Enabling vibration...\n");
		memset(buf, 0x00, 0x400);
		buf[0] = 0x01; // Enabled
		send_subcommand(0x1, 0x48, buf, 1);

		enable_IMU(buf, 0x400);

		printf("Getting calibration data...\n");
		bool result = get_switch_controller_info();

		if (result)
		{
			printf("Successfully initialized %s!\n", this->name.c_str());
		}
		else
		{
			printf("Could not initialise %s! Will try again later.\n", this->name.c_str());
		}
		return result;
	}

	bool init_bt() {
		bool result = true;
		unsigned char buf[0x40];
		memset(buf, 0, 0x40);
		printf("Initialising Bluetooth connection...\n");

		// set blocking to ensure command is recieved:
		hid_set_nonblocking(this->handle, 0);

		// first, check if this is a USB connection
		buf[0] = 0x80;
		buf[1] = 0x01;
		hid_write(this->handle, buf, 2);
		// wait for up to 5 messages for a USB acknowledgement
		for (int idx = 0; idx < 5; idx++)
		{
			if (hid_read_timeout(this->handle, buf, 0x40, 200) && buf[0] == 0x81)
			{
				//printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				//	buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10]);
				printf("Attempting USB connection\n");

				// it's usb!
				is_usb = true;

				init_usb();
				return 1;

				break;
			}
			//printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			//	buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10]);
			printf("Not a USB response...\n");
		}
		memset(buf, 0, 0x40);
		//if (hid_exchange(this->handle, buf, 2))
		//{
		//	printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		//		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10]);
		//	printf("Attempting USB connection\n");
		//	// it's usb!
		//	is_usb = true;
		//
		//	init_usb();
		//	return 1;
		//}
		buf[1] = 0x00;

		// Enable vibration
		printf("Enabling vibration...\n");
		buf[0] = 0x01; // Enabled
		send_subcommand(0x1, 0x48, buf, 1);

		//printf("Set vibration\n");

		// Enable IMU data
		enable_IMU(buf, 0x40);

		// Set input report mode (to push at 60hz)
		// x00	Active polling mode for IR camera data. Answers with more than 300 bytes ID 31 packet
		// x01	Active polling mode
		// x02	Active polling mode for IR camera data.Special IR mode or before configuring it ?
		// x21	Unknown.An input report with this ID has pairing or mcu data or serial flash data or device info
		// x23	MCU update input report ?
		// 30	NPad standard mode. Pushes current state @60Hz. Default in SDK if arg is not in the list
		// 31	NFC mode. Pushes large packets @60Hz
		printf("Set input report mode to 0x30...\n");
		buf[0] = 0x30;
		send_subcommand(0x01, 0x03, buf, 1);

		// @CTCaer

		// get calibration data:
		printf("Getting calibration data...\n");
		result = get_switch_controller_info();

		return result;
	}

	void init_ds4_bt() {
		#ifdef _DEBUG
			printf("initialise, set colour\n");
		#endif
		unsigned char buf[78];
		memset(buf, 0, 78);

		//buf[0] = 0x11;
		//buf[1] = 0x80;
		//buf[3] = 0xff;

		// https://github.com/Ryochan7/DS4Windows/blob/jay/DS4Windows/DS4Library/DS4Device.cs
		buf[0] = 0x15;
		buf[1] = 0xC0 | 1;
		buf[2] = 0xA0;
		buf[3] = 0xf7;
		buf[4] = 0x04;

		//// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
		//buf[0] = 0xa2; // 0x80;
		////buf[1] = 0xff;
		//// trying to do colour stuff
		//// http://eleccelerator.com/wiki/index.php?title=DualShock_4
		//// this is only for bt
		//buf[1] = 0x11;
		//buf[2] = 0xc0;
		//buf[3] = 0x20;
		//buf[4] = 0xf3;
		//buf[5] = 0x04;
		//// rumble
		//buf[7] = 0xFF;
		//buf[8] = 0x00;
		//// colour
		//buf[9] = 0x00;
		//buf[10] = 0x00;
		//buf[11] = 0x00;
		//// flash time
		//buf[12] = 0xff;
		//buf[13] = 0x00;
		//// now we need a CRC-32 of previous bytes
		//uint32_t crc = crc_32(buf, 75);
		//buf[75] = (crc >> 24) & 0xFF;
		//buf[76] = (crc >> 16) & 0xFF;
		//buf[77] = (crc >> 8) & 0xFF;
		//buf[78] = crc & 0xFF;

		//// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
		//buf[0] = 0x80;
		//buf[1] = 0xff;
		//// trying to do colour stuff
		//// http://eleccelerator.com/wiki/index.php?title=DualShock_4
		//// this is only for bt
		//buf[2] = 0x11;
		//// rumble
		//buf[6] = 0xFF;
		//buf[7] = 0xFF;	
		//// colour
		//buf[8] = 0xFF; // 0x00;
		//buf[9] = 0x80; // 0x00;
		//buf[10] = 0x00;
		//// flash time
		//buf[11] = 0xff;
		//buf[12] = 0x00;
		//// now we need a CRC-32 of previous bytes
		//uint32_t crc = crc_32(buf, 75);
		//buf[75] = (crc >> 24) & 0xFF;
		//buf[76] = (crc >> 16) & 0xFF;
		//buf[77] = (crc >> 8) & 0xFF;
		//buf[78] = crc & 0xFF;

		// set blocking:
		// this insures we get the MAC Address
		hid_set_nonblocking(this->handle, 0);

		hid_write(handle, buf, 78);

		// initialise stuff
		memset(factory_stick_cal, 0, 0x12);
		memset(device_colours, 0, 0xC);
		memset(user_stick_cal, 0, 0x16);
		memset(sensor_model, 0, 0x6);
		memset(stick_model, 0, 0x12);
		memset(factory_sensor_cal, 0, 0x18);
		memset(user_sensor_cal, 0, 0x1A);
		memset(factory_sensor_cal_calm, 0, 0xC);
		memset(user_sensor_cal_calm, 0, 0xC);
		memset(sensor_cal, 0, sizeof(sensor_cal));
		memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
		memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
		memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
		memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));
		stick_cal_x_l[0] =
			stick_cal_y_l[0] =
			stick_cal_x_r[0] =
			stick_cal_y_r[0] = 0;
		stick_cal_x_l[1] =
			stick_cal_y_l[1] =
			stick_cal_x_r[1] =
			stick_cal_y_r[1] = 127;
		stick_cal_x_l[2] =
			stick_cal_y_l[2] =
			stick_cal_x_r[2] =
			stick_cal_y_r[2] = 255;
		//// Acc cal origin position
		//sensor_cal[0][0] = 0;
		//sensor_cal[0][1] = 0;
		//sensor_cal[0][2] = 0;
		//
		//// Gyro cal origin position
		//sensor_cal[1][0] = 0;
		//sensor_cal[1][1] = 0;
		//sensor_cal[1][2] = 0;

		enable_gyro_ds4_bt(buf, 78);

		initialised = true;
	}

	// placeholder to get things working quickly. overdue for a refactor
	void init_ds_usb() {
		// initialise stuff
		memset(factory_stick_cal, 0, 0x12);
		memset(device_colours, 0, 0xC);
		memset(user_stick_cal, 0, 0x16);
		memset(sensor_model, 0, 0x6);
		memset(stick_model, 0, 0x12);
		memset(factory_sensor_cal, 0, 0x18);
		memset(user_sensor_cal, 0, 0x1A);
		memset(factory_sensor_cal_calm, 0, 0xC);
		memset(user_sensor_cal_calm, 0, 0xC);
		memset(sensor_cal, 0, sizeof(sensor_cal));
		memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
		memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
		memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
		memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));
		stick_cal_x_l[0] =
			stick_cal_y_l[0] =
			stick_cal_x_r[0] =
			stick_cal_y_r[0] = 0;
		stick_cal_x_l[1] =
			stick_cal_y_l[1] =
			stick_cal_x_r[1] =
			stick_cal_y_r[1] = 127;
		stick_cal_x_l[2] =
			stick_cal_y_l[2] =
			stick_cal_x_r[2] =
			stick_cal_y_r[2] = 255;

		initialised = true;
	}

	// this is mostly copied from init_usb() below, but modified to speak DS4
	void init_ds4_usb() {
		unsigned char buf[31];
		memset(buf, 0, 31);

		// report id?
		buf[0] = 0x05;
		// I dunno what this is
		buf[1] = 0xf7;
		buf[2] = 0x04;
		//// http://eleccelerator.com/wiki/index.php?title=DualShock_4
		//// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
		//// rumble
		//buf[4] = 0x00;
		//buf[5] = 0x00;
		//// colour
		//buf[6] = 0x00;
		////buf[7] = 0xff;
		//buf[7] = 0x00;
		//buf[8] = 0x00;
		//// flash time
		//buf[9] = 0xff;
		//buf[10] = 0x00;
		// now we need a CRC-32 of previous bytes
		//uint32_t = crc_32(buf, 75);
		//buf[75] = 

		// set blocking:
		// this insures we get the MAC Address
		hid_set_nonblocking(this->handle, 0);

		hid_write(handle, buf, 31);

		// initialise stuff
		memset(factory_stick_cal, 0, 0x12);
		memset(device_colours, 0, 0xC);
		memset(user_stick_cal, 0, 0x16);
		memset(sensor_model, 0, 0x6);
		memset(stick_model, 0, 0x12);
		memset(factory_sensor_cal, 0, 0x18);
		memset(user_sensor_cal, 0, 0x1A);
		memset(factory_sensor_cal_calm, 0, 0xC);
		memset(user_sensor_cal_calm, 0, 0xC);
		memset(sensor_cal, 0, sizeof(sensor_cal));
		memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
		memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
		memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
		memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));
		stick_cal_x_l[0] =
			stick_cal_y_l[0] =
			stick_cal_x_r[0] =
			stick_cal_y_r[0] = 0;
		stick_cal_x_l[1] =
			stick_cal_y_l[1] =
			stick_cal_x_r[1] =
			stick_cal_y_r[1] = 127;
		stick_cal_x_l[2] =
			stick_cal_y_l[2] =
			stick_cal_x_r[2] =
			stick_cal_y_r[2] = 255;
		//// Acc cal origin position
		//sensor_cal[0][0] = 0;
		//sensor_cal[0][1] = 0;
		//sensor_cal[0][2] = 0;
		//
		//// Gyro cal origin position
		//sensor_cal[1][0] = 0;
		//sensor_cal[1][1] = 0;
		//sensor_cal[1][2] = 0;

		initialised = true;
	}

	void deinit_ds4_bt() {
		// TODO. For now, init, which stops rumbling and disables light
		init_ds4_bt();

		initialised = false;
	}

	// TODO: implement this
	void deinit_ds4_usb() {
		unsigned char buf[40];
		memset(buf, 0, 40);

		// report id?
		buf[0] = 0x05;
		// don't know what this is
		buf[1] = 0xff;
		// rumble
		buf[4] = 0x00;
		buf[5] = 0x00;
		// colour
		buf[6] = 0x00;
		buf[7] = 0x00;
		buf[8] = 0x00;
		// flash time
		buf[9] = 0x00;
		buf[10] = 0x00;
		// now we need a CRC-32 of previous bytes
		//uint32_t = crc_32(buf, 75);
		//buf[75] = 

		// set non-blocking
		hid_set_nonblocking(this->handle, 1);

		hid_write(handle, buf, 31);

		initialised = false;
	}

	void deinit_usb() {
		unsigned char buf[0x40];
		memset(buf, 0x00, 0x40);

		//Let the Joy-Con talk BT again    
		buf[0] = 0x80;
		buf[1] = 0x05;

		hid_set_nonblocking(this->handle, 1);
		hid_write(handle, buf, 0x2);

		initialised = false;
	}

	void set_ds5_rumble_light(unsigned char smallRumble, unsigned char bigRumble,
                           unsigned char colourR,
                           unsigned char colourG,
                           unsigned char colourB,
                           unsigned char playerlights) {
	    if(!is_usb) {
	        set_ds5_rumble_light_bt(smallRumble, bigRumble, colourR, colourG, colourB, playerlights);
	    }
	    else {
            set_ds5_rumble_light_usb(smallRumble, bigRumble, colourR, colourG, colourB, playerlights);
        }

	}

	void set_ds4_rumble_light(unsigned char smallRumble, unsigned char bigRumble,
		unsigned char colourR,
		unsigned char colourG,
		unsigned char colourB) {
		if (!is_usb) {
			set_ds4_rumble_light_bt(smallRumble, bigRumble, colourR, colourG, colourB);
		}
		else {
			set_ds4_rumble_light_usb(smallRumble, bigRumble, colourR, colourG, colourB);
		}
	}

	void set_ds4_rumble_light_usb(unsigned char smallRumble, unsigned char bigRumble,
		unsigned char colourR,
		unsigned char colourG,
		unsigned char colourB) {
		// todo: based on bluetoothness, switch report id to 0x11, offset everything by 2 -- basically use init stuff as basis
		unsigned char buf[40];
		memset(buf, 0, 40);

		// report id?
		buf[0] = 0x05;
		// don't know what this is
		buf[1] = 0xff;
		// rumble
		buf[4] = smallRumble;
		buf[5] = bigRumble;
		// colour
		buf[6] = colourR;
		buf[7] = colourG;
		buf[8] = colourB;
		// flash time
		buf[9] = 0xff;
		buf[10] = 0x00;
		// now we need a CRC-32 of previous bytes
		//uint32_t = crc_32(buf, 75);
		//buf[75] = 

		hid_write(handle, buf, 31);
	}

	void set_ds4_rumble_light_bt(unsigned char smallRumble, unsigned char bigRumble,
		unsigned char colourR,
		unsigned char colourG,
		unsigned char colourB) {
		unsigned char buf[79];
		memset(buf, 0, 79);

		// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
		//buf[0] = 0xa2; // 0x80;
		//buf[1] = 0xff;
		// trying to do colour stuff
		// http://eleccelerator.com/wiki/index.php?title=DualShock_4
		// this is only for bt

		buf[0] = 0xa2; // Output report header, needs to be included in crc32
		buf[1] = 0x11; // Output report 0x11
		buf[2] = 0xc0; // HID + CRC according to hid-sony
		buf[3] = 0x20; // ????
		buf[4] = 0x07; // Set blink + leds + motor
		buf[5] = 0x00;
		buf[6] = 0x00;
		// rumble
		buf[7] = smallRumble;
		buf[8] = bigRumble;
		// colour
		buf[9] = colourR;
		buf[10] = colourG;
		buf[11] = colourB;
		// flash time
		buf[12] = 0xff;
		buf[13] = 0x00;
		// now we need a CRC-32 of previous bytes

		/*
		// test
        buf[0] = 0xa2; // Output report header, needs to be included in crc32
        buf[1] = 0x11; // Output report 0x11
        buf[2] = 0xc0; // HID + CRC according to hid-sony
        buf[3] = 0x00; // ????
        buf[4] = 0x07; // Set blink + leds + motor
        buf[5] = 0x00;
        buf[6] = 0x00;
        buf[7] = 0xff;
        buf[8] = 0xff;
        buf[9] = 0xff;
        buf[10] = 0xff;
        buf[11] = 0xff;
        buf[12] = 0xff;
        buf[22] = 0x43;
        buf[23] = 0x43;
        buf[25] = 0x4d;
        buf[26] = 0x85;
*/

		uint32_t crc = crc_32(buf, 75);
		memcpy(&buf[75], &crc, 4);
		//buf[75] = (crc >> 24) & 0xFF;
		//buf[76] = (crc >> 16) & 0xFF;
		//buf[77] = (crc >> 8) & 0xFF;
		//buf[78] = crc & 0xFF;

		hid_write(handle, &buf[1], 78);
	}

    void set_ds5_rumble_light_usb(unsigned char smallRumble, unsigned char bigRumble,
                                 unsigned char colourR,
                                 unsigned char colourG,
                                 unsigned char colourB,
                                 unsigned char playerlights) { // DS5 actually has player lights.
        unsigned char buf[79];
        memset(buf, 0, 79);

        // https://github.com/Ryochan7/DS4Windows/blob/jay/DS4Windows/DS4Library/InputDevices/DualSenseDevice.cs
        // DS4Windows to the rescue.
        // Also thanks to Neilk1 for sharing his doc on the DS5 protocol.

        // Header & Report Information
        buf[0] = 0xa2; // Output report header, needs to be included in crc32
        buf[1] = 0x02; // DualSense output report is 0x02 for USB
        //buf[1] = 0x02; // DATA (0x02)


        buf[2] = 0x03;

        buf[3] = 0x54; // Toggle LED Strips, player lights, motor effect. Ignore Mic LED

        // Rumble emulation bytes.
        buf[4] = smallRumble;
        buf[5] = bigRumble;

        // 7-10 are mostly just audio settings.

        // Mute Button state. 0x00 = off, 0x01 = solid, 0x02 = pulsating.
        buf[10] = 0x00;

        // Skip to about 41, since we are ignoring trigger effect data.
        // Enable LED brightness
        buf[40] = 0x02; // ???
        buf[41] = 0x02;
        buf[44] = 0x02;

        // Controls the player lights, which the DS5 has.
        // Last two bits are unused - unset them to avoid issues.
        buf[45] = playerlights;
        buf[45] &= ~(1 << 7);
        buf[45] &= ~(1 << 8);

        // colour
        buf[46] = colourR;
        buf[47] = colourG;
        buf[48] = colourB;

        // USB does not send CRC32

        //uint32_t crc = crc_32(buf, 74);
        //memcpy(&buf[74], &crc, 4);
        //buf[75] = (crc >> 24) & 0xFF;
        //buf[76] = (crc >> 16) & 0xFF;
        //buf[77] = (crc >> 8) & 0xFF;
        //buf[78] = crc & 0xFF;

        hid_write(handle, &buf[1], 74);
    }

	// Calling the Dualsense anything but the DS5 is confusing, since DS also = DualShock, and the DualSense is the PS5 Controller anyway
    void set_ds5_rumble_light_bt(unsigned char smallRumble, unsigned char bigRumble,
                                 unsigned char colourR,
                                 unsigned char colourG,
                                 unsigned char colourB,
                                 unsigned char playerlights) { // DS5 actually has player lights.
        unsigned char buf[79];
        memset(buf, 0, 79);

        // https://github.com/Ryochan7/DS4Windows/blob/jay/DS4Windows/DS4Library/InputDevices/DualSenseDevice.cs
        // DS4Windows to the rescue.
        // Also thanks to Neilk1 for sharing his doc on the DS5 protocol.

        // Header & Report Information
        buf[0] = 0xa2; // Output report header, needs to be included in crc32
        buf[1] = 0x31; // DualSense output report is 0x31
        buf[2] = 0x02; // DATA (0x02)

        // Comment stolen from DS4Windows:
        // 0x01 Set the main motors (also requires flag 0x02)
        // 0x02 Set the main motors (also requires flag 0x01)
        // 0x04 Set the right trigger motor
        // 0x08 Set the left trigger motor
        // 0x10 Enable modification of audio volume
        // 0x20 Enable internal speaker (even while headset is connected)
        // 0x40 Enable modification of microphone volume
        // 0x80 Enable internal mic (even while headset is connected)
        buf[3] = 0x03;

        // Comment stolen from DS4Windows:
        // 0x01 Toggling microphone LED, 0x02 Toggling Audio/Mic Mute
        // 0x04 Toggling LED strips on the sides of the Touchpad, 0x08 Turn off all LED lights
        // 0x10 Toggle player LED lights below Touchpad, 0x20 ???
        // 0x40 Adjust overall motor/effect power, 0x80 ???
        buf[4] = 0x54; // Toggle LED Strips, player lights, motor effect. Ignore Mic LED

        // Rumble emulation bytes.
        buf[5] = smallRumble;
        buf[6] = bigRumble;

        // 7-10 are mostly just audio settings.

        // Mute Button state. 0x00 = off, 0x01 = solid, 0x02 = pulsating.
        buf[11] = 0x00;

        // Skip to about 41, since we are ignoring trigger effect data.
        // Enable LED brightness
        buf[41] = 0x02; // ???
        buf[44] = 0x02;
        buf[45] = 0x02;

        // Last two bits are unused - unset them to avoid issues.
        buf[46] = playerlights;
        buf[46] &= ~(1 << 7);
        buf[46] &= ~(1 << 8);

        // colour
        buf[47] = colourR;
        buf[48] = colourG;
        buf[49] = colourB;

        uint32_t crc = crc_32(buf, 75);
        memcpy(&buf[75], &crc, 4);
        //buf[75] = (crc >> 24) & 0xFF;
        //buf[76] = (crc >> 16) & 0xFF;
        //buf[77] = (crc >> 8) & 0xFF;
        //buf[78] = crc & 0xFF;

        hid_write(handle, &buf[1], 78);
    }

	//// mfosse credits Hypersect (Ryan Juckett), but I've removed deadzones so the consuming application can deal with them
	//// http://blog.hypersect.com/interpreting-analog-sticks/
	void CalcAnalogStick2
	(
		float &pOutX,       // out: resulting stick X value
		float &pOutY,       // out: resulting stick Y value
		uint16_t x,              // in: initial stick X value
		uint16_t y,              // in: initial stick Y value
		uint16_t x_calc[3],      // calc -X, CenterX, +X
		uint16_t y_calc[3]       // calc -Y, CenterY, +Y
	)
	{

		float x_f, y_f;

		// convert to float based on calibration and valid ranges per +/-axis
		x = clamp(x, x_calc[0], x_calc[2]);
		y = clamp(y, y_calc[0], y_calc[2]);
		if (x >= x_calc[1]) {
			x_f = (float)(x - x_calc[1]) / (float)(x_calc[2] - x_calc[1]);
		}
		else {
			x_f = -((float)(x - x_calc[1]) / (float)(x_calc[0] - x_calc[1]));
		}
		if (y >= y_calc[1]) {
			y_f = (float)(y - y_calc[1]) / (float)(y_calc[2] - y_calc[1]);
		}
		else {
			y_f = -((float)(y - y_calc[1]) / (float)(y_calc[0] - y_calc[1]));
		}

		pOutX = x_f;
		pOutY = y_f;
	}

	// SPI (@CTCaer):
	bool get_spi_data(uint32_t offset, const uint8_t read_len, uint8_t *test_buf) {
		int res;
		uint8_t buf[0x100];
		while (1) {
			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr *)buf;
			auto pkt = (brcm_cmd_01 *)(hdr + 1);
			hdr->cmd = 1;
			hdr->rumble[0] = timing_byte;

			buf[1] = timing_byte;

			timing_byte++;
			if (timing_byte > 0xF) {
				timing_byte = 0x0;
			}
			pkt->subcmd = 0x10;
			pkt->offset = offset;
			pkt->size = read_len;

			for (int i = 11; i < 22; ++i) {
				buf[i] = buf[i + 3];
			}

			res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt));

			res = hid_read_timeout(handle, buf, sizeof(buf), 1000);
			if (res == 0)
			{
				return false;
			}

			if ((*(uint16_t*)&buf[0xD] == 0x1090) && (*(uint32_t*)&buf[0xF] == offset)) {
				break;
			}
		}
		if (res >= 0x14 + read_len) {
			for (int i = 0; i < read_len; i++) {
				test_buf[i] = buf[0x14 + i];
			}
		}

		return true;
	}

	int write_spi_data(uint32_t offset, const uint8_t write_len, uint8_t* test_buf) {
		int res;
		uint8_t buf[0x100];
		int error_writing = 0;
		while (1) {
			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr *)buf;
			auto pkt = (brcm_cmd_01 *)(hdr + 1);
			hdr->cmd = 1;
			hdr->rumble[0] = timing_byte;
			timing_byte++;
			if (timing_byte > 0xF) {
				timing_byte = 0x0;
			}
			pkt->subcmd = 0x11;
			pkt->offset = offset;
			pkt->size = write_len;
			for (int i = 0; i < write_len; i++) {
				buf[0x10 + i] = test_buf[i];
			}
			res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt) + write_len);

			res = hid_read(handle, buf, sizeof(buf));

			if (*(uint16_t*)&buf[0xD] == 0x1180)
				break;

			error_writing++;
			if (error_writing == 125) {
				return 1;
			}
		}

		return 0;

	}
};