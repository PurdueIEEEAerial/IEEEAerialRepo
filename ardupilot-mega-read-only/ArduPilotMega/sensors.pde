// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

void init_barometer(void)
{
	int flashcount;
	long 			ground_pressure = 0;
	int 			ground_temperature;

	#if HIL_MODE == HIL_MODE_SENSORS
		hil.update();					// look for inbound hil packets for initialization
	#endif

	while(ground_pressure == 0){
		barometer.Read(); 					// Get initial data from absolute pressure sensor
		ground_pressure 	= barometer.Press;
		ground_temperature 	= barometer.Temp;
		delay(20);
		//Serial.printf("barometer.Press %ld\n", barometer.Press);
	}

	for(int i = 0; i < 30; i++){		// We take some readings...

		#if HIL_MODE == HIL_MODE_SENSORS
			hil.update(); 				// look for inbound hil packets
		#endif

		barometer.Read(); 				// Get initial data from absolute pressure sensor
		ground_pressure		= (ground_pressure * 9l   + barometer.Press) / 10l;
		ground_temperature	= (ground_temperature * 9 + barometer.Temp) / 10;

		delay(20);
		if(flashcount == 5) {
			digitalWrite(C_LED_PIN, LOW);
			digitalWrite(A_LED_PIN, HIGH);
			digitalWrite(B_LED_PIN, LOW);
		}

		if(flashcount >= 10) {
			flashcount = 0;
			digitalWrite(C_LED_PIN, HIGH);
			digitalWrite(A_LED_PIN, LOW);
			digitalWrite(B_LED_PIN, HIGH);
		}
		flashcount++;
	}
	
	g.ground_pressure.set_and_save(ground_pressure);
	g.ground_temperature.set_and_save(ground_temperature / 10.0f);
	abs_pressure = ground_pressure;
	
Serial.printf("abs_pressure %ld\n", abs_pressure);
	SendDebugln("barometer calibration complete.");
}

long read_barometer(void)
{
 	float x, scaling, temp;

	barometer.Read();		// Get new data from absolute pressure sensor


	//abs_pressure 			= (abs_pressure + barometer.Press) >> 1;		// Small filtering
	abs_pressure 			= ((float)abs_pressure * .7) + ((float)barometer.Press * .3);		// large filtering
	scaling 				= (float)g.ground_pressure / (float)abs_pressure;
	temp 					= ((float)g.ground_temperature) + 273.15f;
	x 						= log(scaling) * temp * 29271.267f;
	return 	(x / 10);
}

// in M/S * 100
void read_airspeed(void)
{
	#if GPS_PROTOCOL != GPS_PROTOCOL_IMU	// Xplane will supply the airspeed
		airspeed_raw 		= ((float)adc.Ch(AIRSPEED_CH) * .25) + (airspeed_raw * .75);
		airspeed_pressure 	= max(((int)airspeed_raw - airspeed_offset), 0);
		airspeed 			= sqrt((float)airspeed_pressure * g.airspeed_ratio) * 100;
	#endif

	calc_airspeed_errors();
}

void zero_airspeed(void)
{
	airspeed_raw = (float)adc.Ch(AIRSPEED_CH);
	for(int c = 0; c < 50; c++){
		delay(20);
		airspeed_raw = (airspeed_raw * .90) + ((float)adc.Ch(AIRSPEED_CH) * .10);
	}
	airspeed_offset = airspeed_raw;
}

#endif // HIL_MODE != HIL_MODE_ATTITUDE

#if BATTERY_EVENT == 1
void read_battery(void)
{
	battery_voltage1 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN1)) * .1 + battery_voltage1 * .9;
	battery_voltage2 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN2)) * .1 + battery_voltage2 * .9;
	battery_voltage3 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN3)) * .1 + battery_voltage3 * .9;
	battery_voltage4 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN4)) * .1 + battery_voltage4 * .9;

	#if BATTERY_TYPE == 0
		if(battery_voltage3 < LOW_VOLTAGE)
			low_battery_event();
		battery_voltage = battery_voltage3; // set total battery voltage, for telemetry stream
	#endif

	#if BATTERY_TYPE == 1
		if(battery_voltage4 < LOW_VOLTAGE)
			low_battery_event();
		battery_voltage = battery_voltage4; // set total battery voltage, for telemetry stream
	#endif
}
#endif


void read_current(void)
{
	current_voltage 	= CURRENT_VOLTAGE(analogRead(VOLTAGE_PIN_0)) * .1 	+ current_voltage * .9; //reads power sensor voltage pin
	current_amps 		= CURRENT_AMPS(analogRead(CURRENT_PIN_1)) * .1 		+ current_amps * .9; //reads power sensor current pin
	current_total		+= (current_amps * 0.27777) / delta_ms_medium_loop;
}


//v: 10.9453, a: 17.4023, mah: 8.2
