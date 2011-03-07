/*
Message Prefixes
!!!		Position    		Low rate telemetry
+++		Attitude    		High rate telemetry
###		Mode        		Change in control mode
%%%		Waypoint    		Current and previous waypoints
XXX		Alert       		Text alert  - NOTE: Alert message generation is not localized to a function
PPP		IMU Performance		Sent every 1/2 minute for performance monitoring

Message Suffix
***    All messages use this suffix
*/

/*void print_launch_params(void)
{
	Serial.print("wp_index = \t\t");
	Serial.println(wp_index,DEC);
	Serial.print("wp_total = \t\t");
	Serial.println(wp_total,DEC);
	Serial.print("ch1_trim = \t\t");
	Serial.println(ch1_trim,DEC);
	Serial.print("ch2_trim = \t\t");
	Serial.println(ch2_trim,DEC);
	Serial.print("ch3_trim = \t\t");
	Serial.println(ch3_trim,DEC);	
	Serial.print("ch1_min = \t");
	Serial.println(ch1_min,DEC);
	Serial.print("ch1_max = \t");
	Serial.println(ch1_max,DEC);
	Serial.print("ch2_min = \t");
	Serial.println(ch2_min,DEC);
	Serial.print("ch2_max = \t");
	Serial.println(ch2_max,DEC);
	Serial.print("Home Lat = \t\t");
	Serial.println(home.lat,DEC);
	Serial.print("Home Long = \t\t");
	Serial.println(home.lng,DEC);
	Serial.print("Home altitude = \t");
	Serial.println(home.alt,DEC);
}*/

void print_gains(void){
	Serial.print("%%");
	Serial.print(head_P * 100,DEC);
	Serial.print("|");
	Serial.print(head_I * 100,DEC);
	Serial.print("|");
	Serial.print(head_D * 100,DEC);
	Serial.print("|");
	Serial.print(pitch_P * 100,DEC);
	Serial.print("|");
	Serial.print(pitch_I * 100,DEC);
	Serial.print("|");
	Serial.print(rudder_P * 100,DEC);	
	Serial.print("|");
	Serial.print(rudder_I * 100,DEC);
	Serial.print("|");
	Serial.print(elevator_P * 100,DEC);
	Serial.print("|");
	Serial.print(elevator_I * 100,DEC);
	Serial.print("|");
	Serial.print(roll_max/100,DEC);
	Serial.print("|");
	Serial.println(pitch_comp * 100,DEC);
}
void print_radio()
{
	Serial.print("R:");
	Serial.print(ch1_in);
	Serial.print("\tE:");
	Serial.print(ch2_in);
	Serial.print("\tT:");
	Serial.println(ch3_in);
}
void print_current_waypoint(){
		Serial.print("prev_WP: ");
		Serial.print("\t");
		Serial.print(prev_WP.lat,DEC);
		Serial.print("\t");
		Serial.print(prev_WP.lng,DEC);
		Serial.print("\t");
		Serial.println(prev_WP.alt,DEC);
		
		Serial.print("next_WP:#");
		Serial.print(wp_index,DEC);
		Serial.print("\t");
		Serial.print(next_WP.lat,DEC);
		Serial.print("\t");
		Serial.print(next_WP.lng,DEC);
		Serial.print("\t");
		Serial.println(next_WP.alt,DEC);
}

void print_control_mode(void)
{
	switch (control_mode){
		case MANUAL:
			Serial.println("##0| MANUAL");
			break;
		case STABILIZE:
			Serial.println("##1| STABILIZE");
			break;
		case FLY_BY_WIRE:
			Serial.println("##2| FLY BY WIRE");
			break;
		case AUTO:
			Serial.println("##5| AUTOPILOT");
			break;
		case RTL:
			Serial.println("##6| RTL");
			break;
		case LOITER:
			Serial.println("##7| LOITER");
			break;
	}
	print_new_wp_info();
}

#if GROUNDSTATION == 1
void print_position()
{
	//!!377659260|-1224329073|5543|79|-56|5543|0|7982
	Serial.print("!!");
	Serial.print(current_loc.lat,DEC);					// 0
	Serial.print("|");
	Serial.print(current_loc.lng,DEC);					// 1
	Serial.print("|");
	Serial.print(current_loc.alt,DEC);					// 2
	Serial.print("|");
	Serial.print(ground_speed,DEC);					// 3
	Serial.print("|");	
	Serial.print(airspeed_current,DEC);			// 4
	Serial.print("|");
	Serial.print(get_altitude_above_home(),DEC);	// 5
	Serial.print("|");
	Serial.print(climb_rate,DEC);					// 6
	Serial.print("|");
	Serial.print(wp_distance,DEC);					// 7
	Serial.print("|");
	Serial.print(nav_airspeed,DEC);					// 8
	Serial.print("|");
	Serial.print(throttle_cruise,DEC);					// 9
	Serial.print("|");
	Serial.println(altitude_error,DEC);					// 10
}

void print_attitude()
{
	Serial.print("++");
	Serial.print(ch1_out,DEC);
	Serial.print("|");
	Serial.print(ch2_out,DEC);
	Serial.print("|");
	Serial.print(ch3_out,DEC);
	Serial.print("|");
	Serial.print(roll_sensor,DEC);
	Serial.print("|");
	Serial.print(pitch_sensor,DEC);
	Serial.print("|");
	Serial.print(ir_max,DEC);				// 9
	Serial.print("|");
	Serial.print(ground_course,DEC);				// 9
	Serial.print("|");
	Serial.print(target_bearing,DEC);				// 10
	Serial.print("|");
	Serial.print(nav_roll,DEC);				// 10
	Serial.print("|");
	Serial.print(derivative_roll,DEC);				// 10
	Serial.println("|");
	
}

// required by Groundstation to plot lateral tracking course 
void print_new_wp_info()
{
	Serial.print("??");
	Serial.print(wp_index,DEC);			//0
	Serial.print("|");
	Serial.print(prev_WP.lat,DEC);		//1
	Serial.print("|");
	Serial.print(prev_WP.lng,DEC);		//2
	Serial.print("|");
	Serial.print(prev_WP.alt,DEC);		//3
	Serial.print("|");
	Serial.print(next_WP.lat,DEC);		//4
	Serial.print("|");
	Serial.print(next_WP.lng,DEC);		//5
	Serial.print("|");
	Serial.print(next_WP.alt,DEC);		//6
	Serial.print("|");
	Serial.print(wp_totalDistance,DEC);	//7
	Serial.print("|");
	Serial.print(ch1_trim,DEC);			//8
	Serial.print("|");
	Serial.println(ch2_trim,DEC);			//9
}

#endif


#if GROUNDSTATION == 0
void print_position(void)
{
			Serial.print("!!!");
			Serial.print("LAT:");
			Serial.print(current_loc.lat/10,DEC);
			Serial.print(",LON:");
			Serial.print(current_loc.lng/10,DEC); //wp_current_lat
			Serial.print(",SPD:");
			Serial.print(ground_speed/100,DEC);		
			Serial.print(",CRT:");
			Serial.print(climb_rate,DEC);
			Serial.print(",ALT:");
			Serial.print(current_loc.alt,DEC);
			Serial.print(",ALH:");
			Serial.print(next_WP.alt/100,DEC);
			Serial.print(",CRS:");
			Serial.print(ground_course/100,DEC);
			Serial.print(",BER:");
			Serial.print(target_bearing/100,DEC);
			Serial.print(",WPN:");
			Serial.print(wp_index,DEC);//Actually is the waypoint.
			Serial.print(",DST:");
			Serial.print(wp_distance,DEC);
			Serial.print(",BTV:");
			Serial.print(battery_voltage,DEC);
			Serial.print(",RSP:");
			Serial.print(servo_roll/100,DEC);
			Serial.print(",TOW:");
			Serial.print(iTOW);
			Serial.println(",***");
			print_telemetry = false;
}
void print_new_wp_info()
{

}
void print_attitude(void)
{
	Serial.print("+++");
	Serial.print("ASP:");
	Serial.print(airspeed_current,DEC);
	Serial.print(",THH:");
	Serial.print(servo_throttle/125,DEC);
	Serial.print (",RLL:");
	Serial.print(roll_sensor/100,DEC);
	Serial.print (",PCH:");
	Serial.print(pitch_sensor/100,DEC);
	//Serial.print (",ir_max:");
	//Serial.print(ir_max,DEC);
	Serial.println(",***");
}
#endif

void print_current_loc(){
		Serial.print("loc: ");
		Serial.print(current_loc.lat,DEC);
		Serial.print("\t");
		Serial.print(current_loc.lng,DEC);
		Serial.print("\t");
		Serial.println(current_loc.alt,DEC);
}

void print_waypoints(byte wp_tot){
	
	Serial.print("wp_total: ");
	Serial.println(wp_tot, DEC);

	// create a location struct to hold the temp Waypoints for printing
	//Location tmp;
	struct Location tmp = get_loc_with_index(0);
	
	Serial.print("home: \t");
	Serial.print(tmp.lat, DEC);
	Serial.print("\t");
	Serial.print(tmp.lng, DEC);
	Serial.print("\t");
	Serial.println(tmp.alt,DEC);	

	for (int i = 1; i <= wp_tot; i++){
		tmp = get_loc_with_index(i);
		Serial.print("wp #");
		Serial.print(i);
		Serial.print("\t");
		Serial.print(tmp.lat, DEC);
		Serial.print("\t");
		Serial.print(tmp.lng, DEC);
		Serial.print("\t");
		Serial.println(tmp.alt,DEC);
	}
}


