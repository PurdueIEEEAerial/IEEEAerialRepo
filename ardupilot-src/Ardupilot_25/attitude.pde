/****************************************************************
	Function that will integrate waypoints, radio, stabilization and failsafe
 ****************************************************************/
void stabilize_AP_mix()
{
	if(GPS_fix == FAILED_GPS){
		nav_roll = roll_max / 2;
	}

	float inf 	= (float)ch1_in - (float)ch1_trim;
	inf 		= abs(inf);									
	inf 		= min(inf, 400.0);							
	inf 		= ((400.0 - inf) /400.0);

	servo_roll 		= calc_attitude_roll(nav_roll - roll_sensor);
	servo_roll 		= inf * servo_roll;	// scale down roll to mix in stick later

	inf 		= (float)ch2_in - (float)ch2_trim;				
	inf 		= abs(inf);									
	inf 		= min(inf, 400.0);							
	inf 		= ((400.0 - inf) /400.0);

	servo_pitch 		= calc_attitude_pitch(nav_pitch - pitch_sensor);
	servo_pitch 		= inf * servo_pitch;	// scale down pitch to mix in stick later

	// add in some pitch up while turning
	// ----------------------------------
	servo_pitch += abs(nav_roll * PITCH_COMP);
	
	// write out the servo PWM values
	// ------------------------------
	set_degrees_mix();

	
	// NOTE - Airspeed is stored and used in the program as an integer pressure value
	// Use the formula:  pressure = 0.1254 * speed * speed 
	// where speed is the airspeed in meters per second.
	// For example if you want airspeed_min to be 10 meters per second use a value of 13
	// 100 MPH = (about) 45 M/S = (about) 254 calculated pressure value

	// throttle control with airspeed compensation
	// -------------------------------------------
	#if AIRSPEED_SENSOR == 1
		servo_throttle = throttle_cruise + calc_attitude_throttle(throttle_cruise + nav_airspeed - airspeed_current); // function located in control_attitude.pde 
	#else
		// we don't have an airsensor
		// --------------------------
		servo_throttle = throttle_cruise + calc_attitude_throttle(nav_airspeed); // function located in control_attitude.pde 
	#endif
}

//Function that controls rudder/aileron, elevator and throttle to produce desired attitude and airspeed.
void stabilize_AP()     
{
	if(GPS_fix == FAILED_GPS){       
		nav_roll = roll_max / 2;   		// If we have lost gps and have no ability to navigate we will circle at a gentle bank angle
	}									// This is a second form of failsafe, different from losing radio signal.

	servo_roll 		= calc_attitude_roll(nav_roll - roll_sensor);
	servo_pitch 	= calc_attitude_pitch(nav_pitch - pitch_sensor);
	
	// add in some pitch up while turning to keep the nose from dropping 
	// -----------------------------------------------------------------
	servo_pitch += abs(nav_roll * PITCH_COMP);
	
	// write out the servo PWM values
	// ------------------------------
	set_degrees();

	// throttle control with airspeed compensation
	// -------------------------------------------
	servo_throttle = throttle_cruise + calc_attitude_throttle(throttle_cruise + nav_airspeed - airspeed_current); // function located in control_attitude.pde 
}

//This function is like stabilize_AP(), but is only used for STABILIZE mode
void stabilize()
{
	// Setup proportional influence based on stick position
	// more stick = less stabilize
	// ---------------------------------------------------------------
	float inf 	= (float)ch1_in - (float)ch1_trim;				
	inf 		= abs(inf);									
	inf 		= min(inf, 200.0);							
	inf 		= ((200.0 - inf) /200.0);					

	servo_roll 		= calc_attitude_roll(-roll_sensor);
	servo_roll 		= inf * servo_roll;// scale down roll to mix in stick later

	// Setup proportional influence based on stick position
	// more stick = less stabilize
	// ----------------------------------------------------
	inf 		= (float)ch2_in - (float)ch2_trim;			
	inf 		= abs(inf);
	inf 		= min(inf, 200.0);					
	inf 		= ((200.0 - inf) /200.0);

	servo_pitch 		= calc_attitude_pitch(-pitch_sensor);
	servo_pitch 		= inf * servo_pitch;// scale down pitch to mix in stick later
	
	#if THROTTLE_IN == 0
		servo_throttle = throttle_cruise;
	#endif
	
	set_degrees_mix();
}


