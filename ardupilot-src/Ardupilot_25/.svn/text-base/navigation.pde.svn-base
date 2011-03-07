float dlat,dlng;

void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (invalid_location || GPS_fix == BAD_GPS)
	{
		nav_roll = 0;
		nav_pitch = 0;
		return;
	}
	if(GPS_update & GPS_BOTH){
		GPS_update = GPS_NONE;
		
		// where we should be heading 
		// --------------------------
		target_bearing 	= get_bearing(&current_loc, &next_WP);
	
		// waypoint distance from plane
		// ----------------------------
		wp_distance = getDistance(&current_loc, &next_WP);
	
		// Course Error
		// ------------
		bearing_error = target_bearing - ground_course;
		// negative error = left turn
		// positive error = right turn
	
		
		if(control_mode == LOITER){
			float power;
			if (wp_distance < LOITER_RADIUS){
				power = (float)wp_distance / (float)LOITER_RADIUS;
				//Serial.print("inside power ");
				//Serial.println((power*100),DEC);
				bearing_error += 18000;
				bearing_error += power  * 9000;
			}else if (wp_distance < (LOITER_RADIUS*2)){
				power = (float)((LOITER_RADIUS*2) - wp_distance) / (float)LOITER_RADIUS;
				//Serial.print("outside power ");
				//Serial.println((power*100),DEC);
				bearing_error += power * -9000;
			}
		}
	
		// Wrap the values
		// ---------------
		if (bearing_error > 18000)	bearing_error -= 36000;
		if (bearing_error < -18000)	bearing_error += 36000;
	
		// Crosstrack Error
		// ----------------
		long crosstrack_error = (target_bearing - crosstrack_bearing);
	
		// Wrap the values
		// ---------------
		if (crosstrack_error > 18000)	crosstrack_error -= 36000;
		if (crosstrack_error < -18000)	crosstrack_error += 36000;
	
		
		// Add in Crosstrack Error
		// -----------------------
		if (abs(bearing_error) < 25) {
			//bearing_error += sin(radians(crosstrack_error/100)) * wp_distance * XTRACK_GAIN;
		} else if (abs(bearing_error) > 35) {
			// reset crosstrack_bearing
			//reset_crosstrack();
		}
		
		// Calculate the required roll of the plane
		// ----------------------------------------
		nav_roll = calc_nav_roll(bearing_error);
		// in degrees - positive = right turn; 35° limit
			
		// Altitude error
		// --------------
		altitude_error 	= (next_WP.alt - current_loc.alt);
		// altitude_error = "how much to climb"
		// lower than WP =  positive number
		
		// Altitude converted to elevator pitch in degrees
		// ------------------------------------------------
		nav_pitch = calc_nav_pitch(altitude_error);
		// pitch down is negative
	
	
		// Throttle is in percent or airspeed value, depaending on your setup.
		// change AIRSPEED_SENSOR in your header file to reflect your setup.
		// 
		// throttle cruising speed is the % throttle you want to be at if things are level and no wind
		// Altitude based throttle control + 15as throttle adjustment
		// ---------------------------------------------------------
		nav_airspeed = calc_nav_airspeed(altitude_error);
			
	
	
		// Are we there yet?
		// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
		// ------------------------------------------------------------------------
		if (control_mode == LOITER){
			// nothing to do really;
			
		}else if (wp_distance < 0){
			//
			// something went wrong!!!
			// if our waypoints are too large we can get an wrapped number
			// ----------------------------------------------------------------	
			/*
			Serial.print("wp_distance error, loc: ");
			Serial.print(current_loc.lat);
			Serial.print(", ");
			Serial.print(current_loc.lng);
			Serial.print("  next_WP ");
			Serial.print(next_WP.lat);
			Serial.print(", ");
			Serial.println(next_WP.lng);
			*/
		}else if(wp_distance < wp_radius) {  //10 meters
		
			//int alt_distance = get_alt_distance(&current_loc, &next_WP);
			//if(alt_distance < WP_ALTITUDE_RADIUS){
				waypoint_event(EVENT_WILL_REACH_WAYPOINT);
				reached_waypoint();
			//}
		}
	}
}

/****************************************************************
Function that will read and store the current altitude when you switch to autopilot mode.
 ****************************************************************/

int get_altitude_above_home(void)
{
	// This is the altitude above the home location
	// The GPS gives us altitude at Sea Level
	// if you slope soar, you should see a negative number sometimes
	// -------------------------------------------------------------
	return current_loc.alt - home.alt;
}

long getDistance(struct Location *loc1, struct Location *loc2)
{
	if(loc1->lat == 0 || loc1->lng == 0) 
		return -1;
	if(loc2->lat == 0 || loc2->lng == 0) 
		return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong  	= ((float)(loc2->lng - loc1->lng)) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

long get_alt_distance(struct Location *loc1, struct Location *loc2)
{
	return abs(loc1->alt - loc2->alt);
}

float getArea(struct Location *loc1, struct Location *loc2)
{
	return sq((float)(loc2->lat - loc1->lat)) + (sq((float)(loc2->lng - loc1->lng)) * scaleLongDown);
}

long get_bearing2(struct Location *loc1, struct Location *loc2)
{
	return 18000 + atan2((float)(loc1->lng - loc2->lng) * scaleLongDown, (float)(loc1->lat - loc2->lat)) * 5729.57795;
}

long get_bearing(struct Location *loc1, struct Location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	long bearing =  9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}


