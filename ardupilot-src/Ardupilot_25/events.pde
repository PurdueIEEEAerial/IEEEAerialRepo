/*
	This event will be called when the failsafe changes
	boolean failsafe reflects the current state
*/
void failsafe_event()
{
	if (failsafe == true){
		// This is an example of how to handle a failsafe.
		// Control modes are numbers. Autopilot (5) and higher are under 
		// computer navigation and don't need intervention.
		if (control_mode < AUTO){
			set_mode(RTL);
		}
	}else{
		
	}
}

/*
	This event will be called when the switch changes
	It is up to the user how to react to a swicth change event
	options are: MANUAL, STABILIZE, FLY_BY_WIRE, AUTO, RTL, LOITER 
	see: defines.h
	
	The three switch postions can be handled by most radios.
	Adjust your seetings to make sure all three positions work.
	If you don't have a 3 postion switch, try a two position one 
	and note which case below activates in each position.
*/
void switch_event(byte switchPosition)
{
	switch(switchPosition)
	{
		case 1: // First position
		set_mode(POSITION_1);
		break;

		case 2: // middle position
		//set_mode(RTL);
		set_mode(POSITION_2);
		break;

		case 3: // last position
		set_mode(POSITION_3);
		break;
	}
}

void waypoint_event(byte event)
{
	switch(event)
	{
		case EVENT_WILL_REACH_WAYPOINT:
			// called just before wp_index is incemented
			Serial.print("# Reached WP:");
			Serial.println(wp_index,DEC);
			break;
			
		case EVENT_SET_NEW_WAYPOINT_INDEX:
			// called just after wp_index is incemented
			//Serial.print("Now going to WP:");
			//Serial.println(wp_index,DEC);
			break;

		case EVENT_LOADED_WAYPOINT:
			//Serial.print("Loaded WP index:");
			//Serial.println(wp_index,DEC);
			print_current_waypoint();
			
			// custom loitering code
			/*
			if (wp_index == 2){
				set_mode(LOITER);
				elapsedTime = 0;
			}*/
			break;
			
		// called when the pattern to be flown is automatically restarted
		case EVENT_LOOP: 
			//Serial.println("Looped WP Index");
			print_current_waypoint();
			break;			
			
	}
}

void gps_event(void)
{


}

// called after every single control loop
void mainLoop_event(void)
{
/*
	if (control_mode == LOITER){
		if (wp_index == 2 && elapsedTime > 120000 ){ // 2 minutes
			elapsedTime = 0;
			// our waypoints index is not altered during LOITER
			// All we need to do is reload the waypoint
			load_waypoint();
			// and return to Autopilot mode!
			set_mode(AUTO);
		}
	}
*/
}

void low_battery_event(void)
{
	Serial.println("# Low Battery");
	set_mode(RTL);
}



		
