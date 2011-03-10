// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

void init_commands()
{
	//read_EEPROM_waypoint_info();
    g.waypoint_index.set_and_save(0);
	command_must_index	= 0;
	command_may_index	= 0;
	next_command.id 	= CMD_BLANK;
}

void update_auto()
{
	if (g.waypoint_index == g.waypoint_total) {
		do_RTL();
	}
}

// this is only used by an air-start
void reload_commands_airstart()
{
	init_commands();
	g.waypoint_index.load();        // XXX can we assume it's been loaded already by ::load_all?
	decrement_WP_index();
}

// Getters
// -------
struct Location get_wp_with_index(int i)
{
	struct Location temp;
	long mem;


	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i > g.waypoint_total) {
		temp.id = CMD_BLANK;
	}else{
		// read WP position
		mem = (WP_START_BYTE) + (i * WP_SIZE);
		temp.id = eeprom_read_byte((uint8_t*)mem);
		mem++;
		temp.p1 = eeprom_read_byte((uint8_t*)mem);
		mem++;
		temp.alt = (long)eeprom_read_dword((uint32_t*)mem);
		mem += 4;
		temp.lat = (long)eeprom_read_dword((uint32_t*)mem);
		mem += 4;
		temp.lng = (long)eeprom_read_dword((uint32_t*)mem);
	}
	return temp;
}

// Setters
// -------
void set_wp_with_index(struct Location temp, int i)
{
	i = constrain(i, 0, g.waypoint_total.get());
	uint32_t mem = WP_START_BYTE + (i * WP_SIZE);

	eeprom_write_byte((uint8_t *)	mem, temp.id);

	mem++;
	eeprom_write_byte((uint8_t *)	mem, temp.p1);

	mem++;
	eeprom_write_dword((uint32_t *)	mem, temp.alt);

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lat);

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lng);
}

void increment_WP_index()
{
    if (g.waypoint_index < g.waypoint_total) {
        g.waypoint_index.set_and_save(g.waypoint_index + 1);
		SendDebug("MSG <increment_WP_index> WP index is incremented to ");
	}else{
		//SendDebug("MSG <increment_WP_index> Failed to increment WP index of ");
		// This message is used excessively at the end of a mission
	}
    SendDebugln(g.waypoint_index,DEC);
}

void decrement_WP_index()
{
    if (g.waypoint_index > 0) {
        g.waypoint_index.set_and_save(g.waypoint_index - 1);
    }
}

long read_alt_to_hold()
{
	if(g.RTL_altitude < 0)
		return current_loc.alt;
	else
		return g.RTL_altitude + home.alt;
}


void loiter_at_location()
{
	next_WP = current_loc;
}


//********************************************************************************
//This function sets the waypoint and modes for Return to Launch
//********************************************************************************
struct Location get_LOITER_home_wp()
{
	//so we know where we are navigating from
	next_WP = current_loc;

	// read home position
	struct Location temp 	= get_wp_with_index(0);
	temp.id 				= MAV_CMD_NAV_LOITER_UNLIM;
	temp.alt 				= read_alt_to_hold();
	return temp;
}

/*
This function stores waypoint commands
It looks to see what the next command type is and finds the last command.
*/
void set_next_WP(struct Location *wp)
{
	//GCS.send_text(SEVERITY_LOW,"load WP");
	SendDebug("MSG <set_next_wp> wp_index: ");
	SendDebugln(g.waypoint_index, DEC);
	gcs.send_message(MSG_COMMAND_LIST, g.waypoint_index);

	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = next_WP;

	// Load the next_WP slot
	// ---------------------
	next_WP = *wp;

	// used to control FBW and limit the rate of climb
	// -----------------------------------------------
	target_altitude = current_loc.alt;

	if(prev_WP.id != MAV_CMD_NAV_TAKEOFF && prev_WP.alt != home.alt && (next_WP.id == MAV_CMD_NAV_WAYPOINT || next_WP.id == MAV_CMD_NAV_LAND))
		offset_altitude = next_WP.alt - prev_WP.alt;
	else
		offset_altitude = 0;

	// zero out our loiter vals to watch for missed waypoints
	loiter_delta 		= 0;
	loiter_sum 			= 0;
	loiter_total 		= 0;

	// this is used to offset the shrinking longitude as we go towards the poles
	float rads 			= (abs(next_WP.lat)/t7) * 0.0174532925;
	scaleLongDown 		= cos(rads);
	scaleLongUp 		= 1.0f/cos(rads);

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(&current_loc, &next_WP);
	wp_distance 		= wp_totalDistance;
	target_bearing 		= get_bearing(&current_loc, &next_WP);

	// to check if we have missed the WP
	// ----------------------------
	old_target_bearing 	= target_bearing;

	// set a new crosstrack bearing
	// ----------------------------
	reset_crosstrack();

	gcs.print_current_waypoints();
}

// run this at setup on the ground
// -------------------------------
void init_home()
{
	SendDebugln("MSG: <init_home> init home");

	// block until we get a good fix
	// -----------------------------
	while (!g_gps->new_data || !g_gps->fix) {
		g_gps->update();
	}

	home.id 	= MAV_CMD_NAV_WAYPOINT;
	home.lng 	= g_gps->longitude;				// Lon * 10**7
	home.lat 	= g_gps->latitude;				// Lat * 10**7
	home.alt 	= max(g_gps->altitude, 0);
	home_is_set = true;

	Serial.printf("gps alt: %ld", home.alt);

	// Save Home to EEPROM
	// -------------------
	set_wp_with_index(home, 0);

	// Save prev loc
	// -------------
	next_WP = prev_WP = home;
}



