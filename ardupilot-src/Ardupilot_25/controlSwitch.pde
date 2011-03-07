
byte switchPosition 	= 0;
byte oldSwitchPosition 	= 0;

void read_control_switch()
{
	byte switchPosition = readSwitch();
	if (oldSwitchPosition != switchPosition){
		if(failsafe) {
			set_failsafe(false);
		}
		// we have moved the switch
		// tell user about it, it is up to them to deal with it!
		switch_event(switchPosition);
		oldSwitchPosition = switchPosition;

		// reset all the integrators
		// -------------------------
		//integrators[I_RUDDER] = 0;
		//integrators[I_ELEVATOR] = 0;
		integrators[I_THROTTLE] = 0;
		integrators[I_ROLL] = 0;
		last_errors[I_ROLL] = 0;//derivitives
	}
}

void reset_control_switch()
{
	oldSwitchPosition = 0;
	read_control_switch();
}

byte readSwitch(void){
	if(digitalRead(4) == HIGH){
		if(digitalRead(5) == HIGH){

			// Middle Switch Position
			// ----------------------
			return 2;

		}else{
			// 3rd Switch Position
			// -------------------
			return 3;
		}
	}else{
		// 1st Switch Position
		// ----------------------
		return 1;
	}
}

void initControlSwitch()
{
	oldSwitchPosition = switchPosition = readSwitch();
}
