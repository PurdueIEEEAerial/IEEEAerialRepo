/*
Attitude control functions:
These functions make sure the aircraft holds the desired pitch and roll.
Pitch and roll targets are set by the control_navigation routines.
*/


/*
This function takes the Roll error and outputs a Rudder/Aileron value.
ROll_MAX limits the roll to 45°
head_p adjusts the response based on the error
	a low RUDDER_P of .3 tells the plane to use 13.5° rudder to overcome a 45° error
	a  RUDDER_P of .6 tells the plane to use 27° rudder to overcome a 45° error
	
head_I makes the plane try harder to hold the correct roll
*/

float calc_attitude_roll(float error)
{
	// Integrator
	// ----------
	integrators[I_RUDDER] +=  (error * RUDDER_I * (float)deltaMiliSeconds)/1000.0f;
	// 2/24/10 - Fixed mistake in calc_attitude_roll

	// Limit Integrator
	// ----------------	
	integrators[I_RUDDER] = constrain(integrators[I_RUDDER], -1000, 1000); // +-10 degrees

	// Sum the errors
	// --------------	
	error = (error * RUDDER_P) + integrators[I_RUDDER];
	error = constrain(error, -RUDDER_MAX, RUDDER_MAX); // +-25°
	return error;
}

float calc_attitude_pitch(float error)
{	
	// Integrator
	// ----------
	integrators[I_ELEVATOR] +=  (error * ELEVATOR_I * (float)deltaMiliSeconds)/1000.0f;
	
	// Limit Integrator
	// ----------------
	integrators[I_ELEVATOR] = constrain(integrators[I_ELEVATOR], -1000, 1000); // +-10 degrees
	
	// Sum the errors
	// --------------			
	error = (error * ELEVATOR_P) + integrators[I_ELEVATOR];
	return error;
}


int calc_attitude_throttle(float error)
{
	static float old_throttle_output;
	static float integrators[I_THROTTLE];
	
	integrators[I_THROTTLE] +=  (error * THROTTLE_I * deltaMiliSeconds)/1000.0;

	// Limit Integrator
	// ----------------
	integrators[I_THROTTLE] = constrain(integrators[I_THROTTLE], 0, THROTTLE_I_MAX);

	// Sum the errors
	// --------------			
	error = (error * THROTTLE_ABSOLUTE) + (THROTTLE_P * error) + integrators[I_THROTTLE];
	error = constrain(error, 0, (THROTTLE_MAX - throttle_cruise));

	// smooth output
	// --------------
	old_throttle_output = (old_throttle_output*.90) + (error *.10);

	return old_throttle_output; //Returns the result
}
