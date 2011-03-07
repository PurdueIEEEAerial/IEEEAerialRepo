// These are the control functions for navigation 
// ----------------------------------------------

/*
This function takes the heading error and outputs a roll value.
ROll_MAX limits the roll to 45
head_p adjusts the response based on the error
	a low head_p of .5 makes the plan start to level out at 90• error
	a head_p of 1 makes the plan start to level out at 45• error
	a head_p of 2 makes the plan start to level out at 22.5• error
head_I makes the plane try harder to hold the correct roll
*/
/*
float calc_nav_roll(float error)
{
	static float err_rate_limiter = 0;
	integrators[I_ROLL] +=  (error * head_I * deltaMiliSeconds)/1000.0;
	
	// Limit Integrator
	// ----------------
	integrators[I_ROLL] = constrain(integrators[I_ROLL], -HEAD_I_MAX, HEAD_I_MAX);

	// Sum the errors
	// --------------
	error = (head_P * error) + integrators[I_ROLL];
	
	// rate of change dampener
	if(error > err_rate_limiter){
		err_rate_limiter += 1000;
		err_rate_limiter = min(err_rate_limiter, error);
	}else{
		err_rate_limiter -= 1000;
		err_rate_limiter = max(err_rate_limiter, error);
	}
	err_rate_limiter = constrain(err_rate_limiter, -roll_max, roll_max);

	// Adjust gain based on ground speed - We need lower nav gain going in to a headwind, etc.
	// This does not make provisions for wind speed in excess of airframe speed

	nav_gain_scaler = (float)ground_speed/ 1000;
	nav_gain_scaler = constrain(nav_gain_scaler, 0.4, 1);

	err_rate_limiter *= nav_gain_scaler;
	return (err_rate_limiter);	
}*/

float calc_nav_roll(float error)
{
	static float err_rate_limiter = 0;

	integrators[I_ROLL] +=  (error * head_I * deltaMiliSeconds)/1000.0;
	
	// Limit Integrator
	// ----------------
	integrators[I_ROLL] = constrain(integrators[I_ROLL], -HEAD_I_MAX, HEAD_I_MAX);

	derivative_roll = (error * last_errors[I_ROLL]) * 1000 / deltaMiliSeconds;

	// Sum the errors
	// --------------
	error = (head_P * error) + integrators[I_ROLL] + (derivative_roll * head_D);
	
	last_errors[I_ROLL] = error;
	error = constrain(error, -roll_max, roll_max);

	// Adjust gain based on ground speed - We need lower nav gain going in to a headwind, etc.
	// this is a cheap approx
	nav_gain_scaler = (float)ground_speed/ 1200;  // 1000 = 10 m/s = 22.4mph
	nav_gain_scaler = constrain(nav_gain_scaler, 0.4, 1);
	
	error *= nav_gain_scaler;
	
	return (error);	
}


// input 	: meters
// range 	: +- 20m
// output 	: degrees for servos
// range 	: +- 15•
// p calc = 20 / 15 = 1.5
long calc_nav_pitch(long error)
{
	error = error * pitch_P;
	// 20 * 1.5 = 15;
	return constrain(error, ALTITUDE_PITCH_MIN, ALTITUDE_PITCH_MAX);
	//						-1500 (15•)		,	0 (0•) so we don't stall
}


// input 	: pressure sensor offset
// range 	: +- 20m
// output 	: Percent for throttle
// range 	: +- 15 Airspeed
// p calc = 20 / 15 = 1.33
long calc_nav_airspeed(long error)
{
	error = error * altitude_throttle_P;
	return constrain(error, ALTITUDE_AIRSPEED_MIN, ALTITUDE_AIRSPEED_MAX);// -15 : 15
}


