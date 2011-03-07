  /***************************************/
 /*ArduPilot 2.5 Header file, good luck!*/
/***************************************/

//Hardware Configuration
#define SHIELD_VERSION 1 // Old (red) shield versions is 0, the new (blue) shield version is 1, -1 = no shield
#define AIRSPEED_SENSOR 1 // 1 if an airspeed sensor is attached and pitot tube connected. 0 if not (throttle will be restricted to altitude error)
#define GROUNDSTATION 1 // 0 = Ardupilot GS

// Flight Modes
// these Flight modes can be changed either here or directly in events.pde
// options are MANUAL, STABILIZE, FLY_BY_WIRE, AUTO, RTL, LOITER
#define POSITION_1 MANUAL 
#define POSITION_2 FLY_BY_WIRE
#define POSITION_3 STABILIZE
// Try and run FLY_BY_WIRE to verify you have good gains set up correctly 
// before you try Autopilot and wreck your plane. I'll sleep better that way...
	

// Airframe settings
//1-1
#define GPS_PROTOCOL 2 // 0 = NMEA, 1=SIRF, 2=uBlox, 3 = IMU, 5 = Simulated GPS mode (Debug)
//1-2
#define REVERSE_ROLL 1 //To reverse servo roll, PUT -1 to invert it!!!
//1-3
#define REVERSE_PITCH 1 //To reverse servo pitch, PUT -1 to invert it!!!
// NOTE - IF USING ELEVONS 1-2 AND 1-3 SHOULD BE 1
//1-4
#define REVERSE_ELEVONS 1 //Use 1 or -1 if you need to reverse roll direction
//1-5
#define REVERSE_CH1_ELEVON 1 //To reverse channel 1 elevon servo, PUT -1 to reverse it!!!
//1-6
#define REVERSE_CH2_ELEVON 1 //To reverse channel 2 elevon servo, PUT -1 to reverse it!!!
//1-7
#define REVERSE_THROTTLE 0 // 0 = Normal mode. 1 = Reverse mode - Try and reverse your radio first!
//1-8
#define INPUT_VOLTAGE 5200.0 // (Millivolts) voltage your power regulator is feeding your ArduPilot to have an accurate pressure and battery level readings. (you need a multimeter to measure and set this of course)
//1-9
#define BATTERY_EVENT 0 // (boolean) 0 = don't read battery, 1 = read battery voltage (only if you have it wired up!)
//1-10
#define MIXING_MODE 0 //Servo mixing mode 0 = Normal, 1 = V-tail (v tail not tested yet). 


// IR sensors
//2-1
#define ENABLE_Z_SENSOR 1  // 0 = no Z sensor, 1 = use Z sensor (no Z requires field calibration with each flight)
//2-2
#define XY_SENSOR_LOCATION 0 	//XY Thermopiles Sensor placement
//Mounted right side up: 		0 = cable in front, 1 = cable behind
//Mounted upside down: 			2 = cable in front, 3 = cable behind
//2-3
#define PITCH_TRIM 0 //(Degrees +- 5) allows you to offset bad IR placement
//2-4
#define ROLL_TRIM 0 // (Degrees +- 5) allows you to offset bad IR placement


// RADIO
//3-1
#define SET_RADIO_LIMITS 0 // 0 = no, 1 = set the limits of the Channels with the radio at launch each time
//3-2
#define RADIO_TYPE 0 // 0 = sequential PWM pulses(Fasst, Spektrums), 1 = simultaneous PWM pulses (Corona RP8D1)
//3-3
#define CH1_MIN 1000 // (Microseconds) Range of Ailerons/ Rudder
//3-4
#define CH1_MAX 2000 // (Microseconds)
//3-5
#define CH2_MIN 1000 // (Microseconds) Range of Elevator
//3-6
#define CH2_MAX 2000 // (Microseconds)
//3-7
#define CH3_TRIM 1000 // (Microseconds) Trims are normally set automatically in setup.
//3-8
#define THROTTLE_IN 1 // (boolean) Disables throttle when set to 0
//3-9
#define THROTTLE_OUT 1	 // 1 = throttle, 0 = no throttle at all! (good for saving fingers on the bench.)


//NAVIGATION: HEADING
//Note: Some Gains are now variables
#define HEAD_P .8  //Heading error proportional.
//4-1
#define HEAD_I 0 //heading error integrator. Do not add too much or you will overshoot. 

#define HEAD_D 0 //heading error derivitive

//4-2
#define ROLL_MAX 4000 //(Degrees *100) The maximum output in degrees to control the roll setpoint
//4-3
#define HEAD_I_MAX 500 //(Degrees *100) The maximum output in degrees to control the roll setpoint
//4-4
#define XTRACK_GAIN .3 // amount to compensate for crosstrack - use debug mode to find correct value
//4-5
#define LOITER_RADIUS 45 // radius in meters of a Loiter
//4-6
#define REMEMBER_LAST_WAYPOINT_MODE 1 //If set 1 = will remember the last waypoint even if you restart the autopilot. 0 = Will start from zero everytime you restart the system. 


//NAVIGATION: ALTITUDE
//5-1
#define PITCH_P 1.5 //Altitude error proportional, pitch setpoint
//5-1
#define PITCH_I 0 //Altitude error proportional, pitch setpoint
//5-2
#define ALTITUDE_PITCH_MAX 500 // most the plane will pitch up in degrees to raise altitude
//5-3
#define ALTITUDE_PITCH_MIN -1200 // (Degrees *100) most the plane will pitch down in degrees to lower altitude 


//NAVIGATION: SPEED GAINS
//6-1
#define ALTITUDE_THROTTLE_P 1.33
//6-2
#define ALTITUDE_AIRSPEED_MAX 15 //(0-125) Maximum extra speed for gaining altitude
//6-3
#define ALTITUDE_AIRSPEED_MIN -25 //(0-125) Maximum reduced speed for loosing altitude
//6-4
//#define AIRSPEED_TURN_P 1 // (.25 - 1) The amount we scale down our roll when going slow; 1 = no effect
//6-5
#define AIRSPEED_MAX_TURN_SPEED 25 // Turn full-on above this airspeed, Hint: set less than or equal to throttle_cruise 
//6-6
#define AIRSPEED_MIN_TURN_SPEED 12 // Used to limit the effect

#define CRUISE_AIRSPEED 13      // If we have airspeed sensor pitch is used to maintain desired airspeed and throttle is used for climb/descent
								// NOTE - Airspeed is stored and used in the program as an integer pressure value
                                // Use the formula:  pressure = 0.1254 * speed * speed 
                                // where speed is the airspeed in meters per second.
                                // For example if you want cruising airspeed to be 10 meters per second use a value of 13

//ATTITUDE: Rudder gains / Aileron gains
//7-1
#define RUDDER_P .5 // 	START WITH THIS VALUE TO TUNE - overall proportional term determines how much rudder you use to turn.
//7-2	
#define RUDDER_I 0.0 //	Roll PID integrator
//7-3
#define RUDDER_MAX 2500 // (Degrees *100) Maximum output of Rudder


//ATTITUDE: Elevator gains
//8-1
#define ELEVATOR_P .40 //	Pitch Proportional- overall proportional term determines how much elevator you use to correct pitch
//8-2
#define ELEVATOR_I 0 //	Pitch PID integrator
//8-3
#define PITCH_COMP .10 //<------Very important, Pitch compensation vs. Roll bank angle. 


//ATTITUDE: THROTTLE OUTPUT GAINS
//9-1
#define THROTTLE_ABSOLUTE 1 //Absolute
//9-2
#define THROTTLE_P .2 //Proportional
//9-3
#define THROTTLE_I .004 //Integrator
//9-4
#define THROTTLE_I_MAX 50 // (0-125) 50 = 40% Integrator limit. 
//9-5
#define THROTTLE_CRUISE 40 // (0-125) 30 = 24% throttle, or (int)target airspeed for cruising
//9-6
#define THROTTLE_MAX 90 // (0-125) 60 = 48% maximum throttle


  /*****************/
 /*Debugging Stuff*/
/*****************/
//10-1
#define TURNRATE 85 // (degrees) how fast we turn per second in degrees at full bank
//10-2
#define CLIMBRATE_UP 1000 // (meters * 100) how fast we climb in simulator at 90° 
//10-3
#define CLIMBRATE_DOWN 3000 // (meters * 100) how fast we climb in simulator at 90° 


