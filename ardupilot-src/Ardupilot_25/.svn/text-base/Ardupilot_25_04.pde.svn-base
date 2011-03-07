#include <avr/io.h>
#include <avr/eeprom.h>
#include <math.h>
#include "defines.h"

//To use the header file in your library, use brackets:
//#include <easystar.h>

//To use the header file in your local folder, use quotes:
#include "easystar_25.h"
//#include "jason_EZStar.h"

 /*
ArduPilot By Jordi Munoz and Jason Short
ArduPilot Version 2.5.0.4
Developed by
	-Chris Anderson
	-Jordi Munoz
	-Jason Short
	-Doug Weibel
	-HappyKillMore
	-Jose Julio	
	-Bill Premerlani
	-James Cohen.
	-JB from rotorFX.
	-Automatik.
	-Fefenin
	-Peter Meister
	-Remzibi
	-Your Name Here.
	
*/


// set by the control switch read by the ATTiny
// --------------------------------------------
byte control_mode			= MANUAL;
boolean invalid_location 	= true;		// used to indicate we can't navigate witout good GPS data - the equations will choke

// Radio values
// ------------
int ch1_trim = 1500;			// set by init sequence
int ch2_trim = 1500;			// set by init sequence
int elevon1_trim = 1500;
int elevon2_trim = 1500;
int ch3_trim = CH3_TRIM;	// set by init sequence unless THROTTLE_IN == 0
int ch3_timer_trim = 0; 	// set by init sequence
int ch3_fs;					// set your failsafe throttle >50\u00b5s below ch3 trim

int ch1_temp = 1500;		// Used for elevon mixing
int ch2_temp = 1500;
int ch1_in = 1500;						// store aileron/rudder position from radio
int ch2_in = 1500;						// store elevator position from radio
int ch3_in = CH3_TRIM;					// store thottle position from radio

int ch1_out = 1500;				// actual µs values to servos
int ch2_out = 1500;				// actual µs values to servos
int ch3_out = CH3_TRIM;			// actual µs values to servos

// servo limits - set during initialization
// ----------------------------------------
int ch1_min		= CH1_MIN;		// lowest 	~ 1000
int ch1_max		= CH1_MAX; 		// highest 	~ 2000
int ch2_min		= CH2_MIN;		//
int ch2_max		= CH2_MAX;		//

// If your radio receiver has failsafes set the Throttle to go below the trim value.  We look for this condition to signal failsafe
// ----------------------------------------------------------------
boolean failsafe			= false;		// did our throttle dip below the failsafe value?
byte 	config_tool_options	= 0;			// 

// attitude control output
// -----------------------
float 	servo_roll			= 0;	 		// degrees to servos
float 	servo_pitch			= 0;	 		// degrees to servos
int 	servo_throttle		= 0;			// 0-125 value

// GPS variables
// -------------
int 	GPS_flag			= -1;			// have we achieved first lock and set Home?
boolean GPS_light			= false;		// status of the GPS light
byte 	GPS_fix				= BAD_GPS;		// This variable store the status of the GPS
float 	ground_speed 		= 0;			// centimeters/second
float 	climb_rate 			= 0;			// meters/second
byte 	GPS_update			= GPS_NONE;		// do we have GPS data to work with?
const float t7				= 10000000.0;	// used to scale GPS values for EEPROM storage
boolean print_telemetry		= false;
long 	iTOW 				= 0; //GPS Millisecond Time of Week

// navigation 
// ----------
long 	ground_course 		= 0;			// degrees * 100 dir of plane 
long 	target_bearing		= 0;			// degrees * 100 location of the plane to the target
long 	bearing_error		= 0; 			// degrees * 100
long 	crosstrack_bearing	= 0;			// degrees * 100 location of the plane to the target
int 	altitude_error		= 0;			// meters * 100 we are off in altitude
int 	max_altitude		= 0;			// meters - read by config tool!
byte 	max_speed			= 0;			// m/s
byte 	wp_radius			= 15;			// meters - set by config tool!
boolean wp_mode				= ABS_WP;		// ABS_WP or REL_WP

// these are the values returned from navigation control functions
// ----------------------------------------------------
long 	nav_roll			= 0;					// target roll angle in degrees * 100
long 	nav_pitch			= 0;					// target pitch angle in degrees * 100
int 	throttle_cruise		= THROTTLE_CRUISE;		// target airspeed sensor value - throttle_cruise = airspeed at cruising
int 	nav_airspeed		= THROTTLE_CRUISE;		// target airspeed sensor value - throttle_cruise = airspeed at cruising

// navigation control gains
// ------------------------
float head_P 	 			= HEAD_P; 				// Heading error proportional 
float head_I 	 			= HEAD_I;  				// Heading error integrator
float head_D 	 			= HEAD_D;  				// Heading error integrator
float pitch_P 				= PITCH_P; 				// Altitude error proportional - controls the pitch with elevators 
float pitch_I 				= PITCH_I; 				// Altitude error proportional - controls the pitch with elevators 
float pitch_comp 			= PITCH_COMP; 				// Altitude error proportional - controls the pitch with elevators 

// Attitude control gains
// ------------------------
float rudder_P 				= RUDDER_P; 				// Altitude error proportional - controls the pitch with elevators 
float rudder_I 				= RUDDER_I; 				// Altitude error proportional - controls the pitch with elevators 
float elevator_P 			= ELEVATOR_P; 				// Altitude error proportional - controls the pitch with elevators 
float elevator_I 			= ELEVATOR_I; 				// Altitude error proportional - controls the pitch with elevators 

int roll_max				= ROLL_MAX;					// maximum roll of plane
float 	nav_gain_scaler		= 1;						// limits roll of plane upwind
float derivative_roll 		= 0;

float altitude_throttle_P 	= ALTITUDE_THROTTLE_P; 	// Altitude error proportional - controls the throttle
float integrators[]			= {0,0,0,0};			// PID Integrators
float last_errors[]			= {0,0,0,0};			// PID Integrators


// used to consruct the GPS data from Bytes to ints and longs
// ----------------------------------------------------------
union long_union {
	int32_t dword;
	uint8_t	byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t	byte[2];
} intUnion;



// System Timers
// --------------
unsigned long fast_loopTimer	= 0;	// Time in miliseconds of main control loop
unsigned long slow_loopTimer	= 0;	// Time in miliseconds of main control loop
unsigned long deltaMiliSeconds 	= 0;	// Delta Time in miliseconds
unsigned long elapsedTime 		= 0;	// in miliseconds

// Waypoints
// ---------
long 	wp_distance			= 0;	// meters - distance between plane and next waypoint
long 	wp_totalDistance	= 0;	// meters - distance between old and next waypoint
float 	wp_area				= 0;	// Pre-calculation to speed mapping function
byte 	wp_total			= 0;	// # of waypoints
byte 	wp_index			= 0;	// Current WP index, -1 is RTL
float 	scaleLongDown		= 0;	// cosine of the Latitude used to scale longitude		
float 	scaleLongUp			= 0;	// used to reverse longtitude scaling

// 3D Location vectors
// -------------------
struct Location {
	long lat;
	long lng;
	long alt;
};

struct Location home 				= {0,0,0};		// home location
struct Location prev_WP 			= {0,0,0};		// last waypoint
struct Location current_loc 		= {0,0,0};		// current location
struct Location est_loc 			= {0,0,0};		// for estimation
struct Location next_WP 			= {0,0,0};		// next waypoint

// Sensors 
// --------
long  analog0				= 511;		// Thermopiles - Pitch
long  analog1				= 511;		// Thermopiles - Roll
long  analog2				= 511;		// Thermopiles - Z
float analog3				= 511;		// Airspeed Sensor - is a float to better handle filtering
float analog5				= 511;		// Battery Voltage
float battery_voltage 		= 0;
int ir_max					= 300;		// used to scale Thermopile output to 511
long roll_sensor			= 0;		// how much we're turning in degrees * 100
long pitch_sensor			= 0;		// our angle of attack in degrees * 100
int airspeed_offset			= 0;		// read the analog airspeed sensors to get this
long airspeed_current		= 0;		// airspeed as a pressure value

// Debugging
// ---------
long est_turn_rate			= 0;
long actual_turn_rate		= 0;


// Basic Initialization
//---------------------
void setup() {
	#if GPS_PROTOCOL == 0
		Serial.begin(FIFTY_SEVEN_K_BAUD);
	#endif
	#if GPS_PROTOCOL == 1
		Serial.begin(FIFTY_SEVEN_K_BAUD);
	#endif
	#if GPS_PROTOCOL == 2
		Serial.begin(THIRTY_EIGHT_K_BAUD);
	#endif
	#if GPS_PROTOCOL == 3
		Serial.begin(THIRTY_EIGHT_K_BAUD);
	#endif
	#if GPS_PROTOCOL == 5
		Serial.begin(THIRTY_EIGHT_K_BAUD);
	#endif
	
	init_ardupilot();
}


void loop()
{
	// system Timers
	// ----------------------------------------
	deltaMiliSeconds 	= millis() - fast_loopTimer;
	if (deltaMiliSeconds < 20) {
		// force main loop to run at 50Hz
		return;
	}
	fast_loopTimer			= millis();

	// Uncomment to prove the loop timer
	// Serial.println(deltaMiliSeconds,DEC);
	
	// Read 3-position switch on radio (usually Ch5)
	// -------------------------------------------------
	read_control_switch();

	// Filters radio input - adjust filters in the radio.pde file
	// ----------------------------------------------------------
	read_radio();
	
	// manually change throttle cruise
	// -------------------------------
	#if THROTTLE_IN == 1
	throttle_cruise = ((long)(ch3_in - ch3_trim) * 70) /1000;
	#endif

	// check for throtle failsafe condition
	// ------------------------------------
	throttle_failsafe();

	// Read IR SENSORS
	// ---------------
	read_XY_analogs();

	
	// Output telemtry
	// ---------------
	if(millis() - slow_loopTimer > 100){
		slow_loopTimer = millis();

		// Read in the GPS position
		// ------------------------
		decode_gps();

		// Blink GPS LED if we don't have a fix
		// ------------------------------------
		if(GPS_fix != VALID_GPS){
			GPS_light = !GPS_light;
			if(GPS_light){
				digitalWrite(12, HIGH);	
			}else{
				digitalWrite(12, LOW);	
			}		
		}else{
			if(!GPS_light){
				GPS_light = true;
				digitalWrite(12, HIGH);
			}
		}

		if (print_telemetry){
			print_telemetry = false;
			print_position();

			if(invalid_location && current_loc.lat != 0){
				reset_location();
			}
			
			if(GPS_flag == 0){
				initialize_home();
				GPS_flag = -1;
			}else if(GPS_flag > 0){
				GPS_flag--;
			}

			// GPS event notification
			// ---------------
			gps_event();
		}
		
		
		print_attitude();
		//print_radio();
		
		// reads Battery and Z sensor
		// --------------------------
		read_analogs();
		set_max_altitude_speed();
	}

	if (control_mode == STABILIZE){
		// Attitude Control Loop
		// -----------------------
		stabilize();

	} else if (control_mode == FLY_BY_WIRE){
		// Fake navigation Control Loop
		// ----------------------------
		nav_roll = ((ch1_in - ch1_trim) * ROLL_MAX * REVERSE_ROLL) /500;
		//nav_roll *= head_P;
		nav_roll = constrain(nav_roll, -roll_max, roll_max); //3500

		// nav_pitch is the amount to adjust pitch due to altitude
		nav_pitch = ((ch2_in - ch2_trim) * ROLL_MAX * REVERSE_PITCH) /500;
		//nav_pitch *= pitch_P;
		//nav_pitch = constrain(nav_pitch, ALTITUDE_PITCH_MIN, ALTITUDE_PITCH_MAX);

		// nav_airspeed is the amount to adjust throttle due to altitude
		nav_airspeed = ((ch2_in - ch2_trim) * 50) /500;
		
		
		wp_distance = getDistance(&current_loc, &next_WP);
		
		// Attitude Control Loop
		// -----------------------
		stabilize_AP();

	}else if (control_mode >= AUTO){

		#if GPS_PROTOCOL == 5
			navigate_sim();
		#endif
		
		// Navigation Control Loop
		// -----------------------
		navigate();


		// Attitude Control Loop
		// -----------------------
		stabilize_AP_mix();
		
	}else if (control_mode == MANUAL){

		// set the outs for telemtry
		// -------------------------
		ch1_out = ch1_in;
		ch2_out = ch2_in;
		ch3_out = ch3_in;
		#if GROUNDSTATION == 1
			readCommands_GS();
		#endif
	}
	
	// send the throttle value to the PWM out
	// --------------------------------------
	update_throttle();

	// Send an event notice of the main loop
	// -------------------------------------
	mainLoop_event();
}
