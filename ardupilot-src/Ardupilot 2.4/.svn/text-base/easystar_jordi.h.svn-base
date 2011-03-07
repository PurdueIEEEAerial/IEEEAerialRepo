  /***********************************/
 /*ArduPilot Header file, good luck!*/
/***********************************/

//Hardware Configuration
//0-1
#define SHIELD_VERSION 1 // Old (red) shield versions is 0, the new (blue) shield version is 1

// Airframe settings
//1-1
#define ALT_HOLD_HOME 0 //What altitude you want to hold for home, 0 = the one you are when you switch to RTL, 1 = the altitude defined by the configuration tool.
//1-2
#define REVERSE_X_SENSOR 1 //XY Thermopiles Sensor, 1 = cable behind, 0 = cable in front
//1-3
#define MIXING_MODE 0 //Servo mixing mode 0 = Normal, 1 = V-tail (v tail not tested yet). 
//1-4
#define REVERSE_ROLL 1 //To reverse roll servo, change to -1
//1-5
#define REVERSE_PITCH 1 //To reverse pitch servo, chance to -1
//1-6
#define RADIO_SWITCH_ACTION 0 // 0: TX Switch centered = waypoint mode & full = RTL mode. 1: TX Switch centered = RTL & full = waypoint mode.
//1-7
#define GPS_PROTOCOL 2 // 0 = NMEA, 1=SIRF, 2=uBlox, Choose protocol
//1-8
#define ATTITUDE_RATE_OUTPUT 250 // The output rate of attitude data in milliseconds. Useful if you want to increase the output rate.
//1-9
#define POSITION_RATE_OUTPUT 4  //This number will be multiplied by ATTITUDE_RATE_OUTPUT, the result is the refresh rate in milliseconds.
//1-10
#define REMEMBER_LAST_WAYPOINT_MODE 0 // If set to 1 will remember the last waypoint even if you restart the autopilot. If 0, will start from zero every time you restart the system. 
//1-11
#define INTPUT_VOLTAGE 5200.0 // voltage in millis your power regulator is feeding your ArduPilot. Measure and set this to have an accurate pressure and battery level readings. (you need a multimeter to measure this of course).
//1-12
#define REVERSE_THROTTLE 0 // 0 = Normal mode. 1 = Reverse mode... 


// Fly by wire settings
//
//(Note: If you disconnect the GPS you will fly by wire.) 
//ALWAYS leave your stick's centered when you ArduPilot is booting up.

//2-1
#define FLY_BY_WIRE_GAIN_ROLL .5 //Decrease the value to increase the response of the sticks. DESIRED_ROLL = //STICK_POSITION*FLY_BY_WIRE_GAIN_ROLL
//2-2
#define FLY_BY_WIRE_GAIN_PITCH .5 //The same as roll. 
//2-3
#define FLY_BY_WIRE_SPEED_SETPOINT 20 //The airspeed you want to hold in fly by wire mode.
//2-4
#define GPS_ERROR_SPEED_SETPOINT 3 // In -m/s; , in case of GPS failure the airplane will enter into stabilization mode only and will try to maintain the airspeed set here. 
//2-5
#define REV_FLY_BY_WIRE_CH1 1 //-1 will invert it
//2-6
#define REV_FLY_BY_WIRE_CH2 1 //-1 will invert it


//Autopilot PID gains. 
//(Note: All the PID control loop gains and limits...) 
//3-1
#define SERVO_AILE_MAX 2400 //Range of Ailerons 
//3-2
#define SERVO_AILE_MIN 600
//3-3
#define SERVO_ELEV_MAX 2400 //Range of Elevator
//3-4
#define SERVO_ELEV_MIN 600

//HEADING GAINS
//4-1
#define head_P .7 //Heading error proportional (same used to move the rudder)... DO not add too much or you will oscillate left and right.  (drunk driver effect)
//4-2
#define head_I .1 //heading error integrator. Do not add too much or you will overshoot. 
//4-3
#define head_D 0 //Derivative not used, but someday....
//4-4
#define head_error_max 35 //35 The maximum output in degrees to control the roll setpoint
//4-5
#define head_error_min -35 //-35 The min output in degrees to control the roll setpoint

//ROLL GAINS
//5-1
#define roll_abs .2 //Set point absolute...(Not Used)
//5-3
#define roll_P .35 //roll PID proportional
//5-3
#define roll_I .35 //roll PID integrator
//5-4
#define roll_min -25 //PID output limit in servo degrees
//5-5
#define roll_max  25 //PID output limit in servo degrees
//5-6
#define roll_Integrator_max 10 //Limit the integrator, to avoid overshoots
//5-7
#define roll_Integrator_min -10

//PITCH GAINS
//6-1
#define pitch_P .65 //Pitch Proportional
//6-2
#define pitch_I .35 //Pitch integrator
//6-3
#define pitch_min -25 //Pitch limits
//6-4
#define pitch_max  25 
//6-5
#define pitch_Integrator_max 10 //Pitch integrator limits
//6-6
#define pitch_Integrator_min -10
//6-7
#define PITCH_COMP .30 //<------Very important, Pitch compensation vs. Roll bank angle. 

//THROTTLE GAINS
//7-1
#define throttle_max 1800 //Servo range In milliseconds.
//7-2
#define throttle_min 1200 // 
//7-3
#define throttle_dead_zone 20 //In percent %
//7-4
#define throttle_absolute 3 //Absolute
//7-5
#define throttle_kp 3 //Proportional
//7-6
#define throttle_ki 1 //Integrator
//7-7
#define throttle_output_max 85 //Limits
//7-8
#define throttle_Integrator_max 70 //Integrator limit.


//More PID gains for altitude and speed. 
//8-1
#define ALTITUDE_ERROR_MAX 10 //
//8-2
#define ALTITUDE_ERROR_MIN -10 //
//8-3
#define ALTITUDE_ERROR_PITCH_PROPORTIONAL 1.5 //Altitude error proportional, pitch setpoint
//8-4  
#define ALTITUDE_ERROR_PITCH_MAX 15 //The limit of pitch travel (in degrees) used for altitude hold, if you add to much you may stall... 
//8-5
#define ALTITUDE_ERROR_PITCH_MIN -15 //The limit of pitch travel (in degrees) used for altitude hold, if you add to much you may dive
//8-6
#define AIRSPEED_CENTRAL 34 //Airspeed  central point in m/s, normal flight... This value is the lowest airspeed that makes your plane flight steady. 
//8-7
#define ALTITUDE_ERROR_AIRSPEED_PROPORTIONAL 3
//8-8
#define ALTITUDE_ERROR_AIRSPEED_MAX 30
//8-9
#define ALTITUDE_ERROR_AIRSPEED_MIN -10


  /*****************/
 /*Debugging Stuff*/
/*****************/

//9-1
#define FAKE_BEARING 0 //If set to 1, will fake the bearing and will try to always head to the defined DESIRED_FAKE_BEARING
//9-2
#define DESIRED_FAKE_BEARING 45 //Will try to go NorthEast, you can change that to 0 = NORTH, 90 = EAST, 180 = SOUTH, 270 = WEST or whatever!
//9-3
#define FAKE_GPS_LOCK 0 //If  set to 1 will jump the GPS lock process to set home position. FOR TESTING ONLY!
//9-4
#define PRINT_WAYPOINTS 0 //If set to 1, at bootup will print all the waypoints set in the eeprom! 
//9-5
#define TEST_THROTTLE 0 // If set to 1 will test the throttle increasing the speed slowly. 
//9-6
#define WALK_AROUND 1 //Must be "0" to test the GPS and heading against the servo and "1" for normal operation
//9-7
#define CALIBRATE_SERVOS 0// Use to move the servos center, left and right or center right and left. You must adjust using 3-1 and 3-2. 
//9-8
#define TEST_SENSORS 0 // Set 1 for enable, overwrite the servos with the raw IR sensors data, to test orientation. This will overwrite everything.
//9-9
#define PRINT_EEPROM 0 // If set to 1, sends content of the EEPROM to the serial port for debugging purpose