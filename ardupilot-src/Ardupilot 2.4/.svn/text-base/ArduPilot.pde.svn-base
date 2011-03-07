/*By Chris Anderson & Jordi Munoz*/
/* ArduPilot Version 2.4.5 */
//
/* Updated version by Jean-Louis Naudin */
// Project Hosting : http://code.google.com/p/ardupilotdev/
// 
// Do we need historial here? 
//  Last Update : 11-09-09
// 
// 11-01-09 JLN : Added starting_wp = 1
// 11-02-09 JLN : Added GPS: gpsfix status for telemetry (System tab )
// 11-02-09 JLN : Added WPT: current_wp for telemetry (System tab )
// 11-04-09 JLN : Added HappyKillmore code for GPS Emulator (GPS_EMUL tab)
// 11-04-09 JLN : added Get_Home_Position(void) -> stored into : home_lon, home_lat, home_alt (see Waypoint tab)
// 11-04-09 JLN : Added A-1 GPS_PROTOCOL 3 in the easyglider24.h header file for GPS Emulator
// 11-05-09 JLN : added Save_Home_Position(void) -> to store the virtual home postion and alt for the GPS Emulator (see System tab)
// 11-08-09 JLN : correction of the air_speed_offset bug which gives -289 (!!!) of airspeed...( Sensors tab )
// 11-09-09 JLN : Air_speed_offset bug now corrected, added zasp_req=1 a request flag for Zero Air Speed bias during the catch_analogs Mux

/* Special thanks to: 
  -Bill Premerlani
  -HappyKillMore
  -James Cohen.
  -JB from rotorFX.
  -Automatik.
  -You Name Here.
  -Fefenin
  -Peter Meister
  -Remzibi
  If i forgot a name please tell me!!*/
/*
Todo:
-Add cross-track error. 
-Improve AirSpeed hold and make it in function with GroundSpeed.
 */
 
#include <avr/eeprom.h>
#include <avr/io.h>
#include "Aeasyglider24.h" //Loading the airframe settings. The Header file must bu located in the same directory than the project

//GPS SIRF configuration strings... 
#define SIRF_BAUD_RATE_4800    "$PSRF100,1,4800,8,1,0*0E\r\n"
#define SIRF_BAUD_RATE_9600    "$PSRF100,1,9600,8,1,0*0D\r\n"
#define SIRF_BAUD_RATE_19200    "$PSRF100,1,19200,8,1,0*38\r\n"
#define SIRF_BAUD_RATE_38400    "$PSRF100,1,38400,8,1,0*3D\r\n"  
#define SIRF_BAUD_RATE_57600    "$PSRF100,1,57600,8,1,0*36\r\n"
#define GSA_ON   "$PSRF103,2,0,1,1*27\r\n"   // enable GSA
#define GSA_OFF  "$PSRF103,2,0,0,1*26\r\n"   // disable GSA
#define GSV_ON   "$PSRF103,3,0,1,1*26\r\n"  // enable GSV
#define GSV_OFF  "$PSRF103,3,0,0,1*27\r\n"  // disable GSV
#define USE_WAAS   1     //1 = Enable, 0 = Disable, good in USA, slower FIX... 
#define WAAS_ON    "$PSRF151,1*3F\r\n"       // enable WAAS
#define WAAS_OFF   "$PSRF151,0*3E\r\n"       // disable WAAS

//GPS Locosys configuration strings...
#define USE_SBAS 0
#define SBAS_ON "$PMTK313,1*2E\r\n"
#define SBAS_OFF "$PMTK313,0*2F\r\n"

#define NMEA_OUTPUT_5HZ "$PMTK314,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 5HZ  
#define NMEA_OUTPUT_4HZ "$PMTK314,0,4,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 4HZ 
#define NMEA_OUTPUT_3HZ "$PMTK314,0,3,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 3HZ 
#define NMEA_OUTPUT_2HZ "$PMTK314,0,2,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 2HZ 
#define NMEA_OUTPUT_1HZ "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 1HZ

#define LOCOSYS_REFRESH_RATE_200 "$PMTK220,200*2C" //200 milliseconds 
#define LOCOSYS_REFRESH_RATE_250 "$PMTK220,250*29\r\n" //250 milliseconds

#define LOCOSYS_BAUD_RATE_4800 "$PMTK251,4800*14\r\n"
#define LOCOSYS_BAUD_RATE_9600 "$PMTK251,9600*17\r\n"
#define LOCOSYS_BAUD_RATE_19200 "$PMTK251,19200*22\r\n"
#define LOCOSYS_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define Battery_Voltage(x) (x*(INTPUT_VOLTAGE/1024.0))/(100000.0/(240000.0+100000.0))/1000.0 //I explained this in the Voltage Divider Theory! 
#define Pressure_MTS(x) sqrt((((x*(INTPUT_VOLTAGE/1024.0))/1.0)*2.0)/1.225) //Converts to AirSpeed in meters for second, i will later explain this. 
#define Pressure_KMH(x) Pressure_MTS(x)*3.6 //Kilometers per hour, never use English system please. This stuff is international. 

#define PID_dt 20 //Servo refresh time in milliseconds 
#define Start_Byte 0x0E //Used to read the EEPROM 


/***************************************************************************
 General variables
 **************************************************************************/
 
const float kp[]={
  roll_P,pitch_P}
,ki[]={
  roll_I,pitch_I}; //PI gains


union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t  byte[2];
} intUnion;

/*Flight GPS variables*/
int gpsFix=1; //This variable store the status of the GPS
float lat=0; // store the Latitude from the gps
float lon=0;// Store guess what?
float alt_MSL=0; //This is the alt.
float ground_speed=0;// This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
float ground_course=0;//This is the runaway direction of you "plane" in degrees
float climb_rate=0; //This is the velocity you plane will impact the ground (in case of being negative) in meters for seconds
char data_update_event=0; 

unsigned long PID_timer=0;
unsigned long NAV_timer=0;

//
uint8_t MuxSel=0;
uint8_t analog_reference = DEFAULT;
int16_t analog_buffer[6];
volatile uint8_t ADC_flag=0;
//
float analog0=511; //Roll Analog  
float analog1=511; //Pitch Analog
float analog2=511; //Z Analog
float analog3=511; //I have to change this name is the airspeed sensor... SOON! 
float analog5=511;

int max_ir=0;
int air_speed_offset=0;
int air_speed_hold=0;
unsigned int zasp_req=0; 

byte throttle_set_point=0;
int roll_set_point=0; //Stores the desired set point of roll and pitch
int pitch_set_point=0;
unsigned int hold_Alt=0; 
unsigned int rx_Ch[2];

unsigned int wp_distance=0; 
unsigned int wp_bearing=0;
//byte wp_Mode;

byte options=0; 
char roll_trim= -4; //You know like the transmitter trims 
char pitch_trim= 10; 
int max_alt=0;
int max_spd=0;
byte wp_number=3; //Number of waypoints defined.... Go to to Waypoint tab...
byte current_wp=0;//Store the current waypoint...
byte wp_radius=20; //Radius of the waypoint, normally set as 20 meters.
unsigned int launch_alt=0; //Launch 

//home position parameters. 
float home_lat=0;
float home_lon=0;
unsigned int home_alt=0; 
byte starting_wp=1;//Store the starting waypoint...

//Stores the actual waypoint we are trying to reach. 
float wp_current_lat=0;
float wp_current_lon=0;
unsigned int wp_current_alt=0; 
int last_waypoint=999;

long refresh_rate=0;
const float t7=1000000.0;

float Batt_Volt =0;
/***************************************************************************
 NMEA variables
 **************************************************************************/
#if GPS_PROTOCOL == 0 // This condition is used by the compiler only....
/*GPS Pointers*/
char *token; //Some pointers
char *search = ",";
char *brkb, *pEnd;
char gps_buffer[200]; //The traditional buffer.
#endif
/***************************************************************************
GPS EMULATOR variables
 **************************************************************************/
#if GPS_PROTOCOL == 3 // This condition is used by the compiler only....
/*GPS Pointers*/
char *token; //Some pointers
char *search = ",";
char *brkb, *pEnd;
char gps_buffer[200]; //The traditional buffer.
char gps_GGA[80]; // added for GPS emulator
char gps_RMC[80]; // added for GPS emulator
//virtual home position parameters. 
float vhome_lat=48.422562;
float vhome_lon=4.492571;
unsigned int vhome_alt=20;
#endif
/***************************************************************************
 SIRF variables
 **************************************************************************/
#if GPS_PROTOCOL == 1
 //GPS stuff please read SiRF-Binary-Protocol-Reference-Manual page 87 for more information
 byte gps_buffer[90]={ 0x24,0x50,0x53,0x52,0x46,0x31,0x30,0x30,0x2C,0x30,0x2C,
  0x35,0x37,0x36,0x30,0x30,0x2C,0x38,0x2C,0x31,0x2C,0x30,
  0x2A,0x33,0x37,0x0D,0x0A}; //Buffer that store all the payload coming from the GPS
 
 const byte gps_ender[]={0xB0,0xB3};  //Used to configure Sirf GPS
#endif
/***************************************************************************
 uBlox Variables
 **************************************************************************/
#if GPS_PROTOCOL == 2
//uBlox Checksum
byte ck_a=0;
byte ck_b=0;
long iTOW=0; //GPS Millisecond Time of Week
long alt=0; //Height above Ellipsoid 
long speed_3d=0; //Speed (3-D)  (not used)
#endif


/***************************************************************************

 **************************************************************************/
void setup()
{
  init_ardupilot();  //Initialize ardupilot...
  current_wp=starting_wp;
  zasp_req=1; // zero air speed request flag
}
void loop()//Main Loop
{
  navigation();//Calculate all the setpoints for the roll, pitch and throttle. Function localted in "Amain" tab.
  stabilization();//Control of all the actuators using IRs and Pressure Sensor... and the desired Setpoints for each one. Function localted in "Amain" tab.
  catch_analogs(); //Reads and average the sensors when is doing nothing... Function localted in "Sensors" tab.
  decode_gps();  //Reads and average the GPS when is doing nothing...

  #if GPS_PROTOCOL == 3
  // print_remzibi(); //Function localted in "System" tab.
   print_data_emulator();  // Required for GPS emulator
  #else
   print_data(); //Function localted in "System" tab. Set to normal flight
  #endif
  
  if(gpsFix != 0x00)//No valid GPS
  {
  read_Ch1(); //reading the receiver with very short delays... Function localted in "Servos" tab.
  read_Ch2(); //Reading the receiver with very short delays... Function localted in "Servos" tab.
  }
}


