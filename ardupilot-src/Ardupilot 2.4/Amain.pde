//v2.4.2 
// Jason testing Subversion
/****************************************************************
 * navigation(), this part will calculate the setpoints of the PID that control the servos... 
 ****************************************************************/
void navigation(void)
{
  if((data_update_event&0x01)==0x01) //Verify if we have new GPS data.. Is like a flag... 
  {
    data_update_event&=0xFE; //Resetting flag, 0xFE is equeal to ~0x01 (not) 

    float dt_t = (float)(millis()-NAV_timer)/1000.0; //Timer

    if (Tx_Switch_Status()==0x01) //Check if we are in mode 1 (transmitter switch in middle position) if yes enter to waypoint mode
    {
      Set_New_Waypoint(current_wp); //Loading current waypoint.
      update_distance(); //Updating the distance between the current position and the destioantion waypoints
      Waypoint_Switcher(wp_distance, wp_radius); //Waypoint switcher... 
    }
    else //If we are not in mode 2 please return home (waypoint 0)
    {
      //Here we put home position. 
      Set_New_Waypoint(0); //Waypoint zero is home.
      update_distance(); //This one will activate the nuclear missile and update the distance. 
    }

    wp_bearing=calc_bearing(lat, lon, wp_current_lat, wp_current_lon); //Calculating bearing

    //Now calculates the roll set point, in order to turn to the desired heading.. 
#if FAKE_BEARING == 1
    roll_set_point= calc_roll(heading_error((int)DESIRED_FAKE_BEARING, ground_course),dt_t); //Faking the bearing.
#endif
#if FAKE_BEARING == 0
    roll_set_point= calc_roll(heading_error((int)wp_bearing, ground_course),dt_t); //Calculate the roll set point depending the heading error. 
#endif
    NAV_timer=millis(); //Resetting timer    
  }

  if((data_update_event&0x02)==0x02)///Checking new GPS data flag
  {
    int alt_error = 0;
    data_update_event&=0xFD; //Clearing flag 
    
    alt_error =altitude_error(Hold_Current_Altitude(), Get_Relative_Altitude()); //Holding the altitude before switch to RTL. 

    //Here will take the altitude error to control the throttle, has proportional and limits.
    air_speed_hold=constrain(AIRSPEED_CENTRAL+(alt_error*ALTITUDE_ERROR_AIRSPEED_PROPORTIONAL),ALTITUDE_ERROR_AIRSPEED_MIN,ALTITUDE_ERROR_AIRSPEED_MAX); 

    //Here will take the altitude error to move the elevator, has proportional and limits
    pitch_set_point=constrain(alt_error*ALTITUDE_ERROR_PITCH_PROPORTIONAL,ALTITUDE_ERROR_PITCH_MIN,ALTITUDE_ERROR_PITCH_MAX);

    maxis(); //Storing max altitude and speed. 
  }
}

/****************************************************************
 * stabilization(), this part will control the aircraft! I guess is the most important part... 
 ****************************************************************/
void stabilization(void)
{
  unsigned int t_dt=millis()-PID_timer; //Timer... 

  int Pitch=0;
  int Roll=0;

  if(t_dt > PID_dt)
  {
    //If valid gps data, navigate, else just stabilize... 
    if(gpsFix == 0x00) 
    {
      //Calculating Roll
      Roll=PID_roll(PID_error(roll_set_point, (roll_trim+get_roll())*WALK_AROUND),roll_abs,t_dt); 
      //Calculating Pitch
      Pitch=PID_pitch(PID_error(pitch_set_point+((abs(get_roll())*PITCH_COMP)), pitch_trim+get_pitch()),t_dt);
      //Sending the values to the servos. 
      pulse_servos(Roll,Pitch);
    }
    else //This is fly by wire
    {
      if((abs(read_Ch1())>25) || (abs(read_Ch2())>25))//Fly by wire, check if we move the sticks 
      {
        Roll=PID_roll(PID_error(((float)read_Ch1()*FLY_BY_WIRE_GAIN_ROLL), roll_trim+get_roll()),roll_abs,t_dt);
        Pitch=PID_pitch(PID_error(((float)read_Ch2()*FLY_BY_WIRE_GAIN_PITCH), pitch_trim+get_pitch()),t_dt);
        pulse_servos(Roll,Pitch);
        air_speed_hold=FLY_BY_WIRE_SPEED_SETPOINT;
      }
      else //This is Just stabilize, no navigation. 
      {
        Roll=PID_roll(PID_error(roll_trim, roll_trim+get_roll()),roll_abs,t_dt);
        Pitch=PID_pitch(PID_error(pitch_trim, pitch_trim+get_pitch()),t_dt);
        pulse_servos(Roll,Pitch);//Stabilization only 
        air_speed_hold=GPS_ERROR_SPEED_SETPOINT;
      }      
    }

    //Throttle
    throttle_set_point=PID_throttle(air_speed_hold, airSpeed(), t_dt);
    
    if(digitalRead(6) == LOW)//Verifying the bing plug is inserted
      pulse_servo_0(0); //If yes: Don't pulse the motor....
    else
      pulse_servo_0((throttle_set_point)); // If not: Run normally... 

    PID_timer=millis(); //resetting timer... 
  } 
}

