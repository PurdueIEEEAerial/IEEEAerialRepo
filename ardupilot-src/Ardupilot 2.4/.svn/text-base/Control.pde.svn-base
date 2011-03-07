//V2.4.1
/****************************************************************
 * This function calculate the desired roll angle... 
 ****************************************************************/
int calc_roll(float error,float dt)
{
  static float I;
  static float D;
  static float previous_error;
  
  //Integratior part
  I+= (float)error*dt; //1000 microseconds / 1000 = 1 millisecond
  I= constrain(I,-10,10); //Limits
  I=reset(I);
  error=(error*head_P)+(I*head_I);
  
  //Derivation part
  D=(error-previous_error)/dt;
  previous_error=error;
  
  error=(error*head_P)+(I*head_I)+(D*head_D);
  
  return constrain(error,head_error_min,head_error_max); //Limiting the roll.... 
}

/****************************************************************
 * Altitude hold error... 
 ****************************************************************/
int altitude_error(int PID_set_Point, int PID_current_Point)
{
  int PID_error=PID_set_Point-PID_current_Point; //Computes the error

  return constrain((PID_error),ALTITUDE_ERROR_MIN,ALTITUDE_ERROR_MAX); 
}

/***************************************************************************
 * //Computes heading error, and choose the shortest way to reach the desired heading.
 ***************************************************************************/
int heading_error(int PID_set_Point, int PID_current_Point)
{
 float PID_error = PID_set_Point - PID_current_Point;

 if (PID_error > 180) {
   PID_error -= 360;
 }

 if (PID_error < -180) {
   PID_error += 360;
 }

 return PID_error;
}
/*****************************************
 * Proportional Integrator Control for Throttle
 *****************************************/
byte PID_throttle(int PID_set_Point, int PID_current_Point, int dt)
{
  //int roll_previous_error=0;
  static int throttle_output;
  static float old_throttle_output;
  static float throttle_Integrator;
  //float roll_D=0;

  int PID_error=PID_set_Point-PID_current_Point; //Computes the error

  
  throttle_output= (throttle_kp*PID_error);//Adding proportional

  throttle_Integrator+= (float)(((float)PID_error*((float)dt/1000.0))*(float)throttle_ki); 
  throttle_Integrator=constrain(throttle_Integrator,0,throttle_Integrator_max);
  throttle_Integrator=reset(throttle_Integrator);
  throttle_output+=throttle_Integrator;
  throttle_output+=((float)PID_set_Point*(float)throttle_absolute);

  //Plus all the PID results and limit the output... 
  throttle_output = constrain(throttle_output,0,throttle_max); //
  
  old_throttle_output = (float)(((float)old_throttle_output*.90) + ((float)throttle_output*.10)); 
  
  return old_throttle_output; //Returns the result       
}

/*****************************************
 * Proportional Integrator Control for roll
 *****************************************/
int PID_roll(int PID_error, byte absolute, int dt)
{

  //int roll_previous_error=0;
  static int roll_output;
  static float roll_Integrator;
  //float roll_D=0;

  //int PID_error=PID_set_Point-PID_current_Point; //Computes the error

  roll_output= (kp[0]*PID_error);//Adding proportional

  roll_Integrator+= (float)(((float)PID_error*((float)dt/1000.0))*(float)ki[0]); 
  roll_Integrator=constrain(roll_Integrator,roll_Integrator_min,roll_Integrator_max);
  roll_Integrator=reset(roll_Integrator);
  roll_output+=roll_Integrator;

  //roll_output+=((float)PID_set_Point*(float)absolute);

  //Plus all the PID results and limit the output... 
  roll_output = constrain(roll_output,roll_min,roll_max); //

  return roll_output; //Returns the result       
}

/*****************************************
 * PID ERROR
 *****************************************/
 int PID_error(int PID_set_Point, int PID_current_Point)
{
 
  int PID_error=PID_set_Point-PID_current_Point;
 
 return  PID_error;
}
/*****************************************
 * Proportional Integrator Control for pitch
 *****************************************/
int PID_pitch(int PID_error, int dt)
{ 
  
  static int pitch_output;
  static float pitch_Integrator;
  //static int pitch_previous_error=0;
  //static float pitch_D=0;  
  //int PID_error=PID_set_Point-PID_current_Point; //Computes the error

  pitch_output= (float)((float)kp[1]*(float)PID_error);//Adding proportional

  pitch_Integrator+= (float)(((float)PID_error*((float)dt/1000.0))*(float)ki[1]); 
  pitch_Integrator=constrain(pitch_Integrator,pitch_Integrator_min,pitch_Integrator_max);
  pitch_Integrator=reset(pitch_Integrator);
  pitch_output+=pitch_Integrator;

  pitch_output = constrain(pitch_output,pitch_min,pitch_max); //PID_P+PID_D

  return pitch_output;  
}

