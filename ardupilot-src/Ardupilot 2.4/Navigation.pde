/****************************************************************
 ****************************************************************/
long Get_Relative_Altitude()
{
  long relative=alt_MSL-launch_alt;
  if(relative <= 0)
    relative=0;  
  return relative;
}

/****************************************************************
 * Function that will read and store the current altitude when you switch to autopilot mode.
 ****************************************************************/
unsigned int Hold_Current_Altitude(void)//
{
#if ALT_HOLD_HOME == 1
  return wp_current_alt;
#else
  if(Tx_Switch_Status()==0x00)//Excutes only when we are in manual or in waypoint mode
  {
    hold_Alt=Get_Relative_Altitude();//Updating the current altitude until we switch to RTL
    return hold_Alt;
  }
  else
  {
    if(Tx_Switch_Status()==0x02)//RTL=0x02
    {
      return hold_Alt;
    }
    else
    {
      return wp_current_alt;//WP=0x01
    }
  }
#endif
}


void update_distance(void)
{
  wp_distance = calc_dist(lat, lon, wp_current_lat, wp_current_lon); 
}

/*************************************************************************
 * //Function to calculate the course between two waypoints
 * //I'm using the real formulas--no lookup table fakes!
 *************************************************************************/
int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float calc2;
  float bear_calc;
  float diflon;
  //I've to spplit all the calculation in several steps. If i try it to do it in a single line the arduino will explode.
  flat1=radians(flat1);
  flat2=radians(flat2);

  diflon=radians((flon2)-(flon1));

  calc=sin(diflon)*cos(flat2);
  calc2=cos(flat1)*sin(flat2)-sin(flat1)*cos(flat2)*cos(diflon);

  calc=atan2(calc,calc2);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc;
  }
  return bear_calc;
}
/*************************************************************************
 * //Function to calculate the distance between two waypoints
 * //I'm using  a really good approach
 *************************************************************************/
unsigned int calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters, i love the metric system.. =)
  return dist_calc;
}
