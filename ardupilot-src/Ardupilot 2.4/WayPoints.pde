/*****************************************************************************
 * This funtion will switch waypoint's dipending in the radius 
 *****************************************************************************/
// 11-04-09 : JLN added Get_Home_Position(void) for GPS Emulator -> stored into : home_lon, home_lat, home_alt

void Waypoint_Switcher(int distance, byte radius)
{
  static byte locked=0; //This variable allows only one switch... 
  if((distance < radius)&&(locked==0))   
  {
    locked=1; //Locking the switcher until the next waypoints
    current_wp++; //incrementing the waypoint number
    current_wp= (current_wp>wp_number) ? starting_wp : current_wp; //Verefy if the waypoint is more than the waypoints defined, i so restart the counter to zero. 
    //Set_New_Waypoint(current_wp);
    //Storing in eeprom... 
    eeprom_busy_wait(); 
    eeprom_write_byte((byte*)0x0A,current_wp); //Saving currrent waypoints to eempro, right now is not used. 
  }
  else
  {
    if(distance > radius) 
      locked=0; //Reset the lock
  }
}
/*****************************************************************************
 This routine looks more clomplex that it is. Is used to extract the waypoints from the eeprom 
 one by one when needed, so we don't saturate the RAM. 
 *****************************************************************************/
void Set_New_Waypoint(int current_waypoint)
{
    unsigned int mem_position; //Memory position on the EEPROM
  
    if(current_waypoint!=last_waypoint)//I put this to void innecesary access to the eeprom...
    {
      last_waypoint=current_waypoint;
      //Start byte is where the EEPROM location where the waypoints starts, 
      //every waypoint has 10 bytes so i multiply that by the current waypoint + the startbyte = the waypoint EEPROM position.
    mem_position = (unsigned int)((unsigned int)Start_Byte+(((unsigned int)current_waypoint)*10)); 
    eeprom_busy_wait();
    wp_current_lat=to_float_6((long)eeprom_read_dword((unsigned long*)mem_position));
    mem_position+=4;
    eeprom_busy_wait();
    wp_current_lon=to_float_6((long)eeprom_read_dword((unsigned long*)mem_position)); 
    mem_position+=4;
    eeprom_busy_wait();
    wp_current_alt=(int)eeprom_read_word((unsigned int*)mem_position);  
    }
}

