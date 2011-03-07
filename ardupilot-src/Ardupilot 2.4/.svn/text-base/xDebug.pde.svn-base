/*****************/
/*Debugging Stuff*/
/*****************/

#if PRINT_WAYPOINTS == 1
void print_waypoints(void)
{  
  Serial.print("# of WP: ");
  Serial.println((int)wp_number);
  for(int x=0; x<=(int)wp_number; x++)
  {
    Set_New_Waypoint(x);
    Serial.print(x);
    Serial.print(" Lat: ");
    Serial.print((long)(wp_current_lat*t7));
    Serial.print(" Lon: ");
    Serial.print((long)(wp_current_lon*t7));
    Serial.print(" Alt: ");
    Serial.println(wp_current_alt);
    delay(200);
  }
    #if GPS_PROTOCOL == 3 
    Get_Home_Position();
    Serial.print("Virtual Home");
    Serial.print(" Lat: ");
    Serial.print((long)(home_lat*t7));
    Serial.print(" Lon: ");
    Serial.print((long)(home_lon*t7));
    Serial.print(" Alt: ");
    Serial.println(home_alt);
    #endif
  Set_New_Waypoint(0);
  delay(3000);
}
#endif

#if TEST_THROTTLE == 1
void test_throttle(void)
{
  Serial.println("Ready to test the throttle?");
  delay(1000);
  for(int x=0; x<40; x++)
  {
    pulse_servo_0(x+throttle_dead_zone);
    Serial.print(x);
    delay(100);
  }
  pulse_servo_0(0); 
}
#endif

#if CALIBRATE_SERVOS == 1
void calibrate_servos(void)
{
  while(1)
  {
    Serial.begin(57600);
    pulse_servos(0,0);
    Serial.print(OCR1A/2);
    Serial.print(" ");
    Serial.println(OCR1B/2);
    delay(2000);
    pulse_servos(45,45);
    Serial.print(OCR1A/2);
    Serial.print(" ");
    Serial.println(OCR1B/2);
    delay(2000);
    pulse_servos(-45,-45);
    Serial.print(OCR1A/2);
    Serial.print(" ");
    Serial.println(OCR1B/2);
    delay(2000); 

  }
}
#endif


#if PRINT_EEPROM == 1
void Print_EEPROM(void)
{
  /*
  for(int c=0x00; c<=0x3F; c++)
  {
    Serial.print("0x");
    Serial.print(c,HEX);
    Serial.print("-> ");
    for(int a=0x00; a<=0x0F; a++)
    {
    Serial.print(" 0x");  
    Serial.print(a,HEX);
    Serial.print(":");
    eeprom_busy_wait();
    Serial.print((int)(byte)eeprom_read_byte((byte*)((c*16)+a)),HEX);
    
    }*/
      for(unsigned int c=0x00; c<=0x200; c+=0x02)
  {
    Serial.print(c,HEX);
    Serial.print(":");
    eeprom_busy_wait();
    Serial.print((unsigned int)eeprom_read_word((unsigned int*)c),HEX);
    Serial.println(" ");
  }
  delay(2000);
}
#endif

