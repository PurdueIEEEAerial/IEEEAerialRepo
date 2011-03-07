#if GPS_PROTOCOL == 3
/****************************************************************
 Parsing stuff for NMEA - GPS EMULATOR VERSION
 ****************************************************************/
void init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(9600);
//  Serial.begin(38400);
//  delay(1000);
//  Serial.print(LOCOSYS_BAUD_RATE_38400);
//  Serial.begin(38400);
//  delay(500);
//  Serial.print(LOCOSYS_REFRESH_RATE_250);
//  delay(500);
//  Serial.print(NMEA_OUTPUT_4HZ);
//  delay(500);
//  Serial.print(SBAS_OFF);
 
  delay(1000);
  Serial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(500);
  Serial.println("$PMTK313,1*2e"); 
  delay(500);
  Serial.println("$PMTK301,2*2e"); 
  delay(500);
  Serial.println("$PMTK220,200*2C"); // 5Hz
  delay(500); 
//  Serial.println("$PMTK251,38400*27");
//  delay(500);
  
  
  /* EM406 example init
  Serial.begin(4800); //Universal Sincronus Asyncronus Receiveing Transmiting 
  delay(1000);
  Serial.print(SIRF_BAUD_RATE_9600);
 
  Serial.begin(9600);
  delay(1000);
  
  Serial.print(GSV_OFF);
  Serial.print(GSA_OFF);
  
  #if USE_WAAS ==1
  Serial.print(WAAS_ON);
  #else
  Serial.print(WAAS_OFF);
  #endif*/
  
  Wait_GPS_Fix();
}
void fast_init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(9600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  //Serial.begin(38400);
}

void decode_gps(void)
{
  const char head_rmc[]="GPRMC"; //GPS NMEA header to look for
  const char head_gga[]="GPGGA"; //GPS NMEA header to look for
  
  static unsigned long GPS_timer=0; //used to turn off the LED if no data is received. 
  
  static byte unlock=1; //some kind of event flag
  static byte checksum=0; //the checksum generated
  static byte checksum_received=0; //Checksum received
  static byte counter=0; //general counter

  //Temporary variables for some tasks, specially used in the GPS parsing part (Look at the NMEA_Parser tab)
  unsigned long temp=0;
  unsigned long temp2=0;
  unsigned long temp3=0;


  while(Serial.available() > 0)
  {
    if(unlock==0)
    {
      gps_buffer[0]=Serial.read();//puts a byte in the buffer

      if(gps_buffer[0]=='$')//Verify if is the preamble $
      {
        counter = 0;
        checksum = 0; 
        unlock=1; 
      }
    }
    /*************************************************/
    else
    {
      gps_buffer[counter]=Serial.read();


      if(gps_buffer[counter]==0x0A)//Looks for \F
      {

        unlock=0;


        if (strncmp (gps_buffer,head_rmc,5) == 0)//looking for rmc head....
        {

          /*Generating and parsing received checksum, */
          for(int x=0; x<100; x++)
          {
            if(gps_buffer[x]=='*')
            { 
              checksum_received=strtol(&gps_buffer[x+1],NULL,16);//Parsing received checksum...
              break; 
            }
            else
            {
              checksum^=gps_buffer[x]; //XOR the received data... 
            }
          }

          if(checksum_received==checksum)//Checking checksum
          {
            /* Token will point to the data between comma "'", returns the data in the order received */
            /*THE GPRMC order is: UTC, UTC status ,Lat, N/S indicator, Lon, E/W indicator, speed, course, date, mode, checksum*/

            strcpy(gps_RMC,gps_buffer);
            token = strtok_r(gps_buffer, search, &brkb); //Contains the header GPRMC, not used

            token = strtok_r(NULL, search, &brkb); //UTC Time, not used
            //time=  atol (token);
            token = strtok_r(NULL, search, &brkb); //Valid UTC data? maybe not used... 


            //Longitude in degrees, decimal minutes. (ej. 4750.1234 degrees decimal minutes = 47.835390 decimal degrees)
            //Where 47 are degrees and 50 the minutes and .1234 the decimals of the minutes.
            //To convert to decimal degrees, devide the minutes by 60 (including decimals), 
            //Example: "50.1234/60=.835390", then add the degrees, ex: "47+.835390=47.835390" decimal degrees
            token = strtok_r(NULL, search, &brkb); //Contains Latitude in degrees decimal minutes... 

            //taking only degrees, and minutes without decimals, 
            //strtol stop parsing till reach the decimal point "."  result example 4750, eliminates .1234
            temp=strtol (token,&pEnd,10);

            //takes only the decimals of the minutes
            //result example 1234. 
            temp2=strtol (pEnd+1,NULL,10);

            //joining degrees, minutes, and the decimals of minute, now without the point...
            //Before was 4750.1234, now the result example is 47501234...
            temp3=(temp*10000)+(temp2);


            //modulo to leave only the decimal minutes, eliminating only the degrees.. 
            //Before was 47501234, the result example is 501234.
            temp3=temp3%1000000;


            //Dividing to obtain only the de degrees, before was 4750 
            //The result example is 47 (4750/100=47)
            temp/=100;

            //Joining everything and converting to float variable... 
            //First i convert the decimal minutes to degrees decimals stored in "temp3", example: 501234/600000= .835390
            //Then i add the degrees stored in "temp" and add the result from the first step, example 47+.835390=47.835390 
            //The result is stored in "lat" variable... 
            lat=temp+((float)temp3/600000);


            token = strtok_r(NULL, search, &brkb); //lat, north or south?
            //If the char is equal to S (south), multiply the result by -1.. 
            if(*token=='S'){
              lat=lat*-1;
            }

            //This the same procedure use in lat, but now for Lon....
            token = strtok_r(NULL, search, &brkb);
            temp=strtol (token,&pEnd,10); 
            temp2=strtol (pEnd+1,NULL,10); 
            temp3=(temp*10000)+(temp2);
            temp3=temp3%1000000; 
            temp/=100;
            lon=temp+((float)temp3/600000);

            token = strtok_r(NULL, search, &brkb); //lon, east or west?
            if(*token=='W'){
              lon=lon*-1;
            }

            token = strtok_r(NULL, search, &brkb); //Speed overground?
            ground_speed= atoi(token);

            token = strtok_r(NULL, search, &brkb); //Course?
            ground_course= atoi(token);

            data_update_event|=0x01; //Update the flag to indicate the new data has arrived. 

            digitalWrite(13,HIGH);
            Serial.print("$");
            Serial.println(gps_RMC);
            delay(10);
            digitalWrite(13,LOW);
            
          }
          checksum=0;
        }//End of the GPRMC parsing

        if (strncmp (gps_buffer,head_gga,5) == 0)//now looking for GPGGA head....
        {
          /*Generating and parsing received checksum, */
          for(int x=0; x<100; x++)
          {
            if(gps_buffer[x]=='*')
            { 
              checksum_received=strtol(&gps_buffer[x+1],NULL,16);//Parsing received checksum...
              break; 
            }
            else
            {
              checksum^=gps_buffer[x]; //XOR the received data... 
            }
          }

          if(checksum_received==checksum)//Checking checksum
          {
            strcpy(gps_GGA,gps_buffer);

            token = strtok_r(gps_buffer, search, &brkb);//GPGGA header, not used anymore
            token = strtok_r(NULL, search, &brkb);//UTC, not used!!
            token = strtok_r(NULL, search, &brkb);//lat, not used!!
            token = strtok_r(NULL, search, &brkb);//north/south, nope...
            token = strtok_r(NULL, search, &brkb);//lon, not used!!
            token = strtok_r(NULL, search, &brkb);//wets/east, nope
            token = strtok_r(NULL, search, &brkb);//Position fix, used!!
            gpsFix =atoi(token); 
            if(gpsFix >=1)
            gpsFix=0;
            else
            gpsFix=1;
            token = strtok_r(NULL, search, &brkb); //sats in use!! Nein...
            token = strtok_r(NULL, search, &brkb);//HDOP, not needed
            token = strtok_r(NULL, search, &brkb);//ALTITUDE, is the only meaning of this string.. in meters of course. 
            alt_MSL=atoi(token);
            if(alt_MSL<0){
              alt_MSL=0;
            }

            if(gpsFix==0x00) digitalWrite(12,HIGH); //Status LED...
            else digitalWrite(12,LOW);
            
            digitalWrite(13,HIGH);  
            Serial.print("$"); 
            Serial.println(gps_GGA);
            delay(10);  
            digitalWrite(13,LOW); 

            data_update_event|=0x02; //Update the flag to indicate the new data has arrived.
          }
          checksum=0; //Restarting the checksum
        }

        for(int a=0; a<=counter; a++)//restarting the buffer
        {
          gps_buffer[a]=0;
        } 
        counter=0; //Restarting the counter
        GPS_timer=millis(); //Restarting timer...
      }
      else
      {
        counter++; //Incrementing counter
        if (counter >= 200)
        {
          //Serial.flush();
          counter = 0; 
          checksum = 0;
          unlock = 0;
        }
      }
    }
  }
  
  if(millis() - GPS_timer > 2000){
      digitalWrite(12, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED... 
      gpsFix=1; 
    }  
}

void Save_Home_Position(void)
{
  launch_alt=vhome_alt;//Storing launch altitude... 
  current_wp=0;

  eeprom_busy_wait(); 
  eeprom_write_byte((byte*)0x0A,starting_wp);

  eeprom_busy_wait(); 
  //eeprom_write_word((unsigned int*)0x01,max_ir);//Saving Infrared Calibration.. air_speed_bias
  eeprom_write_word((unsigned int*)0x01,air_speed_offset);//Saving Airspeed..

  eeprom_busy_wait();
  eeprom_write_word((unsigned int*)0x0C,vhome_alt);//Saving virtual home altitude

  eeprom_busy_wait();
  eeprom_write_dword((unsigned long*)0x0E,(long)((float)vhome_lat*(float)t7));//Saving virtual home position.

  eeprom_busy_wait();
  eeprom_write_dword((unsigned long*)0x12,(long)((float)vhome_lon*(float)t7));

}

void Get_Home_Position(void)
{    
    eeprom_busy_wait(); 
    home_lat=to_float_6((long)eeprom_read_dword((unsigned long*)0x0E));
    eeprom_busy_wait();
    home_lon=to_float_6((long)eeprom_read_dword((unsigned long*)0x12)); 
    eeprom_busy_wait();
    home_alt=(int)eeprom_read_word((unsigned int*)0x0C);//Restoring launch altitude from eeprom...  
    launch_alt=home_alt; 
}

void print_remzibi_debug(void)
{
  static unsigned long timer2=0;
  
  if(millis()-timer2 > ATTITUDE_RATE_OUTPUT)
  {   
    digitalWrite(13,HIGH);
    Serial.print("$M");
    Serial.print("81"); // Column
    Serial.print("09"); // Row
    Serial.print("A9"); // First Character
    Serial.print("00"); // Last Character
    Serial.print("P:");  
    Serial.print(pitch_set_point);  
    Serial.print(",R:");  
    Serial.print((int)roll_set_point);  
    Serial.print(",T:");
    Serial.print((int)throttle_set_point);
    Serial.print(",S:");
    Serial.print((int)airSpeed());
    Serial.println("        ");

    timer2=millis(); 
    digitalWrite(13,LOW);
  }
}

void print_data_emulator(void)
{
  static unsigned long timer3=0;
  static byte counter;
  
  if(millis()-timer3 > ATTITUDE_RATE_OUTPUT)
  {   
    digitalWrite(13,HIGH);
    Serial.print("!!!");
    Serial.print("ASO:");
    Serial.print((int)air_speed_offset);
    Serial.print(",AN3:");
    Serial.print((int)analog3);
    Serial.print(",ASP:");
    Serial.print((int)airSpeed()); 
    Serial.print (",STT:");
    Serial.print((int)Tx_Switch_Status());
    Serial.print (",WPN:");
    Serial.print((int)last_waypoint);  //This is the target waypoint.
    Serial.print (",DST:");
    Serial.print(wp_distance);
    Serial.print (",RER:");
    Serial.print((int)roll_set_point);
    Serial.print(",THH:");
    Serial.print((int)throttle_set_point);
    Serial.print (",PSET:");
    Serial.print(pitch_set_point);
    Serial.print (",GPS:");
    Serial.print(gpsFix);
    Serial.println(",***");
    timer3=millis(); 
    digitalWrite(13,LOW);
  }
}

#endif

