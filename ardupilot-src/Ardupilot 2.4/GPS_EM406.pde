#if GPS_PROTOCOL == 1

/****************************************************************
 Parsing stuff for SIRF binary protocol. 
 ****************************************************************/
void init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  delay(500);
  change_to_sirf_protocol();
  Serial.begin(57600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  delay(500);//Waits fot the GPS to start_UP
  configure_gps();//Function to configure GPS, to output only the desired msg's
  Wait_GPS_Fix();
}
/****************************************************************
 ****************************************************************/
void fast_init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(57600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  delay(100);//Waits fot the GPS to start_UP
  configure_gps();//Function to configure GPS, to output only the desired msg's
}
/****************************************************************
 ****************************************************************/
void decode_gps(void)
{
  static unsigned long GPS_timer=0;
  static byte gps_counter=0; //Another gps counter for the buffer
  static byte GPS_step=0;
  static byte gps_ok=0;//Counter to verify the reciving info
  const byte read_gps_header[]={
    0xA0,0xA2,0x00,0x5B,0x29      };//Used to verify the payload msg header

  if(millis()-GPS_timer > 200) //Timer to execute the loop every 200 ms only.
  {
    if(Serial.available()>0)//Ok, let me see, the buffer is empty?
    {
      switch(GPS_step) //Let see in which step i'am.
      {
      case 0: //This case will verify the header, to know when the payload will begin 
        while(Serial.available()>0)  //Loop if data available
        {
          if(Serial.read()==read_gps_header[gps_ok]) //Checking if the head bytes are equal.. 
          {
            gps_ok++; //If yes increment 1
          }
          else{ 
            gps_ok=0; //Otherwise restart.
          }
          if(gps_ok >= 5) //Ohh 5 bytes are correct, that means jump to the next step, and break the loop
          {
            gps_ok=0;
            GPS_step++;
            break;
          }
        }
        break; 
      case 1: //Will receive all the payload and join the received bytes... 
        while(Serial.available()>0) //Loop if theres something in the buffer 
        {
          gps_buffer[gps_counter++]=Serial.read(); //Read data and store it in the temp buffer

          if(gps_counter>=92) //If we got 92 bytes (all the payload) then... 
          {
            gps_counter=0; //Restart the counter... 
            while(Serial.available()==0){
            }//Wait for the ender bytes 
            if((Serial.read()==gps_ender[0])&&(Serial.read()==gps_ender[1])) //Check if we are in the end of the payload, then... 
            {
              unsigned int gps_checksum_verif=0x29;//Restart the checksum value 
              for(int j=0; j<90; j++)//Checksum verifycation, chachan! this is the moment of the true
              {
                gps_checksum_verif+=gps_buffer[j];//I simply plus all the payload
              }
              if(((gps_buffer[90]<<8)|gps_buffer[91]) == gps_checksum_verif) //Now i will check if the checksum i just made with the received checksum are equal, if yes... 
              {
                GPS_join_data(); //Joing the data
                data_update_event|=0x01;
                data_update_event|=0x02;
                //GPS_print_data(); // and print the values... 
              }
            }
            GPS_step=0; //Restarting.... 
            break;
          }
        }
        break;
      }
      GPS_timer=millis(); //Restarting timer... 
    }
    if(millis() - GPS_timer > 2000){
      digitalWrite(12, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED... 
      gpsFix=1; 
    }
  }
}
/****************************************************************
 ****************************************************************/
void GPS_join_data(void)
{
  gpsFix=(int)gps_buffer[0]<<8; //Here i'm joining the status GPS 2 Bytes to 1 Int. 
  gpsFix|=(int)gps_buffer[1];

  if(gpsFix == 0) //If the status GPS is equals to cero YEAAHH! turn ON the LED
  {
    digitalWrite(12,HIGH);
  }
  else{
    digitalWrite(12,LOW);
  }//If not UHH something is really wrong, don't look at me! is GPS fault!

  byte j=22;//Switching the byte position on the buffer

  //lat = to_float_7(join_4_bytes(gps_buffer[j++], gps_buffer[j++], gps_buffer[j++], gps_buffer[j++]));//Joining latitude Bytes
  longUnion.byte[3] = gps_buffer[j++];
  longUnion.byte[2] = gps_buffer[j++];
  longUnion.byte[1] = gps_buffer[j++];
  longUnion.byte[0] = gps_buffer[j++];
  lat=to_float_7(longUnion.dword);

  //lon = to_float_7(join_4_bytes(gps_buffer[j++], gps_buffer[j++], gps_buffer[j++], gps_buffer[j++]));//Joining longitude Bytes

  longUnion.byte[3] = gps_buffer[j++];
  longUnion.byte[2] = gps_buffer[j++];
  longUnion.byte[1] = gps_buffer[j++];
  longUnion.byte[0] = gps_buffer[j++];
  lon=to_float_7(longUnion.dword);

  j=34;//Switching the byte position on the buffer
  //alt_MSL = (float)(join_4_bytes(gps_buffer[j++], gps_buffer[j++], gps_buffer[j++], gps_buffer[j++]))/100.0;//Joining altitude Bytes
  longUnion.byte[3] = gps_buffer[j++];
  longUnion.byte[2] = gps_buffer[j++];
  longUnion.byte[1] = gps_buffer[j++];
  longUnion.byte[0] = gps_buffer[j++];
  alt_MSL = (float)longUnion.dword/100.0;

  j=39;//Switching the byte position on the buffer
  //ground_speed= (float)join_2_bytes(gps_buffer[j++], gps_buffer[j++])/100.0;//Joining Ground Speed Bytes
  intUnion.byte[1] = gps_buffer[j++];
  intUnion.byte[0] = gps_buffer[j++];
  ground_speed= (float)intUnion.word/100.0;

  if(ground_speed>=.5)//Only updates data if we are really moving... 
  {
    //ground_course=(float)join_2_bytes(gps_buffer[j++], gps_buffer[j++])/100.0;//Joining Course Bytes
    intUnion.byte[1] = gps_buffer[j++];
    intUnion.byte[0] = gps_buffer[j++];
    ground_course= (float)intUnion.word/100.0;
    ground_course=abs(ground_course);//The GPS has a BUG sometimes give you the correct value but negative, weird!! 
  }
  j=45;//Switching the byte position on the buffer
  //climb_rate=(float)join_2_bytes(gps_buffer[j++], gps_buffer[j++])/100.0;  //Joining climb rate Bytes
  intUnion.byte[1] = gps_buffer[j++];
  intUnion.byte[0] = gps_buffer[j++];
  climb_rate= (float)intUnion.word/100.0;

  for(int j=0; j<94; j++)
  {
    gps_buffer[j]=0; //Now is time to say good bye to the buffer..=(.. Every byte was like a son.. 
  } 
}
/****************************************************************
 ****************************************************************/
void configure_gps(void)
{
  const byte gps_header[]={
    0xA0,0xA2,0x00,0x08,0xA6,0x00      };//Used to configure Sirf GPS
  const byte gps_payload[]={
    0x02,0x04,0x07,0x09,0x1B      };//Used to configure Sirf GPS
  const byte gps_checksum[]={
    0xA8,0xAA,0xAD,0xAF,0xC1      };//Used to configure Sirf GPS
  const byte cero=0x00;//Used to configure Sirf GPS

  for(int z=0; z<2; z++)
  { 
    for(int x=0; x<5; x++)//Print all messages to setup GPS
    {
      for(int y=0; y<6; y++)
      {
        Serial.print(byte(gps_header[y]));//Prints the msg header, is the same header for all msg..  
      } 
      Serial.print(byte(gps_payload[x]));//Prints the payload, is not the same for every msg
      for(int y=0; y<6; y++)
      {
        Serial.print(byte(cero)); //Prints 6 ceros
      } 
      Serial.print(byte(gps_checksum[x])); //Print the Checksum
      Serial.print(byte(gps_ender[0]));  //Print the Ender of the string, is same on all msg's. 
      Serial.print(byte(gps_ender[1]));  //ender  
    }
  }  
}

/****************************************************************
 ****************************************************************/

void change_to_sirf_protocol(void)
{
  digitalWrite(13, HIGH);
  Serial.begin(4800); //First try in 4800
  delay(300);
  for (byte x=0; x<=28; x++)
  {
    Serial.print(byte(gps_buffer[x]));//Sending special bytes declared at the beginning 
  }  
  digitalWrite(13, LOW);
  delay(300);
  digitalWrite(13, HIGH);
  Serial.begin(9600); //Then try in 9600 
  delay(300);
  for (byte x=0; x<=28; x++)
  {
    Serial.print(byte(gps_buffer[x]));
  }  
  digitalWrite(13, LOW);  
}  
#endif

/****************************************************************
 ****************************************************************/
void Wait_GPS_Fix(void)//Wait GPS fix...
{
  do
  {
    decode_gps();
    digitalWrite(12,HIGH);
    delay(25);
    digitalWrite(12,LOW);
    delay(25);
  }
  while(gpsFix!=0);//

#if GPS_PROTOCOL == 0 //If NMEA
  do
  {
    decode_gps(); //Reading and parsing GPS data  
    digitalWrite(12,HIGH);
    delay(25);
    digitalWrite(12,LOW);
    delay(25);
  }
  while((data_update_event&0x01!=0x01)&(data_update_event&0x02!=0x02));
#endif
}

/****************************************************************
 ****************************************************************/
float to_float_7(long value) 
{
  return (float)value/(float)10000000;
}
float to_float_6(long value) 
{
  return (float)value/(float)1000000;
}


