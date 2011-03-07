/**************************************************************
 * Configuring the PWM hadware... If you want to understand this you must read the Data Sheet of atmega168..  
 ***************************************************************/
void Init_servos(void)//This part will configure the PWM to control the servo 100% by hardware, and not waste CPU time.. 
{   
  digitalWrite(10,LOW);//Defining servo output pins
  pinMode(10,OUTPUT);
  digitalWrite(9,LOW);
  pinMode(9,OUTPUT);
  /*Timer 1 settings for fast PWM*/

    //Remember the registers not declared here remains zero by default... 
  TCCR1A =((1<<WGM11)|(1<<COM1B1)|(1<<COM1A1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR1A = 3000; //the period of servo 1, remember 2us resolution, 3000/2 = 1500us the pulse period of the servo...    
  OCR1B = 3000; //the period of servo 2, 3000/2=1500 us, more or less is the central position... 
  ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000, 
  //must be 50hz because is the servo standard (every 20 ms, and 1hz = 1sec) 1000ms/20ms=50hz, elementary school stuff... 

  /*************************************************************/
  /*From here everthing was made to create the throttle servo*/

  TIMSK1 |=(1 << ICIE1); //See page 136, timer 1 interrupt mask

  //Setting up the Timer 2
  TCCR2A = (1<<WGM21); //CTC mode
  TCCR2B =(1<<CS20)|(1<<CS22); //prescaler 128, at 16mhz (128/16)=8, the counter will increment 1 every 8us
  OCR2A = 138; //1500us/8; The top, when the counter reaches the value definied here will execute the interrupt, 187 is the servo centered... 
  //OCR2B = 138;
  //TIMSK2 = (1<<OCIE2A)|(1<<OCIE2B); //interrupt masks for counter A and B
  TIMSK2 = (1<<OCIE2A); //Don't touch!! you have nothing to do here. 
  sei();//Enabling interrupts
}
/*************************************************************************
 * 
 *************************************************************************/

ISR(TIMER1_CAPT_vect)//This is a timer 1 interrupts, executed every 20us 
{
  TCNT2=0; //restarting the counter of timer 2
  PORTB |= 0x01; //Putting the pin high!
  //stabilization_interrupt();
  //PORTB |= 0x09; //Putting the pins high (pint 8 and 11 of arduino)...
}
/*************************************************************************
 * 
 *************************************************************************/
ISR(TIMER2_COMPA_vect ) //Interrupt of timer 2 compare A
{
  PORTB &= 0xFE;//Putting the pin low

  /*
  if((PORTB&0x01)==1)
   PORTB &= 0xFE;
   else
   PORTB |= 0x01;*/
}
/*************************************************************************
 * 
 *************************************************************************/
/*
ISR(TIMER2_COMPB_vect ) //Interrupt of timer 2 compare B, not used now
 {
 //PORTB &= 0xF7;//Putting the pin low
 }*/

/*************************************************************************
 * You must change this if you have another kind of aircraft... 
 *************************************************************************/
void pulse_servos(int roll, int pitch) //Normal mode 
{
  
  #if TEST_SENSORS == 1
  roll= constrain(get_roll(),-45,45);
  pitch = constrain(get_pitch(),-45,45);
  #endif
  
#if MIXING_MODE == 0

  pulse_servo_1(90+(REVERSE_ROLL*roll));
  pulse_servo_2(90+(REVERSE_PITCH*pitch));

#endif
  /*V tail mode*/
#if MIXING_MODE == 1
  pulse_servo_1(90-((REVERSE_PITCH*pitch)+(REVERSE_ROLL*roll)));

  pulse_servo_2(90+((REVERSE_PITCH*pitch)-(REVERSE_ROLL*roll)));

#endif
}
/**************************************************************
 * Function to pulse servo 0
 ***************************************************************/
void pulse_servo_0(long porcent)//Will convert the angle to the equivalent servo position... 
{
  porcent=constrain(porcent,0,100);
  //OCR2A=(((porcent*(long)(throttle_max-throttle_min))/100L)+throttle_min)/8L;
  #if REVERSE_THROTTLE == 1
  porcent=100-porcent;//reversing thrrotle
  #endif 
  OCR2A=(byte)(porcent+137L);
  //Serial.println((porcent+137L));
}

/**************************************************************
 * Function to pulse servo 1
 ***************************************************************/
void pulse_servo_1(long angle)//Will convert the angle to the equivalent servo position... 
{
  angle=constrain(angle,0,180);
  OCR1A=(((angle*(SERVO_AILE_MAX-SERVO_AILE_MIN))/180L)+SERVO_AILE_MIN)*2L;
}

/**************************************************************
 * Function to pulse the servo 2... 
 ***************************************************************/
void pulse_servo_2(long angle)//Will convert the angle to the equivalent servo position... 
{
  angle=constrain(angle,0,180);
  OCR1B=(((angle*(SERVO_ELEV_MAX-SERVO_ELEV_MIN))/180L)+SERVO_ELEV_MIN)*2L; //Scaling
}

/**************************************************************
 * Function to test the servos.. 
 ***************************************************************/
void test_servos(void)
{
  pinMode(4,OUTPUT); //MUX pin 
  digitalWrite(4,HIGH);

  for(int j=0; j<=3; j++)
  {
    switch(j)
    {
    case 0: 
      pulse_servos(-45, 0); 
      break;
    case 1: 
      pulse_servos(45, 0); 
      break;
    case 2: 
      pulse_servos(0, -45); 
      break;
    case 3: 
      pulse_servos(0, 45); 
      break;
    }
    delay(800);
  }
  pulse_servos(0,0);  

  digitalWrite(4,LOW); 
  pinMode(4,INPUT);
}

/**************************************************************
 * Function to read the channels 1 and 2.. 
 ***************************************************************/
int read_Ch1(void)
{
  int temp;
  static int ch;
  temp=pulseIn(2,HIGH, 5000);
  ch= (temp!=0) ? temp : ch; 
  return (REV_FLY_BY_WIRE_CH1*(ch-rx_Ch[0]));
}

int read_Ch2(void)
{
  int temp;
  static int ch;
  temp=pulseIn(3,HIGH, 5000);
  ch= (temp!=0) ? temp : ch; 
  return (REV_FLY_BY_WIRE_CH2*(ch-rx_Ch[1])); //Scary he?
}




