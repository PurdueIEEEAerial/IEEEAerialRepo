#if GROUNDSTATION == 1

#define GS_BUF_LEN 60
char GS_buffer[GS_BUF_LEN];

void readCommands_GS(void)
{
	static byte bufferPointer = 0;
	static byte header[2];
	const byte read_GS_header[] 	= {0x21, 0x21}; //Used to verify the payload msg header

	if(Serial.available()){
	
		Serial.println("Serial.available");
		bufferPointer = 0;
		header[0] = Serial.read();
		header[1] = Serial.read();
		
		byte test = 0;

		if(header[0] == read_GS_header[0]) test++;
		if(header[1] == read_GS_header[1]) test++;
		
		if(test == 2){
			// Block until we read full command 
			// --------------------------------
			delay(40);
			byte incoming_val = 0;

			// Ground Station communication 
			// ----------------------------
			while(Serial.available() > 0) 
			{
				incoming_val = Serial.read();		 

				if (incoming_val != 13 && incoming_val != 10 ) {	 
					GS_buffer[bufferPointer++] = incoming_val;	
				}

				if(bufferPointer >= GS_BUF_LEN){
					Serial.println("Big buffer overrun");
					bufferPointer = 0;
					GS_buffer[0] = 1;
					Serial.flush();
					memset(GS_buffer,0,sizeof(GS_buffer));
					return;
				}
			}
			parseCommand_GS(GS_buffer);
			
			// clear buffer of old data
			// ------------------------
			memset(GS_buffer,0,sizeof(GS_buffer));

		}else{
			Serial.flush();
		}
	}
}

// Commands can be sent as !!a:100|b:200|c:1
// -----------------------------------------
void parseCommand_GS(char *buffer)
{
	Serial.println("got cmd ");
	char *token, *saveptr1, *saveptr2;
	
	for (int j = 1;; j++, buffer = NULL) {
		token = strtok_r(buffer, "|", &saveptr1);
		if (token == NULL) break;	
		
		char * cmd 		= strtok_r(token, ":", &saveptr2);
		long value		= strtol(strtok_r (NULL,":", &saveptr2), NULL,0);
		
		///*
		Serial.print("cmd ");
		Serial.print(cmd[0]);
		Serial.print("\tval ");
		Serial.println(value);
		Serial.println("");
		/*
float head_P 	 			= HEAD_P; 				// Heading error proportional 
float head_I 	 			= HEAD_I;  				// Heading error integrator
float pitch_P 				= PITCH_P; 				// Altitude error proportional - controls the pitch with elevators 

// Attitude control gains
// ------------------------
float rudder_P 				= RUDDER_P; 				// Altitude error proportional - controls the pitch with elevators 
float rudder_I 				= RUDDER_I; 				// Altitude error proportional - controls the pitch with elevators 
float elevator_P 			= ELEVATOR_P; 				// Altitude error proportional - controls the pitch with elevators 
float elevator_I 			= ELEVATOR_I; 				// Altitude error proportional - controls the pitch with elevators 
		
		*/
		switch(cmd[0]){
		
			// Heading control gains
			// ------------------------
			case 'h':
			head_P = (float)value/100.0;
			break;

			case 'H':
			head_I = (float)value/100.0;
			break;
			
			case 'Z':
			head_D = (float)value/100.0;
			break;

			case 'p':
			pitch_P = (float)value/100.0;
			break;

			case 'P':	
			pitch_I = (float)value/100.0;
			break;

			// Attitude control gains
			// ------------------------
			case 'r':
			rudder_P = (float)value/100.0;
			break;

			case 'R':
			rudder_I = (float)value/100.0;
			break;

			case 'e':
			elevator_P = (float)value/100.0;
			break;

			case 'E':
			elevator_I = (float)value/100.0;
			break;
			
			// Roll Limit
			// ----------
			case 'l':
			roll_max = (int)value * 100;
			break;
	
			// pitch comp
			// ----------
			case 'c':
			pitch_comp = (float)value/100.0;
			break;


			//case 't':
			//throttle_P = (float)value/100.0;
			//break;
			
			case 'w':
			while (value >0){
				demo_servos();
				delay(200);
				value--;
			}
			break;
	
			//case 'r': // return Home
			//return_to_launch();
			//break;

			case 's':
			reached_waypoint();
			load_waypoint();
			break;
			
			case 'z'://reset
			wp_index = 1;
			load_waypoint();
			break;
			
			case 'd'://reset
			print_gains();
			break;

		}
		//*/
	}
	print_gains();
}

#endif