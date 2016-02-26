/***********************************************************
*                         DEFINES                          *
***********************************************************/

// Led Outputs
#define LASER1_PIN              2   //  LEDs for wing lasers here
#define LASER2_PIN              3   // 
#define LASER3_PIN              4   // 
#define LASER4_PIN              5		
#define TPDO_RED_PIN            6
#define TPDO_WHT_PIN            7	// Torpedo pins  - white and red led
#define ENG_PIN					8   // Engine LED pins. All 4 LEDs on same pin

// Sound Triggers
#define SND_ENG_PIN				12
#define SND_LASER_PIN			11
#define SND_TPDO_PIN			10
#define SND_SFOIL_PIN			9 

//Remote Pins
#define REMOTE_A_PIN          A0	// next 4 pins are for the remote input
#define REMOTE_B_PIN          A1	// they are reversed on the receiver board (as normal)
#define REMOTE_C_PIN          A2
#define REMOTE_D_PIN          A3


/***********************************************************
*                      GLOBAL VARS                         *
***********************************************************/
uint8_t  remoteBtnA_state = 0;       // Init variables and Set initial button states
uint8_t  remoteBtnB_state = 0;       // Init variables and Set initial button states
uint8_t  remoteBtnC_state = 0;       // Init variables and Set initial button states
uint8_t  remoteBtnD_state = 0;       // Init variables and Set initial button states
uint8_t  systemState      = 0;       // Used to track system intended state for light control
uint8_t  engineState	  = 0;		 // tracks engine state


/***********************************************************
*                          SETUP                           *
***********************************************************/
void setup(){
	Serial.begin(9600);
	
	//Input Pins Below
	pinMode(REMOTE_A_PIN, INPUT);
	pinMode(REMOTE_B_PIN, INPUT);
	pinMode(REMOTE_C_PIN, INPUT);
	pinMode(REMOTE_D_PIN, INPUT);
	
	
	//Output Pins - LED
	pinMode (LASER1_PIN, OUTPUT);
	pinMode (LASER2_PIN, OUTPUT);
	pinMode (LASER3_PIN, OUTPUT);
	pinMode (LASER4_PIN, OUTPUT);
	pinMode (TPDO_RED_PIN, OUTPUT);
	pinMode (TPDO_WHT_PIN, OUTPUT);
	pinMode (ENG_PIN, OUTPUT);
	
	//Output Pins - Sound
	pinMode (SND_ENG_PIN, OUTPUT);
	digitalWrite(SND_ENG_PIN, HIGH);
	
	pinMode (SND_LASER_PIN, OUTPUT);
	digitalWrite (SND_LASER_PIN, HIGH);
	
	pinMode (SND_TPDO_PIN, OUTPUT);
	digitalWrite (SND_TPDO_PIN, HIGH);
	
	pinMode (SND_SFOIL_PIN, OUTPUT);
	digitalWrite (SND_SFOIL_PIN, HIGH);
	
	pinMode (13, OUTPUT);
	digitalWrite(13, HIGH);
	
	
	
	}


/***********************************************************
*                          LOOP                            *
***********************************************************/
void loop(){
 readRemoteButtonStates() ;
  if (remoteBtnA_state){            // If A button is being pressed
    Serial.println("Remote Button A Detected");
		enginesToggle();                    // Turn on engine sound and lights
		delay(1000);
  }

  if (remoteBtnB_state){            // If B button is being pressed
    Serial.println("Remote Button B Detected");
		laserFire();           // toggles through brightness values
  }

  if (remoteBtnC_state){            // If C button is being pressed
    Serial.println("Remote Button C Detected");
		torpedoFire();	
  }

  if (remoteBtnD_state){            // If D button is being pressed
    Serial.println("Remote Button D Detected");
		sfoilActuate();
  }
  
}

/***********************************************************
*                 readRemoteButtonStates                   *
***********************************************************/
void readRemoteButtonStates(){
  // Read digitial inputs for the RF remote buttons and store as state
	remoteBtnA_state = digitalRead(REMOTE_A_PIN);
	remoteBtnB_state = digitalRead(REMOTE_B_PIN);
	remoteBtnC_state = digitalRead(REMOTE_C_PIN);
	remoteBtnD_state = digitalRead(REMOTE_D_PIN);
}

/***********************************************************
*                 engineToggle   		                   *
***********************************************************/
void enginesToggle(){
	Serial.println("engineToggle");
	if (!engineState){
		digitalWrite(ENG_PIN, HIGH);
		digitalWrite(SND_ENG_PIN, LOW);
		delay(50);
		engineState = 1;
		return;
	}
	if (engineState){
		digitalWrite(ENG_PIN, LOW);
		digitalWrite(SND_ENG_PIN, HIGH);
		delay(50);
		engineState = 0;
		return;
	}
	
	
	
}

/***********************************************************
*                 laserFire   	  		                   *
***********************************************************/
void laserFire(){
	Serial.println("laserFire");
		delay(5);
		digitalWrite(LASER1_PIN, HIGH);
		digitalWrite(SND_LASER_PIN, LOW);
		delay(100);
		digitalWrite(SND_LASER_PIN, HIGH);
		delay(300);
		digitalWrite(LASER1_PIN, LOW);
		//
		digitalWrite(LASER2_PIN, HIGH);
		digitalWrite(SND_LASER_PIN, LOW);
		delay(100);
		digitalWrite(SND_LASER_PIN, HIGH);
		delay(300);
		digitalWrite(LASER2_PIN, LOW);
		//
		digitalWrite(LASER3_PIN, HIGH);
		digitalWrite(SND_LASER_PIN, LOW);
		delay(100);
		digitalWrite(SND_LASER_PIN, HIGH);
		delay(300);
		digitalWrite(LASER3_PIN, LOW);
		//
		digitalWrite(LASER4_PIN, HIGH);
		digitalWrite(SND_LASER_PIN, LOW);
		delay(100);
		digitalWrite(SND_LASER_PIN, HIGH);
		delay(300);
		digitalWrite(LASER4_PIN, LOW);
		
	}
	
/***********************************************************
*                torpedo  	  		                   *
***********************************************************/
void torpedoFire(){
	Serial.println("torpedo fire");
		delay(5);
		digitalWrite(TPDO_WHT_PIN, HIGH);
		digitalWrite(SND_TPDO_PIN, LOW);
		delay(50);
		digitalWrite(TPDO_WHT_PIN, LOW);
		delay(50);
		digitalWrite(SND_TPDO_PIN, HIGH);
		digitalWrite(TPDO_RED_PIN, HIGH);
		delay(500);
		digitalWrite(TPDO_RED_PIN, LOW);
		delay(2000);
	}
/***********************************************************
*                Sfoil  	  		                   *
***********************************************************/
void sfoilActuate(){
	Serial.println("sfoil");
		delay(5);

		digitalWrite(SND_SFOIL_PIN, LOW);
		delay(100);
		digitalWrite(SND_SFOIL_PIN, HIGH);

	}
	
	
