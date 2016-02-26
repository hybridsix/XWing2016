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
#define REMOTE_A_PIN          A3	// next 4 pins are for the remote input
#define REMOTE_B_PIN          A2	// they are reversed on the receiver board (as normal)
#define REMOTE_C_PIN          A1
#define REMOTE_D_PIN          A0


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
	pinMode (SND_LASER_PIN, OUTPUT);
	pinMode (SND_TPDO_PIN, OUTPUT);
	pinMode (SND_SFOIL_PIN, OUTPUT);
	
	
	
	}


/***********************************************************
*                          LOOP                            *
***********************************************************/
void loop(){
 readRemoteButtonStates() ;
  
  if (remoteBtnA_state){            // If A button is being pressed
    Serial.println("Remote Button A Detected");
		enginesToggle();                    // Turn on engine sound and lights
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
*                 readRemoteButtonStates                   *
***********************************************************/
void enginesToggle(){
	if (!engineState){
		digitalWrite(ENG_PIN, HIGH);
		digitalWrite(SND_ENG_PIN, LOW);
	}
	else {
		digitalWrite(ENG_PIN, LOW);	
		digitalWrite(SND_ENG_PIN, HIGH)
	}
	
	
	
}
