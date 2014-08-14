#include <phys253.h>
#include <LiquidCrystal.h>
#include <servo253.h> 

// ARTIFACT ARM TEST CODE
// Code runs through all four parts of the arm and sees if it is working as functioned before
// conducting the actual code.
//
// SETUP:
// 	1. Artifact Sensor
// 		- Place object or artifact between the sensors, should display detected/not detected
// 	2. Arm XY
// 		- Adjust knob 6, arm should rotate according to the displayed angle
// 	3. Arm Z
// 		- Press Start for arm to move up, press stop for arm to move down
//		- Adjust knob 7 greater than 500 to move on
// 	4. Arm Magnet
// 		- Press a switch on the magnet, should displayed detected/not detected
// LOOP:
// 	When the artifact arm detects an artifact, arm grabs the artifact and drops it in the basket


// PINS 0-7 INPUT
// PINS 8-15 OUTPUT
// SERVO 0 ARM

int aaZstep = 8;		  // DIGITAL O
int aaZdirection = 9;     // DIGITAL O
int aaMagnet = 1;
int aaSensor = 0; 

int minStepDelay = 1000;

void setup()
{
	portMode(0, INPUT) ;
	portMode(1, OUTPUT) ;
	RCServo0.attach(RCServo0Output) ;
	RCServo1.attach(RCServo1Output) ;
	RCServo2.attach(RCServo2Output) ;

	// ARTIFACT ARM SENSOR
                LCD.clear();
		LCD.print("Begin aaSensor");
		delay(3000);

		while ( !startbutton() ) {
			LCD.clear();

			int objDetected = digitalRead(aaSensor);
			if ( objDetected ) {
                                LCD.print(objDetected);
                                LCD.setCursor(0,1);
				LCD.print("Artifact Detected!");
			}
			else {
                                  LCD.print(objDetected);
                                LCD.setCursor(0,1);
				LCD.print("Artifact NOT Detected!");
			}
			delay(200);
		}

	// ARTIFACT ARM XY (Servo Test)	  
                LCD.clear();
  		LCD.print("Begin aaXY");
		delay(3000);

		while ( !startbutton() ) {
			LCD.clear();

			int servoAnalog = knob(6);
			int angle = 0.176*servoAnalog;

			LCD.print(angle);
			RCServo0.write(angle);
			delay(200);
		}

	// ARTIFACT ARM Z (Stepper Test)	
                LCD.clear();
  		LCD.print("Begin aaZ");
		delay(3000);

		if (knob(7) > 500) {
			while ( !startbutton() ) {
				LCD.clear();
				LCD.print("KNOB 7 < 500?");
				LCD.setCursor(0,1);
				int knob7 = knob(7);
				LCD.print(knob7);
				delay(200);
			}
		}
                LCD.clear();
		LCD.print("Begin aaZ");
		delay(3000);

		while ( knob(7) < 500 ) {
			int velocity = knob(6);
			LCD.clear();
			LCD.print("V = "); LCD.print(velocity);
			LCD.setCursor(0,1);
			LCD.print("K7 = "); LCD.print(knob(7));

			int stepDelay = (102400) - (velocity*10) + minStepDelay;

			if ( startbutton() ) {
				stepperControl(100, minStepDelay);
			}
                        if ( stopbutton() ) {
                               stepperControl(-100, minStepDelay); 
                        }
        
		}


	// ARTIFACT ARM MAGNET  
                LCD.clear();
  		LCD.print("Begin aaMagnet");
		delay(3000);

		while ( !startbutton() ) {
			LCD.clear();

			int magnetOn = digitalRead(aaMagnet);

			if ( magnetOn ) {
				LCD.print("Magnet ON");
			}
			else {
				LCD.print("Magnet OFF");
			}

			delay(300);
		}
                LCD.clear();
		LCD.print("Beginning Actual Arm Code!");
		delay(3000);

}

void loop () {
	LCD.clear();
	LCD.print("TODO TODO TODO");
}

void stepperControl ( int steps, int stepDelay ) {
	if (steps < 0) { 
		digitalWrite(aaZdirection, LOW); // Set counter-clockwise direction 
	}
	else { 
		digitalWrite(aaZdirection, HIGH); // Set clockwise direction 
	} 
	// Moves desired number of steps 
	// Motor rotates one step when STEP_PIN changes from HIGH to LOW 
	
	for(int i = 0; i < abs(steps); i++)  { 
		digitalWrite(aaZstep, HIGH); 
		delayMicroseconds(stepDelay/2); 
		digitalWrite(aaZstep, LOW); 
 		delayMicroseconds(stepDelay/2); 
	} 
}
