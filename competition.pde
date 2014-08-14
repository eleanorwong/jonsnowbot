/*
        /\___/\         WELCOME TO ENPH253 ROBOT CODE (Team 1)
  /   /  .   . \        
  \   \    ^   /        last updated: july 15, 2014
  \  /        \         version: 0.1
  \ /______  \          author: eleanor wong

---------------------------------------------------------------
PINS AND NUMBERS
---------------------------------------------------------------

  Digital                          Analog
  -------------------------------------------------------------
     0        Artifact Sensor        0        Left QRD
     1        Magnet Sensor          1        Right QRD
     2        Zipline Sensor         2        Left IR
     8        Stepper Signal (O)     3        Right IR
     9        Stepper Direction (O)

  Servos                           Motors
  -------------------------------------------------------------
     0        Artifact Arm           0        Left Rear Motor
     1        Artifact Sensor        1        Right Rear Motor
     2        Zipline Arm    

*/

// REMINDER: motor.stop()

// SOME LIBRARIES
    #include <phys253.h>
    #include <LiquidCrystal.h>
    #include <servo253.h> 

// GLOBAL VARIABLES
     int knobChoice;
     int mode;
     int function;

    // DIGITAL I                                  // DIGITAL O                          // PWM
        int aaSensor = 0;                             int aaZstep = 8;                      int motorLeft = 0;
                                                      int aaZdirection = 9;

    // ANALOG I/O
        int leftQRDpin = 0; int rightQRDpin = 1;
        int leftIRpin = 3; int rightIRpin = 4;

    // QRD DRIVE
        int threshold = 250; int defaultSpeed = 400;

        int kp = 111; int kd = 183; double d; double p; double con;
        int currentPeriod = 0; int displayCounter = 0;

        int leftQRD; int rightQRD;
        int error; int prevError = 0; int recentError; int lastTime;

        int speedLeft; int speedRight;

    // IR DRIVE
        int startIR = false;
        int irDefaultSpeed = 610;
        int irKP = 28; int irKD = 0; 
        int irStartThreshold = 50;

        int irErrorThresholdSmall = 5;
        int irQRDthresh = 300;

        int irLeft; int irRight;
        int irError; int irPrevError; int irRecentError;
        double irCon; double irP; double irD;

        int irCurrentPeriod; int irLastTime; int irDisplayCounter;
        int reTapeFollow;

        int sweepSpeed = 500;

    // ARTIFACT ARM
        int minStepDelay = 800;
        int lastDetect = 0;
        int time1 = 0; int time2 = 0;
        int artifactThreshold = 30;

        int downMin = -300; int downMax = 3300;
        int upMin = 300;    int upMax = -3300;

        int aaCount = 0;
        int artifactDelay = 950;

        int sensorOut = 180; int sensorIn = 60;

        int armOut = 180; int armIn = 0;

        int pickup = 0;

void setup() {
    portMode(0, INPUT) ;
    portMode(1, OUTPUT) ;
    RCServo0.attach(RCServo0Output) ;
    RCServo1.attach(RCServo1Output) ;
    RCServo2.attach(RCServo2Output) ;

    RCServo1.write(sensorIn);
    RCServo0.write(armIn);

    LCD.clear();
    LCD.print("1: AARM 2: MOTOR 3: IR");
    LCD.setCursor(0,1);
    LCD.print("4: COMP SETUP");

    delay(2000);

    // SETUP
    while ( !startbutton() ) {
        LCD.clear();
        int knobVal = knob(6);
        knobChoice = knobVal/205;
        LCD.print(knobChoice);

        delay(200);
    }

    if ( knobChoice == 0 ) {
        functionTest();
    }

    if ( knobChoice == 1 ) {
        aaTest();
    }
    else if ( knobChoice == 2 ) {
        motorTest();
    }
    else if ( knobChoice == 3 ) {
        irTest();
    }
    else if ( knobChoice == 4 ) {
        competitionSetup();
    }

}

void loop () {
    // KP KD LOOP FOR FAST ADJUSTING
        if ( stopbutton() ) {
            while ( !startbutton() ) {
                motor.speed(0, 0);
                motor.speed(1, 0);
                LCD.clear();
                kp = knob(6);
                kd = knob(7);
                LCD.print("KP = "); LCD.print(kp);

                LCD.setCursor(0, 1);
                LCD.print("KD = "); LCD.print(kd);
                delay(500);
            }
            LCD.clear();
            LCD.print("STARTING");
            delay(2000);
        }

            tapeFollow();
            artifact();

            if ( aaCount == 3 && !reTapeFollow ) {
                turn180();
                reTapeFollow = true;
            }

        // if ( checkIR() ) {
        //     while ( checkIR() ) {
        //         irFollow();
        //         artifact();
        //     }
        // }

        // else if ( mode == 1 ) {
        //     tapeFollow();
        //     artifact();

        //     if ( checkIR() ) {
        //         while ( aaCount < 3 ) {
        //             irFollow();
        //             artifact();
        //         }

        //         turn180rocks();
                
        //         irLeft = analogRead(leftIRpin);
        //         irRight = analogRead(rightIRpin);

        //         while ( irLeft < irQRDthresh && irRight < irQRDthresh ) {
        //             irFollow();
        //         }

        //         sweep();
        //     }

        // }

        // else if ( mode == 2 ) {
        //     tapeFollow();
        //     artifact();

        //     if ( checkIR() ) {
        //         while ( aaCount < 3 ) {
        //             irFollow();
        //             artifact();
        //         }

        //         zipline();
        //     }
        // }

    // if ( checkIR() && tapeFinished() ) {
    //     while ( !gotIdol() ) {
    //         irFollow();
    //         idol();
    //     }

    //     zipline();
    // }
}

void aaTest() {
    // ARTIFACT SENSOR SERVO 
        LCD.clear();
        LCD.print("Begin sensorYZ");
        delay(3000);

        while ( !startbutton() ) {
          LCD.clear();

          int servoAnalog = knob(6);
          int angle = 0.176*servoAnalog;

          LCD.print(angle);
          RCServo1.write(angle);
          delay(200);
        }
  
    // ARTIFACT SENSOR
        LCD.clear();
        LCD.print("Begin aaSensor");
        delay(3000);

        while ( !startbutton() ) {
          LCD.clear();

          int objDetected = !digitalRead(aaSensor);
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

        while ( knob(7) < 500 ) {
          int velocity = knob(6);
          LCD.clear();
          LCD.print("STOP = UP; START = DOWN");
          LCD.setCursor(0,1);
          LCD.print("K7>500 TO CONT");

          if ( startbutton() ) {
            stepperControl(100, minStepDelay);
          }
          if ( stopbutton() ) {
            stepperControl(-100, minStepDelay); 
          }
        }

    // ARTIFACT DELAY (While Driving)
        LCD.clear();
        LCD.print("Begin artifact delay");
        delay(2000);

        while ( !startbutton() ) {
          LCD.clear();
          artifactDelay = knob(6);
          LCD.print(artifactDelay);
          delay(500);
        }
    
    LCD.clear();
    LCD.print("Begin Full Arm Code");
    delay(3000);

    while ( !stopbutton() ) {
      artifact();
    }
}

void motorTest() {
    // MOTOR TEST ( HBRIDGE )
        LCD.clear();
        LCD.print("BEGIN MOTOR TEST");
        delay(2000);

        while ( !startbutton() ) {
            LCD.clear();

            int knobVal0 = knob(6);
            int knobVal1 = knob(7);
            int motorSpeed0 = 2 * knobVal0 - 1023 ;
            int motorSpeed1 = 2 * knobVal1 - 1023;

            LCD.print("MOTOR0");
            LCD.print(motorSpeed0);
            LCD.setCursor(0,1);
            LCD.print("MOTOR1");
            LCD.print(motorSpeed1);

            motor.speed(0, motorSpeed0);
            motor.speed(1, motorSpeed1);
            delay(200);
        }

      motor.stop_all();

    // QRD TEST
    LCD.clear();
    LCD.print("BEGIN QRD TEST + THRESHOLD");
    delay(2000);

    while ( !startbutton() ) {
        LCD.clear();
        leftQRD = analogRead(0);
        rightQRD = analogRead(1);
        LCD.print( "L:" ); LCD.print( leftQRD );
        LCD.print( "R:" ); LCD.print( rightQRD );
                  LCD.setCursor(0, 1);
        LCD.print( "T: "); LCD.print( knob(6) );
        threshold = knob(6);
        delay(500);
    }

    // KP AND KD DRIVE VARIABLES
    LCD.clear();
    LCD.print("SET INITIAL KD AND KP");
    delay(2000);

    while ( !startbutton() ) {
        LCD.clear();
        kp = knob(6);
        kd = knob(7);
        LCD.print("KP = "); LCD.print(kp);
              LCD.setCursor(0, 1);
        LCD.print("KD = "); LCD.print(kd);
        delay(500);
    }

    // INITIAL SPEED
    LCD.clear();
    LCD.print("SET SPEED");
    delay(2000);

    while ( !startbutton() ) {
        LCD.clear();
        defaultSpeed = knob(6);
        LCD.print(defaultSpeed);
        delay(500);
    }

    // ARTIFACT DELAY (While Driving)
        LCD.clear();
        LCD.print("SET ARM DELAY");
        delay(2000);

        while ( !startbutton() ) {
          LCD.clear();
          artifactDelay = knob(6);
          LCD.print(artifactDelay);
          delay(500);
        }
}

void irTest() {
    LCD.clear();
    LCD.print("BEGIN IR TEST");
    delay(2000);
  
    while ( !startbutton() ) {
        LCD.clear();
        int irLeft = analogRead(leftIRpin);
        int irRight = analogRead(rightIRpin);
        LCD.print( "L:" ); LCD.print( irLeft );
        LCD.print( "R:" ); LCD.print( irRight );
                  LCD.setCursor(0, 1);
        LCD.print( "L-R: "); LCD.print( knob(6) );
        int dx = knob(6);
        delay(500);
    }

    // KP AND KD DRIVE VARIABLES
    LCD.clear();
    LCD.print("SET INITIAL KD AND KP");
    delay(2000);

    while ( !startbutton() ) {
        LCD.clear();
        irKP = knob(6);
        irKD = knob(7);
        LCD.print("irKP = "); LCD.print(irKP);
              LCD.setCursor(0, 1);
        LCD.print("irKD = "); LCD.print(irKD);
        delay(500);
    }

    // INITIAL SPEED
    LCD.clear();
    LCD.print("SET SPEED");
    delay(2000);

    while ( !startbutton() ) {
        LCD.clear();
        irDefaultSpeed = knob(6);
        LCD.print(irDefaultSpeed);
        delay(500);
    }

    LCD.clear();
    LCD.print("BEGIN IR FOLLOW");
    delay(2000);

    while ( !startbutton() ) {
    // KP KD LOOP FOR FAST ADJUSTING
        if ( stopbutton() ) {
            while ( !startbutton() ) {
                motor.speed(0, 0);
                motor.speed(1, 0);
                LCD.clear();
                irKP = knob(6);
                irKD = knob(7);
                LCD.print("irKP = "); LCD.print(irKP);

                LCD.setCursor(0, 1);
                LCD.print("irKD = "); LCD.print(irKD);
                delay(500);
            }
            LCD.clear();
            LCD.print("STARTING");
            delay(2000);
        }

        irFollow();
        artifact();
    }
}

void competitionSetup() {
    // COMPETITION MODE
        LCD.clear();
        LCD.print("COMPETITION MODE");
        delay(2000);

    // MOTOR TEST ( HBRIDGE )
        LCD.clear();
        LCD.print("BEGIN MOTOR TEST");
        delay(2000);

        while ( !startbutton() ) {
            LCD.clear();

            int knobVal0 = knob(6);
            int knobVal1 = knob(7);
            int motorSpeed0 = 2 * knobVal0 - 1023 ;
            int motorSpeed1 = 2 * knobVal1 - 1023;

            LCD.print("MOTOR0");
            LCD.print(motorSpeed0);
            LCD.setCursor(0,1);
            LCD.print("MOTOR1");
            LCD.print(motorSpeed1);

            motor.speed(0, motorSpeed0);
            motor.speed(1, motorSpeed1);
            delay(200);
        }

      motor.stop_all();

    // QRD TEST
        LCD.clear();
        LCD.print("BEGIN QRD TEST + THRESHOLD");
        delay(2000);

        while ( !startbutton() ) {
            LCD.clear();
            leftQRD = analogRead(0);
            rightQRD = analogRead(1);
            LCD.print( "L:" ); LCD.print( leftQRD );
            LCD.print( "R:" ); LCD.print( rightQRD );
                      LCD.setCursor(0, 1);
            LCD.print( "T: "); LCD.print( knob(6) );
            threshold = knob(6);
            delay(500);
        }

    // ARTIFACT SENSOR
        LCD.clear();
        LCD.print("Begin aaSensor");
        delay(3000);

        while ( !startbutton() ) {
          LCD.clear();

          int objDetected = !digitalRead(aaSensor);
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

    LCD.clear();
    LCD.print("Time to roll!");
    delay(2000);
}

void tapeFollow () {
    // WHAT DOES THE QRD SEE?
        leftQRD = analogRead(leftQRDpin);
        rightQRD = analogRead(rightQRDpin);

    // LETS SEE HOW MUCH WE'LL NEED TO TURN
        if ( (leftQRD > threshold) && (rightQRD > threshold) ) {
            error = 0;     // Both QRDS are on the tape!
        }
        else if ( (leftQRD > threshold) && (rightQRD < threshold) ) {
            error = -1;    // Turn LEFT because right QRD off the track
        }
        else if ( (leftQRD < threshold) && (rightQRD > threshold) ) {
            error = 1;     // Turn RIGHT because left QRD is off the track
        }

        if ( (leftQRD < threshold ) && (rightQRD < threshold) ) {
            if ( prevError > 0 ) {    // If last error was positive, i.e. too left, TURN really RIGHt
                error = 5;
            }
            if ( prevError <= 0 ) {
                error = -5;
            }
        }
        if ( error != prevError ) {
            recentError = prevError;
            lastTime = currentPeriod;
            currentPeriod = 1;
        }

    // LETS CALCULATE HOW FAST WE NEED TO TURN
        // Proportional Error
        p = kp * error;

        // Derivative Error
        d = (int) ( kd * ( (float)( error - recentError ) ) / ( (float)( lastTime + currentPeriod ) ) ) ; // dError/dt
    
        // Total Error
        con = p + d;

    // OKAY TIME TO SET THE SPEED!
        speedLeft = defaultSpeed + (int) con;
        speedRight = -defaultSpeed + (int) con;

        if ( speedLeft > 1000 ) {
            speedLeft = 1000;
        }
        else if ( speedLeft < -1000 ) {
            speedLeft = -1000;
        }

        if ( speedRight > 1000 ) {
            speedRight = 1000;
        }
        else if ( speedRight < -1000 ) {
            speedRight = -1000;
        }

        motor.speed(0, speedLeft);   // Left motor
        motor.speed(1, speedRight);    // Right motor

    // LETS DISPLAY SOME INFORMATION BECAUSE DEBUG
        if ( displayCounter == 200 ) {
            LCD.clear();
            LCD.print( "R:" ); LCD.print( rightQRD );
            LCD.print( "L:" ); LCD.print( leftQRD );
            LCD.print( "KP:" ); LCD.print( kp );
            LCD.print( "KD:" ); LCD.print( kd );
            LCD.print( "p:" ); LCD.print( p );
            LCD.print( "d:" ); LCD.print( d );
            
            LCD.setCursor(0, 1);
            LCD.print("sL: "); LCD.print ( speedLeft );
            LCD.print("sR: "); LCD.print ( speedRight);
            displayCounter = 0;
        }

    // YAY YOU TAPE FOLLOWED A BIT, TRY AGAIN LATER <3
        displayCounter++;
        currentPeriod++;

        prevError = error;
}

int artifactDetected() {
    return digitalRead(aaSensor);
}

void stepperControl ( int steps, int stepDelay ) {
  if (steps < 0) { 
    digitalWrite(aaZdirection, 0); // Set counter-clockwise direction 
  }
  else if ( steps > 0 ) { 
    digitalWrite(aaZdirection, 1); // Set clockwise direction 
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



void artifact ( ) {
      RCServo0.write(armOut);
      RCServo1.write(sensorOut);

      if (  lastDetect == false && artifactDetected() ) {
        lastDetect = true;
        LCD.clear();
        LCD.print("Artifact Detected");
        time1 = millis();
      }
      if ( lastDetect == true && !artifactDetected() ) {
        lastDetect = false;
        LCD.clear();
        LCD.print("Artifact Not Detected");
        time2 = millis();

        if ( abs(time1 - time2) > artifactThreshold ) {
          pickup = true;
        }
        else {
          time1 = 0;
          time2 = 0;
        }
      }
      if ( pickup == true ) {
          LCD.clear();
          LCD.print("ARM TIME");

          if ( aaCount == 0 ) {
            motorBreak();
            defaultSpeed = 450;
          }
          else if ( aaCount == 1 ) {
            delay(artifactDelay);
            defaultSpeed = 400;
          }
          else {
            motorBreak();
          }



          delay(100);
          motor.stop_all();
          delay(200);

          inSensor();

          delay(300);
          armDown();

          delay(300);
          
          armUp();

          armOff();

          delay(100);

          outSensor();
          
          delay(200);

          armOn();

          delay(500);
          LCD.clear();

          time1 = 0;
          time2 = 0; 
          lastDetect = false;
          pickup = false;

          delay(200);

          aaCount++;
          delay(100);
      }

}

void servoControl( int servoNum, int degStart, int degEnd ) {
  if ( degStart > degEnd ) {
    if ( servoNum == 0 ) {
      for ( int i = degStart; i >= degEnd; i-- ) {
        RCServo0.write(i);
        delay(8);
      }
    }
    else if ( servoNum == 1 ) {
      for ( int i = degStart; i >= degEnd; i-- ) {
        RCServo1.write(i);
        delay(8);
      }
    } 
  }

  if ( degStart < degEnd ) {
    if ( servoNum == 0 ) {
      for ( int i = degStart; i <= degEnd; i++ ) {
        RCServo0.write(i);
        delay(8);
      }
    }
    else if ( servoNum == 1 ) {
      for ( int i = degStart; i <= degEnd; i++ ) {
        RCServo1.write(i);
        delay(8);
      }
    } 
  }
}

void outSensor() {
    servoControl(1, sensorIn, sensorOut);
}

void inSensor() {
    servoControl(1, sensorOut, sensorIn);
}

void armOn() {
    RCServo0.write(armOut);
}

void armOff() {
    servoControl(0, armOut, armIn);
}

void armDown() {
    stepperControl(downMax, minStepDelay);
}

void armUp() {
    stepperControl(upMax, minStepDelay);
}

void turn180() {
    RCServo1.write(sensorIn);

    LCD.clear();
    LCD.print("TURN180");
    inSensor();
    motor.speed(0, 500);
    motor.speed(1, 500);
    delay(300);

    reTapeFollow = false;
    leftQRD = analogRead(leftQRDpin);
    rightQRD = analogRead(rightQRDpin);

    while ( reTapeFollow == false  ) {
            leftQRD = analogRead(leftQRDpin);
            rightQRD = analogRead(rightQRDpin);
            LCD.clear();
            LCD.print( "R:" ); LCD.print( rightQRD );
            LCD.print( "L:" ); LCD.print( leftQRD );
            LCD.setCursor(0,1);
            LCD.print(threshold);

        if ( leftQRD > threshold && rightQRD > threshold ) {
            reTapeFollow = true;
        }
    }
}

void turn180rocks() {
    LCD.clear();
    LCD.print("TURN180 ROCKS");
    motor.speed(0, -600);
    motor.speed(0, 600);
    delay(200);

    int reIR = false;

    while ( reIR == false ) {
        if ( checkIR() ) {
            reIR = true;
        }
    }
}

void sweep() {
    LCD.clear();
    LCD.print("SWEEP");

    while ( analogRead(leftQRDpin) < threshold && analogRead(rightQRDpin) < threshold ) {
        motor.speed(0, sweepSpeed);
        motor.speed(1, sweepSpeed);
        delay(500);
        sweepSpeed += 50;
    }
}

void motorBreak() {
    motor.speed(0, -700);
    motor.speed(1, 700);
}

int checkIR () {
    irLeft = analogRead(leftIRpin);
    irRight = analogRead(rightIRpin);
    if ( irLeft > irStartThreshold || irRight > irStartThreshold ) {
        startIR = true;
        return true;
    }
    else {
        return false;
    }
}

    void irFollow () {
        // WHAT DO THE PHOTOTRANSISTORS SEE?
            irLeft = analogRead(leftIRpin);
            irRight = analogRead(rightIRpin);

        // NEED TO CALLIBRATE THE PHOTOTRANSISTORS BECAUSE BIQUAD :(

        // TURN LOGIC TIME!
            // If Right > Left, error is positive, turn right, if Left > Right, error is negative turn left
            int irError = irRight - irLeft;

            if ( irError != irPrevError ) {
                    irRecentError = irPrevError;
                    irLastTime = irCurrentPeriod;
                    irCurrentPeriod = 1;
            }

        // LETS CALCULATE HOW FAST WE NEED TO TURN!
            // Proportional Error
            irP = irKP * irError;

            // Derivative Error
            irD = (int) ( irKD * ( (float)( irError - irRecentError ) ) / ( (float)( irLastTime + irCurrentPeriod ) ) ) ; // dError/dt
            
            // Total Error
            irCon = irP + irD;
       
        // OKAY NOW LETS ACTUALLY TELL OUR MOTORS TO RUN FAST (or slow)
            int irSpeedLeft = irDefaultSpeed + (int) irCon;
            int irSpeedRight = -irDefaultSpeed + (int) irCon;

            if ( irSpeedLeft > 1000 ) {
                irSpeedLeft = 1000;
            }
            else if ( irSpeedLeft < -1000 ) {
                irSpeedLeft = -1000;
            }

            if ( irSpeedRight > 1000 ) {
                irSpeedRight = 1000;
            }
            else if ( irSpeedRight < -1000 ) {
                irSpeedRight = -1000;
            }

            motor.speed(0, irSpeedLeft);   // Left motor
            motor.speed(1, irSpeedRight);    // Right motor

        // LETS DISPLAY SOME INFORMATION BECAUSE YEAH
            if ( irDisplayCounter == 500 ) {
                LCD.clear();
                LCD.print( "R:" ); LCD.print( irLeft );
                LCD.print( "L:" ); LCD.print( irRight );
                LCD.print( "KP:" ); LCD.print( irKP );
                LCD.print( "KD:" ); LCD.print( irKD );
                LCD.print( "p:" ); LCD.print( irP );
                LCD.print( "d:" ); LCD.print( irD );
               
                LCD.setCursor(0, 1);
                LCD.print("sL: "); LCD.print ( irSpeedLeft );
                LCD.print("sR: "); LCD.print ( irSpeedRight);
                irDisplayCounter = 0;
            }

        // YAY YOU FOLLOWED IR FOR A BIT, TRY AGAIN NEXT LOOP!
            irDisplayCounter++;
            irCurrentPeriod++;

            irPrevError = irError;
    }

    void zipline() {
        LCD.clear();
        LCD.print("ZIPLINE");
        delay(2000);
    }

    void functionTest() {
        LCD.clear();
        LCD.print("FUNCTION TEST");
        delay(2000);

        while ( !startbutton() ) {
            int knobChoiceFunction = knob(6);
            function = knobChoiceFunction/105;

            if ( function == 0 ) {
                LCD.clear();
                LCD.print("tapeFollow");
            }
            else if ( function == 1 ) {
                LCD.clear();
                LCD.print("artifact");
            }
            else if ( function == 2 ) {
                LCD.clear();
                LCD.print("artifactDetected");
            }
            else if ( function == 3) {
                LCD.clear();
                LCD.print("turn180");
            }
            else if ( function == 4 ) {
                LCD.clear();
                LCD.print("turn180rocks");
            }
            else if ( function == 5 ) {
                LCD.clear();
                LCD.print("sweep");
            }
            else if ( function ==6 ) {
                LCD.clear();
                LCD.print("irFollow");
            }
            else if ( function == 7 ) {
                LCD.clear();
                LCD.print("zipline");
            }

            delay(300);
        }

        LCD.clear();
        LCD.print("PRESS START TO RUN");
        delay(2000);

        if ( function == 0 ) {
            while ( !stopbutton() ) {
                tapeFollow();
            }
        }   
        else if ( function == 1 ) {
            while ( !stopbutton() ) {
                artifact();
            }
        }
        else if ( function == 2 ) {
            while ( !stopbutton() ) {
              LCD.clear();

              int objDetected = !digitalRead(aaSensor);
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
        }

        else if ( function == 3 ) {
            while ( !stopbutton() ) {
                turn180();

                if ( reTapeFollow ) {
                    while ( reTapeFollow ) {
                        tapeFollow();
                    }
                }
            }
        }
        else if ( function == 4 ) {
            while ( !stopbutton() ) {
                turn180rocks();

                motor.stop_all();
                while ( !startbutton() ) {
                    LCD.clear();
                    LCD.print("FOUNDIR");
                    delay(200);
                }
            }
        }
        else if ( function == 5 ) {
            while ( !stopbutton() ) {
                sweep();
                tapeFollow();
            }
        }
        else if ( function == 6 ) {
            while ( !stopbutton() ) {
                irFollow();
            }
        }
    }