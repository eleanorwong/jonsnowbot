#include <phys253.h>
#include <LiquidCrystal.h>
#include <servo253.h> 

int threshold = 250; int defaultSpeed = 450;

int kp = 2; int kd = 2; double d; double p; double con;
int currentPeriod = 0; int displayCounter = 0;

int leftQRD; int rightQRD;
int error; int prevError = 0; int recentError; int lastTime;

int leftQRDpin = 0;
int rightQRDpin = 1;

int speedLeft; int speedRight;

void setup()
{
  portMode(0, INPUT) ;
  portMode(1, OUTPUT) ;
  
  while ( !startbutton() ) {
    LCD.clear();
    leftQRD = analogRead(0);
    rightQRD = analogRead(1);
    LCD.print( "R:" ); LCD.print( rightQRD );
    LCD.print( "L:" ); LCD.print( leftQRD );
              LCD.setCursor(0, 1);
    LCD.print( "T: "); LCD.print( knob(6) );
    threshold = knob(6);
    delay(500);
  }

  while ( !startbutton () ) {
  	LCD.clear();
  	kp = knob(6);
  	kd = knob(7);
  	LCD.print("KP = "); LCD.print(kp);
          LCD.setCursor(0, 1);
  	LCD.print("KD = "); LCD.print(kd);
  	delay(500);
  }
  
  LCD.clear();
  LCD.print("Time to roll!");
  delay(2000);
}

void loop() {
  if ( stopbutton() ) {
    while ( !startbutton() ) {
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

            if ( speedLeft > 700 ) {
                speedLeft = 700;
            }
            else if ( speedLeft < -700 ) {
                speedLeft = -700;
            }

            if ( speedRight > 700 ) {
                speedRight = 700;
            }
            else if ( speedRight < -700 ) {
                speedRight = -700;
            }

            motor.speed(0, speedLeft);   // Left motor
            motor.speed(1, speedRight);    // Right motor

        // LETS DISPLAY SOME INFORMATION BECAUSE DEBUG
            if ( displayCounter == 500 ) {
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

