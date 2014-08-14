#include <phys253.h>
#include <LiquidCrystal.h>
#include <servo253.h> 

int dx = 20; int defaultSpeed = 450;

int irKP = 2; int irKD = 2; double irD; double irP; double irCon;
int irCurrentPeriod = 0; int irDisplayCounter = 0;

int irLeft; int irRight;
int irError; int irPrevError = 0; int irRecentError; int irLastTime;

int lowIRthresh = 15; int highIRthresh = 40;

int leftQRDpin = 0;
int rightQRDpin = 1;

int speedLeft; int speedRight;

void setup()
{
  portMode(0, INPUT) ;
  portMode(1, OUTPUT) ;

  LCD.clear();
  LCD.print("BEGIN MOTOR TEST");
  delay(2000);

  while ( !startbutton() ) {
    LCD.clear();
    
    int knobVal0 = knob(6);
    int knobVal1 = knob(7);
    int motorSpeed0 = 2 * knobVal0 - 1023 ;
    int motorSpeed1 = 2 * knobVal1 - 1023;
    
    LCD.print("MOTOR0: ");
    LCD.print(motorSpeed0);
    LCD.setCursor(0,1);
    LCD.print("MOTOR1: ");
    LCD.print(motorSpeed1);
    
    motor.speed(0, motorSpeed0);
    motor.speed(1, motorSpeed1);
    delay(200);
  }

  LCD.clear();
  LCD.print("BEGIN IR TEST");
  delay(2000);
  
  while ( !startbutton() ) {
    LCD.clear();
    irLeft = analogRead(2);
    irRight = analogRead(3);
    LCD.print( "R:" ); LCD.print( irLeft );
    LCD.print( "L:" ); LCD.print( irRight );
              LCD.setCursor(0, 1);
    LCD.print( "L-R: "); LCD.print( knob(6) );
    dx = knob(6);
    delay(500);
  }

  LCD.clear();
  LCD.print("SET L/H IR THRESH");
  delay(2000);

  while ( !startbutton() ) {
    LCD.clear();

    irLeft = analogRead(2);
    irRight = analogRead(3);
    LCD.print( "IRL:" ); LCD.print( irLeft );
    LCD.print( "IRR:" ); LCD.print( irRight );
              LCD.setCursor(0, 1);

    lowIRthresh = knob(6);
    highIRthresh = knob(7);
    LCD.print( "L:" ); LCD.print( lowIRthresh );
    LCD.print( " H:" ); LCD.print( highIRthresh );
    delay(500);
  }

  LCD.clear();
  LCD.print("SET INITIAL KD AND KP");
  delay(2000);

  while ( !startbutton () ) {
    LCD.clear();
    irKP = knob(6);
    irKD = knob(7);
    LCD.print("KP = "); LCD.print(irKP);
          LCD.setCursor(0, 1);
    LCD.print("KD = "); LCD.print(irKD);
    delay(500);
  }
  
  LCD.clear();
  LCD.print("Time to roll!");
  delay(2000);
}

void loop() {
  if ( stopbutton() ) {
    while ( !startbutton() ) {
      motor.stop_all();
      LCD.clear();
      irKP = knob(6);
      irKD = knob(7);
      LCD.print("KP = "); LCD.print(irKP);
      
      LCD.setCursor(0, 1);
      LCD.print("KD = "); LCD.print(irKD);
      delay(500);
      }
      LCD.clear();
      LCD.print("STARTING");
      delay(2000);
  }

  irFollow();
}


    void irFollow () {
        // WHAT DO THE PHOTOTRANSISTORS SEE?
            irLeft = analogRead(2);
            irRight = analogRead(3);

        // NEED TO CALLIBRATE THE PHOTOTRANSISTORS BECAUSE BIQUAD :(
            irLeft = irLeft - dx;

        // TURN LOGIC TIME!
            // If Left and Right are approximately the same, no error!
            if ( abs( irLeft - irRight) <= lowIRthresh ) {
                irError = 0;
            }
            // If Left is a bit higher than right, turn left!
            if ( irLeft - irRight >= lowIRthresh && irLeft - irRight <= highIRthresh ) {
                irError = -1;
            }
            // If Right is a bit higher than left, turn right!
            if ( irRight - irLeft >= lowIRthresh && irRight - irLeft <= highIRthresh) {
                irError = 1;
            }
            // If left is A LOT higher than right, turn really left!
            if ( irLeft - irRight >= highIRthresh ) {
                irError = -5;
            }
            // If right is A LOT higher than left, turn really right!
            if ( irRight - irLeft >= highIRthresh ) {
                irError = 5;
            }
            
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
            speedLeft = defaultSpeed + (int) irCon;
            speedRight = -defaultSpeed + (int) irCon;

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
                LCD.print("sL: "); LCD.print ( speedLeft );
                LCD.print("sR: "); LCD.print ( speedRight);
                irDisplayCounter = 0;
            }

        // YAY YOU FOLLOWED IR FOR A BIT, TRY AGAIN NEXT LOOP!
            irDisplayCounter++;
            irCurrentPeriod++;

            irPrevError = irError;
    }
