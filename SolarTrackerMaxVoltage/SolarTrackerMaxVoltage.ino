/*
  Arduino solar tracker
  www.sciencebuddies.org
*/

// include servo library
#include <Servo.h>

// declare variables for pins
const int light1Pin = A0;
const int light2Pin = A1;
const int solarPin = A2;
const int servoPin = 9;

// variable for servo angle
int angle = 90;

// variables for light sensor readings and difference between them
int light1Reading;
int light2Reading;
int difference;

// margin for difference to move motor
int margin = 20;

// variables for measuring solar panel
int solarReading;
float solarVoltage;

// create servo object and give it a name
Servo servo1;

void setup() {   // put your setup code here, to run once:
  servo1.attach(servoPin);  // attach servo to pin
  servo1.write(angle);      // set initial servo angle
  Serial.begin(9600);       // initialize serial communication
}

void loop() {  // put your main code here, to run repeatedly:
  // get light sensor values
  light1Reading = analogRead(light1Pin);
  light2Reading = analogRead(light2Pin);
  // calculate difference
  difference = light2Reading - light1Reading;

  // move the servo depending on the difference
  if(difference > margin){  // increase angle if difference is greater than positive margin
    if(angle<180){          // only increase the angle if it is already less than 180 degrees
      angle = angle + 1;
    }
  }
  else if(difference < (-1*margin)){  // decrease angle if difference is less than negative margin
    if(angle>0){                      // only decrease the angle if it is already greater than than 0 degrees
      angle = angle - 1;
    }
  }
  else{  // if difference is within +/- margin, do nothing  
         // this prevents the servo from constantly bouncing back and forth due to small fluctuations
  }

  // move to new servo position
  servo1.write(angle);

  // read solar panel with analog pin
  solarReading = analogRead(solarPin);
  // convert the ADC reading (0-1023) to a voltage (0-5.0)
  solarVoltage = solarReading * 5.0 / 1023 ;

  // print sensor readings - use for calibration
  Serial.print("Sensor 1 reading: ");
  Serial.print(light1Reading);
  Serial.print(" Sensor 2 reading: ");
  Serial.print(light2Reading);
  Serial.print(" Solar panel voltage: ");
  Serial.println(solarVoltage);

  // short delay before next reading
  delay(100);
}
