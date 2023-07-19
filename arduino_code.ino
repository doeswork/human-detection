#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo2; // create second servo object to control a servo

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(10);  // attaches the second servo on pin 10 to the servo object
  Serial.begin(9600);  // start serial communication at 9600bps
}

void loop() {
  if (Serial.available()) {  // if data is available to read
    int angle = Serial.parseInt();  // read it and store it in 'angle'
    
    // Make sure the angle is within the servo's range
    if (angle >= 0 && angle <= 180) {
      myservo.write(angle);  // sets the servo position according to the scaled value
    }
    
    if (Serial.available()) { // if more data is available to read
      int angle2 = Serial.parseInt(); // read it and store it in 'angle2'
      
      // Make sure the angle is within the servo's range
      if (angle2 >= 0 && angle2 <= 180) {
        myservo2.write(angle2);  // sets the second servo position according to the scaled value
      }
    }
  }
}
