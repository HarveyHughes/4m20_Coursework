
#include <Servo.h>

Servo motor1; 
Servo motor2; 
Servo motor3; 

int p1 = 0;    // variable to store the servo position
int p2 = 0; 
int p3 = 0; 
int d1 = 30;
int d2 = 30;
int d3 = 150;
int s1=0;
int s2=0;
int s3=0;
int n=100;
int i=0;

void setup() {
  // put your setup code here, to run once:
  motor1.attach(11);

}

void loop() {
  // put your main code here, to run repeatedly:

    motor1.write(100);
    delay(25);                       // waits 15ms for the servo to reach the position
  
 
}
