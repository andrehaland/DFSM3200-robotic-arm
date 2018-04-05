#include <Servo.h>

/*
*
*	This is our test of the servo-motors. We did it in three steps:
*		1:	Manually writing values (degrees) to the servo i.e servo.write(90)
*		2:	Provided values for the servo with a potentiometer
*		3:	Provided values for the servo by using joystick.
*			a:	Were able to control a motor for the X-axis and a motor for the Y-axis
*
*	Currently the datastream from the Arduino to the robot seem to have a lot of interference,
*	which prevents the robot from moving smoothly. In our first experiments we used very long cables
*	that may have caused the intereference.
*
*/

int xVal;
int yVal;
Servo servo;

void setup() {
	Serial.begin(9600);
	pinMode(A0, INPUT);
	servo.attach(7);
	pinMode(A2, INPUT);
}

void loop() {
	delay(15);
	xVal = analogRead(A0) * 5; // Attempt to calibrate the joystick in regards to the analog input
	xVal = map(xVal, 4, 999 * 5, 0, 180);
	servo.write(xVal);
	yVal = analogRead(A2);

	Serial.print("( ");
	Serial.print(xVal);
	Serial.println(" )");
}
