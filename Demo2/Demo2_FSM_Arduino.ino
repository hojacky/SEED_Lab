//SEED_LAB group 14 Arduino motor reading and spinning
 //Name the pins for clarification

#include <Encoder.h>
#include <Wire.h>
#include <string>
#include <iostream>

#define PI 3.1415926535897932384626433832795
//  const double r = .05;
double distance = 2658;//((2658/(2*PI*0.245))/1600)*PI;//3ft
//double distance = 13290;//5ft
//double distance = 19606;//7ft
//double angularDistance = 150;////150 for 90 deg degrees to turn
double angularDistance = 255;//135 deg
//double angularDistance = 353;// 180 deg
const double WheelRadius = 0.245;// radius of wheel(meters)
double D_L = 0;
double D_R= 0;
double leftControllerOutput;
double rightControllerOutput;
double leftAngularOutput;
double rightAngularOutput;
//PID values
double Kp = 3.5;    //5.5; //V/rad
double Kd = 0.5;    //0.5; //V/mil/rad
double Ki = 0.0001; //0.0001; //V/rad*mil
double I_L = 0;
double I_R;
double e_Lpast;
double e_Rpast;
int Ts = 50;
double Tc = 0;
double lefterror = 0;
double righterror = 0;
double strErr_L = 0;
double strErr_R = 0;
double r_L = 0; //quadrant;
double r_R = 0; //quadrant;
double y_L = 0;
double y_R = 0;
double Tc2 = 0;
  
//Motor Driver Variables 
int newPos = 0;
int phiold = 0;
int expected_pos;
bool turn = 1;
bool straight = 0;
double voltageR;
double voltageL;

//set up encoder pins
Encoder motorEncoder(2, 5);
Encoder motorEncoder2(3, 6);
  
#define SLAVE_ADDRESS 0x04  

enum FSM {
	turn, straight
};
  
void setup() {
	Serial.begin(9600);
	//Serial.println("Wheel positions");
  
  	//Open i2C
  	//Wire.begin(SLAVE_ADDRESS);
  
	//  Wire.onReceive(receiveData);
	//  Wire.onRequest(sendData);
  

 	distance = (distance/(2*PI*WheelRadius));
	distance = (distance/1600)*PI;

	//angularDistance = (angularDistance/(4*PI*axleRadius));
 	angularDistance = (angularDistance*PI)/180;   //Convert to radians
  	//set pins 4,7,8,9,10 as output
  	pinMode(4, OUTPUT);
  	pinMode(7, OUTPUT);
  	pinMode(8, OUTPUT);
  	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
	pinMode(6,OUTPUT);
	digitalWrite(6, HIGH);
	//set the enable pins high
	digitalWrite(4, HIGH);
	digitalWrite(7, HIGH);
	digitalWrite(8, HIGH);
	analogWrite(9, LOW);
  	analogWrite(10, LOW);
  	//set pin 12 to input
  	pinMode(12, INPUT);

 	r_L = angularDistance;
  	r_R = angularDistance;
  	y_L = motorEncoder.read();
  	y_R = motorEncoder2.read();

  	attachInterrupt(digitalPinToInterrupt(2),LeftAngle,CHANGE);
  	attachInterrupt(digitalPinToInterrupt(3),RightAngle,CHANGE);

}
