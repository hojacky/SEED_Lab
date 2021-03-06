 //SEED_LAB group 14 Arduino motor reading and spinning
 //Name the pins for clarification

  #include <Encoder.h>
  #include <Wire.h>

  #define PI 3.1415926535897932384626433832795
//  const double r = .05;
  //double distance = 7509;//3ft
  //double distance = 13290;//5ft
  double distance = 19606;//7ft
  double angularDistance = 0;//degrees to turn
  const double WheelRadius = 0.245;// radius of wheel need to measure also units?
  const double axleRadius = 0.323;//need to measure
  const double d = .1;
  const double N = 6.28;
  const double halfPi = 1.35;
  const double Pi = 3.14;
  const double threeHalfPi = 4.67;
  double dot_theta1 = 0;
  double vel1 = 0;
  double mvolt = 0;
  volatile int aVal = 0;
  volatile int bVal = 0;
  volatile double time1;
  volatile double totaltime1 = 0;
  volatile double oldtime1 = 0;
  volatile double t1 = 0;
  int D = 0;
  double xnew = 0;
  double xold = 0;
  double ynew = 0;
  double yold = 0;
  double step_theta = 0;
  double leftControllerOutput;
  double rightControllerOutput;
  double leftAngularOutput;
  double rightAngularOutput;
  //PID values
  double Kp = 5.5;    //5.5; //V/rad
  double Kd = 0.5;    //0.5; //V/mil/rad
  double Ki = 0.0001; //0.0001; //V/rad*mil
  int I = 0;
  int e_past = 0;
  int Ts = 50;
  int Tc = 0;
  int poop = 0;
  int lastPos = 100;
  int quadrant = 1;
  double lefterror = 0;
  double righterror = 0;
  double leftang = 0;
  double rightang = 0;

  //Motor Driver Variables 
  int newPos = 0;
  int phiold = 0;
  int expected_pos;


  
  //set up encoder pins
  Encoder motorEncoder(2, 5);
  Encoder motorEncoder2(3, 6);
  
#define SLAVE_ADDRESS 0x04  
  
void setup() {
  Serial.begin(9600);
//  Serial.println("Wheel positions");
  
  //Open i2C
  //Wire.begin(SLAVE_ADDRESS);
  
//  Wire.onReceive(receiveData);
//  Wire.onRequest(sendData);

  distance = (distance/(2*PI*WheelRadius));
  distance = (distance/1600)*PI;

//  angularDistance = (angularDistance/(4*PI*axleRadius));
  angularDistance = (angularDistance*PI)/180;   
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

}

// void receiveData(int){
//
//  expected_pos = Wire.read();
//  // compare to calculated value and computer %Error 
//  // Send back %Error and display via LCD
// }
 



//void sendData(){
////  Serial.print(quadrant);
//  Wire.write(quadrant);
//
//}




void LeftWheel(){
//   if(expected_pos  == 1) {
//    quadrant = halfPi;
//  }else if(expected_pos == 2){
//    quadrant = Pi;  
//  }else if(expected_pos == 3){
//    quadrant = threeHalfPi;
//  }else if(expected_pos == 4){
//     quadrant = N;
//  }
 Tc = millis();
 double r = distance; //quadrant;
 Serial.println("Desired Distance left");
 Serial.println(r);
 double y = motorEncoder.read();
 Serial.println("Current Position left");
 //Serial.println(y);
 y = (y/1600)*PI;
 Serial.println(y);
 lefterror = r + y;
 if(Tc > 0){
  D = (lefterror - e_past)/Ts; //rad/mil
  e_past = lefterror;
 }
 else{
  D = 0;
 }
 Serial.println("Error Left");
 Serial.println(lefterror);
 if (lefterror < 0){
  leftControllerOutput = 0;
 }else{
  I = I + (Ts*lefterror); //rad*mil
  leftControllerOutput = (Kp*lefterror) + (Ki*I) + (Kd*D);
 }
 if(leftControllerOutput > 0){
  digitalWrite(7, LOW);
 }else{
  digitalWrite(7, HIGH);
 }
 leftControllerOutput = abs(leftControllerOutput);
 leftControllerOutput = (leftControllerOutput/8.5)*255;
 if(leftControllerOutput > 255){
  leftControllerOutput = 255;
 }
 analogWrite(9,leftControllerOutput);
// Serial.println("PWM voltage and Error left");
// Serial.println(leftControllerOutput);
 Ts = Ts - Tc;
 delay(50);

 
  //if a character is detected on the serial monitor, reset wheel to zero
   if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    motorEncoder.write(0);
  }
 }




 void RightWheel(){
//   if(expected_pos  == 1) {
//    quadrant = halfPi;
//  }else if(expected_pos == 2){
//    quadrant = Pi;  
//  }else if(expected_pos == 3){
//    quadrant = threeHalfPi;
//  }else if(expected_pos == 4){
//     quadrant = N;
//  }
 Tc = millis();
 double r = distance; //quadrant;
 Serial.println("Desired Distance right");
 Serial.println(r);
 double y = motorEncoder2.read();
 Serial.println("Current Position right");
 //Serial.println(y);
 y = (y/1600)*PI;
 Serial.println(y);
 righterror = r - y;
 Serial.println("Desired error right");
 Serial.println(righterror);
 if(Tc > 0){
  D = (righterror - e_past)/Ts; //rad/mil
  e_past = righterror;
 } else{
  D = 0;
 }
 if (righterror < 0){
  rightControllerOutput = 0;
 }else{
   I = I + (Ts*righterror); //rad*mil
   rightControllerOutput = (Kp*righterror) + (Ki*I) + (Kd*D);
 }
// Serial.println("Error Right");
// Serial.println(righterror);

 if(angularDistance < 0){
  digitalWrite(8, LOW);
 }else{
  digitalWrite(8, HIGH);
 }
 rightControllerOutput = abs(rightControllerOutput);
 rightControllerOutput = (rightControllerOutput/8.5)*255;
 if(rightControllerOutput > 255){
  rightControllerOutput = 255;
 }
 analogWrite(10,rightControllerOutput);
//  Serial.println("PWM voltage and Error right");
// Serial.println(rightControllerOutput);

 Ts = Ts - Tc;
 delay(50);

 
  //if a character is detected on the serial monitor, reset wheel to zero
   if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    motorEncoder2.write(0);
   }
 }





 void leftAngular(){
 Tc = millis();
 double x = angularDistance; //quadrant;

 double z = motorEncoder.read();
 z = (z/1600)*PI;

 leftang = x - z;
 if(Tc > 0){
  D = (leftang - e_past)/Ts; //rad/mil
  e_past = leftang;
 }
 else{
  D = 0;
 }
 Serial.println("Left Ang Err");
 Serial.println(leftang);
 I = I + (Ts*leftang); //rad*mil
 leftAngularOutput = (Kp*leftang) + (Ki*I) + (Kd*D);
 if(angularDistance < 0){
  digitalWrite(7, LOW);
 }else{
  digitalWrite(7, HIGH);
 }
 if(leftang < 0){
  analogWrite(9, LOW);
 }
 leftAngularOutput = abs(leftAngularOutput);
 leftAngularOutput = (leftAngularOutput/8.5)*255;
 if(leftAngularOutput > 255){
  leftAngularOutput = 255;
 }
 analogWrite(9, leftAngularOutput);
 Ts = Ts - Tc;
 delay(50);
 }


 void rightAngular(){
 Tc = millis();

 double x = angularDistance; //quadrant;
 double z = motorEncoder2.read();
 z = (z/1600)*PI;
 rightang = x + z;
 if(Tc > 0){
  D = (rightang - e_past)/Ts; //rad/mil
  e_past = rightang;
 }
 else{
  D = 0;
 }
 Serial.println("Right Ang Err");
 Serial.println(rightang);
 I = I + (Ts*rightang); //rad*mil
 if (rightang < 0){
  analogWrite(10, LOW);
 }

 rightAngularOutput = (Kp*rightang) + (Ki*I) + (Kd*D);
 if(rightAngularOutput < 0){
  digitalWrite(8, LOW);
 }else{
  digitalWrite(8, HIGH);
 }
 rightAngularOutput = abs(rightAngularOutput);
 rightAngularOutput = (rightAngularOutput/8.5)*255;
 if(rightAngularOutput > 255){
  rightAngularOutput = 255;
 }
 analogWrite(10, rightAngularOutput);

 Ts = Ts - Tc;
 delay(50);
 }


 void loop() { 
// if(angularDistance != 0){
//    leftAngular(); 
//    rightAngular();
// }
 LeftWheel();
 RightWheel();


}
