
  //SEED_LAB group 14 Arduino motor reading and spinning
 //Name the pins for clarification

  #include <Encoder.h>
  #include <Wire.h>

  #define PI 3.1415926535897932384626433832795
//  const double r = .05;
  double distance = 0;
  double angularDistance = 0;
  const double WheelRadius = 0.5;// radius of wheel need to measure also units?
  const double axleRadius = 0.5;//need to measure
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
  int leftControllerOutput;
  int rightControllerOutput;
  int leftAngularOutput;
  int rightAngularOutput;
  //PID values
  double Kp = 3.5; //V/rad
  double Kd = 0;//1; //V/mil/rad
  double Ki = 0; //0.1; //V/rad*mil
  int I = 0;
  int e_past = 0;
  int Ts = 50;
  int Tc = 0;
  int poop = 0;
  int lastPos = 100;
  int quadrant = 1;

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
  Wire.begin(SLAVE_ADDRESS);
  
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);


  
  //set pins 4,7,8,9,10 as output
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  //set the enable pins high
  digitalWrite(4, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  analogWrite(9, LOW);
  digitalWrite(10, HIGH);
  //set pin 12 to input
  pinMode(12, INPUT);

}

 void receiveData(int){

  expected_pos = Wire.read();
  // compare to calculated value and computer %Error 
  // Send back %Error and display via LCD
 }
 
void loop() { 
 if(angularDistance != 0){
    leftAngular(); 
    rightAngular();
 }
 LeftWheel();
 RightWheel();


}


void sendData(){
//  Serial.print(quadrant);
  Wire.write(quadrant);

}



















void LeftWheel()[
   distance = (distance/(2*PI*WheelRadius));
   distance = (distance*1600)/PI;
   if(expected_pos  == 1) {
    quadrant = halfPi;
  }else if(expected_pos == 2){
    quadrant = Pi;  
  }else if(expected_pos == 3){
    quadrant = threeHalfPi;
  }else if(expected_pos == 4){
     quadrant = N;
  }
 Tc = millis();
// analogWrite(9,LOW);
 double r = distance; //quadrant;
 Serial.print("Desired");
 Serial.print(r);
 //r = (r/1600)*PI;
 double y = motorEncoder.read();
 y = (y/1600)*PI;
 r = r + y;
 double error = r - y;
 if(Tc > 0){
  D = (error - e_past)/Tc; //rad/mil
  e_past = error;
 }
 else{
  D = 0;
 }
 Serial.println("ERROR");
 Serial.print(error);
 I = I + (Ts*error); //rad*mil
 Serial.print("Integral");
 Serial.print(I);
 leftControllerOutput = (Kp*error) + (Ki*I) + (Kd*D);
 if(leftControllerOutput < 0){
  digitalWrite(7, LOW);
 }else{
  digitalWrite(7, HIGH);
 }
 leftControllerOutput = abs(leftControllerOutput);
 leftControllerOutput = (leftControllerOutput/7.5)*255;
 if(leftControllerOutput > 255){
  leftControllerOutput = 255;
 }
 analogWrite(9,leftControllerOutput);
 Serial.println("PWM voltage and Error");
 Serial.println(leftControllerOutput);
// Serial.println(e_past);
 Ts = Ts - Tc;
 delay(50);
// Serial.print("Current time");
// Serial.println(Tc); 
 
  //if a character is detected on the serial monitor, reset wheel to zero
   if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    motorEncoder.write(0);
  }
 }


















 void RightWheel(){
     distance = (distance/(2*PI*WheelRadius));
   distance = (distance*1600)/PI;
   if(expected_pos  == 1) {
    quadrant = halfPi;
  }else if(expected_pos == 2){
    quadrant = Pi;  
  }else if(expected_pos == 3){
    quadrant = threeHalfPi;
  }else if(expected_pos == 4){
     quadrant = N;
  }
 Tc = millis();
// analogWrite(9,LOW);
 double r = distance; //quadrant;
 Serial.print("Desired");
 Serial.print(r);
 //r = (r/1600)*PI;
 double y = motorEncoder2.read();
 y = (y/1600)*PI;
 r = r + y;
 double error = r - y;
 if(Tc > 0){
  D = (error - e_past)/Tc; //rad/mil
  e_past = error;
 }
 else{
  D = 0;
 }
 Serial.println("ERROR");
 Serial.print(error);
 I = I + (Ts*error); //rad*mil
 Serial.print("Integral");
 Serial.print(I);
 rightControllerOutput = (Kp*error) + (Ki*I) + (Kd*D);
 if(rightControllerOutput < 0){
  digitalWrite(7, LOW);
 }else{
  digitalWrite(7, HIGH);
 }
 rightControllerOutput = abs(rightControllerOutput);
 rightControllerOutput = (rightControllerOutput/7.5)*255;
 if(rightControllerOutput > 255){
  rightControllerOutput = 255;
 }
 analogWrite(9,rightControllerOutput);
 Serial.println("PWM voltage and Error");
 Serial.println(rightControllerOutput );
// Serial.println(e_past);
 Ts = Ts - Tc;
 delay(50);
// Serial.print("Current time");
// Serial.println(Tc); 
 
  //if a character is detected on the serial monitor, reset wheel to zero
   if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    motorEncoder.write(0);
   }
 }

















 void leftAngular(){
 angularDistance = (angularDistance/(4*PI*axleRadius));
 angularDistance = (angularDistance*1600)/PI;

 
 
 Tc = millis();
// analogWrite(9,LOW);
 double x = angularDistance; //quadrant;
 Serial.print("Desired");
 Serial.print(x);
 //x = (x/1600)*PI;
 double z = motorEncoder.read();
 z = (z/1600)*PI;
 x = x + z;
 double error = x - z;
 if(Tc > 0){
  D = (error - e_past)/Tc; //rad/mil
  e_past = error;
 }
 else{
  D = 0;
 }
 Serial.println("ERROR");
 Serial.print(error);
 I = I + (Ts*error); //rad*mil
 Serial.print("Integral");
 Serial.print(I);
 leftAngularOutput = (Kp*error) + (Ki*I) + (Kd*D);
 if(leftAngularOutput < 0){
  digitalWrite(7, LOW);
 }else{
  digitalWrite(7, HIGH);
 }
 leftAngularOutput = abs(leftAngularOutput);
 leftAngularOutput = (leftAngularOutput/7.5)*255;
 if(leftAngularOutput > 255){
  leftAngularOutput = 255;
 }
 Serial.println("PWM voltage and Error");
 Serial.println(leftAngularOutput);
// Serial.println(e_past);
 Ts = Ts - Tc;
 delay(50);
 }


 void rightAngular(){
 angularDistance = (angularDistance/(4*PI*axleRadius));
 angularDistance = (angularDistance*1600)/PI;

 
 
 Tc = millis();
// analogWrite(9,LOW);
 double x = angularDistance; //quadrant;
 Serial.print("Desired");
 Serial.print(x);
 //x = (x/1600)*PI;
 double z = motorEncoder2.read();
 z = (z/1600)*PI;
 x = x + z;
 double error = x - z;
 if(Tc > 0){
  D = (error - e_past)/Tc; //rad/mil
  e_past = error;
 }
 else{
  D = 0;
 }
 Serial.println("ERROR");
 Serial.print(error);
 I = I + (Ts*error); //rad*mil
 Serial.print("Integral");
 Serial.print(I);
 rightAngularOutput = (Kp*error) + (Ki*I) + (Kd*D);
 if(rightAngularOutput > 0){
  digitalWrite(7, LOW);
 }else{
  digitalWrite(7, HIGH);
 }
 rightAngularOutput = abs(rightAngularOutput);
 rightAngularOutput = (rightAngularOutput/7.5)*255;
 if(rightAngularOutput > 255){
  rightAngularOutput = 255;
 }
 Serial.println("PWM voltage and Error");
 Serial.println(rightAngularOutput);
// Serial.println(e_past);
 Ts = Ts - Tc;
 delay(50);
 }
 
