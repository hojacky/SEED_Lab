 


  
  //SEED_LAB group 14 Arduino motor reading and spinning
 //Name the pins for clarification
 
 
 
  #include <Encoder.h>
  #include <Wire.h>

  
  #define PI 3.1415926535897932384626433832795
  const double r = .05;
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
  int realControllerOutput;
  //PID values
  double Kp = 2.5;
  double Kd = 1;
  double Ki = 0.1;
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
  Serial.println("Expected Position");
  Serial.println(expected_pos);

  if(expected_pos  == 1) {
    quadrant = halfPi;
  }else if(expected_pos == 2){
    quadrant = Pi;  
  }else if(expected_pos == 3){
    quadrant = threeHalfPi;
  }else if(expected_pos == 4){
     quadrant = N;
  }
  
  // compare to calculated value and computer %Error 
  // Send back %Error and display via LCD

  
 }
 
void loop() { 
 Tc = millis();
 analogWrite(9,LOW);
 int r = quadrant;
 Serial.print("Desired");
 Serial.print(r);
 //r = (r/1600)*PI;
 int y = motorEncoder.read();
 y = (y/1600)*PI;
 int error = r - y;
 if(Tc > 0){
  D = (error - e_past)/Tc;
  e_past = error;
 }
 else{
  D = 0;
 }
 I = I + (Ts*error);
 Serial.print("Integral");
 Serial.print(I);
 realControllerOutput = (Kp*error) + (Ki*I) + (Kd*D);
 if(realControllerOutput < 0){
  digitalWrite(7, LOW);
 }else{
  digitalWrite(7, HIGH);
 }
 realControllerOutput = abs(realControllerOutput);
 analogWrite(9,realControllerOutput);
 Serial.println("PWM voltage and Error");
 Serial.println(realControllerOutput);
// Serial.println(e_past);
 poop = millis();
 Tc = poop - Tc;
 Ts = Ts - Tc;
 delay(Ts);
// Serial.print("Current time");
// Serial.println(Tc); 
 
  //if a character is detected on the serial monitor, reset wheel to zero
   if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    motorEncoder.write(0);
  }

}


void sendData(){
//  Serial.print(quadrant);
  Wire.write(quadrant);

}
