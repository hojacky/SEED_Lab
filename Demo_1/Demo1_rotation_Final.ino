 //SEED_LAB group 14 Arduino motor reading and spinning
 //Name the pins for clarification

  #include <Encoder.h>
  #include <Wire.h>

  #define PI 3.1415926535897932384626433832795
//  const double r = .05;
  double distance = 2658;//((2658/(2*PI*0.245))/1600)*PI;//3ft
  //double distance = 13290;//5ft
  //double distance = 19606;//7ft
  //double angularDistance = 150;////150 for 90 deg degrees to turn
  double angularDistance = 255;//135 deg
//  double angularDistance = 353;// 180 deg
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
  double str_Err_L = 0;
  double str_Err_R = 0;
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
  
void setup() {
  Serial.begin(9600);
//  Serial.println("Wheel positions");
  
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




void LeftAngle(){
  r_L = y_L;
  y_L = motorEncoder.read();

 }


 void RightAngle(){
  r_R = y_R;
  y_R = motorEncoder2.read();

 }



 void loop() { 

if (turn) {
///////////////////////////////////////////Left and Right Angle////////////////////////////////////////////////////
Tc = millis();

 y_L = -1*(y_L/1600)*PI;
 y_R = (y_R/1600)*PI;
 
 lefterror = angularDistance + y_L;
 righterror = angularDistance - y_R;

 if(Tc > 0){
  D_L = (y_L - r_L)/Ts; //rad/mil
  D_R = (y_R - r_R)/Ts; //rad/mil
 }
 else{
  D_L = 0;
  D_R = 0;
 }

  I_L = I_L + (Ts*lefterror); //rad*mil
  I_R = I_R + (Ts*righterror); //rad*mil
  leftControllerOutput = (Kp*lefterror);//+ (Ki*I_L) + (Kd*D_L);  
  rightControllerOutput = (Kp*righterror);//+ (Ki*I_R) + (Kd*D_R); 
 if (signbit(leftControllerOutput)){
  if (abs(leftControllerOutput) > 8.5) {
    leftControllerOutput = -8.5;
  }
 } else if (abs(leftControllerOutput) > 8.5){
  leftControllerOutput = 8.5;
 }
 
 if (signbit(rightControllerOutput)){
  if (abs(rightControllerOutput) > 8.5) {
    rightControllerOutput = -8.5;
  }
 } else if (abs(rightControllerOutput) > 8.5){
  rightControllerOutput = 8.5;
 }
 
// voltageL = leftControllerOutput;
// voltageR = rightControllerOutput;
  
 if(leftControllerOutput > 0){ 
  digitalWrite(7, LOW);
 }else{
  digitalWrite(7, HIGH);
 }

 if(rightControllerOutput < 0){
  digitalWrite(8, LOW);
 }else{
  digitalWrite(8, HIGH);
 }
 
 leftControllerOutput = abs(leftControllerOutput);
 leftControllerOutput = (leftControllerOutput/8.5)*255;
 if(leftControllerOutput > 255){
    leftControllerOutput = 255;
 }

 rightControllerOutput = abs(rightControllerOutput);
 rightControllerOutput = (rightControllerOutput/8.5)*255;
 if(rightControllerOutput > 255){
    rightControllerOutput = 255;
 }
 digitalWrite(7, HIGH);
 analogWrite(9,leftControllerOutput);
 analogWrite(10,rightControllerOutput);

 Tc2 = millis();
 delay(50-(Tc2-Tc));
 
}
 
  if ((righterror < 0.5) && (lefterror < 0.5)) {//&& (abs(rightControllerOutput) < 0.5) && (abs(leftControllerOutput) < 0.5)) {
    analogWrite(10,0);
    analogWrite(9,0);
    turn = 0;
    if ((!turn) && (!straight)){
      motorEncoder.write(0);
      motorEncoder2.write(0);
      //distance = 12*1.0*2*PI / 5.75*PI;
    }
    straight = 1;
    r_L = distance;
    r_R = distance;
  }
  if (straight) {
    Tc = millis();
    y_L = (y_L/1600)*PI;
    y_R = (y_R/1600)*PI;
    strErr_L = distance + y_L;
    strErr_R = distance - y_R;
   
    if(Tc > 0){
    D_L = (lefterror - e_Lpast)/Ts; //rad/mil
    e_Lpast = lefterror;
   }
   else{
    D_L = 0;
   }
 
   if(Tc > 0){
    D_R = (righterror - e_Rpast)/Ts; //rad/mil
    e_Rpast = righterror;
   } else{
    D_R = 0;
   }
   
   if (lefterror < 0){
    leftControllerOutput = 0;
   }else{
      I_L = I_L + (Ts*lefterror); //rad*mil
      leftControllerOutput = (Kp*lefterror);// + (Ki*I_L) + (Kd*D_L)));
   }
   if (righterror < 0){
    rightControllerOutput = 0;
   }else{
     I_R = I_R + (Ts*righterror); //rad*mil
     rightControllerOutput = (Kp*righterror);// + (Ki*I_R) + (Kd*D_R);
   } 
   
   if(leftControllerOutput > 0){
    digitalWrite(7, LOW);
   }else{
    digitalWrite(7, HIGH);
   }
   if(angularDistance < 0){
    digitalWrite(8, LOW);
   }else{
    digitalWrite(8, HIGH);
   }
   
   leftControllerOutput = abs(leftControllerOutput);
   leftControllerOutput = (leftControllerOutput/8.5)*255;
   if(leftControllerOutput > 255){
    leftControllerOutput = 255;
   }
   rightControllerOutput = abs(rightControllerOutput);
   rightControllerOutput = (rightControllerOutput/8.5)*255;
   if(rightControllerOutput > 255){
    rightControllerOutput = 255;
   }
   analogWrite(9,leftControllerOutput);
   analogWrite(10,rightControllerOutput);
   
   Tc2 = millis();
   delay(50-(Tc2-Tc));
   
   
   
//      Serial.print("Tell go a foot ");
//      Serial.println(distance);
      RightWheel();
      LeftWheel();
  }

}
