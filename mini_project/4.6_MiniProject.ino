  //TORY WILSON
  //EENG 350 - SEED - FE
  //ASSIGNMENT 2 - Localization
  //Name the pins for clarification
  #include <Encoder.h>
  #define PI 3.1415926535897932384626433832795
  const int PIDReference = 7;
  const int PIDoutput = 8;
  const int oneA = 2;
  const int oneB = 5;
  const double r = .05;
  const double d = .1;
  const double N = 16.0;
  int aState1;
  int aState2;
  int bState1;
  int bState2;
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
  step_theta = 0;
  int realControllerOutput;
  //PID values
  int Kp = 5;
  int Kd = 2;
  int Ki = 0;
  int I = 0;
  int e_past = 0;
  int Ts = 0;
  int Tc = 0;
void setup() {
  Serial.begin(9600);
  
//  pinMode(oneA, INPUT_PULLUP); //Initialize using the PULLUP RESISTOR
//  pinMode(oneB, INPUT_PULLUP); //Initialize using the PULLUP RESISTOR

  Encoder motorEncoder(oneA, oneB);


  attachInterrupt(digitalPinToInterrupt(oneA), interrupt1, CHANGE); //Initializes interrupt()
}

void loop(){
  aState2 = digitalRead(oneA); //Read the states after entering the code for 1A
  bState2 = digitalRead(twoA); //Read the states after entering the code for 2A

  //PID control
  Ts = millis();
 int r = digitalRead(PIDReference);
 int y = digitalRead(PIDOutput);
 int error = r - y;
 if(Ts > 0){
  D = (error - e_past)/Ts;
  e_past = error;
 }
 else{
  D = 0;
 }
 I = I + (Ts*error);
 realControllerOutput = (Kp*error) + (Ki*I) + (Kd*D);
 Ts = Ts - Tc;
 Tc = Ts;
 Serial.print("Current time");
 Serial.println(Tc);
  position();

//    prints out current state
//  Serial.print(tim); //rotary encoder A state
//  Serial.print("\t"); //carriage return  
//  Serial.print(vel1); //rotary encoder A state
//  Serial.print("\t"); //carriage return
//  Serial.print(step_theta); //rotary encoder A state
//  Serial.print("\t"); //carriage return
//  Serial.println(); //carriage 
//  delay(500);
//
  Serial.print(xnew); //rotary encoder A state
  Serial.print("\t"); //carriage return
  Serial.print(ynew); //rotary encoder A state
  Serial.print("\t");//carriage return
  Serial.println(); //carriage 
}

 void interrupt1(){
  aState2 = digitalRead(oneA); //Read the states after entering the code for 1A
  bState2 = digitalRead(oneB); //Read the states after entering the code for 2A

  if (aState2 == bState2){ //checks for clockwise or counter-clockwise actions
    aVal = aVal - 2;
  }else{
    aVal = aVal + 2;
  }
  
  //time calculations
  time1 = millis();
  totaltime1 = time1;
  if(oldtime1 == totaltime1){
    t1 = 0.0;
    dot_theta1 = 0.0;
    step_theta = step_theta;
  }else{
    t1 = (((double)totaltime1 - (double)oldtime1)/1000);    
    dot_theta1 = ((4*PI)/(t1*N));
    step_theta = 1/t1;
  }
  oldtime1 = totaltime1;
 
  //velocity calculations
  vel1 = r*dot_theta1;

 
  
 }



 void position(){
  //time calculations
 
  xnew = xold + ((t1 * cos(phiold)*vel1));
  ynew = yold + ((t1 * sin(phiold)*vel1));

  xold = xnew;
  yold = ynew;
 }
