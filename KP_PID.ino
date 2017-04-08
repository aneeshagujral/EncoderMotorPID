#include <PID_v1.h>

double Setpoint, Input, Output,Input_1,Output_1,Setpoint_1;
PID myPID(&Input, &Output, &Setpoint,4.91,1,0.1, DIRECT); //for m1 kp=1.7 gives nearer pwm for 20rpm and kd=0.01 for making it faster, ki to remove offset
PID myPID_1(&Input_1, &Output_1, &Setpoint_1,5.21,1,0.1, DIRECT); //for m2 kp=1.4 gives nearer pwm for 20rpm

unsigned long encA=2;
unsigned long encB=3;
unsigned long n=0,q=0;
float r=768;  //RPR(Rev/pulse)
int a=1;
int analog=5;
int analog1=6;
unsigned long PPM,PPM1;
float rpm,rpm1;
float offset;
float kp;
int s;
unsigned long m,previousmillis=0;
void setup()
{
  Serial.begin(9600);
  pinMode(13,OUTPUT);
 
  attachInterrupt(digitalPinToInterrupt(encA),encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(encB),encoder1,RISING);
  Setpoint = 25;    //rpm value for motor1
  Setpoint_1=25;   //rpm value for motor2

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID_1.SetMode(AUTOMATIC);
  
}
void loop()
{
  digitalWrite(13,HIGH);
  unsigned long currentmillis=millis();   // counting milliseconds since the time arduino starts
  
  //int y=map(offset,0,103,0,255);
  analogWrite(analog,Output);
  analogWrite(analog1,Output_1);
  
  if(currentmillis-previousmillis==1000) //due to computing PID it sometimes takes time to compute and doesnt enter th loop so added OR condition so that enters the loop everytime
  {
      if(n==m&q==s)  //condition for making the rpm 0 when the motor stops
      {
        n=0;
        q=0;
      }
      
      PPM=(60*n/a);  //PPM (pulses/min)
      PPM1=(60*q/a);  //PPM (pulses/min)
      
      rpm=PPM/r;
      rpm1=PPM1/r;
      Input = rpm;
      Input_1 = rpm1;
      myPID.Compute();
      myPID_1.Compute();
      Serial.println("Motor 1");
      Serial.println("Output PWM 1");
      Serial.println(Output);   //will be generated in pwm
      
      Serial.println("Input rpm");
      Serial.println(Input);  //print RPM
      Serial.println("Current RPM1:");
      Serial.println(rpm);
     // offset=Setpoint-Input;
      
      Serial.println("Motor 2");
      Serial.println("Output PWM 2");
      Serial.println(Output_1);   //will be generated in pwm
      Serial.println("Input rpm 2");
      Serial.println(Input_1);  //print RPM
      Serial.println("Current RPM2:");
      Serial.println(rpm1);
      
      
      a=a+1;
      m=n;
      s=q;
      previousmillis=currentmillis; 
   }
 
 }

void encoder()
{
  n++;  //number of pulses
  
}

void encoder1()
{
  q++;  //number of pulses
  
}



