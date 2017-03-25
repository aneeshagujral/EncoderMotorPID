#include <PID_v1.h>

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,2.01,0,0, DIRECT); //kp=2.01 gives nearer pwm for 20rpm
unsigned long encA=2;
unsigned long n=0;
float r=768;  //RPR(Rev/pulse)
int a=1;
int analog=5;
unsigned long PPM;
float rpm;
int value=15;   //pwm value
float kp;
unsigned long m,previousmillis=0;
int value1=15;
void setup()
{
  Serial.begin(9600);
  pinMode(13,OUTPUT);
 
  attachInterrupt(digitalPinToInterrupt(encA),encoder,RISING);
  
  Setpoint = 25;    //rpm value for motor

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
}
void loop()
{
  digitalWrite(13,HIGH);
  unsigned long currentmillis=millis();   // counting milliseconds since the time arduino starts
  analogWrite(analog,value);
  
  Input = rpm;
  if(currentmillis-previousmillis>=6000) //due to computing PID it sometimes takes time to compute and doesnt enter th loop so added OR condition so that enters the loop everytime
  {
      if(n==m)  //condition for making the rpm 0 when the motor stops
      {
        n=0;
      }
      
      PPM=(10*n/a);  //PPM (pulses/min)
      rpm=PPM/r;
      myPID.Compute();
      Serial.println("Setpoint=60");
      Serial.println("Output PWM");
      Serial.println(Output);   //will be generated in pwm
      Serial.println("Input rpm");
      Serial.println(Input);  //print RPM
      Serial.println("Current RPM:");
      Serial.println(rpm);
      value=Output+value1;
      
      previousmillis=currentmillis; 
      a=a+1;
      m=n;
   }
 
 }

void encoder()
{
  n++;  //number of pulses
  
}

