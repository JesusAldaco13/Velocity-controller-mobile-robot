#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <math.h>
#include <Encoder.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);

Encoder le(2,2);
Encoder re(3,3);
 
// Variables for storing the calculated velocity
double wR;
double wL; 
double LdVal = 0;
double RdVal = 0;
double Radius = 0.05;
double Length = 0.14;

double wd = 0;
double vd = 0.5;  
double wdr;
double wdl;
double wdr_p=0;
double wdl_p=0;
double wrf;
double wlf;
double wrf_p=0;
double wlf_p=0;

double CR;
double CR_p=0;
double CR_pp=0;
double CL;
double CL_p=0;
double CL_pp=0;

double Lerror;
double Lerror_p = 0;
double Lerror_pp = 0;
double Rerror;
double Rerror_p = 0;
double Rerror_pp = 0;
      
int PWMR;
int PWML;

double kp = 0.288;   
double ki = 1.426;    

double alpha = 100;  
double h = ki/kp;

long L; 
long R;	   
long L_last=0;
long R_last=0;	
unsigned long Time=0;
unsigned long sample_time=100; 
double td=0.100; 


void setup()
{
   
  AFMS.begin();  
  
  Serial.begin(9600);
  
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
 
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

 delay(1000);
}

void loop()
{     
  if (millis()<20000) 
  {
      if(millis()-Time>sample_time)
        {
           Time = millis();
           GetSpeeds();    
        }   
  }
  
  
  else 
  {  
    rightMotor->setSpeed(0);
    leftMotor->setSpeed(0);
  }
   
}


void GetSpeeds()
{
    wdr= (2*vd - Length*wd)/(2*Radius);
    wdl= (2*vd + Length*wd)/(2*Radius);
    
    wrf = ( (td*h)*wdr + (td*h)*wdr_p - (td*h - 2)*wrf_p)/(2 + td*h);
    wlf = ( (td*h)*wdl + (td*h)*wdl_p - (td*h - 2)*wlf_p)/(2 + td*h);
    
    wrf_p = wrf;
    wlf_p = wlf;
    wdr_p = wdr;
    wdl_p = wdl;
     
    
    L = le.read();
    R = re.read();	
   
   LdVal = (double)( L- L_last)/ (td);
   RdVal = (double)( R -R_last)/(td);

   wL = LdVal*2*3.14159 /32;  
   wR = RdVal*2*3.14159 /32;
     
   Rerror = wrf - wR;
   Lerror = wlf - wL;
      
   
   CL = ((alpha*td*td*ki+2*alpha*td*kp)*Lerror + (2*alpha*td*td*ki)*Lerror_p + (alpha*td*td*ki-2*alpha*td*kp)*Lerror_pp + 8*CL_p - (4-2*alpha*td)*CL_pp)/(2*alpha*td + 4);
   CR = ((alpha*td*td*ki+2*alpha*td*kp)*Rerror + (2*alpha*td*td*ki)*Rerror_p + (alpha*td*td*ki-2*alpha*td*kp)*Rerror_pp + 8*CR_p - (4-2*alpha*td)*CR_pp)/(2*alpha*td + 4);
   
   CR_pp = CR_p;
   CR_p = CR;
   CL_pp = CL_p;
   CL_p = CL;
   Lerror_pp = Lerror_p;
   Lerror_p = Lerror;
   Rerror_pp = Rerror_p;
   Rerror_p = Rerror;
     
   PWMR = int(255.0*CR/5.15);
   PWML = int(255.0*CL/5.15);
     
     if (PWMR>=255) {PWMR=255;}
   else if (PWMR<=0) {PWMR=0;}
 
   if (PWML>=255) {PWML=255;}
   else if (PWML<=0) {PWML=0;}
       
   leftMotor->setSpeed(PWML);
   leftMotor->run(FORWARD);
   rightMotor->setSpeed(PWMR);
   rightMotor->run(FORWARD);
  
   L_last = L;
   R_last = R;	
   
  Serial.print("  ");
  Serial.print( wL );  //
  Serial.print("  ");
  Serial.print(  wR); // 
  Serial.print("  ");
  Serial.print( CL );  //
  Serial.print("  ");
  Serial.println(  CR); // 
  
  
}
