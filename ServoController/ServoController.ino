#include <Servo.h>
#include <math.h>

//Define your Touch screen connections
#define X1 A0
#define X2 A1
#define Y1 A2
#define Y2 A3
//Define your screen resolution as per your Touch screen (Max: 1024)
#define Xres 860
#define Yres 430

#define ga 9.80665
#define pi 3.14159265359

#define thetaLim 120

// for interrupt
uint32_t interval = 80;
static uint32_t nextTime = 0;

// target initialy 0, 0
int tX = 545, tY = 215;

Servo A,B,C;

double x, y; 
double prevx = 0, prevy = 0; //prevVx = 0, prevVy = 0;
double delPosx = 0,delPosy = 0,delPos = 0, vel= 0, intv = (double)interval/1000, delVelx = 0, delVely = 0;
double Ax = 216, Ay = 150, Bx = 767, By = 39, Cx = 616, Cy = 466;
double sze = 0;
double m = 0.95;
int angA, angB, angC, dangA, dangB, dangC;

double Kpx = 0.15, Kdx = 3.6, Kpy = 0.125, Kdy = 4, Kix = 1, Kiy = 1;
//double Kpx = 1, Kdx = 1, Kpy = 1, Kdy = 1, Kix = 1, Kiy = 1;

int refA = 151, refB = 125, refC = 145;

int pos = 0;

void setup() {
  // put your setup code here, to run once:
  A.attach(5);
  C.attach(6);
  B.attach(7);

  // servo initiation
  A.write(154); 
  B.write(125);
  C.write(151);
  Serial.begin(9600);

  prevx = tX;
  prevy = tY;

  // variable initiation 
  sze = sqrt((Ax-tX)*(Ax-tX) + (Ay-tY)*(Ay-tY));
  Ax = (double)(Ax - tX)/sze;
  Ay = (double)(Ay - tY)/sze;
  sze = sqrt((Bx-tX)*(Bx-tX) + (By-tY)*(By-tY));
  Bx = (double)(Bx - tX)/sze;
  By = (double)(By - tY)/sze;
  sze = sqrt((Cx-tX)*(Cx-tX) + (Cy-tY)*(Cy-tY));
  Cx = (double)(Cx - tX)/sze;
  Cy = (double)(Cy - tY)/sze;

  angA = 154;
  angB = 125;
  angC = 151;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - nextTime >= interval){
    unsigned long tstart = millis();
    nextTime += interval;
    ////////////////////////// sense position////////////////////////////////
    ////get x position 
    pinMode(Y1,INPUT);
    pinMode(Y2,INPUT);  
    digitalWrite(Y2,LOW);
    pinMode(X1,OUTPUT);
    digitalWrite(X1,HIGH);
    pinMode(X2,OUTPUT);
    digitalWrite(X2,LOW);
    x = (analogRead(Y1))/(1024/Xres); //Reads X axis touch position
    
    pinMode(X1,INPUT);
    pinMode(X2,INPUT);
    digitalWrite(X2,LOW);
    pinMode(Y1,OUTPUT);
    digitalWrite(Y1,HIGH);
    pinMode(Y2,OUTPUT);
    digitalWrite(Y2,LOW);
    y = (analogRead(X1))/(1024/Yres); //Reads Y axis touch position
    
    //Display X and Y on Serial Monitor
    
    Serial.print("X = ");  
    Serial.print(x);
    Serial.print(" Y = ");
    Serial.println(y);
    
    /////////////////////////////////////////////////////////
    ///// with A
    // pd control
    delPosx = Kpx*(x - tX)/(4*1000);// in m
    delPosy = Kpy*(y - tY)/(4*1000);

    delVelx = Kdx*(delPosx - prevx)/intv; 
    delVely = Kdy*(delPosy - prevy)/intv;

    delPos = (Ax*delPosx + Ay*delPosy);
    vel = (Ax*delVelx + Ay*delVely);
    dangA = (asin((7/5)*asin((1/(ga*m))*(vel/intv + (delPos/(intv*intv)))))*(360/(2*pi))); //  360/2*pi to converted into degree, 700 for radius in mm,  190 for l1 + l2 in mm, 1000 for ga in mm
    // with B
    delPos = (Bx*delPosx + By*delPosy);
    vel = (Bx*delVelx + By*delVely);
    dangB = (asin((7/5)*asin((1/(ga*m))*(vel/intv + (delPos/(intv*intv)))))*(360/(2*pi)));
    // with C
    delPos = (Cx*delPosx + Cy*delPosy);
    vel = (Cx*delVelx + Cy*delVely);
    dangC = (asin((7/5)*asin((1/(ga*m))*(vel/intv + (delPos/(intv*intv)))))*(360/(2*pi)));
    /////
    ///////////range limiter 
    angA = 154 - dangA;
    angB = 125 - dangB;
    angC = 151 - dangC;

    //Serial.println(vel);
    //Serial.println(delPos);
    Serial.println(asin((7/5)*asin((1/ga)*(vel/intv + (delPos/(intv*intv))))));
    //Serial.println(dangA);
    //Serial.println(dangB);
    //Serial.println(dangC);
    //Serial.println(((double)vel/(double)intv));

    //// limiter to stop from falling apart
    // A 179 - 119
    angA = (angA >= (refA+30))?(refA+30):angA;
    angA = (angA <= (refA-30))?(refA-30):angA;
    // B 171 - 111
    angB = (angB >= (refB+30))?(refA+30):angB;
    angB = (angB <= (refA-30))?(refA-30):angB;
    // C 172 - 112
    angC = (angC >= (refC+30))?(refC+30):angC;
    angC = (angC <= (refA-30))?(refA-30):angC;

    A.write(angA);
    B.write(angB);
    C.write(angC);

    prevx = delPosx;
    prevy = delPosy;
    //prevVx = delVelx;
    //prevVy = delVely;
    
    /////////////////////Controller ////////////////////////
    // 1 PID control
    if((millis() - tstart) >= interval) Serial.println("0");
  }
}