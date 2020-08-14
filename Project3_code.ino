#include <SparkFun_TB6612.h>
#include <math.h>

//------------Line Following Setup---------------//
//Assign Pin for Line Following Module
const int right_Sensor=A1; //A1
const int left_Sensor=A0; //A0
const int middle_Sensor=A2; //A2
const float finaldistance= 5;
const double dbetweenMx=2;
const double dbetweenMy=0.5; 
double dY; 
float angleOfTurn = 0; 
int motorspeedDiff; 
float velocity;

//Initiate Left, Right, Middle Values for Sensor
int left_Val=0;
int right_Val=0;
int middle_Val=0;

//Define Sensor Reading Threshold to Determine Line Presence
const int threshold_Lvl=500;

//------------------END------------------------//


//------------Motor Setup---------------------//
//Pin Initialization
#define AIN1  2
#define BIN1  7
#define AIN2  4
#define BIN2  8
#define PWMA  5
#define PWMB  6
#define STBY  9

//Offset Initialization
const int offsetA=1;
const int offsetB=1;

// Motor Initialization
Motor motor1= Motor(AIN1, AIN2,PWMA,offsetA,STBY); //left motor
Motor motor2= Motor(BIN1, BIN2,PWMB,offsetB,STBY);//right motor


//------------------END------------------------//


//------------US Setup---------------------//
//Pin Initialization
const int trigPin=11;
const int echoPin=10;

// Variable Declarations
long duration;
int distance;

//------------------END------------------------//




//
int timer = 0;
int timerForY = 0; 
int curDir = 2; // 1 = left, 2 = straught, 3 = right
int lastDir = 2; // 1 = left, 2 = straught, 3 = right
const int Kturn = 10; // turn constant that we'll play with
const int kSpeed= 510; //max speed of both motors
float r = 1; //1 is straight line, increases as curvature increases 
float lastOutput; // this value will be set to last command we needed to do 
  // ie if last thing to do was turn right, take in
const float curveCst = 2; 
int turnDirection = 0; //turn direction is determined here in binary 
int middleWasOnBlack = 1; 
double turnRadius; //radius for any turn can be found
void setup() {
  // put your setup code here, to run once:
  pinMode(right_Sensor,INPUT);
  pinMode(left_Sensor,INPUT);
  pinMode(middle_Sensor,INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  Serial.begin(9600);//
  
}

void loop() {
  
  int motorspeed1 = 255;
  int motorspeed2 = 255;
  
  
//-------------Line Following Module Code---------------
  left_Val=analogRead(left_Sensor);
  right_Val=analogRead(right_Sensor);
  middle_Val=analogRead(middle_Sensor);

  if(left_Val> threshold_Lvl) //if left sensor begins sensing hte line
  {
    curDir = 1;
    turnDirection = 1 ; // turn counter clockwise
    timerForY = 0; //if sensor starts sensing the line we need to reset the timer so we can determine further turns
  }
  else if(right_Val > threshold_Lvl)
  {
    curDir = 3;
    turnDirection = -1; //turn clockwise
    timerForY = 0;
  }
  else if(middleWasOnBlack == 1 )
      {
      if(middle_Val < threshold_Lvl) //if the middle sensor stops sensing the blackline start the timer so we can determine the turn ratio
      {
      timerForY = timerForY+ 2; //swap to 2 when we change our delay 
      }
      }
  else if(timerForY>0)
  {
      timerForY = timerForY+2;
    
  }
  if(middle_Val > threshold_Lvl) //if the middle sensor senses the line we set middleWasOnBLack to 1 for use later
  {
    middleWasOnBlack = 1;
    r = 1; 
    turnDirection = 0; 
  }
  else
  {
    middleWasOnBlack = 0;
  }
  // dY = velocity/timerForY-0.5;
  r = r*timerForY*curveCst + 1; 
   
  motorspeedDiff = motorspeed1-motorspeed2; 
  int unitSpeed = motorspeed1 + motorspeed2; // possibly used? 
  motorspeed2 = (turnDirection*Kturn+kSpeed/r)/2; 
  motorspeed1 = motorspeed2 + turnDirection*Kturn; 
    
    
  /*float turnAngle(double y)
  {
       double angle = atan2(y, 2);
       return angle; 
  }
  */
  angleOfTurn = atan2(dY, 2);

 /* float motorspeedRatio(float x, float y)
  {
      
    
  }
  */
  // here set the motorspeeds accordingly ->
  // motorspeed1, motorspeed2 = 255* some constant determined above
  /* if(turnDirection>0) // turn counterclockwise adjust motorspeed to this
  {
      turnRadius = (15*motorspeed1)/(motorspeed2-motorspeed1);
    
  }
    else if(turnDirection<0) //turn clockwise
  {

      turnRadius = (15*motorspeed2)/(motorspeed1-motorspeed2);
  }
  // if no turns -> do nothing
  */
//--------------------------------------------------//

//---------------Ultrasound Module Code-------------------//
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration=pulseIn(echoPin,HIGH);
  distance=duration*0.034/2;
  //Serial.printf("%d, %d, %d, %d", distance, left_Val, middle_Val, right_Val);
  Serial.print('{');
  Serial.print(distance);
  Serial.print(',');
  Serial.print(left_Val);
  Serial.print(',');
  Serial.print(middle_Val);
  Serial.print(',');
  Serial.print(right_Val);
  Serial.print('}');
  Serial.println();
  if(distance<finaldistance+3 && distance>finaldistance+1)
  {
      float motorspeedtemp = 25.5*(finaldistance-distance);
      constrain(motorspeedtemp,0, 255); 
      motorspeed1 = motorspeedtemp;
      motorspeed2 = motorspeedtemp;
  }
  else if(distance <= finaldistance+1)
  {
    motorspeed1 = 0 ;
    motorspeed2 = 0 ;
  }
  /*
  else if(distance<finaldistance-1)
  {
    
    float motorspeedtemp = 25.5*(finaldistance-distance)*(finaldistance-distance);
    constrain(motorspeedtemp,0, 255);
    motorspeedtemp = motorspeedtemp *-1;
    motorspeed1 = motorspeedtemp;
    motorspeed2 = motorspeedtemp; 
  }
 
  */
  //---------------------------------------------------//


  //--------------Motor Driving Code--------------------//
   motor1.drive(motorspeed1);
    motor2.drive(motorspeed2);
  // reset delay to 2 later
  delay(2);
  timer = timer + 2;//chnage to 2 later
  if (curDir == lastDir) {
      timerForY = timerForY + 2;
    }
  else {
    timerForY = 0;
  }
  //----------------------------------------------------//
}




  
