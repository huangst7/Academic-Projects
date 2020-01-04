//Dec.1st---V1
//delay when reading sensors
//Complete "forward" function
//add "send to Matlab and wait for response" function (send, keep checking response, keep calculating compass)
//add "read walls" function (head north, read sensor, send to matlab and wait)

#include <Servo.h>
#include <Wire.h>
//#include <SoftwareSerial.h>
#include <LedControl.h>

int North=0;
int East=90;
int South=180;
int West=270;
int targetAngle;
int ref;  //ref can be 0;90;270;180
int currentAngle;
int LorR;
int dif;

//----------UR SENSOR---------// F = front  L = left  R = right  B = back
int trigPinF = 51;
int echoPinF = 50;
long durationF;
int FDis;

int trigPinL = 47;
int echoPinL = 46;
long durationL;
int LDis;
int oldLDis;

int trigPinR = 43;
int echoPinR = 42;
long durationR;
int RDis;
int oldRDis;

int trigPinB = 39;
int echoPinB = 38;
long durationB;
int BDis;

//----------IR SENSOR-----------// TO BE COMPLETED
int IRDis;

//----------GRIPPER--------------//
Servo RGrip;
Servo LGrip;

//----------WHEEL---------------//
Servo Rwheel;
Servo Lwheel;

//--------Define Pin------------//
int RGripPin = 8;
int LGripPin = 9;
int RwheelPin = 10;
int LwheelPin = 11;

//----------Define Compass-------------//

long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;

float angelX = 0, angelY = 0, angelZ = 0;
int angle;

long timePast = 0;
long timePresent = 0;

//Compass serial print
long t_now = 0;
long t_last_print = 0;
int T_print = 200;

//-----------------Communication with Matlab----------------/////
int row = 0;
int col = 0;
int cmd = 0;
String input;
int action;


//------------------Logic----------------------//
int checkPoint = 0; //1= checkPoint detected, 0=haven't reach checkpoint
int confirmCheckCase = 0; // 0=false, 1=true

//----------------LED Matrix--------------//
LedControl lc=LedControl(6,5,4,1);  // Pins: DIN,CLK,CS, # of Display connected
unsigned long delayTime=200;  // Delay between Frames

byte set23[] =
{
   B11101111,
   B00100001,
   B00100001,
   B11101111,
   B10000001,
   B10000001,
   B10000001,
   B11101111
};
byte set25[] =
{
   B11101111,
   B00101000,
   B00101000,
   B11101111,
   B10000001,
   B10000001,
   B10000001,
   B11101111
};
byte set32[] =
{
   B11101111,
   B00100001,
   B00100001,
   B11101111,
   B00101000,
   B00101000,
   B00101000,
   B11101111
};
byte set35[] =
{
   B11101111,
   B00101000,
   B00101000,
   B11101111,
   B00100001,
   B00100001,
   B00100001,
   B11101111
};
byte set37[] =
{
   B11101111,
   B00100001,
   B00100001,
   B11100001,
   B00100001,
   B00100001,
   B00100001,
   B11100001
};
byte set39[] =
{
   B11101111,
   B00101001,
   B00101001,
   B11101111,
   B00100001,
   B00100001,
   B00100001,
   B11101111
};
byte set52[] =
{
   B11101111,
   B10000001,
   B10000001,
   B11101111,
   B00101000,
   B00101000,
   B00101000,
   B11101111
};
byte set54[] =
{
   B11101001,
   B10001001,
   B10001001,
   B11101111,
   B00100001,
   B00100001,
   B00100001,
   B11100001
};
byte set57[] =
{
   B11101111,
   B10000001,
   B10000001,
   B11100001,
   B00100001,
   B00100001,
   B00100001,
   B11100001
};
byte setarrow[] =
{
   B00011000,
   B00100100,
   B01000010,
   B10011001,
   B00011000,
   B00011000,
   B00011000,
   B00011000
};
byte setarrowl[] =
{
   B00010000,
   B00100000,
   B01000000,
   B10011111,
   B10011111,
   B01000000,
   B00100000,
   B00010000
};
byte setarrowr[] =
{
   B00001000,
   B00000100,
   B00000010,
   B11111001,
   B11111001,
   B00000010,
   B00000100,
   B00001000
};
byte setcali[] =
{
   B01111110,
   B01000010,
   B00111100,
   B00011000,
   B00011000,
   B00100100,
   B01111110,
   B01111110
};
byte setcheck[] =
{
   B00000000,
   B00000001,
   B00000010,
   B00000100,
   B10001000,
   B01010000,
   B00100000,
   B00000000
};
byte setgo[] =
{
   B01110110,
   B10001001,
   B10001001,
   B10001001,
   B10111001,
   B10011001,
   B10011001,
   B01100110
};
byte setwhat[] =
{
   B00111100,
   B01000010,
   B01000010,
   B00001100,
   B00010000,
   B00011000,
   B00000000,
   B00011000
};
byte set666[] =
{
   B00011000,
   B00011000,
   B00011000,
   B00011000,
   B00011000,
   B00000000,
   B00011000,
   B00011000
};
byte set66666[] =
{
   B01100110,
   B01100110,
   B01100110,
   B01100110,
   B01100110,
   B00000000,
   B01100110,
   B01100110
};

//buzzer////////////////////////////////////////////////////////
const int buzzerPin = 3;

const int note_f = 349;
const int note_gS = 415;
const int note_a = 440;
const int note_cH = 523;
const int note_eH = 659;
const int note_fH = 698;
const int ending = 0;
////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------
 * -----------------------Setup--------------------------------------
 ------------------------------------------------------------------*/
void setup() {
Serial.begin(115200);
//Serial.println("Begin Setup");
//============== UR sensor ===============//
  pinMode(trigPinF,OUTPUT);
  pinMode(echoPinF, INPUT);

  pinMode(trigPinL,OUTPUT);
  pinMode(echoPinL, INPUT);

  pinMode(trigPinR,OUTPUT);
  pinMode(echoPinR, INPUT);

  pinMode(trigPinB,OUTPUT);
  pinMode(echoPinB, INPUT);

//============== Gripper ===============//
RGrip.attach (RGripPin);
LGrip.attach (LGripPin);
delay (100);
GRIP();

//================ Wheel ===============//
Rwheel.attach (RwheelPin);
Lwheel.attach (LwheelPin);

//============LED Matrix===============//
lc.shutdown(0,false);  // Wake up displays
lc.setIntensity(0,5);  // Set intensity levels
lc.clearDisplay(0);  // Clear Displays

//=============buzzer================//
  pinMode(buzzerPin, OUTPUT);

//================ Compass ===============//
delay(1000); //setup delay
Wire.begin();
  setUpMPU();
  callibrateGyroValues();
  timePresent = millis();
//=============Read initial ref==============//
ref = Read_COMPASS();
//Serial.print("Setup Ready, Facing North, current Ref Angle:  ");
//Serial.println(ref);
}

/*-------------------------------------------------------------------
 * -----------------------Main Loop----------------------------------
 ------------------------------------------------------------------*/
void loop() {
//Serial.println("Enter Main Loop");
showgo();
locolization();
Delay(2000);

}

/*-------------------------------------------------------------------
 * -----------------------Funtions-----------------------------------
 ------------------------------------------------------------------*/

//----------------UR SENSORS-----------------//
int Read_F(){
    digitalWrite(trigPinF, LOW); 
    delay (2);
    digitalWrite(trigPinF, HIGH);
    delay(10);
    digitalWrite(trigPinF, LOW);
    durationF = pulseIn(echoPinF, HIGH);
    FDis = durationF*0.034/2;
    //Serial.print("F-Dis:  ");
    //Serial.println(FDis);
    return FDis;
}
int Read_L(){
    digitalWrite(trigPinL, LOW);
    delay (2);
    digitalWrite(trigPinL, HIGH);
    delay(10);
    digitalWrite(trigPinL, LOW);
    durationL = pulseIn(echoPinL, HIGH);
    LDis = durationL*0.034/2;
    //Serial.print("L-Dis:  ");
    //Serial.println(LDis);
    return LDis;
    
}
int Read_R(){
    digitalWrite(trigPinR, LOW);
    delay (2);
    digitalWrite(trigPinR, HIGH);
    delay(10);
    digitalWrite(trigPinR, LOW);
    durationR = pulseIn(echoPinR, HIGH);
    RDis = durationR*0.034/2;
    //Serial.print("R-Dis:  ");
    //Serial.println(RDis);
    return RDis;
}

int Read_B(){
    digitalWrite(trigPinB, LOW);
    delay (2);
    digitalWrite(trigPinB, HIGH);
    delay(10);
    digitalWrite(trigPinB, LOW);
    durationB = pulseIn(echoPinB, HIGH);
    BDis = durationB*0.034/2;
    //Serial.print("B-Dis:  ");
    //Serial.println(BDis);
    return BDis;
}

//-----------------IR SENSOR------------------//
int Read_IR(){
  IRDis = analogRead(A0);
  //THIS IS NOT REAL DISTANCE TO CM
  //NEED TO MODIFY
  return IRDis;
}

//-------------------Grip--------------------//
void DROP(){
  RGrip.write(0); 
  LGrip.write(55); 
}
void GRIP(){
   RGrip.write(65); 
   LGrip.write(0); 
}

//-------------Basic Movements------------------//
void Continuous_F(){
  Rwheel.writeMicroseconds(1360);
  Lwheel.writeMicroseconds(1600);
}
 
void Continuous_B(){
  Rwheel.writeMicroseconds(1567);
  Lwheel.writeMicroseconds(1400);
}

void Continuous_R(){
  Rwheel.writeMicroseconds(1599);
  Lwheel.writeMicroseconds(1600);
}

void Continuous_L(){
  Rwheel.writeMicroseconds(1380);
  Lwheel.writeMicroseconds(1380);
}
void Brake(){
  Rwheel.writeMicroseconds(1500);
  Lwheel.writeMicroseconds(1500);
}

void slowRight(){
  Rwheel.writeMicroseconds(1540);
  Lwheel.writeMicroseconds(1540);
}

void slowLeft(){
  Rwheel.writeMicroseconds(1410);
  Lwheel.writeMicroseconds(1410);
}

void Delay(int dur){    
  for (int i=0;i<dur;i+=20){
    readAndProcessGyroData();
    delay(20);
  }
}

void bareR(){     
  Rwheel.writeMicroseconds(1457);
  Lwheel.writeMicroseconds(1710);
  Delay(300);
  Continuous_F();
  Delay(200);
  headRef();
}

void bareL(){                              
  Rwheel.writeMicroseconds(1257);
  Lwheel.writeMicroseconds(1510);
  Delay(300);
  Continuous_F();
  Delay(200);
  headRef();
}

void goStraight(){
  headDirectionNoBrake(ref);
  //Serial.println("In the go straight function ");
  int Lread=Read_L();
  int Rread=Read_R();
  if(Lread<=20 && Rread<=20){//BothWall mode
      if (abs(Lread-Rread) <= 2){
        Continuous_F();
        }
      else if (Lread-Rread > 2){
        bareL();
        }
      else if (Lread-Rread < 2){
        bareR();
        }
    }
  else if(Lread<=20 && Rread>20){      //LeftWall mode
      if (abs(Lread-7) <= 1){
        Continuous_F();
        }
      else if (Lread-7 > 1){
        bareL();
        }
      else if (7-Lread > 1){
        bareR();
        }
    }
  else if(Lread>20 && Rread<=20){       //RightWall mode
      if (abs(Rread-7) <= 1){
        Continuous_F();
        }
      else if (Rread-7 > 1){
        bareR();
        }
      else if (7-Rread > 1){
        bareL();
        } 
    }
  else {
     Continuous_F();
    }
 
}

//--------------------------Turn to angle------------------------------//
void headDirection(int targetAngle){
    currentAngle=Read_COMPASS();
    dif = currentAngle - targetAngle;
    while (abs(dif)>20 && abs(dif)<340){
      currentAngle = Read_COMPASS();
     // Serial.println(currentAngle);
      dif = currentAngle - targetAngle;
      LorR = Determine_LR(targetAngle,currentAngle);
      if (LorR == 1){
        Continuous_R();
        }
      if (LorR == -1){
        Continuous_L();
        } 
      }
    while (abs(dif)>2 && abs(dif)<358){
      currentAngle = Read_COMPASS();
      dif = currentAngle - targetAngle;
      LorR = Determine_LR(targetAngle,currentAngle);
      if (LorR == 1){
        slowRight();
      }
      if (LorR == -1){
        slowLeft();
      }  
  }
  Brake();
  showarrow();
}

void headDirectionNoBrake(int targetAngle){
    currentAngle=Read_COMPASS();
    dif = currentAngle - targetAngle;
    while (abs(dif)>20 && abs(dif)<340){
      currentAngle = Read_COMPASS();
     // Serial.println(currentAngle);
      dif = currentAngle - targetAngle;
      LorR = Determine_LR(targetAngle,currentAngle);
      if (LorR == 1){
        Continuous_R();
        }
      if (LorR == -1){
        Continuous_L();
        } 
      }
    while (abs(dif)>2 && abs(dif)<358){
      currentAngle = Read_COMPASS();
      dif = currentAngle - targetAngle;
      LorR = Determine_LR(targetAngle,currentAngle);
      if (LorR == 1){
        slowRight();
      }
      if (LorR == -1){
        slowLeft();
      }  
  }
}
//--------------------------Turn to ref------------------------------//
void headRef(){
  headDirection(ref);
}

//------------------------Determ L or R-------------------------// return 1: Right turn, return -1: left turn
int Determine_LR(int targetAngle,int currentAngle){
    if (targetAngle >=0 && targetAngle <=180){
    int target_inverse = targetAngle +180;
      if (targetAngle<currentAngle && currentAngle<target_inverse){
        showarrowl();
        return -1;
        }
      else {
        showarrowr();
        return 1;
        }
    }
    else {
    int target_inverse = targetAngle -180;
      if (target_inverse<currentAngle && currentAngle<targetAngle){
        showarrowr();
        return 1;
        }
      else {
        showarrowl();
        return -1;
        }
    }
    }

//------------------------------------------------------Read compass---------------------------------------------------------------//
int Read_COMPASS(){
  readAndProcessGyroData();
  //Serial.println(angle);
  //print_results();
  delay(20);
  return angle;
}

void setUpMPU() {
  // power management
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // configure gyro
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  // configure accelerometer
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();  
}

void callibrateGyroValues() {
  //Serial.println("Prepare Cali No Touch!");
  showcali();
  delay(1000);
  //Serial.println("start cali");
  for (int i=0; i<5000; i++) {
    getGyroValues();
    gyroZCalli = gyroZCalli + gyroZPresent;
  }
  gyroZCalli = gyroZCalli/5000;  
  //Serial.println("end cali");
  showcheck();
}

void readAndProcessGyroData() {
  gyroZPast = gyroZPresent;                                   // Assign Present gyro reaging to past gyro reading
  timePast = timePresent;                                     // Assign Present time to past time
  timePresent = millis();                                     // get the current time in milli seconds, it is the present time
  
  getGyroValues();                                            // get gyro readings
  calculateAngle();                                           // calculate the Angle  
}

void getGyroValues() {
  Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU 
  Wire.write(0x43);                                           // Access the starting register of gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);                              // Request for 6 bytes from gyro registers (43 - 48)
  while(Wire.available() < 6);                                // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read()<<8|Wire.read();                  // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read()<<8|Wire.read();                  // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read()<<8|Wire.read();                  //Store last two bytes into gyroZPresent
}

void calculateAngle() {  
  // same equation can be written as 
  // angelZ = angelZ + ((timePresentZ - timePastZ)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) / (2*1000*131);
  // 1/(1000*2*131) = 0.00000382
  // 1000 --> convert milli seconds into seconds
  // 2 --> comes when calculation area of trapezium
  // substacted the callibated result two times because there are two gyro readings
  //angelX = angelX + ((timePresent - timePast)*(gyroXPresent + gyroXPast - 2*gyroXCalli)) * 0.00000382;
  //angelY = angelY + ((timePresent - timePast)*(gyroYPresent + gyroYPast - 2*gyroYCalli)) * 0.00000382;
  if (abs(gyroZPresent-gyroZCalli)<100){
    gyroZPresent=gyroZCalli;
  }
  angelZ = angelZ + ((timePresent - timePast)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) * 0.00000388;
  //Serial.println(angelZ);
  angle = angelZ;
  angle = angle % 360;

  if (angle > 0 ){
    angle = 360 - angle;
  }
  else {
    angle = - angle;
  }
}
/*
void print_results() {
  t_now = millis();
  if (t_now - t_last_print >= T_print){
    t_last_print = t_now;
    Serial.println();
  }
}
*/
int checkIfNeedStop(){      //1 = stopped
  headDirectionNoBrake(ref);
  //Serial.println ("Now in the chech if need stop function");
  LDis=Read_L();
  RDis=Read_R();
  FDis=Read_F();

  if (FDis <=14){
    FDis=Read_F();
    if (FDis <=14){
    Brake();
    //Serial.println("front wall, stop");
    return 1;
   }
   //Serial.println("////////////////////////////////////////////////////////////////////");
   //Serial.println("////////////////////////////////////////////////////////////////////");
    oldLDis = LDis;
    oldRDis = RDis;
    goStraight();
    return 0;
  }
   else if((LDis-oldLDis>12 && oldLDis < 18) || (RDis-oldRDis>12 && oldRDis < 18)){
    //Serial.println("Intersection Detected!");
    halfStepFoward(ref);
    return 1;
   }
   else{
    oldLDis = LDis;
    oldRDis = RDis;
    goStraight();
    return 0;
   }
}
void findCheckPoint(){
  headDirection(ref);
  oldLDis = Read_L();
  oldRDis = Read_R();
  checkPoint = 0;
  while (checkPoint != 1){
  checkPoint = checkIfNeedStop();
}
  //Serial.println("Checkpoint!");
  showwhat();
  }

void oneStepFoward(int direct){
  int duration = 3600;
  for (int i=0;i<duration;i+=200){
  headDirection(direct);
  Continuous_F();
  Delay(200);  
  }    
  Brake();
  }


void halfStepFoward(int direct){
  int duration = 1200;
  for (int i=0;i<duration;i+=200){
  headDirection(direct);
  Continuous_F();
  Delay(200);  
  }    
  Brake();
  }

//=========================Locolization=====================================
void locolization(){
  
  Delay(100);

  while (cmd == 0){
    if(Serial.available()>0){
      input = Serial.readString();
      action = input.toInt();
      cmd = 1;
      }
    Delay(20);
    }


  if (action == 0){ //Face North and check case
    Serial.println(doubleCheckCase());
  }
  
  else if (action == 1){
    //Move North 1 step;
    oneStepFoward(North);
    Serial.println(doubleCheckCase());
  }

  else if (action == 2){
    //Move West 1 step;
    oneStepFoward(West);
    Serial.println(doubleCheckCase());
  }

  else if (action == 3){
    //Move East 1 step;
    oneStepFoward(East);
    Serial.println(doubleCheckCase());
  }

  else if (action == 4){
    //Move South 1 step;
    oneStepFoward(South);
    Serial.println(doubleCheckCase());
  }

  else if (action == 5){
    //Turn to North, keep going until intersect;
    ref = North;
    findCheckPoint();   
    Serial.println("I see an Intersection while going North!");
  }
  
  else if (action == 6){
    //Turn to West, keep going until checkpoint;
    ref = West;
    findCheckPoint(); 
    Serial.println("I see an Intersection while going West!");
  }

  else if (action == 7){
    //Turn to East, keep going until checkpoint;
    ref = East;
    findCheckPoint(); 
    Serial.println("I see an Intersection while going East!");
  }

  else if (action == 8){
    //Turn to South, keep going until checkpoint;
    ref = South;
    findCheckPoint(); 
    Serial.println("I see an Intersection while going South!");
  }

  else if (action == 9){
    //go forward, until pick up block;
    //Serial.println("OPEN GRIPEER");
    DROP();
    blockMove ();
    buzzer2();
    Serial.println("i got block");
  }
  
  else if (action == 10){
    DROP();
    show66666();
    Continuous_B();
    Delay (1500);
    Brake();
    GRIP();
    buzzer1();
    buzzer2();
    Serial.println("I am done");
  }
 else if (action == 99){
    show666();
    buzzer1();
    Serial.println("local");
  }
 else if (action == 23){
    show23();
    Serial.println("LED ON");
  }
 else if (action == 25){
    show25();
    Serial.println("LED ON");
  }
  else if (action == 32){
    show32();
    Serial.println("LED ON");
  }
  else if (action == 35){
    show35();
    Serial.println("LED ON");
  }
  else if (action == 37){
    show37();
    Serial.println("LED ON");
  }
  else if (action == 39){
    show39();
    Serial.println("LED ON");
  }
  else if (action == 52){
    show52();
    Serial.println("LED ON");
  }
  else if (action == 54){
    show54();
    Serial.println("LED ON");
  }
  else if (action == 57){
    show57();
    Serial.println("LED ON");
  }
  cmd = 0;

  
  }

int Check_Case(){

headDirection(North);
 int F;
 int L;
 int R;
 int B;


 if (Read_F()<20) 
  F = 0;
 else 
  F = 1;
 if (Read_L()<20) 
  L = 0;
 else 
  L = 1;
 if (Read_R()<20) 
  R = 0;
 else 
  R = 1;
 if (Read_B()<20)
  B = 0;
 else 
  B = 1;

 if(F==0 && L==0 && B==1 && R==1){
     return 1;
 }
 
 if(F==0 && L==1 && B==1 && R==1){
     return 2;
 }

 if(F==0 && L==1 && B==1 && R==0){
     return 3;
 }

 if(F==1 && L==0 && B==1 && R==1){
     return 4;
 }
 
 if(F==1 && L==1 && B==1 && R==1){
     return 5;
 }

 if(F==1 && L==1 && B==1 && R==0){
     return 6;
 }

 if(F==1 && L==1 && B==0 && R==1){
     return 7;
 } 

 if(F==1 && L==0 && B==0 && R==0){
     return 8;
 } 

 if(F==0 && L==1 && B==0 && R==1){
     return 11;
 }

 if(F==0 && L==0 && B==1 && R==0){
     return 22;
 }

 if(F==1 && L==1 && B==0 && R==0){
     return 33;
 }

 if(F==1 && L==0 && B==0 && R==1){
     return 44;
 }

 if(F==1 && L==0 && B==1 && R==0){
     return 55;
 } 
}

int doubleCheckCase(){
  confirmCheckCase = 0;
  int oldCheckCase;
  int newCheckCase;
  while (confirmCheckCase == 0){
  oldCheckCase = Check_Case();
  newCheckCase = Check_Case();
  if (oldCheckCase == newCheckCase){
    confirmCheckCase = 1;
  }
}
  return newCheckCase;
}



//=================LED Functions==================//
void show23()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set23[i]);
  }
}
void show25()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set25[i]);
  }
}
void show32()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set32[i]);
  }
}
void show35()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set35[i]);
  }
}
void show37()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set37[i]);
  }
}
void show39()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set39[i]);
  }
}
void show52()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set52[i]);
  }
}
void show54()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set54[i]);
  }
}
void show57()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set57[i]);
  }
}
void showarrowl()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,setarrowl[i]);
  }
}
void showarrowr()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,setarrowr[i]);
  }
}
void showarrow()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,setarrow[i]);
  }
}
void showcali()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,setcali[i]);
  }
}
void showcheck()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,setcheck[i]);
  }
}
void showgo()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,setgo[i]);
  }
}
void showwhat()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,setwhat[i]);
  }
}
void show666()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set666[i]);
  }
}
void show66666()
{
  for (int i = 0; i < 8; i++)  
  {
    lc.setRow(0,i,set66666[i]);
  }
}

void blockMove (){
 int blockWall = Read_F();
 int oldblockWall = Read_F();
 while (blockWall > 15 || oldblockWall > 15){
   blockGoStraight();
   blockWall = Read_F();
   oldblockWall = Read_F();
   int IRDis = Read_IR();
    if (IRDis > 300){
      show666();
    }
 }
Brake();
GRIP();
}


//=====================Loading Zone Move==================//
void blockGoStraight(){
  headDirectionNoBrake(ref);
  //Serial.println("In the go straight function ");
  int Lread=Read_L();
  int Rread=Read_R();
  if(Lread<=20 && Rread<=20){//BothWall mode
      if (abs(Lread-Rread) <= 2){
        Continuous_F();
        }
      else if (Lread-Rread > 2){
        blockBareL();
        }
      else if (Lread-Rread < 2){
        blockBareR();
        }
    }
  else if(Lread<=20 && Rread>20){      //LeftWall mode
      if (abs(Lread-7) <= 1){
        Continuous_F();
        }
      else if (Lread-7 > 1){
        blockBareL();
        }
      else if (7-Lread > 1){
        blockBareR();
        }
    }
  else if(Lread>20 && Rread<=20){       //RightWall mode
      if (abs(Rread-7) <= 1){
        Continuous_F();
        }
      else if (Rread-7 > 1){
        blockBareR();
        }
      else if (7-Rread > 1){
        blockBareL();
        } 
    }
  else {
     Continuous_F();
    }
 
}

void blockBareR(){     
  Rwheel.writeMicroseconds(1457);
  Lwheel.writeMicroseconds(1710);
  Delay(120);
  Continuous_F();
  Delay(100);
  headRef();
}

void blockBareL(){                              
  Rwheel.writeMicroseconds(1257);
  Lwheel.writeMicroseconds(1510);
  Delay(120);
  Continuous_F();
  Delay(100);
  headRef();
}
void buzzer1(){
  beep(note_a, 500);
  beep(note_a, 500);    
  beep(note_a, 500);
  beep(note_f, 350);
  beep(note_cH, 150);  
  beep(note_a, 500);
  beep(note_f, 350);
  beep(note_cH, 150);
  beep(note_a, 650);
  beep(ending,10);
}
void buzzer2(){
  beep(note_eH, 500);
  beep(note_eH, 500);  
  beep(note_fH, 350);
  beep(note_cH, 150);
  beep(note_gS, 500);
  beep(note_f, 350);
  beep(note_cH, 150);
  beep(note_a, 650);
  beep(ending,10);
}
void beep(int note, int duration)
{
  
 //Play tone on buzzerPin
 tone(buzzerPin, note);
 Delay(duration);
  
  //Stop tone on buzzerPin
  noTone(buzzerPin);
  Delay(50);

}

