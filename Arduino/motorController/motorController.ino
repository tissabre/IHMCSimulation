#include <TimerThree.h>
#include <Dynamixel_Servo.h>
#include <EasyCAT.h>

EasyCAT EASYCAT;
unsigned long Micros = 0;
unsigned long PreviousCycle = 0;
int previous;
int writebyte0 = 0;
int incomingByte = 0;
int timeout = 50; //milliseconds

byte receivedByte;
float motorPos;
byte sentByte;
servo_error_t error;
byte turnOnSignal;
bool loosened = false;

byte receivedByte2;
float motorCopy;
byte sentByte2;

float max_angle = 0;
float new_max_angle = 6.23;
#define ID1 2
#define ID2 5
#define ID3 3
#define SERVO_DARWIN_BAUD 1000000
#define BIGGEST_ID 6

class Dynamixel {
  private:
    int ID;
    //angles in radians
    float MIN_ANGLE;
    float MAX_ANGLE;

   public:
      //Constructors
      Dynamixel(int _id, float min_rad, float max_rad) {
        ID = _id;
        MIN_ANGLE = min_rad;
        MAX_ANGLE = max_rad;
      }

      int id() {
        return ID;
      }

      float minAngle() {
        return MIN_ANGLE;
      }

      float maxAngle() {
        return MAX_ANGLE;
      }

      void info() {
        Serial.print("ID: ");
        Serial.println(ID);

        Serial.print("Minimum Angle: ");
        Serial.println(MIN_ANGLE);

        Serial.print("Maximum Angle: ");
        Serial.println(MAX_ANGLE);
      }
};

//ALL DARWIN MOTORS
//DO NOT USE ID 1 BECAUSE RESETTING A MOTOR MAKES IT ID 1, WHICH WILL CREATE MULTIPLE ID 1 --> VERY BAD
  Dynamixel SHOULDER_R(100, 3.17, 6.27),            HIGH_ARM_R(3, 2.33, 5.13),              LOW_ARM_R(5,1.6,3.74),
            SHOULDER_L(2, 2*PI-6.27, 2*PI-3.17),    HIGH_ARM_L(4, 2*PI-5.13, 2*PI-2.33),    LOW_ARM_L(6, 2*PI-3.74, 2*PI-1.6);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Timer3.initialize(50);         // initialize timer3, and set a 50us period
  
  EASYCAT.Init();
  servo_init(&Serial1, 4, SERVO_DEFAULT_BAUD);
//  servo_init(&Serial1, 4, SERVO_DARWIN_BAUD);
  
//  servo_set(HIGH_ARM_R.id(), SERVO_REGISTER_GOAL_ANGLE, HIGH_ARM_R.minAngle(), timeout);
//  servo_get(3,SERVO_REGISTER_PRESENT_ANGLE,&motorCopy,timeout);
//    servo_loosen_motor(5);
//  servo_changeID_from_to(1,3);
//  loosenAll();
}

void loop() {
//    delay(2);
//    Micros = micros();
//    if (Micros - PreviousCycle >= 50) 
//    { 
//        PreviousCycle = Micros;
//        EASYCAT.MainTask();                                 // execute the EasyCAT task
//        Application(); 
//    }

//  helloMotion();
//  mimicArm();
}

void helloMotion() {
  MOVE(SHOULDER_R, 4.79);
  MOVE(HIGH_ARM_R, 3.90);
  MOVE(LOW_ARM_R, 3.65);
  delay(500);
  MOVE(LOW_ARM_R, 2.5);
}

void mimicArm() {
    float pos_should_r, pos_high_a_r, pos_low_a_r;
//read
   pos_should_r = READ(SHOULDER_R);
   pos_high_a_r = READ(HIGH_ARM_R);
   pos_low_a_r = READ(LOW_ARM_R);
    
//mimic
   MOVE(SHOULDER_L,  2*PI - pos_should_r);
   MOVE(HIGH_ARM_L,  2*PI - pos_high_a_r);
   MOVE(LOW_ARM_L,   2*PI - pos_low_a_r);
}

void testDynamixelWrite(float angle) {
  //  tests if any Dynamixel works
  servo_set(SERVO_BROADCAST_ID, SERVO_REGISTER_GOAL_ANGLE, angle, timeout);
  
}

void testDynamixelRead(int id_start, int id_end) {
  float pos;

  //Show ID 1 independently
  servo_get(1,SERVO_REGISTER_PRESENT_ANGLE,&pos,timeout);
  Serial.print("[");
  Serial.print(pos);
  Serial.print("] ");
  for (int i = id_start; i<=id_end; i++) {
   if (i == 1) 
    servo_get(100,SERVO_REGISTER_PRESENT_ANGLE,&pos,timeout);
   else 
    servo_get(i,SERVO_REGISTER_PRESENT_ANGLE,&pos,timeout);
   
   Serial.print(pos);
   Serial.print(" ");
  }
  Serial.println(" ");
  
}

servo_error_t servo_changeID_from_to(uint8_t from, uint8_t to) {
  servo_error_t error = servo_set(from, SERVO_REGISTER_ID, to, timeout);  //set id back
  delay(25);
  return error;
}

servo_error_t servo_loosen_motor (uint8_t id) {
  servo_factory_reset(id, timeout);
  delay(300);
  return servo_changeID_from_to(SERVO_DEFAULT_ID, id);  //set id back
}

void loosenAll() {
   for (int i = 1; i<=BIGGEST_ID; i++) {
   if (i == 1) 
    servo_loosen_motor(100);
   else 
    servo_loosen_motor(i);
  }
}

float READ(Dynamixel dy) {
  float motorpos;
  servo_get(dy.id(),SERVO_REGISTER_PRESENT_ANGLE,&motorpos,timeout);
  return motorpos;
}
void MOVE(Dynamixel dy, float goal_pos) {
  servo_set(dy.id(), SERVO_REGISTER_GOAL_ANGLE, goal_pos, timeout);
}

float bytetoradian( int Byte)
{
  return ((Byte*2*PI)/256);
}
byte radiantobyte( float Radian)
{
  return ((255*Radian)/(2*PI));
}
void Application()
{
  //Controlling motor 1
  turnOnSignal = EASYCAT.BufferOut.Byte[1];
  switch (turnOnSignal) {
    
    //motor should be loosened
    case 0:
      //this allows motor to be loosened only once
      if (loosened == false) {
          loosened = true;
          servo_loosen_motor(ID1);
      }  
    break;
    
    //motor should be on and locked into position
    case 1:
       loosened = false;
    break;

    default:
    break;
  }

  //Reading from etherCAT and writing to motor
  if(!loosened) {
    receivedByte = EASYCAT.BufferOut.Byte[0];
    error = servo_set(ID1, SERVO_REGISTER_GOAL_ANGLE, bytetoradian(receivedByte), timeout);
    //update bufferIn for next time master reads values (prevents jumps)
    EASYCAT.BufferIn.Byte[0] = receivedByte;
  }

  //Reading from motor and writing to etherCAT
  else {
    error = servo_get(ID1,SERVO_REGISTER_PRESENT_ANGLE,&motorPos,timeout);
    sentByte = radiantobyte(motorPos);
    EASYCAT.BufferIn.Byte[0] = sentByte;  //write to byte 0 of ethercat    
  }

  //Controlling motor 2 with Arduino
//    receivedByte2 = EASYCAT.BufferOut.Byte[2];
//    error = servo_set(ID2, SERVO_REGISTER_GOAL_ANGLE, /*bytetoradian(receivedByte2)*/motorPos, timeout);
    //send motorcopy position to etherCAT
//    error = servo_get(ID1,SERVO_REGISTER_PRESENT_ANGLE,&motorCopy,timeout);
//    sentByte = radiantobyte(motorCopy);
//    EASYCAT.BufferIn.Byte[2] = sentByte;  //write to byte 2 of ethercat    

//  //Controlling motor 2 with IHMC
    receivedByte2 = EASYCAT.BufferOut.Byte[2];
    error = servo_set(ID2, SERVO_REGISTER_GOAL_ANGLE, bytetoradian(receivedByte2), timeout);
}
