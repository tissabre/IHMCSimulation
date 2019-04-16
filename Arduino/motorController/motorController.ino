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

float rad = 0;
float new_max_angle = 6.23;
#define ID1 2
#define ID2 5
#define ID3 3
#define SERVO_DARWIN_BAUD 1000000
#define BIGGEST_ID 20
const long TWOBYTES =  pow(2,16)-1;

class Dynamixel {
  private:
    int ID;

  public:
    //Constructors
    Dynamixel(int _id) {
      ID = _id;
    }

    int id() {
      return ID;
    }

    void info() {
      Serial.print("ID: ");
      Serial.println(ID);
    }
};

//ALL DARWIN MOTORS
//DO NOT USE ID 1 BECAUSE RESETTING A MOTOR MAKES IT ID 1, WHICH WILL CREATE MULTIPLE ID 1 --> VERY BAD
//Syntax: Dynamixel JOINT_NAME(ID, MIN_ANGLE, MAX_ANGLE);

//ARMS
Dynamixel SHOULDER_R(100),  HIGH_ARM_R(3),    LOW_ARM_R(5),
          SHOULDER_L(2),    HIGH_ARM_L(4),    LOW_ARM_L(6);

//LEGS
Dynamixel THIGH1_R(7),      PELVIS_R(9),      THIGH2_R(11),    TIBIA_R(13),       ANKLE1_R(15),         ANKLE2_R(17),
          THIGH1_L(8),      PELVIS_L(10),     THIGH2_L(12),    TIBIA_L(14),       ANKLE1_L(16),         ANKLE2_L(18);

//NECK
Dynamixel PAN(19),          TILT(20);

//Array of motors to send
Dynamixel allMotors[] = {TILT, SHOULDER_R, SHOULDER_L, HIGH_ARM_R, HIGH_ARM_L, LOW_ARM_R, LOW_ARM_L, PAN};//, HIGH_ARM_R, HIGH_ARM_L, LOW_ARM_R, LOW_ARM_L, THIGH1_R, THIGH1_L, PELVIS_R, PELVIS_L, THIGH2_R, THIGH2_L, TIBIA_R, TIBIA_L, ANKLE1_R, ANKLE1_L, ANKLE2_R, ANKLE2_L};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Timer3.initialize(50);         // initialize timer3, and set a 50us period

  EASYCAT.Init();
  servo_init(&Serial1, 4, SERVO_DEFAULT_BAUD);
//    servo_init(&Serial1, 4, SERVO_DARWIN_BAUD);

  //  servo_set(HIGH_ARM_R.id(), SERVO_REGISTER_GOAL_ANGLE, HIGH_ARM_R.minAngle(), timeout);
  //  servo_get(3,SERVO_REGISTER_PRESENT_ANGLE,&motorCopy,timeout);
//      servo_loosen_motor(20);
//    servo_changeID_from_to(100,2);

  //Only need to run once
//  loosenMotors(1, 6);
//  standPosition();    //Make sure the robot is able to stand and is not constrained by any other surrounding object
}

void loop() {
  delay(2);
  Micros = micros();
  if (Micros - PreviousCycle >= 50)
  {
    PreviousCycle = Micros;
    EASYCAT.MainTask();                                 // execute the EasyCAT task
    Application();

}

//    helloMotion();
//    mimicArm();                                                                                                          
  //  mimicLeg();
//    testDynamixelRead(1, BIGGEST_ID);
}

void helloMotion() {
  MOVE(SHOULDER_R, 4.79);
  MOVE(HIGH_ARM_R, 3.90);
  MOVE(LOW_ARM_R, 3.65);
  delay(500);
  MOVE(LOW_ARM_R, 2.5);
}

void standPosition() {
  MOVE(THIGH1_R,  PI);             
  MOVE(PELVIS_R,  PI);
  MOVE(THIGH2_R,  PI);
  MOVE(TIBIA_R,   PI);
  MOVE(ANKLE1_R,  PI);
  MOVE(ANKLE2_R,  PI);

  MOVE(THIGH1_L,  PI);
  MOVE(PELVIS_L,  PI);
  MOVE(THIGH2_L,  PI);
  MOVE(TIBIA_L,   PI);
  MOVE(ANKLE1_L,  PI);
  MOVE(ANKLE2_L,  PI);
}
void mimicArm() {
  float pos_should_r, pos_high_a_r, pos_low_a_r;
  //read
  pos_should_r = READ(SHOULDER_R);
  pos_high_a_r = READ(HIGH_ARM_R);
  pos_low_a_r = READ(LOW_ARM_R);

  //mimic
  MOVE(SHOULDER_L,  2 * PI - pos_should_r);
  MOVE(HIGH_ARM_L,  2 * PI - pos_high_a_r);
  MOVE(LOW_ARM_L,   2 * PI - pos_low_a_r);
}

void mimicLeg() {
  float pos_th1_r, pos_pel_r, pos_th2_r, pos_tib_r, pos_ank1_r, pos_ank2_r;
  //read
  pos_th1_r = READ(THIGH1_R);
  pos_pel_r = READ(PELVIS_R);
  pos_th2_r = READ(THIGH2_R);
  pos_tib_r = READ(TIBIA_R);
  pos_ank1_r = READ(ANKLE1_R);
  pos_ank2_r = READ(ANKLE2_R);

  //mimic
  MOVE(THIGH1_L,  2 * PI - pos_th1_r);
  MOVE(PELVIS_L,  2 * PI - pos_pel_r);
  MOVE(THIGH2_L,   2 * PI - pos_th2_r);
  MOVE(TIBIA_L,  2 * PI - pos_tib_r);
  MOVE(ANKLE1_L,  2 * PI - pos_ank1_r);
  MOVE(ANKLE2_L,   2 * PI - pos_ank2_r);
}

void testDynamixelWrite(float angle) {
  //  tests if any Dynamixel works
  servo_set(SERVO_BROADCAST_ID, SERVO_REGISTER_GOAL_ANGLE, angle, timeout);

}

void testDynamixelRead(int id_start, int id_end) {
  float pos;

  //Show ID 1 independently
  servo_get(1, SERVO_REGISTER_PRESENT_ANGLE, &pos, timeout);
  Serial.print("[");
  Serial.print(pos);
  Serial.print("] ");
  for (int i = id_start; i <= id_end; i = i + 2) {
    if (i == 1)
      servo_get(100, SERVO_REGISTER_PRESENT_ANGLE, &pos, timeout);
    else
      servo_get(i, SERVO_REGISTER_PRESENT_ANGLE, &pos, timeout);

    Serial.print(pos);
    Serial.print(" ");
  }

  Serial.print("\t");

  for (int i = id_start + 1; i <= id_end; i = i + 2) {
    if (i == 1)
      servo_get(100, SERVO_REGISTER_PRESENT_ANGLE, &pos, timeout);
    else
      servo_get(i, SERVO_REGISTER_PRESENT_ANGLE, &pos, timeout);

    Serial.print(pos);
    Serial.print(" ");
  }
  Serial.println();

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

void loosenMotors(int st, int en) {
  for (int i = st; i <= en; i++) {
    if (i == 1)
      servo_loosen_motor(100);
    else
      servo_loosen_motor(i);
  }
}

float READ(Dynamixel dy) {
  float motorpos;
  servo_get(dy.id(), SERVO_REGISTER_PRESENT_ANGLE, &motorpos, timeout);
  return motorpos;
}

void MOVE(Dynamixel dy, float goal_pos) {
  servo_set(dy.id(), SERVO_REGISTER_GOAL_ANGLE, goal_pos, timeout);
}

void SENDPOSITIONS() {
  //ARMS
  //Dynamixel SHOULDER_R(100, 3.17, 6.27),            HIGH_ARM_R(3, 2.33, 5.13),              LOW_ARM_R(5, 1.6, 3.74),
  //          SHOULDER_L(2, 2 * PI - 6.27, 2 * PI - 3.17),    HIGH_ARM_L(4, 2 * PI - 5.13, 2 * PI - 2.33),    LOW_ARM_L(6, 2 * PI - 3.74, 2 * PI - 1.6);
  //
  //LEGS
  //Dynamixel THIGH1_R(7, 3.17, 6.27),                PELVIS_R(9, 2.33, 5.13),                THIGH2_R(11, 1.6, 3.74),                        TIBIA_R(13, 0, 0),                      ANKLE1_R(15, 0, 0),                     ANKLE2_R(17, 0, 0),
  //          THIGH1_L(8, 2 * PI - 6.27, 2 * PI - 3.17),      PELVIS_L(10, 2 * PI - 5.13, 2 * PI - 2.33),     THIGH2_L(12, 2 * PI - 3.74, 2 * PI - 1.6),            TIBIA_L(14, 2 * PI - 0, 0),                 ANKLE1_L(16, 2 * PI - 0, 0),                ANKLE2_L(18, 2 * PI - 0, 0);
  unsigned long whole2bytes;
  byte b0_7, b8_15;
  int eth_pos = 0;

  for (auto dy:allMotors) {
    whole2bytes = radianto2bytes(READ(dy));
    b8_15 = (whole2bytes & 0xFF00) >> 8;
    b0_7 = whole2bytes & 0xFF;
    //whole2bytes = (b8_15 << 8) | b0_7;
    //Exceptions
    if (dy.id() == 100)
      EASYCAT.BufferIn.Byte[3*eth_pos] = 1;
    else if (dy.id() == 19)
      EASYCAT.BufferIn.Byte[3*eth_pos] = 7;
    else if (dy.id() == 20)
      EASYCAT.BufferIn.Byte[3*eth_pos] = 0;
    else
      EASYCAT.BufferIn.Byte[3*eth_pos] = dy.id();
      
    EASYCAT.BufferIn.Byte[3*eth_pos+1] = b8_15;
    EASYCAT.BufferIn.Byte[3*eth_pos+2] = b0_7;
    //////////////////////////////////////////////
    eth_pos++;
  }

//EASYCAT.BufferIn.Byte[0] = 1;
  
//  Serial.print(whole2bytes);
//  Serial.print(" - ");
                                                                                                                                                                                                                                                                                                           
//  unsigned int result = (b8_15 << 8 | b0_7);
//  Serial.println(result);

}

float bytetoradian( int Byte)
{
  return ((Byte * 2 * PI) / 256);
}

byte radiantobyte( float Radian)
{
  return ((255 * Radian) / (2 * PI));
}

long radianto2bytes (float rad) {
  return (((TWOBYTES) * rad) / (2 * PI));
}

void Application()
{
  
  
//  mimic
//  MOVE(SHOULDER_L,  2 * PI - READ(SHOULDER_R));
//  MOVE(HIGH_ARM_L,  2 * PI - READ(HIGH_ARM_R));
//  MOVE(LOW_ARM_L,   2 * PI - READ(LOW_ARM_R));  

  SENDPOSITIONS();
}

