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

#define ID1 2
#define ID2 5


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Timer3.initialize(50);         // initialize timer3, and set a 50us period
  
  EASYCAT.Init();
  servo_init(&Serial1, 4, SERVO_DEFAULT_BAUD);
  
//  tests if any Dynamixel works
//  servo_set(SERVO_BROADCAST_ID, SERVO_REGISTER_GOAL_ANGLE, PI, timeout);
  
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

  
}

servo_error_t servo_changeID_from_to(uint8_t from, uint8_t to) {
  servo_error_t error = servo_set(from, SERVO_REGISTER_ID, to, timeout);  //set id back
  delay(25);
  return error;
}

servo_error_t servo_loosen_motor (uint8_t id) {
  servo_factory_reset(id, timeout);
  delay(200);
  return servo_changeID_from_to(SERVO_DEFAULT_ID, id);  //set id back
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
    receivedByte2 = EASYCAT.BufferOut.Byte[2];
    error = servo_set(ID2, SERVO_REGISTER_GOAL_ANGLE, /*bytetoradian(receivedByte2)*/motorPos, timeout);
    //send motorcopy position to etherCAT
//    error = servo_get(ID1,SERVO_REGISTER_PRESENT_ANGLE,&motorCopy,timeout);
//    sentByte = radiantobyte(motorCopy);
//    EASYCAT.BufferIn.Byte[2] = sentByte;  //write to byte 2 of ethercat    

//  //Controlling motor 2 with Arduino
//    receivedByte3 = EASYCAT.BufferOut.Byte[2];
//    error = servo_set(ID2, SERVO_REGISTER_GOAL_ANGLE, bytetoradian(receivedByte2), timeout);
}

