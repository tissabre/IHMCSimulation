#include <TimerThree.h>

#include <Dynamixel_Servo.h>
#include <EasyCAT.h>



EasyCAT EASYCAT;
unsigned long Micros = 0;
unsigned long PreviousCycle = 0;
int previous;
float writebyte0;
int incomingByte = 0;
  int timeout = 50; //milliseconds

#define ID 1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Timer3.initialize(50);         // initialize timer3, and set a 50us period
  servo_init(&Serial1, 4, 1000000);
  EASYCAT.Init();
}

void loop() {
  // put your main code here, to run repeatedly:
    delay(2);
    Micros = micros();                                    //
    if (Micros - PreviousCycle >= 50)                     // each 500 uS   
    { 
        PreviousCycle = Micros;
        EASYCAT.MainTask();                                 // execute the EasyCAT task
        Application(); 
    }
}

float bytetoradian( int Byte)
{
  return ((Byte*2*PI)/255);
}
byte radiantobyte( float Radian)
{
  return ((255*Radian)/(2*PI));
}
void Application()
{
  float readbyte;
  byte sentbyte = 0;
  servo_error_t error;
//  writebyte0 = bytetoradian(EASYCAT.BufferOut.Byte[0]);
// 
//  error = servo_set(ID, SERVO_REGISTER_GOAL_ANGLE, writebyte0, timeout);


 error = servo_get(ID,SERVO_REGISTER_PRESENT_ANGLE,&readbyte,timeout);
 sentbyte = radiantobyte(readbyte);
  Serial.println(readbyte);
// Serial.println(sentbyte);
//Serial.write((uint8_t)(readbyte*180/PI));
  EASYCAT.BufferIn.Byte[0] = sentbyte;
//  error = servo_set(ID, SERVO_REGISTER_MOVING_SPEED, 100, timeout);

}
