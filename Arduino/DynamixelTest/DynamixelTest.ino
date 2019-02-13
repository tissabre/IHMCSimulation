#include <TimerThree.h>
#include <EasyCAT.h>
EasyCAT EASYCAT;

#include <AX12A.h>

#define DirectionPin  (10u)
#define BaudRate    (1000000ul)
#define ID    (1u)

unsigned long Micros = 0;
unsigned long PreviousCycle = 0;
int previous, writebyte0;
int incomingByte = 0;
void setup()
{
  Timer3.initialize(50);         // initialize timer3, and set a 50us period
  ax12a.begin(BaudRate, DirectionPin, &Serial);
  ax12a.setEndless(ID, ON);

  EASYCAT.Init();
 
}

void loop()
{
    delay(2);
    Micros = micros();                                    //
    if (Micros - PreviousCycle >= 50)                     // each 500 uS   
    { 
        PreviousCycle = Micros;
        EASYCAT.MainTask();                                 // execute the EasyCAT task
        Application(); 
    }
}

void turnLeft() {
    ax12a.ledStatus(ID, OFF);
    ax12a.turn(ID, LEFT, 800);
    delay(50);
}

void turnRight() {
    ax12a.ledStatus(ID, ON);
    ax12a.turn(ID, RIGHT, 800);
    delay(50);
}

void stopMotor() {
  ax12a.turn(ID, RIGHT, 0);
}

int readMotorPos() {
  ax12a.readPosition(ID);
}

void Application ()                                        
{  
//  Moving motor
  writebyte0 = EASYCAT.BufferOut.Byte[0];

  if (writebyte0 < previous) {
    turnLeft();
  }
  else if (writebyte0 > previous) {
    turnRight();
  }
  else {
    stopMotor();  
}
  
  previous = writebyte0;

//  Reading motor positions
//    turnLeft();
//    EASYCAT.BufferIn.Byte[0] = ax12a.readPosition(ID);
//    EASYCAT.BufferIn.Byte[1] = ax12a.readSpeed(ID);
//    EASYCAT.BufferIn.Byte[2] = ax12a.RWStatus(ID);
//    ax12a.setID(ID, 1);
    


}

