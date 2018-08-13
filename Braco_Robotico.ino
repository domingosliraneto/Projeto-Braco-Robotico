#include <PS3BT.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

//Motor1
//pwm
#define RotAntiHorario1  5
#define RotHorario1 6

#define controleVelocPot1 A0
//#define analog1 A5

//Mortor2

#define RotAntiHorario2  3
#define RotHorario2 9

#define controleVelocPot2 A1
//#define analog2 A4

USB Usb;
//USBHub Hub1(&Usb);

#include <Servo.h>
Servo myservo;
int garra = 0;

BTD Btd(&Usb); 
PS3BT PS3(&Btd);
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57);

bool printTemperature;
bool printAngle;

int Ea1 = 0, Ei1 = 0, Ea2 = 0, Ei2 = 0, PosMotor1 = 125, PosMotor2 = 125;

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial);
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); 
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));

  //Motor 1
    pinMode(RotAntiHorario1,OUTPUT);
    pinMode(RotHorario1,OUTPUT);
    //pinMode(analog1, INPUT);
    pinMode(controleVelocPot1, INPUT);
    
    //Motor 2
    pinMode(RotAntiHorario2,OUTPUT);
    pinMode(RotHorario2,OUTPUT);
    //pinMode(analog2, INPUT);
    pinMode(controleVelocPot2, INPUT);
    
     myservo.attach(2);

}

int PID1(int Pot1, int PosRef1)
{
  //Motor 1
  int Kp1 = 2,Ki1 = 0.5, Kd1 = 0, Dt1 = 1;
  int E1 = PosRef1 - Pot1;
  int Ed1 = (E1-Ea1)/Dt1;
  Ei1 += Dt1 * E1;
  
  if(Ei1 > 255)
    Ei1 = 255;
  if(Ei1< - 255)
    Ei1 = - 255;
 
  int pid1 = Kp1*E1 + Ki1*Ei1 + Kd1*Ed1; 
  Ea1 = E1;
  return pid1;

}
//Motor 2
int PID2(int Pot2, int PosRef2)
{
  //Motor 2
  int Kp2 = 2, Ki2 = 0.5, Kd2 = 0, Dt2 = 1;
  int E2 = PosRef2 - Pot2;
  int Ed2 = (E2-Ea2)/Dt2;
  Ei2 += Dt2 * E2;

  if(Ei2 > 255)
    Ei2 = 255;
  if(Ei2 < - 255)
    Ei2 = - 255;
 
  int pid2 = Kp2*E2 + Ki2*Ei2 + Kd2*Ed2; 
  Ea2 = E2;
  return pid2;

}
  //Motor1
  void controleDeVelcidade1 (int PosRef1)
{
  int pwm1 = PID1(analogRead(controleVelocPot1)/4, PosRef1);
  if(pwm1 > 255)
    pwm1 = 255;
  if (pwm1 < -255)
    pwm1 = -255;
    
  if(pwm1 > 0)
    analogWrite(RotHorario1,pwm1);
  else
    analogWrite(RotAntiHorario1,-pwm1);
} 
  
  //Motor2
  void controleDeVelcidade2 (int PosRef2)
{
  int pwm2 = PID2(analogRead(controleVelocPot2)/4, PosRef2);
  if(pwm2 > 255)
    pwm2 = 255;
  if (pwm2 < -255)
    pwm2 = -255;
 
  if(pwm2 > 0)
    analogWrite(RotHorario2,pwm2);
  else
    analogWrite(RotAntiHorario2,-pwm2);
} 

void loop() {
  Usb.Task();
   if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getButtonPress(DOWN)){
      Serial.print(F("\r\nDOWN: "));
      Serial.print(PosMotor1);
      PosMotor1-=1;
      if(PosMotor1 < 0)
        PosMotor1 = 0;
      }
      
      if (PS3.getButtonPress(UP)){
      Serial.print(F("\r\nUP: "));
      Serial.print(PosMotor1);
      PosMotor1+=1;
      if(PosMotor1 > 250)
        PosMotor1 = 250;
      }
      
      if (PS3.getButtonPress(RIGHT)){
      Serial.print(F("\r\nRIGHT: "));
      Serial.print(PosMotor2);
      PosMotor2-=1;
      if(PosMotor2 < 0)
        PosMotor2 = 0;
      }
      
      if (PS3.getButtonPress(LEFT)){
      Serial.print(F("\r\nLEFT: "));
      Serial.print(PosMotor2);
      PosMotor2+=1;
      if(PosMotor2 > 250)
        PosMotor2 = 250;
      }
    }
///////////////////////////////////////
if (PS3.getButtonClick(TRIANGLE)){
  Serial.print(F("\r\nTriangle"));
  garra = 130;
  }
if (PS3.getButtonClick(CROSS)){
  Serial.print(F("\r\nCROSS"));
  garra = 20;
  }
    myservo.write(garra);
    //Motores
    controleDeVelcidade1 (PosMotor2);
    controleDeVelcidade2 (PosMotor1);
    //Motores
    controleDeVelcidade1 (PosMotor2);
    controleDeVelcidade2 (PosMotor1);
}

