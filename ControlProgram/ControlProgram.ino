
#include <Sabertooth.h>
#include <PS3BT.h>
#include <PS3USB.h>
#include <usbhub.h>
#include <LinearActuator.h>
#include <KarKontroller.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#define BRAKE_POS_PIN 15
#define SHIFTER_POS_PIN 14
#define THROTTLE_POS_PIN 13
#define STEERING_POS_PIN 12

Sabertooth throttleClutch(135, Serial1);
Sabertooth steeringBrake(129, Serial1);

LinearActuator throttle(&throttleClutch, 1, THROTTLE_POS_PIN);
LinearActuator shifter(&throttleClutch, 2, SHIFTER_POS_PIN);
LinearActuator brake(&steeringBrake, 2, BRAKE_POS_PIN);
LinearActuator steering(&steeringBrake, 1, STEERING_POS_PIN);

KarKontroller::Config konfig;
KarKontroller kontroller(&throttle, &brake, &shifter, &steering, konfig);

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

//BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
//PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0xCA, 0x8C, 0x79); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
PS3USB PS3(&Usb);

int last_update_time = 0;

void setup() {
  Serial1.begin(38400);
  Serial.begin(115200);
  Sabertooth::autobaud(Serial1);
  throttleClutch.setDeadband(10);
  steeringBrake.setDeadband(10);

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void incrementMotor(int* moto) {
  *moto += 32;
  if (*moto > 127) {
    *moto = -127;
  }
}

int translateHat(int hat) {
  return map(hat, 0, 255, 0, 1023);
}
int last_update = 0;

void loop() {
  Usb.Task();
  if (PS3.PS3Connected) {
/*    Serial.print(F("\r\nThrottle: "));
    Serial.print(throttle.getPosition());
    Serial.print(F("\r\nBrake: "));
    Serial.print(brake.getPosition());    
    Serial.print(F("\r\nSHIFTER: "));
    Serial.print(shifter.getPosition());
    Serial.print(F("\r\nLeftHatX: "));
    Serial.print(translateHat(PS3.getAnalogHat(LeftHatX)));
    Serial.print(F("\tLeftHatY: "));
    Serial.print(translateHat(PS3.getAnalogHat(LeftHatY)));
    Serial.print(F("\tRightHatX: "));
    Serial.print(translateHat(PS3.getAnalogHat(RightHatX)));
    Serial.print(F("\tRightHatY: "));
    Serial.print(translateHat(PS3.getAnalogHat(RightHatX)));
    Serial.println("");*/
    if (millis() - last_update > 10) {
      kontroller.setSteering(PS3.getAnalogHat(RightHatX));
      int brakeThrottle = PS3.getAnalogHat(LeftHatY) - 127;
      if (brakeThrottle < 0) {
        kontroller.setThrottle(0);
        kontroller.setBrake(-2 * brakeThrottle);
      } else {
        kontroller.setBrake(0);
        kontroller.setThrottle(2 * brakeThrottle);
      }
      
      if (PS3.getButtonClick(UP))
        kontroller.setGear(FIRST);
      if (PS3.getButtonClick(DOWN))
        kontroller.setGear(REVERSE);
      if (PS3.getButtonClick(RIGHT))
        kontroller.setGear(PARK);

      if (!kontroller.isStarterOn() && PS3.getButtonPress(START)) {
        kontroller.starterOn();
      }
      if (kontroller.isStarterOn() && !PS3.getButtonPress(START)) {
        kontroller.starterOff();
      }
      
      if (PS3.getButtonClick(SELECT)) {
        if (kontroller.isRunning()) {
          kontroller.turnOff();
        } else {
          kontroller.turnOn();
        }
      }
        
      last_update = millis();
    }
  }    
  kontroller.update();
}
