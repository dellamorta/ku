#include <Sabertooth.h>
#include <PS3BT.h>
#include <PS3USB.h>
#include <usbhub.h>
#include <LinearActuator.h>

#define POS_PIN 12

Sabertooth controller(129, Serial1);
LinearActuator actuator(&controller, 1, POS_PIN);

USB Usb;
PS3USB PS3(&Usb);

int last_update = 0;
int target = 500;

void setup() {
  Serial1.begin(38400);
  Serial.begin(115200);
  Sabertooth::autobaud(Serial1);
  controller.setDeadband(10);
  
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));  
}

void loop() {
  Usb.Task();
  if (PS3.PS3Connected) {
    if (PS3.getButtonPress(LEFT)) {
      target = target - 1;
    } else if (PS3.getButtonPress(RIGHT)) {
      target = target + 1;
    }
    target = constrain(target, 0, 1000);
    
    if (PS3.getButtonClick(UP)) {
      Serial.println(target);
    }
    
    if (millis() - last_update > 10) {
      actuator.goToPosition(target);
    }
    
    actuator.update();
  }
}
