#include "KarKontroller.h"

KarKontroller::Config::Config() {
  throttle.min = 100;
  throttle.max = 900;
  brake.min = 100;
  brake.max = 900;
  steering.min = 100;
  steering.max = 100;
  
  gear_pos[PARK] = 150;
  gear_pos[REVERSE] = 300;
  gear_pos[NEUTRAL] = 450;
  gear_pos[DRIVE] = 600;
  gear_pos[SECOND] = 750;
  gear_pos[FIRST] = 900;
}

KarKontroller::KarKontroller(:inearActuator* throttle,
                             LinearActuator* brake,
                             LinearActuator* shifter,
                             LinearActuator* steering,
                             const Config& konfig)
                             : throttle_(throttle),
                               brake_(brake),
                               shifter_(shifter),
                               steering_(steering),
                               config_(konfig) {}


uint8_t KarKontroller::getThrottle() {
  uint16_t = contstrain()
  return map(value, 
             config_.throttle.min,
             config_.throttle.max,
             0, 255)
}

uint8_t KarKontroller::getThrottleTarget() {
  return throttle_target_;
}

void KarKontroller::setThrottle(uint8_t target) {
  if (state == DRIVE) {
    throttle_target_ = target;
  }
}
  
  // Controls to get and set the brakes.
  // The value is from 0 (off) - 255 (full brake).
  uint8_t getBrake();
  uint8_t getBrakeTarget();
  void setBrake(uint8_t target);
  
  // Controls to set the steering
  // The value is from 0 (full left) - 255 (full right). 127 is dead straight.
  uint8_t getSteering();
  uint8_t getSteeringTarget();
  void setSteering(uint8_t target);
  
  // Controls the gear shift.
  gear_t getGear();
  gear_t getGearTarget
  void setGear(gear_t gear);
  
  // Turns the car on or off. Does nit actually start the car.
  turnOn():
  turnOff();
  boolean isRunning();
  
  // Turns the starter motor on or off.
  void starterOn():
  void starterOff();
  bool isStarterOn();
  
  // Stops the car, puts it in park, and shuts off the engine.
  shutdown();
  
  // Called every loop()
  update();