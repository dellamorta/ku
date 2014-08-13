#include "KarKontroller.h"

Config::Config() {
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

KarKontroller::KarKontroller() {}

KarKontroller::setConfig(const Config& config) {
  config_ = config;
}

// Helper functions for linear actuators.
uint8_t KarKontroller::getLinearActuatorPos(const LinearActuatorConfig& config,
                                            LinearActuator* actuator) {
  uint8_t value = constrain(actuator->getPosition(),
                            config.min, config.max);
  return map(value, config.min, config.max, 0, 255);
}
  
void KarKontroller::setLinearActuatorTarget(uint8_t target,
                                            const LinearActuatorConfig& config,
                                            LinearActuator* actuator) {
  actuator->goToPosition(map(target, 0, 255, config.min, config.max));                            
}


// Throttle functions.
uint8_t KarKontroller::getThrottle() {
  return getLinearActuatorPos(config_.throttle, &throttle_);
}

uint8_t KarKontroller::getThrottleTarget() {
  return throttle_target_;
}

void KarKontroller::setThrottle(uint8_t target) {
  // If we're in drive, use whatever value is passed in. If we're not in drive,
  // use the lower of the existing or new throttle value (slow down)/
  if (state == DRIVE || target < throttle_target_) {
    throttle_target_ = target;
    setLinearActuatorTarget(target, config.throttle, &throttle);
  }
}

// Brake functions.
uint8_t KarKontroller::getBrake() {
  return getLinearActuatorPos(config_.brake, &brake_);
}

uint8_t KarKontroller::getBrakeTarget() {
  return brake_target_;
}

void KarKontroller::setBrake(uint8_t target) {
  // If we're in drive, use whatever value is passed in. If we're not in drive,
  // use the higher of the existing or new brake value (slow down).
  if (state == DRIVE || (target > brake_target_) {
    brake_target_ = target;
    setLinearActuatorTarget(target, config.brake, &brake);
  }
}


// Steering functions.
uint8_t KarKontroller::getSteering() {
  return getLinearActuatorPos(config_.steering, &steering_);
}

uint8_t KarKontroller::getSteeringTarget() {
  return steering_target_;
}

void KarKontroller::setSteering(uint8_t target) {
  // You should always be able to turn the wheels.
  steering_target_ = target;
  setLinearActuatorTarget(target, config.steering, &steering);
}


// Controls the gear shift.
gear_t KarKontroller::getGear() {
  uint8_t value = shifter_.getPosition();
  if (value < PARK) {
    return PARK;
  } else if (value > FIRST) {
    return FIRST;
  }
  // For each set of adjacent gear, find the midpoint between them. Then
  // check to see if the shifter is between the gear set point and the
  // midpoint. If it is in that range, return the equivalent gear.
  for (uint8_t i = 0; i < NUM_GEARS - 1; i++) {
    uint16_t min = config_.gear_pos[i];
    uint16_t max = config_.gear_pos[i+1];
    uint16_t mid_point = min + ((max - min) / 2);
    if (value > min && value <= mid_point) {
      return i;
    } else if (value < max && value > mid_point) {
      return i + 1;
    }
  }
  
  return FIRST;
}

gear_t KarKontroller::getGearTarget() {
  return gear_target_
}

void setGear(gear_t target) {
  if (state_ == DRIVE || state_ == SHIFTING) {
    gear_target_ = target;
    state_ = SHIFTING;
    if (state_ != SHIFTING) {
      last_state_update_ = now();
    }
  }
}
  
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