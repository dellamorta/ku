#include "KarKontroller.h"

#define STARTER_TIMEOUT 5000
#define SHIFTER_TOLERANCE 5

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

KarKontroller::KarKontroller(LinearActuator* throttle,
                             LinearActuator* brake,
                             LinearActuator* shifter,
                             LinearActuator* steering,
                             uint8_t ignition_pin,
                             uint8_t starter_pin,
                             const Config& konfig)
                             : throttle_(throttle),
                               brake_(brake),
                               shifter_(shifter),
                               steering_(steering),
                               ignition_pin_(ignition_pin),
                               starter_pin_(starter_pin),
                               config_(konfig) {}

void KarKontroller::updateState(state_t state) {
  if (state_ != state) {
    state_ = state;
    last_state_update_ = now();
  }
}

// Helper functions for linear actuators.
uint8_t KarKontroller::getLinearActuatorPos(const LinearActuatorConfig& config,
                                            LinearActuator* actuator) {
  uint8_t value = constrain(actuator->getPosition(),
                            config.min, config.max);
  return map(value, config.min, config.max, 0, 255);
}

// Map |target| to the range of |config.min| and |config.max|, then set that
// as |actuator|'s target depth.
void KarKontroller::setLinearActuatorTarget(uint8_t target,
                                            const LinearActuatorConfig& config,
                                            LinearActuator* actuator) {
  actuator->goToPosition(map(target, 0, 255, config.min, config.max));                            
}

KarKontroller::setConfig(const Config& config) {
  config_ = config;
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

bool KarKontroller::isInGear(gear_t gear) {
  uint8_t value = shifter_.getPosition();
  if (value < config_.gear_pos[gear] + SHIFTER_TOLERANCE &&
      value > config_.gear_pos[gear] - SHIFTER_TOLERANCE) {
    return true;
  }
  
  return false;
}

gear_t KarKontroller::getGearTarget() {
  return gear_target_;
}

// Shift gears.
// If you are currently driving, set the state to SHIFTING, set the target
// gear, and begin decelerating.
void KarKontroller::setGear(gear_t target) {
  if (state_ == DRIVE || state_ == SHIFTING) {
    updateState(SHIFTING);
    gear_target_ = target;
    brake_target_ = 255;
    setLinearActuatorTarget(255, config.brake, &brake);
  }
}
  
// Turns the car on or off. Does not actually start the car.
void KarKontroller::turnOn() {
  if (state_ == OFF) {
    updateState(ON);
  }
  digitalWrite(ignition_pin_, HIGH)
}

void KarController::turnOff() {
  if (state_ != OFF) {
    updateState(OFF);
  }
  digitalWrite(ignition_pin_, LOW);
}

bool KarKontroller::isRunning() {
  return state_ != OFF;
}
  
// Turns the starter motor on. Only valid if the car is turned on but not
// already started.
void KarKontroller::starterOn() {
  if (state_ == ON) {
    updateState(START);
    digitalWrite(starter_pin_, HIGH);
  }
}

// Turns the starter motor off, but only if it is currently on.
void KarKontroller::starterOff() {
  if (state_ == START) {
    updateState(DRIVE);
    digitalWrite(starter_pin_, LOW)
  }
}

bool KarKontroller::isStarterOn() {
  return state_ == START;
}
  
// Stops the car, puts it in park, and shuts off the engine.
void KarKontroller::shutdown() {
  updateState(SHUTDOWN);
  gear_target_ = PARK;
  setLinearActuatorTarget(255, config.brake, &brake);
}
  
// Called every loop()
void KarKontroller::update() {

  // Update all the linear actuator positions.
  brake.update();
  throttle.update();
  steering.update();
  shifter.update();
  
  if (gps->newNMEAreceived()) {
    gps->parse(gpd->lastNMEA());
  }  
  
  // Do some special processing for individual states.
  // If you are in the shutdown state, decelerate until you're stopped and then
  // shift to park and turn off the engine.
  if (state_ == SHUTDOWN) {
    //if (gps_.speed < 0.25) {
      if (isInGear(PARK)) {
        digitalWrite(ignition_pin_, LOW);
        updateState(OFF);
      } else {
        shifter_.goToPosition(config_.gear_pos[PARK]);
      }
    //}
  } 
   
  // If you're in the SHIFT state, decelerate until you're stopped and then
  // shift to the new gear.
  } else if (state_ == SHIFT) {
    if (gps_.speed < 0.25) {
      if (isInGear(gear_target_)) {
        updateState(DRIVE);
      } else {
        shifter_.goToPosition(config_.gear_pos[gear_target_]);
      }
    }
    
  // Make sure you don't run the starter for too long by accident.
  } else if ((state_ == START) &&
             (last_state_update_ - now() > STARTER_TIMEOUT) {
    updateState(ON);
    digitalWrite(starter_pin_, OFF);
  }
}
