#ifndef KAR_KONTROLLER_H
#define KAR_KONTROLLER_H

#include "Arduino.h"
#include <LinearActuator.h>

class KarKontroller {
  enum gear_t {
    PARK = 0,
    REVERSE,
    NEUTRAL,
    DRIVE,
    SECOND,
    FIRST,
    NUM_GEARS
  };
  
  struct LinearActuatorConfig {
    uint16_t min_pos;
    uint16_t max_pos;
  };

  class Config {
   public:
    Config();

    LinearActuatorConfig throttle;
    LinearActuatorConfig brake;
    LinearActuatorConfig steering;
    uint16_t gear_pos[gear_t::NUM_GEARS];
  
    //bool readFromEeprom(uint16_t address);
    //void writeToEeprom(uint16_t address);
  
   private:
    //void writeUint16ToEeprom(uint16_t address, uint16_t data);
    //void writeLinearControllerConfigToEeprom(uint16_t address,
    //                                         const LinearActuatorConfig& data);
  };
  
  KarKontroller();
  
  // Sets the config
  void setConfig(const Config& config);
  
  // Members that get and set the car's physical controls.
  // In general, there are 3 functions for each control:
  //   getXXX() - returns the CURRENT value of the control
  //   getXXXTarget() - returns the TARGET value of the contol. This is value
  //                    that the controller is in the process of moving
  //                    towards. If this is the same as getXXX(), the
  //                    the controller is not moving.
  //  setXXX() - sets the TARGET value 
  
  // Controls to get and set the throttle.
  // The value is from 0 (off) - 255 (full throttle).
  uint8_t getThrottle();
  uint8_t getThrottleTarget();
  void setThrottle(uint8_t target);
  
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

 private:
 
  enum state_t {
    OFF,
    START,
    DRIVE,
    SHIFT,
    SHUTDOWN
  }
  
  // The current state and the last time (in ms) that the state changed.
  state_t state_;
  uint32_t last_state_change_;
  
  Config config_;
  
  uint8_t throttle_target_;
  uint8_t brake_target_;
  uint8_t steering_target_;
  gear_t gear_;
}

#endif