#include "Arduino.h"

#include <Sabertooth.h>

#ifndef LinearActuator_h
#define LinearActuator_h

#define LA_HISTORY_LEN 10
#define LA_RAW_POS_MIN 140
#define LA_RAW_POS_MAX 990

#define LA_DEBUG 1

class LinearActuator {
 public:
  LinearActuator(Sabertooth* driver, uint8_t motor, uint8_t pos_pin);

  // Returns the position of the actuator [0...1023], where 1023 is completely
  // extended.
  uint16_t getPosition();
  void goToPosition(uint16_t target_position);

  void setPIDParams(double proportional_gain, 
                    double integral_gain,
                    double derivative_gain);

  // Updates the current position and move towards the target position if
  // applicable. This should be called once a loop.
  void update();

 private:
  void normalizePosition(); 

  // Specifies the motor to control
  Sabertooth* _driver;
  uint8_t _motor;

  // We use a running average of the position to get rid of error.
  uint8_t _pos_pin;
  uint16_t _pos_history[LA_HISTORY_LEN];
  uint8_t _next_pos_index;
  uint16_t _position;

  // PID parameters
  uint16_t _target_position;
  double _p_gain;
  double _i_gain;
  double _d_gain;
  double _previous_error;
  double _error_integral;
  double _previous_output;
  
  uint32_t _last_update;
};

#endif