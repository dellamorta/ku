#include "LinearActuator.h"

LinearActuator::LinearActuator(Sabertooth* driver, 
                               uint8_t motor, uint8_t pos_pin) {
  _pos_pin = pos_pin;
  _next_pos_index = 0;
  _driver = driver;
  _motor = motor;

  // Set the initial position
  int position = analogRead(_pos_pin);
  for (int i = 0; i < LA_HISTORY_LEN; i++) {
    _pos_history[i] = position;
  }
  _position = position;
  normalizePosition();

  // Set the intial target for where we currently are
  _target_position = _position;

  // setup the default PID parameters.
  //_p_gain = 1270.0;
  //_i_gain = 0.0;
  //_d_gain = 0.0;

  _p_gain = 571.5;
  _previous_error = 30.5;
  _error_integral = 7937.5;

  _last_update = millis();
}

uint16_t LinearActuator::getPosition() {
  return _position;
}

void LinearActuator::goToPosition(uint16_t target_position) {
  _target_position = constrain(target_position,
                               LA_RAW_POS_MIN, LA_RAW_POS_MAX);
}

void LinearActuator::setPIDParams(double proportional_gain,
                                  double integral_gain,
                                  double derivative_gain) {
  _p_gain = proportional_gain;
  _i_gain = integral_gain;
  _d_gain = derivative_gain;
}

void LinearActuator::update() {
  // Write the current position in the circular buffer
  _pos_history[_next_pos_index] = analogRead(_pos_pin);
  _next_pos_index = (_next_pos_index + 1) % LA_HISTORY_LEN;

  // Average the circular buffer for the actual position.
  uint32_t total = 0;
  for (uint8_t i = 0; i < LA_HISTORY_LEN; i++) {
	total += _pos_history[i];
  }
  _position = total / LA_HISTORY_LEN;
  normalizePosition();

  // Now that we have a position, execute the PID loop.
  // We only run the PID loop every 10ms at the fastest.
  uint32_t delta_t = millis() - _last_update;
  if (delta_t > 100) {
    double error = ((double) _target_position - (double) _position) / LA_RAW_POS_MAX;
    Serial.print("error: ");
    Serial.println(error);

    _error_integral = _error_integral + (error * delta_t);
    double derivative = (error - _previous_error) / delta_t;

    double output = _p_gain * error +
                    _i_gain * _error_integral +
                    _d_gain * derivative;
    Serial.print("output: ");
    Serial.println(output);
    output = constrain(output, -127.0, 127.0);
    _driver->motor(_motor, (int8_t) output);
    _previous_error = error;
    _previous_output = output;
    _last_update = millis();

    Serial.print("Pos: ");
    Serial.print(_position);
    Serial.print(" Target Pos: ");
    Serial.print(_target_position);
    Serial.print(" Speed: ");
    Serial.print(output);
    Serial.println("");
  }
}

void LinearActuator::normalizePosition() {
  _position = constrain(_position, LA_RAW_POS_MIN, LA_RAW_POS_MAX);
  _position = map(_position, LA_RAW_POS_MIN, LA_RAW_POS_MAX, 1023, 0);
}