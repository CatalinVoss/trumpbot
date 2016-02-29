#include "line_sensor.hpp"

#define LINE_DELTA_THRESH 500 // TODO: set this empirically

line_sensor::line_sensor(int ir1, int ir2, int ir3) {
  _pin1 = ir1;
  _pin2 = ir2;
  _pin3 = ir3;
  _idx = 0;

  for (int i = 0; i < kLineSensorBufferSize; i++) {
    _readings1[i] = 0;
    _readings2[i] = 0;
    _readings3[i] = 0;
  }
}

void line_sensor::read() {
    // We may or may not need this
/*
  // Compute current averages up to int accuracy
  int avg1 = 0;
  for (int i = 0; i< kLineSensorBufferSize; i++) { avg1 += _readings1[i]; }
  avg1 /= kLineSensorBufferSize;
  int avg2 = 0;
  for (int i = 0; i< kLineSensorBufferSize; i++) { avg2 += _readings2[i]; }
  avg2 /= kLineSensorBufferSize;
  int avg3 = 0;
  for (int i = 0; i< kLineSensorBufferSize; i++) { avg3 += _readings3[i]; }
  avg3 /= kLineSensorBufferSize;

  // TODO: do something with these

  // Update queue
  _readings1[_idx] = analogRead(_pin1);
  _readings2[_idx] = analogRead(_pin2);
  _readings3[_idx] = analogRead(_pin3);
  _idx = (_idx+1)%kLineSensorBufferSize; // wrapping index
*/
}

bool line_sensor::is_line(int pin) {
  
  return false;
}