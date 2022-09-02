#include "_20v_Buck_Converter.h"

#define TEST

void setup() {
  #ifdef TEST
  test();
  
  #else
  Serial.begin(115200);
  init_pins();
  output_enable(0b000);
  init_timers(0);
  init_vars();
  //init_PID();
  delay(5000);
  while(soft_start(calc_saturation()));
  #endif
}

void loop() {
  #ifdef TEST
  test_loop();

  #else
  constant_voltage(calc_saturation());
  digitalWrite(FAN, HIGH);
  #endif
}
