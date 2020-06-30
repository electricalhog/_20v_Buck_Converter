#include <_20v_Buck_Converter.h>

void setup() {
  Serial.begin(115200);
  init_pins();
  output_enable(0b000);
  init_timers(0);
  //init_PID();
  delay(5000);
  while(soft_start(calc_saturation()));
}

void loop() {
  write_pwm(constant_voltage(calc_saturation()));
  //Serial.println(read_amps());
  //read_output_voltage();
  
}
