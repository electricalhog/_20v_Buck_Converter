#include <_20v_Buck_Converter.h>

void test(int binary){
  if((binary&0b001) == 0b001){
    Serial.print(1);
  }
  else{
    Serial.print(0);
  }
  if((binary&0b010) == 0b010){
    Serial.print(1);
  }
  else{
    Serial.print(0);
  }
  if((binary&0b100) == 0b100){
    Serial.print(1);
  }
  else{
    Serial.print(0);
  }
}

void setup() {
  Serial.begin(115200);
  init_pins();
  output_enable(0b010);
  init_timers(0);
  //init_PID();
  delay(5000);
  //output_enable(true);
  /*test(0b101);
  Serial.println();
  test(0b111);
  Serial.println();
  test(0b001);*/
}

void loop() {
  //while(soft_start(calc_saturation()));
  //while(soft_start(20.0));
  //constant_voltage(calc_saturation());
  //read_output_voltage();
  //Serial.println();
  write_pwm(20.0);
  
}
