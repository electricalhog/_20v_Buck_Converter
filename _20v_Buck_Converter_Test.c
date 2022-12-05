#include "_20v_Buck_Converter.h"
#include <stdio.h>
#include "pico/stdlib.h"
//float myArray[10];

void test(){
  init_pins();
  init_vars();
  output_enable(0b000);
  //init_timers(50);
  //init_PID();
  sleep_ms(5000);
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN,true);
  gpio_put(PICO_DEFAULT_LED_PIN,true);

  init_timers(0);
  stdio_init_all();
}

void test_loop() {
  read_analog();
  printf("Vin %f ", voltage.input_voltage);
  printf(" Vout %f ", voltage.output_voltage);
  printf(" Amp %f\n", current.output_current);
  sleep_ms(1000);
  // float i;
  // for(i=0.0;i<=100.0;i=i+0.1){
  //   write_pwm(i);
  //   sleep_ms(10);
  // }
  // for(i=100.0;i>=0.0;i=i-0.1){
  //   write_pwm(i);
  //   sleep_ms(10);
  // }

}
