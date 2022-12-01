#include <stdio.h>
#include "pico/stdlib.h"

#include "_20v_Buck_Converter.h"

//#define TEST

int main() {
  #ifdef TEST
  test();
  
  #else
  init_pins();
  output_enable(0b000);
  init_timers(0);
  init_vars();
  //init_PID();
  sleep_ms(5000);
  while(soft_start(calc_saturation()));
  #endif

    while(true) {
        #ifdef TEST
        test_loop();

        #else
        constant_voltage(calc_saturation());
        
        #endif
    }
}
