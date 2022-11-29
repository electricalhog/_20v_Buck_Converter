#include "_20v_Buck_Converter.h"
#include <stdio.h>
#include "pico/stdlib.h"
//float myArray[10];

void test(){
  init_pins();
  output_enable(0b000);
  //init_timers(50);
  //init_PID();
  sleep_ms(5000);
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN,true);
  gpio_put(PICO_DEFAULT_LED_PIN,true);

  //Test PWM Implementation

}

void test_loop() {
  /*
  for(byte i = 0; i < 9; i++){
    myArray[i]=myArray[i+1];
  }
  myArray[9]=read_amps();

  float total = 0.0;

  for(int i = 0; i < 10; i++){
    Serial.print(myArray[i]);
    Serial.print(" ");
    total += myArray[i];
  }
  float average = total/10;
  Serial.println();
  Serial.println(average);
  Serial.println();
  delay(5);
  */
  //while(soft_start(calc_saturation()));
  //while(soft_start(20.0));
  //constant_voltage(calc_saturation());
  //read_output_voltage();
  //Serial.println();
  write_pwm(20.0);
  

}
