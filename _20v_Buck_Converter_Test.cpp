#include "_20v_Buck_Converter.h"

//float myArray[10];

void test(){
  Serial.begin(115200);
  init_pins();
  output_enable(0b000);
  init_timers(0);
  //init_PID();
  delay(5000);
  /*
  for(byte i = 0; i < 10; i++){
    myArray[i]=0.0;
  }
  Serial.println(myArray[0]);
  //output_enable(true);
  test(0b101);
  Serial.println();
  test(0b111);
  Serial.println();
  test(0b001);*/
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
  read_temp();
  Serial.println();
  //write_pwm(20.0);
  

}
