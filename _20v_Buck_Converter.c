#include "_20v_Buck_Converter.h"

#include "hardware/address_mapped.h"
#include "hardware/structs/pwm.h"
#include "hardware/structs/adc.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/pads_bank0.h"

#include "pico/stdlib.h"
#include <stdio.h>

#include "math.h"

double cell_voltage;
double voltage_difference;
double power;
double saturation_duty;
double saturation_time;
struct Voltages voltage;
struct Currents current;
int adc_gpio[] = {INPUT_TAP,OUTPUT_TAP,CURRENT_TAP};

void init_timers(int duty_cycle) { //setup the timers with a initial duty cycle
	//set IO function to PWM
	iobank0_hw->io[PWM_0].ctrl = GPIO_FUNC_PWM<<IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
  iobank0_hw->io[PWM_1].ctrl = GPIO_FUNC_PWM<<IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
  iobank0_hw->io[PWM_2].ctrl = GPIO_FUNC_PWM<<IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
	//set timer top values
	pwm_hw->slice[0].top = (MAX_DUTY-1)&PWM_CH0_TOP_BITS;
  pwm_hw->slice[1].top = (MAX_DUTY-1)&PWM_CH0_TOP_BITS;
  pwm_hw->slice[2].top = (MAX_DUTY-1)&PWM_CH0_TOP_BITS;
	//set timer cc values
	pwm_hw->slice[0].cc = duty_cycle&PWM_CH0_CC_A_BITS;
  pwm_hw->slice[1].cc = duty_cycle&PWM_CH0_CC_A_BITS;
  pwm_hw->slice[2].cc = duty_cycle&PWM_CH0_CC_A_BITS;
	//set timer starting values
	pwm_hw->slice[0].ctr = 0;
  pwm_hw->slice[1].ctr = ((MAX_DUTY/3)-1)&PWM_CH0_CTR_BITS;
  pwm_hw->slice[2].ctr = ((2*MAX_DUTY/3)-1)&PWM_CH0_CTR_BITS;
	//enable GPIO channels
	pwm_hw->en = 1<<0|1<<1|1<<2;
}

void init_pins() { //setup the pins as inputs or outputs
  //initialize and set digital outputs
  gpio_init(DISABLE_0); gpio_set_dir(DISABLE_0, true);
  gpio_init(DISABLE_1); gpio_set_dir(DISABLE_1, true);
  gpio_init(DISABLE_2); gpio_set_dir(DISABLE_2, true);

  gpio_init(FAN); gpio_set_dir(FAN, true);

  //set adc function and high impedance for analog inputs
  for(int i=0;i<3;i++){
    gpio_set_function(adc_gpio[i],GPIO_FUNC_NULL);
    gpio_disable_pulls(adc_gpio[i]);
    gpio_set_input_enabled(adc_gpio[i],false);
  }
  //set adc output as FIFO
  hw_set_bits(&adc_hw->fcs,ADC_FCS_EN_BITS);
  //enable adc
  hw_set_bits(&adc_hw->cs,ADC_CS_EN_BITS);
}

void init_PID() { //setup the PID loop
  //PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  //Input = read_output_voltage();
  //Setpoint = SET_VOLTAGE;
  //myPID.SetMode(AUTOMATIC);
}

void init_vars() { //setup variables for use
  //populate amp_array
  // for (int i = 0; i < AMP_SMOOTHING; i++) {
	// amp_array[i] = 0.0;
  // }
  voltage.input_voltage = 1.0;
  voltage.output_voltage = 2.0;
  current.output_current = 3.0;
}

void write_pwm(double percent_duty) { //set the duty cycle of all output phases
  if (current.output_current < PHASE_1) {
	output_enable(0b010);
  }
  else if (current.output_current > PHASE_1 && current.output_current < PHASE_2) {
	output_enable(0b101);
  }
  else {
	output_enable(0b111);
  }

  double double_CC_value = ((percent_duty * MAX_DUTY) / 100) - 1; //calculate the compare counter value from the duty
  int CC_value = round(double_CC_value); //round and pass to integer variable
  if (CC_value <= 0) { //limit the compare counter value to 1 and greater
	CC_value = 1;
  }
  if (CC_value > (MAX_DUTY - 1)) { //limit the compare counter value to the max that the register can take
	CC_value = (MAX_DUTY - 1);
  }
  //write new values to buffered counter registers
  pwm_hw->slice[0].cc = CC_value&PWM_CH0_CC_A_BITS;
  pwm_hw->slice[1].cc = CC_value&PWM_CH0_CC_A_BITS;
  pwm_hw->slice[2].cc = CC_value&PWM_CH0_CC_A_BITS;
}

void output_enable(int phases) { //enable or disable the gate drivers
  if ((phases & 0b001) == 0b001) {
	gpio_put(DISABLE_0, false);
  }
  else {
	gpio_put(DISABLE_0, true);
  }

  if ((phases & 0b010) == 0b010) {
	gpio_put(DISABLE_1, false);
  }
  else {
	gpio_put(DISABLE_1, true);
  }

  if ((phases & 0b100) == 0b100) {
	gpio_put(DISABLE_2, false);
  }
  else {
	gpio_put(DISABLE_2, true);
  }
}

void read_analog() { //read and update analog sensor values
  // clear any old FIFO values
  while (!(adc_hw->fcs & ADC_FCS_EMPTY_BITS)) (void) adc_hw->fifo;

  //sample ADC channels and save to FIFO
  for(int i=0; i<3;i++){
    //set ADC channel to sample
    hw_write_masked(&adc_hw->cs, (adc_gpio[i]-26) << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
    //start oneshot sample
    hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
    //wait for READY flag
    while (!(adc_hw->cs & ADC_CS_READY_BITS)) tight_loop_contents();
  }

  //throw away ADC values if 3 samples were not collected
  if (((adc_hw->fcs & ADC_FCS_LEVEL_BITS) >> ADC_FCS_LEVEL_LSB) != 3){
    return;
  }
  
  //scale and save recorded values from FIFO
  voltage.input_voltage = (adc_hw->fifo)/VOLTAGE_SCALE;
  voltage.output_voltage = (adc_hw->fifo)/VOLTAGE_SCALE;
  current.output_current = ((adc_hw->fifo)/CURRENT_SCALE)-CURRENT_OFFSET;
}

double calc_saturation() { //calculate and return the saturation current duty cycle
  saturation_time = SATURATION_CURRENT / voltage.output_voltage; //calculate the saturation time in microseconds
  saturation_duty = (saturation_time * FREQUENCY / 10.0); //calculate the precent duty for that saturation time
  if (saturation_duty > 95) {
	saturation_duty = 95;
  }
  return saturation_duty; //return the saturation duty cycle that will saturate the inductor
}

double calc_saturation_boost() { //calculate and return the saturation current duty cycle
  saturation_time = SATURATION_CURRENT / voltage.output_voltage; //calculate the saturation time in microseconds
  saturation_duty = 100 - (100000/(saturation_time * FREQUENCY)); //calculate the precent duty for that saturation time
  if (saturation_duty < 20) {
	saturation_duty = 20;
  }
  return saturation_duty; //return the saturation duty cycle that will saturate the inductor
}

int undervolt_protect() { //identify undervolt scenario
  cell_voltage = voltage.input_voltage / NUM_CELLS;//identify estimated cell voltage
  if (cell_voltage <= CELL_CUTOUT_VOLTAGE) { //if the estimated cell voltage is low, trigger undervolt protect
	return 1;
  }
  else { //else, show that the protection isn't active
	return 0;
  }
}

int overcurrent_protect() { //identify overcurrent scenario
  return (current.output_current > OVER_CURRENT);
}

void constant_voltage(double duty_limit) { //regulate voltage in normal opperation
  voltage_difference = voltage.output_voltage - SET_VOLTAGE; //calculate the error between the desired and actual value
  if (voltage_difference > 0.0) { //run if the voltage is higher than the target
	  power -= 0.1; //increment the power down
  }
  else if (voltage_difference < 0.0) { //run if the voltage is lower than the target
	  power += 0.1; //increment the power up
  }
  if (power < 0.0) { //limit the duty cycle to 0%
	  power = 0.0;
  }
  else if (power > duty_limit) { //limit the duty cycle to the passed paramenter
	  power = duty_limit;
  }
  // if (not FAST) { //print the power value over serial if fast mode isn't enabled
	// Serial.print("Power: ");
	// Serial.println(power);
  // }
  write_pwm(power); //set the pwm duty cycle to the calculated level
}

void constant_boost(double duty_limit) { //regulate voltage in normal opperation
  voltage_difference = voltage.input_voltage - SET_VOLTAGE; //calculate the error between the desired and actual value
  if (voltage_difference > 0.0) { //run if the voltage is higher than the target
	  power += 0.1;
  }
  else if (voltage_difference < 0.0) { //run if the voltage is lower than the target
	  power -= 0.1;
  }
  if (power < duty_limit) { //limit the duty cycle to saturation time
	  power = duty_limit;
  }
  else if (power > 100.0){ //limit the duty cycle to 100%
    power = 100.0;
  }
  // if (not FAST) { //print the power value over serial if fast mode isn't enabled
	// Serial.print("Power: ");
	// Serial.println(power);
  // }
  write_pwm(power); //set the pwm duty cycle to the calculated level
}

int soft_start(double duty_limit) { //limit output capacitor inrush current on startup
  double voltage_difference = voltage.output_voltage - SET_VOLTAGE; //calculate the error between the desired and actual value
  //voltage_difference = -3.1;
  //double power; //set up power variable
  if (voltage_difference < 0.0) { //run if the voltage is lower than the target
    power += (100.0 / MAX_DUTY); //increment the power up
    if (power > duty_limit) { //limit the duty cycle to the passed paramenter
      power = duty_limit;
      return 0; //quit soft start if it hits the duty cycle limit
    }
    // if (not FAST) { //print the power value over serial if fast mode isn't enabled
    //   Serial.print("Power: ");
    //   Serial.println(power);
    // }
    write_pwm(power); //write the power level to the output
    sleep_us(SOFT_START_RATE);
    return 1; //continue running
  }
  else {
	  return 0; //stop running if the voltage target has been reached
  }
}
