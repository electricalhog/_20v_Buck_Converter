#include "_20v_Buck_Converter.h"

#include "hardware/address_mapped.h"
#include "hardware/regs/pwm.h"

float battery_voltage;
float output_voltage;
float amps;
float amp_array[AMP_SMOOTHING];
float temp_amps;
float cell_voltage;
float voltage_difference;
float power;
float saturation_duty;
float saturation_time;
double Setpoint, Input, Output;


void init_timers(int duty_cycle) { //setup the timers with a initial duty cycle
  /*
  // Set up the generic clock (GCLK6) used to clock timers
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120MHz clock source by divisor 1
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK6
                         //GCLK_GENCTRL_SRC_DFLL;      // Generate from 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;   // Generate from 100MHz DPLL clock source
                         GCLK_GENCTRL_SRC_DPLL0;   // Generate from 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 perhipheral channel
                          GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TCC0, PCHCTRL[25]
  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC3 perhipheral channel
                          GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TCC3, PCHCTRL[29]
  GCLK->PCHCTRL[38].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC4 perhipheral channel
                          GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TCC4, PCHCTRL[38]

  // Enable the peripheral multiplexer on each pin
  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;

  // Set the pin peripheral multiplexer to peripheral correct peripheral
  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg |= PORT_PMUX_PMUXO(5);
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXO(5);

  TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 1
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock

  TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC0->SYNCBUSY.bit.WAVE)                    // Wait for synchronization

    TCC4->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 1
                      TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock

  TCC4->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC4 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC4->SYNCBUSY.bit.WAVE)                    // Wait for synchronization

    TCC3->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 1
                      TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock

  TCC3->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC3 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC3->SYNCBUSY.bit.WAVE)                    // Wait for synchronization

    TCC0->PER.reg = MAX_DUTY - 1;                            // Set-up the PER (period) register 100kHz PWM
  while (TCC0->SYNCBUSY.bit.PER);                    // Wait for synchronization

  TCC4->PER.reg = MAX_DUTY - 1;                            // Set-up the PER (period) register 100kHz PWM
  while (TCC4->SYNCBUSY.bit.PER);                    // Wait for synchronization

  TCC3->PER.reg = MAX_DUTY - 1;                            // Set-up the PER (period) register 100kHz PWM
  while (TCC3->SYNCBUSY.bit.PER);                    // Wait for synchronization

  TCC0->CC[2].reg = duty_cycle;                            // Set-up the CC (counter compare), channel 2 register for 50% duty-cycle
  while (TCC0->SYNCBUSY.bit.CC2);                    // Wait for synchronization

  TCC4->CC[1].reg = duty_cycle;                             // Set-up the CC (counter compare), channel 3 register for 25% duty-cycle
  while (TCC4->SYNCBUSY.bit.CC1);                    // Wait for synchronization

  TCC3->CC[1].reg = duty_cycle;                             // Set-up the CC (counter compare), channel 5 register for 12.5% duty-cycle
  while (TCC3->SYNCBUSY.bit.CC1);                    // Wait for synchronization

  TCC0->COUNT.bit.COUNT = 0;                         //Give each timer a different initial value to create a phase shift
  TCC4->COUNT.bit.COUNT = (MAX_DUTY / 3) - 1;        //Give each timer a different initial value to create a phase shift
  //TCC4->COUNT.bit.COUNT = 0;
  TCC3->COUNT.bit.COUNT = (2 * (MAX_DUTY / 3)) - 1;  //Give each timer a different initial value to create a phase shift
  //TCC3->COUNT.bit.COUNT = 0;

  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
  TCC4->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC4
  TCC3->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC3
  while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
  while (TCC4->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
  while (TCC3->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization

  while (TCC0->SYNCBUSY.bit.CTRLB);
  TCC0->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
  while (TCC0->SYNCBUSY.bit.CTRLB);
  while (TCC0->SYNCBUSY.bit.COUNT);

  while (TCC4->SYNCBUSY.bit.CTRLB);
  TCC4->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
  while (TCC4->SYNCBUSY.bit.CTRLB);
  while (TCC4->SYNCBUSY.bit.COUNT);

  while (TCC3->SYNCBUSY.bit.CTRLB);
  TCC3->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
  while (TCC3->SYNCBUSY.bit.CTRLB);
  while (TCC3->SYNCBUSY.bit.COUNT);
  */

 //set timer prescalers to 1 for 120MHz
 _REG_(PWM_CH0_DIV_OFFSET) 1<<PWM_CH0_DIV_INT_LSB;
 _REG_(PWM_CH1_DIV_OFFSET) 1<<PWM_CH1_DIV_INT_LSB;
 _REG_(PWM_CH2_DIV_OFFSET) 1<<PWM_CH2_DIV_INT_LSB;

 //set timer TOP values
 _REG_(PWM_CH0_TOP_OFFSET) MAX_DUTY;
 _REG_(PWM_CH1_TOP_OFFSET) MAX_DUTY;
 _REG_(PWM_CH2_TOP_OFFSET) MAX_DUTY;

 //set 0 for timer CC values
 _REG_(PWM_CH0_CC_OFFSET) 0<<PWM_CH0_CC_A_LSB;
 _REG_(PWM_CH1_CC_OFFSET) 0<<PWM_CH1_CC_A_LSB;
 _REG_(PWM_CH2_CC_OFFSET) 0<<PWM_CH2_CC_A_LSB;

 //set timer CTR values for 120deg phasing
 _REG_(PWM_CH0_CTR_OFFSET) 0;
 _REG_(PWM_CH1_CTR_OFFSET) (MAX_DUTY/3)-1;
 _REG_(PWM_CH2_CTR_OFFSET) (2*MAX_DUTY/3)-1;

 //set enable bit for PWM channels 0, 1, 2
 _REG_(PWM_EN_OFFSET) //PWM_EN register handles lockstep enabling
  1<<PWM_EN_CH0_LSB|
  1<<PWM_EN_CH1_LSB|
  1<<PWM_EN_CH2_LSB;

}

void init_pins() { //setup the pins as inputs or outputs
  /*
  pinMode(DISABLE_1, OUTPUT);
  pinMode(DISABLE_2, OUTPUT);
  pinMode(DISABLE_3, OUTPUT);

  pinMode(FAN, OUTPUT);

  pinMode(OUTPUT_TAP, INPUT);
  pinMode(BATTERY_TAP, INPUT);
  pinMode(CURRENT_TAP, INPUT);
  */
}

void init_PID() { //setup the PID loop
  //PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  //Input = read_output_voltage();
  //Setpoint = SET_VOLTAGE;
  //myPID.SetMode(AUTOMATIC);
}

void init_vars() { //setup variables for use
  //populate amp_array
  for (int i = 0; i < AMP_SMOOTHING; i++) {
    amp_array[i] = 0.0;
  }
}

void write_pwm(float percent_duty) { //set the duty cycle of all output phases
  /*
  float temp_amps = read_amps();
  if (temp_amps < PHASE_1) {
    output_enable(0b010);
  }
  else if (temp_amps > PHASE_1 && temp_amps < PHASE_2) {
    output_enable(0b101);
  }
  else {
    output_enable(0b111);
  }

  float float_CC_value = ((percent_duty * MAX_DUTY) / 100) - 1; //calculate the compare counter value from the duty
  int CC_value = int(float_CC_value); //round and pass to integer variable
  if (CC_value <= 0) { //limit theps compare counter value to 1 and greater
    CC_value = 1;
  }
  if (CC_value > (MAX_DUTY - 1)) { //limit the compare counter value to the max that the register can take
    CC_value = (MAX_DUTY - 1);
  }
  //write new values to buffered counter registers
  TCC0->CCBUF[2].reg = CC_value;
  TCC4->CCBUF[1].reg = CC_value;
  TCC3->CCBUF[1].reg = CC_value;
  */
}

void output_enable(int phases) { //enable or disable the gate drivers
  /*
  if ((phases & 0b001) == 0b001) {
    digitalWrite(DISABLE_1, LOW);
  }
  else {
    digitalWrite(DISABLE_1, HIGH);
  }

  if ((phases & 0b010) == 0b010) {
    digitalWrite(DISABLE_2, LOW);
  }
  else {
    digitalWrite(DISABLE_2, HIGH);
  }

  if ((phases & 0b100) == 0b100) {
    digitalWrite(DISABLE_3, LOW);
  }
  else {
    digitalWrite(DISABLE_3, HIGH);
  }
  */
}

float read_output_voltage() { //read and return the regulated output voltage
  /*
  battery_voltage = analogRead(BATTERY_TAP) / VOLTAGE_SCALE;//get and scale the battery voltage
  output_voltage = analogRead(OUTPUT_TAP) / VOLTAGE_SCALE;//get and scale the sensor voltage for the output

  //if fast mode isn't enabled, run with print statements
  if (not FAST) {
    Serial.print("Battery voltage: ");
    Serial.print(battery_voltage);
    Serial.print("	Output voltage: ");
    Serial.print(output_voltage);
  }

  return output_voltage;//return the true output voltage
  */return 0.0;
}

float read_amps() { //read the amp draw of the power power output
  /*
  float amps = ((analogRead(CURRENT_TAP) / CURRENT_SCALE) - CURRENT_OFFSET); //scale the input
  for (byte i = 0; i < AMP_SMOOTHING - 1; i++) {
    amp_array[i] = amp_array[i + 1];
  }
  float total = 0.0;
  amp_array[AMP_SMOOTHING - 1] = amps;
  for (byte i = 0; i < AMP_SMOOTHING; i++) {
    if (not FAST) {
      //Serial.print(amp_array[i]);
      //Serial.print(" ");
    }
    total += amp_array[i];
  }
  amps = total / AMP_SMOOTHING;
  if (not FAST) { //print the reading if fast mode isn't enabled
    Serial.print("	Current: ");
    Serial.print(amps);
  }
  return amps;//return the reading
  */return 0.0;
}

float calc_saturation() { //calculate and return the saturation current duty cycle
  /*
  saturation_time = SATURATION_CURRENT / (analogRead(BATTERY_TAP) / VOLTAGE_SCALE); //calculate the saturation time in microseconds
  saturation_duty = 100 * (saturation_time / (1 / (FREQUENCY / 1000))); //calculate the precent duty for that saturation time
  if (saturation_duty > 95) {
    saturation_duty = 95;
  }
  return saturation_duty; //return the saturation duty cycle that will saturate the inductor
  */return 0.0;
}

char undervolt_protect() { //identify undervolt scenario
  /*
  battery_voltage = analogRead(BATTERY_TAP) / VOLTAGE_SCALE;//get and scale  the battery voltage
  cell_voltage = battery_voltage / NUM_CELLS;//identify estimated cell voltage
  if (cell_voltage <= CELL_CUTOUT_VOLTAGE) { //if the estimated cell voltage is low, trigger undervolt protect
    return true;
  }
  else { //else, show that the protection isn't active
    return false;
  }
  */return 0;
}

char overcurrent_protect() { //identify overcurrent scenario
  return (read_amps() > OVER_CURRENT);
}

void constant_voltage(float duty_limit) { //regulate voltage in normal opperation
  //Input = read_output_voltage();
  //myPID.Compute();
  //float output_float = Output/2.55;
  //return output_float;
  voltage_difference = read_output_voltage() - SET_VOLTAGE; //calculate the error between the desired and actual value
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
  /*
  if (not FAST) { //print the power value over serial if fast mode isn't enabled
    Serial.print("Power: ");
    Serial.println(power);
  }
  */
  write_pwm(power); //set the pwm duty cycle to the calculated level
}

char soft_start(float duty_limit) { //limit output capacitor inrush current on startup
  /*
  float voltage_difference = read_output_voltage() - SET_VOLTAGE; //calculate the error between the desired and actual value
  //voltage_difference = -3.1;
  //float power; //set up power variable
  if (voltage_difference < 0.0) { //run if the voltage is lower than the target
    power += (100.0 / MAX_DUTY); //increment the power up
    if (power > duty_limit) { //limit the duty cycle to the passed paramenter
      power = duty_limit;
      return false; //quit soft start if it hits the duty cycle limit
    }
    if (not FAST) { //print the power value over serial if fast mode isn't enabled
      Serial.print("Power: ");
      Serial.println(power);
    }
    write_pwm(power); //write the power level to the output
    int i = 0; //set up increment variable
    //while ((i < SOFT_START_RATE) && ((read_output_voltage()-SET_VOLTAGE) < 0.0)){ //wait some time while the voltage is less than the target
    while ((i < SOFT_START_RATE)) {
      delayMicroseconds(1);
      i++;
    }
    return true; //continue running
  }
  else {
    return false; //stop running if the voltage target has been reached
  }
  */return 0;
}
