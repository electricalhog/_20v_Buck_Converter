#ifndef _20V_BUCK_CONVERTER_H
#define _20V_BUCK_CONVERTER_H

//define pins
#define PWM_0 0
#define PWM_1 2
#define PWM_2 4
#define DISABLE_0 1
#define DISABLE_1 3
#define DISABLE_2 5
#define FAN 6
#define INPUT_TAP 26
#define OUTPUT_TAP 27
#define CURRENT_TAP 28


//define constants
#define FREQUENCY 250 //frequency in kHz of the switching
#define MAX_DUTY (120000/FREQUENCY) //what the PER reg should be set to for proper frequency
#define CELL_CUTOUT_VOLTAGE 3.5 //the buck convert will shut down output if estimated cell voltage drops to this level or lower; starts at 3.5
#define SET_VOLTAGE 20.0 //voltage out
#define SET_CURRENT 50 //constant current limit
#define FAST true //run without print statements
#define OVER_CURRENT 80 //over current threshold before the output shuts down
#define NUM_CELLS 8 //the number of series lipo cells on the input
#define VOLTAGE_SCALE 31.7 //the scale for the voltage dividers
#define CURRENT_SCALE 9.77 //the scale for the current measuring
#define CURRENT_OFFSET 2.2 //the offset for the current measuring
#define SATURATION_CURRENT 45 //amps considered to saturate the inductor
#define SOFT_START_RATE 1250 //high values give slower soft start
#define AMP_SMOOTHING 10 //higher values average more

#define PHASE_1 5.0 //Output amps before switching to 2 phase operation
#define PHASE_2 10.0 //Output amps before switching to 3 phase operation

//create global variabless
extern double cell_voltage;
extern double voltage_difference;
extern double power;
extern double saturation_duty;
extern double saturation_time;

extern struct Voltages {
    double input_voltage;
    double output_voltage;
}voltage;

extern struct Currents {
    double output_current;
}current;

//PID TUNING PARAMETERS!!!!!!!!!!!!!!!!!!!!!
//extern double Kp = 12;
//extern double Ki = 9;
//extern double Kd = 1;

//extern PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//define functions
void init_timers(int duty_cycle);
void init_pins();
void init_PID();
void init_vars();
void begin_PID();
void write_pwm(double percent_duty);
void output_enable(int phases);
void read_analog();
double calc_saturation();
int undervolt_protect();
int overcurrent_protect();
void constant_voltage(double duty_limit);
int soft_start(double duty_limit);

//test functions
void test();
void test_loop();

#endif
