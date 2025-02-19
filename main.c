#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "_20v_Buck_Converter.h"

//#define TEST

void core1_main() {
  gpio_init(ENCODER_A);
  gpio_init(ENCODER_B);
  gpio_set_dir(ENCODER_A, GPIO_IN);
  gpio_set_dir(ENCODER_B, GPIO_IN);
  gpio_pull_up(ENCODER_A);
  gpio_pull_up(ENCODER_B);

  // Enable input hysteresis (hardware debounce)
  gpio_set_input_hysteresis_enabled(ENCODER_A, true);
  gpio_set_input_hysteresis_enabled(ENCODER_B, true);

  // Attach encoder interrupts
  gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
  gpio_set_irq_enabled(ENCODER_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  // Core1 does nothing after setting up interrupts
  while (1) __wfi();  // Wait-for-interrupt (low-power idle)
}

int main() {
  stdio_init_all();
  
  multicore_launch_core1(core1_main);
  queue_init(&encoder_queue, sizeof(double), 1); // Initialize the queue with a size of 1

  #ifdef TEST
  test();
  
  #else
  init_pins();
  output_enable(0b000);
  init_timers(0);
  init_vars();
  //init_PID();
  sleep_ms(5000);
  while(soft_start(calc_saturation())){read_analog();}
  #endif

    while(true) {
      #ifdef TEST
            test_loop();

      #else
            read_analog();
            if (queue_try_remove(&encoder_queue, &voltage.target_voltage))
            {
              printf("Target voltage: %.3f\n", voltage.target_voltage);
            }
            constant_voltage(calc_saturation());

      #endif
    }
}
