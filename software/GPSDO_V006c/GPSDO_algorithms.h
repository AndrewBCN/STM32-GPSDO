#ifndef GPSDO_ALGORITHMS_H
  #define GPSDO_ALGORITHMS_H
  #include <Arduino.h>
  // algorithm selector function
  uint16_t adjustVctlPWM(uint16_t previous_PWM_output, uint32_t timer, uint8_t algorithm_no);
  // up to 10 different control loop algorithms, in order from 0 to 9
  // 0 - primitive, very simple control loop
  uint16_t primitive_ctl_loop(uint16_t adjusted_PWM_output, uint32_t lclppscount);
  // 1 - not a control loop, we force the OCXO to drift slowly and regularly
  uint16_t forced_drift_Vctl(uint16_t adjusted_PWM_output, uint32_t lclppscount);
  // 2 - not a control loop, we force the OCXO to "jitter" randomly
  uint16_t random_walk_Vctl(uint16_t adjusted_PWM_output, uint32_t lclppscount);
  // 3 - FLL PID control loop, coefficients set manually
  // 4 - PLL PI (not PID) control loop, coefficients set manually (similar to Lars')
  // 5 - PLL PID control loop, coefficients set manually
  // 6 - FLL PID control loop, genetic algorithm used to find near-optimal coefficients
  // 7 - PLL PID control loop, genetic algorithm used to find near-optimal coefficients
  // 8 - FLL + PLL "hybrid" PID control loop, weights and coefficients set manually
  // 9 - Neural network MLP driven control loop
  
  // global variables from main program
  extern char trendstr[5];
  extern volatile bool cotho_full;
  extern volatile double oavgftho;
  extern volatile double oavgfhun;
#endif
