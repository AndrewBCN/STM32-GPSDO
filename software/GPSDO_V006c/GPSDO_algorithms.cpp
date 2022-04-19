/*  STM32 GPSDO Control Loop Algorithms 
 *  
 */

#include "GPSDO_algorithms.h"
// ---------------------------------------------------------------------------------------------
//    Control loop algorithm selector
// ---------------------------------------------------------------------------------------------
uint16_t adjustVctlPWM(uint16_t previous_PWM_output, uint32_t timer, uint8_t algorithm_no) {
  uint16_t return_PWM_output = previous_PWM_output;

  switch(algorithm_no) {
    case 0:
      return_PWM_output = primitive_ctl_loop(previous_PWM_output, timer);
      break;
      
    case 1:
      return_PWM_output = forced_drift_Vctl(previous_PWM_output, timer);
      break;
      
    case 2:
      return_PWM_output = random_walk_Vctl(previous_PWM_output, timer);
      break;

    // add new control algorithms here and below, and in header file
      
    default:
      return_PWM_output = primitive_ctl_loop(previous_PWM_output, timer);
  }
  return (return_PWM_output);
}
// ---------------------------------------------------------------------------------------------
//    Very primitive Adjust Vctl PWM routine
// ---------------------------------------------------------------------------------------------
uint16_t primitive_ctl_loop(uint16_t adjusted_PWM_output, uint32_t lclppscount) {
// This should reach a stable PWM output value / a stable 10000000.00 frequency
// after an hour or so, and 10000000.000 after eight hours or so

  uint16_t new_PWM_output = adjusted_PWM_output;
  const uint32_t update_periodicity = 429; // only calculate new value every 429s
  
  if ((lclppscount % update_periodicity) == 0) {
    // check first if we have the data, then do ultrafine and very fine frequency
    // adjustment, when we are very close
    // ultimately the objective is 10000000.000 over the last 1000s (16min40s)
    if ((cotho_full) && (oavgftho >= 9999999.990) && (oavgftho <= 10000000.010)) {
     
      // decrease frequency; 1000s based
      if (oavgftho >= 10000000.001) {
        if (oavgftho >= 10000000.005) {
          // decrease PWM by 5 bits = very fine
          new_PWM_output = adjusted_PWM_output - 5;
        strcpy(trendstr, " vf-");
          }
      else {
          // decrease PWM by one bit = ultrafine
          new_PWM_output = adjusted_PWM_output - 1;
        strcpy(trendstr, " uf-");
          }
      }
      // or increase frequency; 1000s based
      else if (oavgftho <= 9999999.999) {
        if (oavgftho <= 9999999.995) {
         // increase PWM by 5 bits = very fine
          new_PWM_output = adjusted_PWM_output + 5;     
        strcpy(trendstr, " vf+");
          }
      else {
          // increase PWM by one bit = ultrafine
          new_PWM_output = adjusted_PWM_output + 1;
        strcpy(trendstr, " uf+");
        }
      }
    }
    ///// next check the 100s values in second place because we are too far off
    // decrease frequency; 100s based
    else if (oavgfhun >= 10000000.01) {
      if (oavgfhun >= 10000000.10) {
        // decrease PWM by 100 bits = coarse
        new_PWM_output = adjusted_PWM_output - 100;
      strcpy(trendstr, " c- ");
        }
      else {
        // decrease PWM by ten bits = fine
        new_PWM_output = adjusted_PWM_output - 10;
      strcpy(trendstr, " f- ");
        }
    }
    // or increase frequency; 100s based
    else if (oavgfhun <= 9999999.99) {
      if (oavgfhun <= 9999999.90) {
       // increase PWM by 100 bits = coarse
        new_PWM_output = adjusted_PWM_output + 100;     
      strcpy(trendstr, " c+ ");
      }
    else {
      // increase PWM by ten bits = fine
        new_PWM_output = adjusted_PWM_output + 10;
        strcpy(trendstr, " f+ ");
      }
    }
    else {
      // here we keep PWM DAC setting, because measured frequency is exactly 10000000.000MHz
      strcpy(trendstr, " hit");
    }
  }
  return(new_PWM_output); // return newly computed value for PWM DAC
} // end adjustVctlPWM

// ---------------------------------------------------------------------------------------------
//    Forced drift : increases PWM DAC by 1 bit every 1000 seconds
// ---------------------------------------------------------------------------------------------
uint16_t forced_drift_Vctl(uint16_t adjusted_PWM_output, uint32_t lclppscount) {

  uint16_t new_PWM_output = adjusted_PWM_output;
  const uint32_t update_periodicity = 1000; // only calculate new value every 1000s
  
  if ((lclppscount % update_periodicity) == 0) {
    new_PWM_output = adjusted_PWM_output + 1;
  }
  return(new_PWM_output); // return newly computed value for PWM DAC
} // end forced_drift

// ---------------------------------------------------------------------------------------------
//    Random walk : adds -1, 0, +1 with equal probabilities to PWM DAC every 5 seconds
// ---------------------------------------------------------------------------------------------
uint16_t random_walk_Vctl(uint16_t adjusted_PWM_output, uint32_t lclppscount) {

  uint16_t new_PWM_output = adjusted_PWM_output;
  const uint32_t update_periodicity = 5; // only calculate new value every 5s
  
  if ((lclppscount % update_periodicity) == 0) {
    new_PWM_output = adjusted_PWM_output + random(-1,2);
  }
  return(new_PWM_output); // return newly computed value for PWM DAC
} // end random_walk
