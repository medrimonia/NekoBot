#include <servos.h>

#include "ServoConfig.h"

void registerServoMotors(){
  servos_register( 4, (char*)"AntLeft2");
  servos_calibrate(0, 1500, 4650, 6240, false);
  servos_register( 5, (char*)"AntRight2");
  servos_calibrate(1, 2900, 4400, 7500, true);
  servos_register( 8, (char*)"AntLeft1");
  servos_calibrate(2, 1500, 4300, 7500, false);
  servos_register( 9, (char*)"AntRight1");
  servos_calibrate(3, 1500, 4400, 7500, true);
  servos_register(11, (char*)"PostLeft1");
  servos_calibrate(4, 1800, 4800, 7350, false);
  servos_register(27, (char*)"PostRight1");
  servos_calibrate(5, 1800, 4300, 7250, true);
  servos_register(26, (char*)"PostLeft2");
  servos_calibrate(6, 2350, 4670, 7250, false);
  servos_register(25, (char*)"PostRight2");
  servos_calibrate(7, 1800, 4270, 6600, true);
  servos_register(16, (char*)"PostLeft3");
  servos_calibrate(8, 1900, 4600, 6350, false);
  servos_register(15, (char*)"PostRight3");
  servos_calibrate(9, 2750, 4600, 7250, true);
}
