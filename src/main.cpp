#include <cstdlib>
#include <wirish/wirish.h>
#include <servos.h>
#include <terminal.h>
#include <main.h>
#include "InverseKinematics.hpp"
#include "Position.hpp"
#include "ServoConfig.h"

//Defining ServoMotors
#define SERVO_AntLeft2 0
#define SERVO_AntRight2 1
#define SERVO_AntLeft1 2
#define SERVO_AntRight1 3

// TIME PARAMETERS
TERMINAL_PARAMETER_DOUBLE(t, "Time", 0.0);
TERMINAL_PARAMETER_DOUBLE(timeSpeed, "Speed of the time", 1.0);

TERMINAL_PARAMETER_BOOL(freeMove,
                        "Disable all control on servoMotors",
                        false);

// IK PARAMETERS
TERMINAL_PARAMETER_DOUBLE(defaultAntX,
                          "Default x for the anterior legs (mm)",
                          0.0);
TERMINAL_PARAMETER_DOUBLE(defaultAntZ,
                          "Default z for the anterior legs (mm)",
                          150.0);


Position<NB_SERVOS> targetPosition;
Position<NB_SERVOS> lastPosition;
Position<NB_SERVOS> lastStatePosition;

void registerServoMotors(){
  servos_register(4, "AntLeft2");
  servos_calibrate(0, 1500, 4450, 7500, false);
  servos_register(5, "AntRight2");
  servos_calibrate(1, 1500, 4450, 7500, true);
  servos_register(8, "AntLeft1");
  servos_calibrate(2, 1500, 4450, 7500, false);
  servos_register(9, "AntRight1");
  servos_calibrate(3, 1500, 4450, 7500, true);
}

/**
 * Vous pouvez écrire du code qui sera exécuté à 
 * l'initialisation ici
 */
void setup(){
  registerServoMotors();
}

void testIK(){
  double actualAngles[2];
  actualAngles[0] = lastPosition.angles[SERVO_AntLeft1];
  actualAngles[1] = lastPosition.angles[SERVO_AntLeft2];
  double wishedAngles[2];
  int r = computeForeLegIK(wishedAngles,
                           actualAngles,
                           defaultAntX,
                           defaultAntZ);
  //TODO if r == -1
  targetPosition.setAntLeftAngles(wishedAngles);
  targetPosition.setAntRightAngles(wishedAngles);
}

/**
 * Foncton appellée à 50hz, c'est ici que vous pouvez mettre
 * à jour les angles moteurs etc.
 */
void tick()
{
    t += 0.02; // 20ms
    if (!freeMove){
      testIK();
      targetPosition.apply();
    }
}

/**
 * Si vous souhaitez écrire ici du code, cette fonction sera
 * apellée en boucle
 */
void loop()
{
    // Pour toucher aux moteurs, utilisez plutôt
    // la fonction tick()
}
