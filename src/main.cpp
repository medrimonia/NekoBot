#include <cstdlib>
#include <wirish/wirish.h>
#include <servos.h>
#include <terminal.h>
#include <main.h>

#include "function.h"
#include "InverseKinematics.hpp"
#include "Position.hpp"
#include "ServoConfig.h"

// TIME PARAMETERS
TERMINAL_PARAMETER_DOUBLE(t, "Time", 0.0);
TERMINAL_PARAMETER_DOUBLE(timeSpeed, "Speed of the time", 1.0);

TERMINAL_PARAMETER_BOOL(freeMove,
                        "Disable all control on servoMotors",
                        true);

// IK PARAMETERS
TERMINAL_PARAMETER_DOUBLE(defaultAntX,
                          "Default x for the anterior legs [mm]",
                          0.0);
TERMINAL_PARAMETER_DOUBLE(defaultAntZ,
                          "Default z for the anterior legs [mm]",
                          150.0);

// WALK PARAMETERS
TERMINAL_PARAMETER_DOUBLE(stepLength, "Length of a step [mm]", 60.0);
TERMINAL_PARAMETER_DOUBLE(stepHeight, "Height of a step [mm]", 30.0);


Position<NB_SERVOS> targetPosition;
Position<NB_SERVOS> lastPosition;
Position<NB_SERVOS> lastStatePosition;
Function walkingX;
Function walkingZ;

void registerServoMotors(){
  servos_register(4, "AntLeft2");
  servos_calibrate(0, 1500, 4650, 7500, false);
  servos_register(5, "AntRight2");
  servos_calibrate(1, 1500, 4400, 7500, true);
  servos_register(8, "AntLeft1");
  servos_calibrate(2, 1500, 4300, 7500, false);
  servos_register(9, "AntRight1");
  servos_calibrate(3, 1500, 4400, 7500, true);
}

#define STEP_DURATION 0.2
void initWalking(){
  // X in the middle, foot in the air
  walkingX.addPoint(0, 0);
  walkingZ.addPoint(0, 1);
  // X forward, foot on ground
  walkingX.addPoint(STEP_DURATION / 2, 1);
  walkingZ.addPoint(STEP_DURATION / 2, 0);
  // X backward, foot on ground
  walkingX.addPoint(1 - STEP_DURATION / 2, -1);
  walkingZ.addPoint(1 - STEP_DURATION / 2, 0);
  // Back to start, in the middle, foot in the air
  walkingX.addPoint(1, 0);
  walkingZ.addPoint(1, 1);
}

/**
 * Vous pouvez écrire du code qui sera exécuté à 
 * l'initialisation ici
 */
void setup(){
  registerServoMotors();
  initWalking();
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

bool isMoving(){
  return true;
}

void move(){
  double time1 = t;
  double time2 = t + 0.5;
  double wishedAntLeft[2];
  double wishedAntRight[2];
  double actualAngles[2] = {0};//TODO, to treat
  computeForeLegIK(wishedAntLeft,
                   actualAngles,
                   defaultAntX + walkingX.getMod(time1) * stepLength / 2,
                   defaultAntZ - walkingZ.getMod(time1) * stepHeight);
  computeForeLegIK(wishedAntRight,
                   actualAngles,
                   defaultAntX + walkingX.getMod(time2) * stepLength / 2,
                   defaultAntZ - walkingZ.getMod(time2) * stepHeight);
  targetPosition.setAntLeftAngles(wishedAntLeft);
  targetPosition.setAntRightAngles(wishedAntRight);
}

/**
 * Foncton appellée à 50hz, c'est ici que vous pouvez mettre
 * à jour les angles moteurs etc.
 */
void tick()
{
    t += 0.02 * timeSpeed; // 20ms
    if (!freeMove){
      if (isMoving()){
        move();
      }
      else{
        testIK();
      }
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
