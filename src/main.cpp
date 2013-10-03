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
                        false);

// IK PARAMETERS
TERMINAL_PARAMETER_DOUBLE(defaultAntX,
                          "Default x for the anterior legs [mm]",
                          0.0);
TERMINAL_PARAMETER_DOUBLE(defaultAntZ,
                          "Default z for the anterior legs [mm]",
                          150.0);

// WALK PARAMETERS
// Classic StepLength is 6
TERMINAL_PARAMETER_DOUBLE(stepLength, "Length of a step [mm]",  0.0);
TERMINAL_PARAMETER_DOUBLE(stepHeight, "Height of a step [mm]", 30.0);

// DEBUG VALUES
TERMINAL_PARAMETER_DOUBLE(failedAntX, "", 0.0);
TERMINAL_PARAMETER_DOUBLE(failedAntZ, "", 0.0);



Position<NB_SERVOS> targetPosition;
Position<NB_SERVOS> lastPosition;
Position<NB_SERVOS> lastStatePosition;
Function walkingX;
Function walkingZ;

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
  double actualForeAngles[2];
  actualForeAngles[0] = lastPosition.angles[SERVO_AntLeft1];
  actualForeAngles[1] = lastPosition.angles[SERVO_AntLeft2];
  double wishedForeAngles[2];
  int r = computeForeLegIK(wishedForeAngles,
                           actualForeAngles,
                           defaultAntX,
                           defaultAntZ);
  double actualRearAngles[3];
  actualRearAngles[0] = lastPosition.angles[SERVO_PostLeft1];
  actualRearAngles[1] = lastPosition.angles[SERVO_PostLeft2];
  actualRearAngles[2] = lastPosition.angles[SERVO_PostLeft3];
  double wishedRearAngles[3];
  int r2 = computeRearLegIK(wishedRearAngles,
                            actualRearAngles,
                            defaultAntX,
                            defaultAntZ);
  //TODO if (r == -1 || r2 == -1)
  targetPosition.setAntLeftAngles(wishedForeAngles);
  targetPosition.setAntRightAngles(wishedForeAngles);
  targetPosition.setPostLeftAngles(wishedRearAngles);
  targetPosition.setPostRightAngles(wishedRearAngles);
}

bool isMoving(){
  return stepLength > 1;
}

void move(){
  double time1 = t;
  double time2 = t + 0.5;
  double wishedAntLeft[2];
  double wishedAntRight[2];
  double actualAngles[2] = {0};//TODO, to treat
  int r1, r2;
  double leftAntX = defaultAntX + walkingX.getMod(time1) * stepLength / 2;
  double leftAntZ = defaultAntZ - walkingZ.getMod(time1) * stepHeight;
  double rightAntX = defaultAntX + walkingX.getMod(time2) * stepLength / 2;
  double rightAntZ = defaultAntZ - walkingZ.getMod(time2) * stepHeight;
  r1 = computeForeLegIK(wishedAntLeft,
                        actualAngles,
                        leftAntX,
                        leftAntZ);
  r2 = computeForeLegIK(wishedAntRight,
                        actualAngles,
                        rightAntX,
                        rightAntZ);
  //If one of the IK failed, 
  if (r1 == -1 || r2 == -1){
    if (r1 == -1){
      failedAntX = leftAntX;
      failedAntZ = leftAntZ;
    }
    if (r2 == -1){
      failedAntX = rightAntX;
      failedAntZ = rightAntZ;
    }
    freeMove = true;
    return;
  }
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
