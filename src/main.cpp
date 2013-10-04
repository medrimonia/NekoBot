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
TERMINAL_PARAMETER_DOUBLE(defaultX,
                          "Default x for legs [mm]",
                          0.0);
TERMINAL_PARAMETER_DOUBLE(defaultZ,
                          "Default z for legs [mm]",
                          150.0);

// WALK PARAMETERS
TERMINAL_PARAMETER_DOUBLE(stepLength, "Length of a step [mm]", 60.0);
TERMINAL_PARAMETER_DOUBLE(stepHeight, "Height of a step [mm]", 30.0);

// SHOOT PARAMETERS
/* -1 -> shooting with left leg
 *  0 -> not shooting
 *  1 -> shooting with right leg
 */
TERMINAL_PARAMETER_INT(shooting, "Is the robot shooting and from which?", 0);
TERMINAL_PARAMETER_DOUBLE(shootPreparationTime,
                          "Time of preparation for shoot",
                          0.7);
TERMINAL_PARAMETER_DOUBLE(shootingTime, "Time of shoot", 0.3);
TERMINAL_PARAMETER_DOUBLE(preparationDeltaX,
                          "Delta x for shoot preparation",
                          -20);
TERMINAL_PARAMETER_DOUBLE(shootDeltaX, "Delta x for shoot", 60);
TERMINAL_PARAMETER_DOUBLE(shootDeltaZ, "Delta z for shoo", -20);

// HIGH LEVEL ORDERS
TERMINAL_PARAMETER_DOUBLE(forwardOrder, "percent of forward Order", 0.0);
TERMINAL_PARAMETER_DOUBLE(rotationOrder, "percent of rotation Order", 0.0);


// STATE VALUES
TERMINAL_PARAMETER_INT(state,
                       "Current state",
                       -1);
TERMINAL_PARAMETER_DOUBLE(stateStartingTime,
                          "Last State Change",
                          0.0);

// DEBUG VALUES
TERMINAL_PARAMETER_DOUBLE(failedAntX, "", -1000.0);
TERMINAL_PARAMETER_DOUBLE(failedAntZ, "", -1000.0);
TERMINAL_PARAMETER_DOUBLE(failedPostX, "", -1000.0);
TERMINAL_PARAMETER_DOUBLE(failedPostZ, "", -1000.0);
TERMINAL_PARAMETER_INT(failedLeg, "", -1);
TERMINAL_PARAMETER_DOUBLE(leftStep, "", -1000.0);
TERMINAL_PARAMETER_DOUBLE(rightStep, "", -1000.0);

/******************************************************************************/
/*                          STATE HANDLING                                    */
/******************************************************************************/

#define STATE_FREE_MOVE 0
#define STATE_UNKNOWN   1
#define STATE_MOVING    2
#define STATE_SHOOTING  3

void updateState(){
  int newState = STATE_UNKNOWN;
  if (freeMove){
    newState = STATE_FREE_MOVE;
  }
  else if(abs(forwardOrder) + abs(rotationOrder) > 0.05){
    newState = STATE_MOVING;
  }
  else if(shooting != 0){
    newState = STATE_SHOOTING;
  }
  if (newState != state){
    stateStartingTime = t;
    state = newState;
  }
};

/******************************************************************************/
/*                           PARAMETERS GETTERS                               */
/******************************************************************************/

double getMoveNormalizer(){
  double totalOrder = abs(forwardOrder) + abs(rotationOrder);
  if (totalOrder < 1){
    return 1;
  }
  return totalOrder;
}

double getFwdOrder(){
  if (abs(forwardOrder) > 1){
    forwardOrder /= abs(forwardOrder);
  }
  return forwardOrder / getMoveNormalizer();
}

double getRotOrder(){
  if (abs(rotationOrder) > 1){
    rotationOrder /= abs(forwardOrder);
  }
  return rotationOrder / getMoveNormalizer();
}

/******************************************************************************/
/*                            GLOBAL_VARIABLES                                */
/******************************************************************************/

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
                           defaultX,
                           defaultZ);
  double actualRearAngles[3];
  actualRearAngles[0] = lastPosition.angles[SERVO_PostLeft1];
  actualRearAngles[1] = lastPosition.angles[SERVO_PostLeft2];
  actualRearAngles[2] = lastPosition.angles[SERVO_PostLeft3];
  double wishedRearAngles[3];
  int r2 = computeRearLegIK(wishedRearAngles,
                            actualRearAngles,
                            defaultX,
                            defaultZ);
  //TODO if (r == -1 || r2 == -1)
  targetPosition.setAntLeftAngles(wishedForeAngles);
  targetPosition.setAntRightAngles(wishedForeAngles);
  targetPosition.setPostLeftAngles(wishedRearAngles);
  targetPosition.setPostRightAngles(wishedRearAngles);
}

bool isMoving(){
  return (abs(getFwdOrder()) + abs(getRotOrder())) > 0.05;
}

void shoot(){
  double shootingLegX, shootingLegZ;
  double elapsedTime = t - stateStartingTime;
  // Shoot is Finished, change shooting and return
  if (elapsedTime > shootPreparationTime + shootingTime){
    shooting = 0;
    return;
  }
  double partDone;
  if (elapsedTime < shootPreparationTime){
    partDone = elapsedTime / shootPreparationTime;
    shootingLegX = defaultX + preparationDeltaX * partDone;
    shootingLegZ = defaultZ + shootDeltaZ;
  }
  //TODO this part doesn't seem to work well
  else{
    partDone = (elapsedTime - shootPreparationTime) / shootingTime;
    shootingLegX = crossfadedValue(defaultX + preparationDeltaX,
                                   defaultX + shootDeltaX,
                                   partDone);
    shootingLegZ = defaultZ;
  }
  double actualShootingLeg[2] = {0};//TODO, to treat
  double wishedShootingLeg[2] = {0};
  int r = computeForeLegIK(wishedShootingLeg,
                           actualShootingLeg,
                           shootingLegX,
                           shootingLegZ);
  if (r == -1){
    failedAntX = shootingLegX;
    failedAntZ = shootingLegZ;
    if (shooting == -1){
      failedLeg = 1;
    }
    else{
      failedLeg = 2;
    }
  }
  if (shooting == -1){
    targetPosition.setAntLeftAngles(wishedShootingLeg);
  }
  if (shooting == 1){
    targetPosition.setAntRightAngles(wishedShootingLeg);
  }
}

void move(){
  double time1 = t;
  double time2 = t + 0.5;
  //double leftStep, rightStep;
  double wishedAntLeft[2];
  double wishedAntRight[2];
  double wishedPostLeft[3];
  double wishedPostRight[3];
  double actualForeAngles[2] = {0};//TODO, to treat
  double actualRearAngles[3] = {0};//TODO, to treat
  double leftAntX, rightAntX, leftPostX, rightPostX;
  double leftAntZ, rightAntZ, leftPostZ, rightPostZ;
  // each leg has it's own value
  leftStep  = stepLength * (getFwdOrder() + getRotOrder());
  rightStep = stepLength * (getFwdOrder() - getRotOrder());
  // Leg positions
  leftAntX   = defaultX + walkingX.getMod(time1) * leftStep / 2.0;
  leftAntZ   = defaultZ - walkingZ.getMod(time1) * stepHeight;
  rightAntX  = defaultX + walkingX.getMod(time2) * rightStep / 2.0;
  rightAntZ  = defaultZ - walkingZ.getMod(time2) * stepHeight;
  leftPostX  = defaultX + walkingX.getMod(time2) * leftStep / 2.0;
  leftPostZ  = defaultZ - walkingZ.getMod(time2) * stepHeight;
  rightPostX = defaultX + walkingX.getMod(time1) * rightStep / 2.0;
  rightPostZ = defaultZ - walkingZ.getMod(time1) * stepHeight;
  // Computing IK
  int r1, r2, r3, r4;
  r1 = computeForeLegIK(wishedAntLeft,
                        actualForeAngles,
                        leftAntX,
                        leftAntZ);
  r2 = computeForeLegIK(wishedAntRight,
                        actualForeAngles,
                        rightAntX,
                        rightAntZ);
  // Inverting side for rear legs
  r3 = computeRearLegIK(wishedPostRight,
                        actualRearAngles,
                        rightPostX,
                        rightPostZ);
  r4 = computeRearLegIK(wishedPostLeft,
                        actualRearAngles,
                        leftPostX,
                        leftPostZ);
  //If one of the IK failed, 
  if (r1 == -1 ||
      r2 == -1 ||
      r3 == -1 ||
      r4 == -1){
    if (r1 == -1){
      failedAntX = leftAntX;
      failedAntZ = leftAntZ;
      failedLeg = 1;
    }
    if (r2 == -1){
      failedAntX = rightAntX;
      failedAntZ = rightAntZ;
      failedLeg = 2;
    }
    if (r4 == -1){
      failedPostX = leftPostX;
      failedPostZ = leftPostZ;
      failedLeg = 4;
    }
    if (r3 == -1){
      failedPostX = rightPostX;
      failedPostZ = rightPostZ;
      failedLeg = 3;
    }
    freeMove = true;
    return;
  }
  targetPosition.setAntLeftAngles(wishedAntLeft);
  targetPosition.setAntRightAngles(wishedAntRight);
  targetPosition.setPostLeftAngles(wishedPostLeft);
  targetPosition.setPostRightAngles(wishedPostRight);
}

/**
 * Foncton appellée à 50hz, c'est ici que vous pouvez mettre
 * à jour les angles moteurs etc.
 */
void tick()
{
    t += 0.02 * timeSpeed; // 20ms
    updateState();
    switch(state){
    case STATE_UNKNOWN: testIK(); break;
    case STATE_MOVING: move(); break;
    case STATE_SHOOTING: shoot(); break;
    }
    if (state != STATE_FREE_MOVE){
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
