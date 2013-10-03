#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "InverseKinematics.hpp"
#include "Utils.hpp"

//#define DEBUG_IK

#define HUMERUS_LENGTH   77.2
#define RADIUS_LENGTH   100.8

//TODO check
#define FEMUR_LENGTH     55
#define TIBIA_LENGTH     77

//TODO check
#define REAR_FOOT_DX    30.0
#define REAR_FOOT_DZ    40.0
#define REAR_FOOT_ANGLE 25.0
#define PELVIS_ANGLE    25.0

#define PI 3.14159265

#define RAD2DEG(x) (x) * 180.0 / PI

/*******************************************************************************
 * In all this file, x and z will be used to describe the position of a part
 * in function of another, the usual notation is : 
 * Distances are all specified in mm
 * x : horizontal distance
 * x+ : forward
 * x- : backward
 * z  : vertical distance (always positive)
 *****************************************************************************/

// Return the angle between 180 and -180
double normalizeAngle(double angle){
  if (angle > 180.0){
    return angle - 360;
  }
  if (angle < -180.0){
    return angle + 360;
  }
  return angle;
}

bool
isValidIKSolution(double alpha,
                  double beta,
                  double l1,
                  double l2,
                  double wantedX,
                  double wantedZ){
  double x = sin(alpha) * l1 + sin(alpha + beta) * l2;
  double z = cos(alpha) * l1 + cos(alpha + beta) * l2;
  return isZero(x - wantedX) && isZero(z - wantedZ);
}

/* compute two solutions to a two degrees of freedom inverse kinematic and
 * place the results in the dst array given as parameter.
 * Result will be placed in dst as following
 * dst[0] -> Humerus angle solution 1
 * dst[1] -> Radius  angle solution 1
 * dst[2] -> Humerus angle solution 2
 * dst[3] -> Radius  angle solution 2
 * return the number of solutions found
 */
int computeIK(double * dst,
              double x,
              double z,
              double l1,
              double l2){
  double possibleBeta[2];
  double possibleAlpha1[2];
  double possibleAlpha2[2];
  int solutionIndex = 0;
  double d = sqrt(x * x + z * z);
  possibleBeta[0]   = PI - acos((l1 * l1 + l2 * l2 - d * d) / (2 * l1 * l2));
  possibleAlpha1[0] = acos((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d));
  possibleAlpha2[0] = acos(z / d);
  // cos can produce + or -
  possibleBeta[1] = -possibleBeta[0];
  possibleAlpha1[1] = -possibleAlpha1[0];
  possibleAlpha2[1] = -possibleAlpha2[0];
  // Find the two possible solutions
  for (int a1 = 0; a1 < 2; a1++){
    for (int a2 = 0; a2 < 2; a2++){
      for (int b = 0; b < 2; b++){
        double alpha = possibleAlpha1[a1] + possibleAlpha2[a2];
        double beta = possibleBeta[b];
        if (isValidIKSolution(alpha, beta, l1, l2, x, z)){
          //Avoiding duplicated Solutions
          bool duplicatedSolution = false;
          for (int i = 0; i < solutionIndex; i++){
            if (isZero(dst[i * 2] - alpha) &&
                isZero(dst[i * 2 + 1] - beta)){
              duplicatedSolution = true;
            }
          }
          if (!duplicatedSolution){
            dst[solutionIndex * 2    ] = alpha;
            dst[solutionIndex * 2 + 1] = beta;
            solutionIndex++;
          }
        }
      }
    }
  }
  // Going to degrees
  for (int i = 0; i < 2 * solutionIndex; i++){
    dst[i] = RAD2DEG(dst[i]);
  }
  return solutionIndex;
}

/* Compute the fore leg angles corresponding to the asked x and z,
 * Place the best solution (closest to the last known) in the given array.
 * Return -1 if IK did not give any possible result.
 */
int computeForeLegIK(double * foreLegComputedAngles,
                     double * foreLegActualAngles,
                     double x,
                     double z){
  double solutions[4];
  int nbSolutions = computeIK(solutions,
                              x,
                              z,
                              HUMERUS_LENGTH,
                              RADIUS_LENGTH);
#ifdef DEBUG_IK
  printf("There's %d solutions before scoring\n", nbSolutions);
#endif
  double solutionBestScore = 0;
  double solutionChoosen = -1;
  for (int solutionNo = 0; solutionNo < nbSolutions; solutionNo++){
    double alpha = normalizeAngle(solutions[solutionNo * 2]);
    double beta  = normalizeAngle(solutions[solutionNo * 2 + 1]);
#ifdef DEBUG_IK
    printf("Solution %d:\n", solutionNo);
    printf("\tAlpha : %f\n", alpha);
    printf("\tBeta  : %f\n", beta);
#endif
    if (alpha <  90 &&
        alpha > -90 &&
        beta  <  160 &&
        beta  >  0){
      double score = 1000;
      score -= abs(alpha - foreLegActualAngles[0]);
      score -= abs(beta - foreLegActualAngles[1]);
      if (alpha > 0) score -= 400;
#ifdef DEBUG_IK
      printf("\tScore : %f\n", score);
#endif
      if (score > solutionBestScore){
        solutionChoosen = solutionNo;
        solutionBestScore = score;
        foreLegComputedAngles[0] = alpha;
        foreLegComputedAngles[1] = beta - 90;
      }
    }
  }
  if (solutionChoosen == -1){
    return -1;
  }
  return 0;
}

/* Compute the fore leg angles corresponding to the asked x and z,
 * Place the best solution (closest to the last known) in the given array.
 * Return -1 if IK did not give any possible result.
 */
int computeRearLegIK(double * rearLegComputedAngles,
                     double * rearLegActualAngles,
                     double x,
                     double z){
  double solutions[4];
  x = x - REAR_FOOT_DX;
  z = z - REAR_FOOT_DZ;
  int nbSolutions = computeIK(solutions,
                              x,
                              z,
                              FEMUR_LENGTH,
                              TIBIA_LENGTH);
#ifdef DEBUG_IK
  printf("There's %d solutions before scoring\n", nbSolutions);
  printf("Reminder: wanted = {%lf,%lf}\n", x, z);
#endif
  double solutionBestScore = 0;
  double solutionChoosen = -1;
  for (int solutionNo = 0; solutionNo < nbSolutions; solutionNo++){
    //computing third angles
    double alpha = normalizeAngle(solutions[solutionNo * 2]);
    double beta  = normalizeAngle(solutions[solutionNo * 2 + 1]);
    double gamma = normalizeAngle(REAR_FOOT_ANGLE - 90 - alpha - beta);
    // Adjusting angles
    alpha -= PELVIS_ANGLE;
    beta += 90;
#ifdef DEBUG_IK
    printf("Solution %d:\n" , solutionNo);
    printf("\tAlpha : %f\n" , alpha);
    printf("\tBeta  : %f\n" , beta);
    printf("\tGamma  : %f\n", gamma);
#endif
    if (alpha <  90 &&
        alpha > -90 &&
        beta  <  90 &&
        beta  > -75 &&
        gamma <  51 &&
        gamma > -90){
      double score = 1000;
      score -= abs(alpha - rearLegActualAngles[0]);
      score -= abs(beta  - rearLegActualAngles[1]);
      score -= abs(gamma - rearLegActualAngles[2]);
      if (alpha < 0) score -= 400;
#ifdef DEBUG_IK
      printf("\tScore : %f\n", score);
#endif
      if (score > solutionBestScore){
        solutionChoosen = solutionNo;
        solutionBestScore = score;
        rearLegComputedAngles[0] = alpha;
        rearLegComputedAngles[1] = beta;
        rearLegComputedAngles[2] = gamma;
      }
    }
  }
  if (solutionChoosen == -1){
    return -1;
  }
  return 0;
}

#ifdef DEBUG_IK
int main(int argc, char ** argv){
  double actualForeAngles[2] = {0,0};
  double computedForeAngles[2];
  computeForeLegIK(computedForeAngles,
                   actualForeAngles,
                   0,
                   150);
  double actualRearAngles[3] = {0};
  double computedRearAngles[3];
  computeRearLegIK(computedRearAngles,
                   actualRearAngles,
                   0,
                   150);
}
#endif
