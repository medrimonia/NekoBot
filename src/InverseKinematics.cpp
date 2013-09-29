#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "InverseKinematics.hpp"
#include "Utils.hpp"

#define HUMERUS_LENGTH  77.2
#define RADIUS_LENGTH   100.8

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
 */
void computeIK(double * dst,
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
          dst[solutionIndex * 2    ] = RAD2DEG(alpha);
          dst[solutionIndex * 2 + 1] = RAD2DEG(beta);
          solutionIndex++;
        }
      }
    }
  }
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
  computeIK(solutions, x, z, HUMERUS_LENGTH, RADIUS_LENGTH);
  double solutionBestScore = 3000;
  double solutionChoosen = -1;
  for (int solutionNo = 0; solutionNo < 2; solutionNo++){
    double alpha = solutions[solutionNo * 2];
    double beta  = solutions[solutionNo * 2 + 1];
    if (alpha <  90 &&
        alpha > -90 &&
        beta  <  90 &&
        beta  > -90){
      double score = abs(alpha - foreLegActualAngles[0]);
      score += abs(beta - foreLegActualAngles[1]);
      if (score < solutionBestScore){
        solutionChoosen = solutionNo;
        solutionBestScore = score;
        foreLegComputedAngles[0] = alpha;
        foreLegComputedAngles[1] = beta;
      }
    }
  }
  if (solutionChoosen == -1){
    return -1;
  }
}

/* Compute the fore leg angles corresponding to the asked x and z,
 * Place the best solution (closest to the last known) in the given array.
 * Return -1 if IK did not give any possible result.
 */
int computeRearLegIK(double * foreLegComputedAngles,
                     double * foreLegActualAngles,
                     double x,
                     double z){
  //TODO
}
