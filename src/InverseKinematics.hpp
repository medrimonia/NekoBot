#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

/* Compute an inverse kinematic for the fore leg according to the specified x
 * and z.
 * Result is placed in the first given array and if multiple solutions are
 * possible, the closest solution to the actual position is returned.
 */
int computeForeLegIK(double * foreLegComputedAngles,
                     double * foreLegActualAngles,
                     double x,
                     double z);

/* Compute an inverse kinematic for the rear leg according to the specified x
 * and z.
 * Result is placed in the first given array and if multiple solutions are
 * possible, the closest solution to the actual position is returned.
 */
int computeRearLegIK(double * rearLegComputedAngles,
                     double * rearLegActualAngles,
                     double x,
                     double z);
#endif//INVERSE_KINEMATICS_HPP
