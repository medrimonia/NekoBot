#ifndef POSITION_HPP
#define POSITION_HPP

#include <servos.h>

#include "ServoConfig.h"
#include "Utils.hpp"

// N is the number of ServoMotors
template<int N>
class Position{
public:
  double angles[N];

  Position(){
    for (int i = 0; i < N; i++){
      angles[i] = 0;
    }
  }

  void setAntLeftAngles(double * values){
    angles[SERVO_AntLeft1] = values[0];
    angles[SERVO_AntLeft2] = values[1];
  }

  void setAntRightAngles(double * values){
    angles[SERVO_AntRight1] = values[0];
    angles[SERVO_AntRight2] = values[1];
  }

  void setPostLeftAngles(double * values){
    angles[SERVO_PostLeft1] = values[0];
    angles[SERVO_PostLeft2] = values[1];
    angles[SERVO_PostLeft3] = values[2];
  }

  void setPostRightAngles(double * values){
    angles[SERVO_PostRight1] = values[0];
    angles[SERVO_PostRight2] = values[1];
    angles[SERVO_PostRight3] = values[2];
  }

  void apply() const{
    for (int i = 0; i < N; i++){
      servos_command(i, angles[i]);
    }
  }

  /* Return the position corresponding to a crossfade of two position
   */
  static Position crossfadePosition(const Position<N> & src,
                                    const Position<N> & target,
                                    double time,
                                    double startingTime,
                                    double smoothLength){
    Position<N> result;
    double partDone = (time - startingTime) / smoothLength;
    partDone = boundDouble(partDone, 0, 1);
    for (int i = 0; i < NB_SERVOS; i++){
      result.angles[i] = src.angles[i] * (1 - partDone) +
                         target.angles[i] * partDone;
    }
    return result;
  }
};

#endif//POSITION_HPP
