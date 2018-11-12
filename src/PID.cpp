#include "PID.h"
#include <vector>
#include <numeric>
#include <math.h>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error, d_error, i_error = 0.0;

  step = 1;

  n_settle_steps = 100;
  n_eval_steps = 2000;
  total_error = 0;


}

void PID::UpdateError(double cte) {
  if (step == 1) {
       // to get correct initial d_error
       p_error = cte;
   }
   d_error = cte - p_error;
   p_error = cte;
   i_error += cte;

   // update total error only if we're past number of settle steps
   if (step % (n_settle_steps + n_eval_steps) > n_settle_steps){
       total_error += pow(cte,2);
   }

   step++;



}

double PID::TotalError() {
  return 0.0;
}

void PID::AddToParameterAtIndex(int index, double amount) {
    if (index == 0) {
        Kp += amount;
    }
    else if (index == 1) {
        Kd += amount;
    }
    else if (index == 2) {
        Ki += amount;
    }
    else {
        std::cout << "AddToParameterAtIndex: index out of bounds";
    }
}
