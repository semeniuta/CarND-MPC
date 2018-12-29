#include "PID.h"

PID::PID() {

  Init(0, 0, 0);

}

void PID::Init(double Kp, double Ki, double Kd) {

  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  error_ = 0.0;
  steer_ = 0.0;

  first_call_ = true;
  cte_d_ = 0.0;
  cte_sum_ = 0.0;
  cte_last_ = 0.0;

}

void PID::UpdateError(double cte) {

  cte_sum_ += cte;

  if (!first_call_) {

    cte_d_ = cte - cte_last_;
    cte_last_ = cte;

  } else {

    // cte_d_ will remain 0.
    first_call_ = false;
  }

  steer_ = -Kp_ * cte - Kd_ * cte_d_ - Ki_ * cte_sum_;

  error_ += (cte * cte);

}

double PID::TotalError() {

  return error_;

}

