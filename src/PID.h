#ifndef PID_H
#define PID_H

class PID {

public:

  /*
   * Constructor
   */
  PID();

  /*
   * Destructor.
   */
  virtual ~PID() = default;

  /*
   * Initialize PID.
   */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Get current steering angle
   */
  double SteeringAngle() {
    return steer_;
  }

  /*
   * Calculate the total PID error.
   */
  double TotalError();

private:

  /*
   * Error
   */
  double error_;

  /*
   * Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;

  /*
   * Steering angle
   */
  double steer_;

  /*
   * Variables for computation of
   * cross-track error derivative and sum
   */
  double cte_last_;
  double cte_d_;
  double cte_sum_;
  bool first_call_;

};

#endif /* PID_H */
