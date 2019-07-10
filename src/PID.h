#ifndef PID_H
#define PID_H

class PID
{
public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  // Normalize cte values [-1, 1]
  double Normalize(double cte);

  int GetCounter();

  void Twiddle(int tw_index, int steps, double total_err);

private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double cte;
  double prev_cte;
  double total_error;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  int counter;

  double tolerance;
  //int tw_index;
  double best_error;
  double epoch_sum_error;
  double dp[3];
  int twiddle_step;
};
#endif // PID_H