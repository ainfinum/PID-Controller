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

  double GetCounter();

  double Twiddle(double cte);

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
  int tw_index;
  double best_error;
  double dp;

#endif // PID_H