#include <iostream>
#include "PID.h"
#include <math.h>

/**
 * The PID class
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  counter = 0;

  dp[0] = Kp;
  dp[1] = Ki;
  dp[2] = Kd;

  tolerance = 0.2;
  //tw_index = 0;
  twiddle_step = 1;
  best_error = 100.0;
  epoch_sum_error = 0.0;
}

void PID::UpdateError(double cte)
{
  prev_cte = p_error;
  p_error = cte;

  i_error += cte;
  d_error = cte - prev_cte;

  counter++;

  if (total_error <= -1 && total_error >= 1)
  {
    i_error += cte;
  }
}

double PID::TotalError()
{
  double err = Kp * p_error + Ki * i_error / counter + Kd * d_error;
  epoch_sum_error += fabs(err);
  return err;
}

double PID::Normalize(double cte)
{
  if (cte > 1)
    cte = 1;
  if (cte < -1)
    cte = -1;
  return cte;
}

int PID::GetCounter()
{
  return counter;
}

void PID::Twiddle(int index, int steps, double total_err)
{

  if (counter % steps == 0)
  {

    if (counter % 600 == 0)
      best_error = 100;

    double p[3];
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;

    //double err = fabs(total_err);
    double err = epoch_sum_error / 50;
    epoch_sum_error = 0.0;
    double sum_dp = dp[0] + dp[1] + dp[2];

    std::cout << "Tunung PID Coefficients "
              << " Counter: " << counter << " Average Err: " << err << " best_error " << best_error << std::endl;
    std::cout << "Twiddle_step " << twiddle_step << " Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;

    if (sum_dp > tolerance)
    {

      switch (twiddle_step)
      {
      case 1:

        p[index] += dp[index];
        twiddle_step = 2;
        std::cout << "Step 1 "
                  << " dp[index]: " << dp[index] << std::endl;
        break;

      case 2:

        if (err < best_error)
        {
          best_error = err;
          dp[index] *= 1.1;
          std::cout << "Step 2/1 "
                    << " dp[index]: " << dp[index] << std::endl;
        }
        else
        {
          p[index] -= 2 * dp[index];
          std::cout << "Step 2/2 "
                    << " dp[index]: " << dp[index] << std::endl;
        }
        twiddle_step = 3;

        break;

      case 3:
        twiddle_step = 1;
        if (err < best_error)
        {
          best_error = err;
          dp[index] *= 1.1;

          std::cout << "Step 3/1 "
                    << " dp[index]: " << dp[index] << std::endl;
        }
        else
        {
          p[index] += dp[index];
          dp[index] *= 0.9;
          std::cout << "Step 3/2 "
                    << " dp[index]: " << dp[index] << std::endl;
        }
        break;
      }
    }

    Kp = p[0];
    Ki = p[1];
    Kd = p[2];

    std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << " best_error " << best_error << std::endl;
    std::cout << "Next twiddle step " << twiddle_step << std::endl;
    std::cout << "  " << std::endl;
  }
}