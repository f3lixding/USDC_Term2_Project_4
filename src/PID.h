#ifndef PID_H
#define PID_H
#include <vector>
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;


  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle variables
  */
  std::vector <double> p;
  std::vector <double> dp;
  int iter;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double kp, double ki, double kd);

  /*
  * Twiddle
  */
  void Twiddle(double tolerance, double angle, int num);
};

#endif /* PID_H */
