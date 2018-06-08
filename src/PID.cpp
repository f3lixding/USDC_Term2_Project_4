#include "PID.h"
#include <cmath>
#include <iostream>


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

	//sizing twiddle vectors
	p.resize(3);
	dp.resize(3);

	//initialize twiddle values
	dp[0] = 1.0;
	dp[1] = 1.0;
	dp[2] = 1.0;

	iter = 0;
}

void PID::UpdateError(double cte) {
	PID::d_error = cte - p_error;
	PID::p_error = cte;
	PID::i_error += cte;
	
	iter += 1;
}

void PID::Twiddle(double tolerance, double angle, int num) {
	
	//need to minimize steer angle as well as cte.
	double best_cte = p_error;
	double new_cte;
	double rho = p_error / sin(angle);
	
	//error function, which is just total error
	double best_err = std::abs(this->TotalError(Kp, Ki, Kd));
	double err;
	double err_deg;

	//intialize parameters
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;
	int c = 0;
	//main twiddle algorithm
	while (std::abs(dp[num]) > tolerance) {
		c += 1;
		
		p[num] += dp[num];
		err = std::abs(TotalError(p[0], p[1], p[2]));
		err_deg = err * (180 / M_PI);
		new_cte = best_cte + (rho*sin(err_deg)); 
		if ((std::abs(err) < std::abs(best_err)) && (std::abs(new_cte) < std::abs(best_cte))) {
			best_err = err;
			best_cte = new_cte; 
			dp[num] *= 1.1;
		}
		else {
			p[num] -= 2 * dp[num];
			err = std::abs(TotalError(p[0], p[1], p[2]));
			err_deg = err * (180 / M_PI);
			new_cte = best_cte + (rho*sin(err_deg));
			if ((std::abs(err) < std::abs(best_err)) && (std::abs(new_cte) < std::abs(best_cte))) {
				best_err = err;
				best_cte = new_cte;
				dp[num] *= 1.05;
			}
			else {
				p[num] += dp[num];
				dp[num] *= 0.95;
			}
		}



		
	}
	if (c != 0) {
		std::cout << "==================================================twiddle ran " << c << " times==============================================================" << endl;
	}

	Kp = p[0];
	Ki = p[1];
	Kd = p[2];

	// resetting the dp for next iteration

	dp[2] = 1.0;
	/*
	dp[0] = Kp * 0.1;
	dp[1] = Ki * 0.1;
	dp[2] = Kd * 0.1;
	*/
}

double PID::TotalError(double kp, double ki, double kd) {
	return -kp * p_error - ki * i_error - kd * d_error;
}

