#ifndef PID_H
#define PID_H
#include <chrono>
#include <vector>
using namespace std::chrono;

enum TwidleState{ Initial, CheckIncrease, CheckDecrease };

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

	double dKp;
	double dKi;
	double dKd;

	double best_err;
	double current_error;

	std::vector<double*> params;
	std::vector<double*> dParams;

	int twiddleParamIndex;
	TwidleState twiddleState;
	int n;
	int index;
	int best_error_index;
	int total_index;
	bool is_twiddle_on;

  high_resolution_clock::time_point previous;
  bool timeInitialized;
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
  double TotalError();

  void Twiddle();
};

#endif /* PID_H */
