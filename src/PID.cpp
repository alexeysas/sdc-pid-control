#include "PID.h"
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std;

typedef std::chrono::duration<float> fsec;
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
	this->Kp = Kp;
	this->Kd = Kd;
	this->Ki = Ki;

	// initilize parameters deltas (for twiddle)
	this->dKp = Kp / 10;
	this->dKi = Ki / 10;
	this->dKd = Kd / 10;

	// initialize errors
	d_error = 0;
	p_error = 0;
	i_error = 0;

	// combine parameters to array for twiddle
	params = { &this->Kp , &this->Kd, &this->Ki };
	dParams = { &this->dKp , &this->dKd, &this->dKi };
	
	// initialize twiddle
	twiddleParamIndex = 0;
	twiddleState = Initial;
	n = 100;
	index = 0;
	current_error = 0;
	best_err = -1;
	best_error_index = 0;
	total_index = 0;
	is_twiddle_on = false;

	timeInitialized = false;
}

void PID::UpdateError(double cte) {

	auto now = std::chrono::high_resolution_clock::now();

	// fist we need to initialize time parameters to be sure that differential time dependent 
	// part of the controller has time and previous values initialized
	if (!timeInitialized)
	{
		previous = now;
		timeInitialized = true;
		p_error = cte;
		return;
	}
	
	// calculate time delta between telemetry snapshots
	fsec fs = now - previous;
	double delta = fs.count();

	// calculate moving average error (used for twiddle)
	if (index >= n) {
		current_error += cte * cte / n;
	}

	// if twiddle is on - use it
	if (index == 2 * n - 1 && is_twiddle_on) {
		Twiddle();
		cout << "Params: " << this->Kp << "  " << this->Kd << "  " << this->Ki << endl;
		cout << "dParams: " << this->dKp << "  " << this->dKd << "  " << this->dKi << endl;
		cout << "Current Error: " << current_error << " Best Error: " << best_err << endl;
	}

	std::cout << delta << std::endl;
	// calculate proportianal, diferential and integral errors
	//d_error = (cte - p_error) / delta;
	
	d_error = (cte - p_error);
	p_error = cte;
	i_error += cte;
		
	// reset moving average error (used for twiddle)
	if (index == 2 * n - 1) {
		index = 0;
		current_error = 0;
		total_index += 2 * n;

		cout << "total index: " << total_index << "best_error_index = " << best_error_index << endl;

		// as we do not have a straight line - error might be biased to some specific curvature - so we can reset best error from time to time to make sure that we are constantly tuning our parameters  
		if (best_error_index < total_index - 20 * n) {
			best_err = -1;
			best_error_index = total_index;
		}
	}

	index += 1;
	previous = now;
}

double PID::TotalError() {

	return Kp*p_error + Kd*d_error + Ki*i_error;
}


void PID::Twiddle() {
	
	// force to tune only specific parameter
	// twiddleParamIndex = 2;

	double* param = params[twiddleParamIndex];
	double* dParam = dParams[twiddleParamIndex];

	if (best_err == -1) {
		cout << current_error << endl;
		best_err = current_error;
	}

	switch (twiddleState)
	{
		case Initial:
			*param = *param + *dParam;
			twiddleState = CheckIncrease;
			break;
		case CheckIncrease:
			if (current_error < best_err) {
				best_err = current_error;
				*dParam = *dParam * 1.05;
				twiddleState = Initial;
				twiddleParamIndex = (twiddleParamIndex + 1) % params.size();
			}
			else {
				*param = *param - 2 * (*dParam);
				twiddleState = CheckDecrease;
			}
			break;
		case CheckDecrease:
			if (current_error < best_err) {
				best_err = current_error;
				*dParam = *dParam * 1.05;
			}
			else {
				*param = *param + *dParam;
				*dParam = *dParam * 0.95;
			}
			twiddleState = Initial;
			twiddleParamIndex = (twiddleParamIndex + 1) % params.size();
			break;
		default:
			break;
	}

}
