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

	this->dKp = Kp / 10;
	this->dKi = Ki / 10;
	this->dKd = Kd / 10;

	d_error = 0;
	p_error = 0;
	i_error = 0;

	params = { &this->Kp , &this->Kd, &this->Ki };
	dParams = { &this->dKp , &this->dKd, &this->dKi };
	twiddleParamIndex = 0;
	twiddleState = Initial;
	n = 300;
	index = 0;
	current_error = 0;
	best_err = -1;
	timeInitialized = false;
	best_error_index = 0;
	total_index = 0;
}

void PID::UpdateError(double cte) {

	auto now = std::chrono::high_resolution_clock::now();

	if (!timeInitialized)
	{
		previous = now;
		timeInitialized = true;
		p_error = cte;
		return;
	}

	fsec fs = now - previous;
	double secs = fs.count();

	if (index >= n) {
		current_error += cte * cte;
	}

	//if (dKp + dKi + dKd > 0.2 && index == 2 * n - 1) {
	if (index == 2 * n - 1) {
		Twiddle();
		cout << "Params: " << this->Kp << "  " << this->Kd << "  " << this->Ki << endl;
		cout << "dParams: " << this->dKp << "  " << this->dKd << "  " << this->dKi << endl;
		cout << "Current Error: " << current_error << " Best Error: " << best_err << endl;
	}

	d_error = (cte - p_error) / secs;
	p_error = cte;
	i_error += cte;
		
	if (index == 2 * n - 1) {
		index = 0;
		current_error = 0;
		total_index += 2 * n;

		cout << "total index: " << total_index << "best_error_index = " << best_error_index << endl;

		if (best_error_index < total_index - 20 * n) {
			best_err = -1;
			best_error_index = total_index;
			cout << "dsdsdsd";
		}
	}

	index += 1;
	previous = now;
}

double PID::TotalError() {

	return Kp*p_error + Kd*d_error + Ki*i_error;
}


void PID::Twiddle() {

	twiddleParamIndex = 0;

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
				*dParam = *dParam * 1.1;
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
				*dParam = *dParam * 1.1;
			}
			else {
				*param = *param + *dParam;
				*dParam = *dParam * 0.9;
			}
			twiddleState = Initial;
			twiddleParamIndex = (twiddleParamIndex + 1) % params.size();
			break;
		default:
			break;
	}

}