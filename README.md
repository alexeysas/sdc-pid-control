# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Goal

The goal of the project is to build a PID controller and tune the PID hyperparameters so vehicle is able to drive one lap in the simulator.

## Reflection

PID algorithm implemented consists of three different components: P, I, D.

* P - proportional part of the controller
  This is main part of the controller. The idea is simple  -  we need to set steering angle proporionaly to the Cross Track Error - CTE difference between reference trajectory and current trajectory.  So basicly: steereng angle = -tau * CTE. This will allows car to set stiring angle 
  
  The problem with having only P component is that it tends to overshoot the real trajectry and never concerge to it.  This can be easily seen assuming CTE = 0 and according to forumal above wheels are set to straight position - however car is set slightly towards trajectory forcing to cross it and return back bounsing around.  First video demonstraites this behaviour.
  
* D - differential part of the controller - introduced to fix the bouncing prorblem above.  The idea is that compensate the steering angle by addibg tau_d * (CTE_NOW - CTE_PREVIOUS) / dt

* I - integral part of the controller - this correction is nesseary to compensate possible controller biases (for example: wheels are not straight when steering angle is 0, or as for our project we have a lot more left turns than right turns - this also might be compensated by integral part of the controller in some degree). THe idea is simple there - if controller is biased - the average error over time will converge to some constant (not ot zero) so we are adding correction term which tau_i * SUM(CTE)  


Student describes the effect of the  component of the PID algorithm in their implementation. Is it what you expected?

Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

Describe how the final hyperparameters were chosen.

Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!
