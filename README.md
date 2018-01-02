# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Goal

The goal of the project is to build a PID controller and tune the PID hyperparameters so vehicle is able to drive one lap in the simulator.

## Reflection

### PID Overview

PID algorithm implemented consists of three different components: P, I, D.

* P - proportional part of the controller
  This is main part of the controller. The idea is simple  -  we need to set steering angle proporionaly to the Cross Track Error - CTE difference between reference trajectory and current trajectory.  So basicly: steereng angle = -tau * CTE. This will allows car to set stiring angle 
  
  The problem with having only P component is that it tends to overshoot the real trajectry and never concerge to it.  This can be easily seen assuming CTE = 0 and according to forumal above wheels are set to straight position - however car is set slightly towards trajectory forcing to cross it and return back bounsing around.  First video demonstraites this behaviour.
 
 So, for P-Controller we have: steereng angle = -tau * CTE
  
* D - differential part of the controller - introduced to fix the bouncing prorblem above.  The idea is that compensate the steering angle by addibg tau_d * (CTE_NOW - CTE_PREVIOUS) / dt

So, for PD-Controller we have: steereng angle = -tau * CTE - tau_d * (CTE - CTE_PREVIOUS) / dt
 
* I - integral part of the controller - this correction is nesseary to compensate possible controller biases (for example: wheels are not straight when steering angle is 0, or as for our project we have a lot more left turns than right turns - this also might be compensated by integral part of the controller in some degree). THe idea is simple there - if controller is biased - the average error over time will converge to some constant (not ot zero) so we are adding correction term which tau_i * SUM(CTE)  

So finnaly, for PID-Controller we have: steereng angle = -tau * CTE - tau_d * (CTE - CTE_PREVIOUS) / dt - tau_i * SUM(CTE)

### Parameters tunning

For the finnal parameters I've used combined approach - both manual tunning + twiddle. There are a lot of challenges to make twiddle useful for the parameters tuning for this project. Dor example, different road culvatures and car physics effects makes total error to vary a lot for different track parts and previous car behaviour - so it become unreliable metric. And idealy each twiddle iteration must be based on the whole track - which is time consuming.  However, I've still partially used foloowing algorithm below:

* Select arbitrary resonable value for the the parameters manual.
* Make twiddle to work only for onr specific parameter - fixing all other
* Run simulator to see if there is strong parameter decresing or increasing trend.
* Change parameter manualy following trend identified.

The final parameters are following: tau_p = 0.12, tau_d = 0.025,  tau_i = 0.0001


