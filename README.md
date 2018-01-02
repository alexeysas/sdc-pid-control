# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Goal

The goal of the project is to build a PID controller and tune the PID hyperparameters so vehicle is able to drive one lap in the simulator.

## Reflection

### PID Overview

PID algorithm implemented consists of three different components: P, I, D.

* P - proportional part of the controller
  This is main part of the controller. The idea is simple -  we need to set steering angle proportionally to the Cross Track Error - CTE (difference between reference trajectory and current trajectory).  So basically: steering angle = -tau * CTE. This will force car to set steering angle to reduce CTE which mean to move closer to the reference trajectory.
  The problem with having only P component is that it tends to overshoot the real trajectory and never converge to it.  This can be easily seen assuming CTE = 0 and according to the formula above wheels are set to straight position (steering angle = 0). However, car is set slightly towards trajectory forcing to cross it and return bouncing around.  
  
  Video: https://github.com/alexeysas/sdc-pid-control/blob/master/video/P.mp4 demonstrates this behavior.

  So, for P-Controller we have:  steering angle = -tau * CTE

* D - differential part of the controller - introduced to fix the bouncing problem above.  The idea is that we need to compensate the steering angle when it is close to the reference trajectory by adding previous error change:  -tau_d * (CTE - CTE_PREVIOUS) / dt. So, for example for the positive CTE sequence which is heading towards 0 we will have negative component CTE - CTE_PREVIOUS – which works as correction preventing crossing  reference trajectory  and heavily bouncing around.

  So, for PD-Controller we have:  steering angle = -tau * CTE - tau_d * (CTE - CTE_PREVIOUS) / dt
  
  Video: https://github.com/alexeysas/sdc-pid-control/blob/master/video/PD.mp4 demonstrates PD controller improvement.

* I - integral part of the controller - this correction is necessary to compensate possible controller biases (for example: wheels are not straight when steering angle is 0, or as for our project we have a lot more left turns than right turns - this also might be compensated by integral part of the controller in some degree). The idea is simple there - if controller is biased - the average error over time will converge to some non-zero constant for PD controller - so we need to add correction term to compensate this error.  This can be done by adding integral part:   tau_i * SUM(CTE)  

  So finally, for PID-Controller implemented we have: steering angle = -tau_p * CTE - tau_d * (CTE - CTE_PREVIOUS) / dt - tau_i *     SUM(CTE).
  
  Video: https://github.com/alexeysas/sdc-pid-control/blob/master/video/PD.mp4 demonstrates full PID controller behavior.

### Parameters tuning

For the final parameters, I've used combined approach - both manual tuning + twiddle algorithm. There are a lot of challenges to make twiddle useful for the parameters tuning for this project. For example, different road culvatures and car physics effects makes total error to vary a lot for different track parts and previous car behavior making it unreliable metric. Additionally, ideally each twiddle iteration must be based on the whole track - which is time consuming.  However, I've still partially used twiddle implementation following algorithm below:

* Select arbitrary reasonable value for the parameters manual.
* Make twiddle to work only for one specific parameter - fixing all other
* Run simulator to see if there is strong parameter decreasing or increasing trend.
* Change parameter manually following trend identified.

The final parameters are following: tau_p = 0.12, tau_d = 0.25,  tau_i = 0.00001

Additionally, speed controller was implemented to make constant speed = 30mhp - this makes control a lot more stable.

