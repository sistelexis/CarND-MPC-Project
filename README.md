# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies + Basic Build Instructions + Tips + Editor Settings + Code Style + Hints!

In order to find all the information related to this project, please go to the [provided base code github page](https://github.com/udacity/CarND-MPC-Project).

## Project Instructions and Rubric

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Rubric

### Compilation

#### Code must compile without errors with cmake and make.

The code compiles as expected.

### Implementation

#### Student describes their model in detail. This includes the state, actuators and update equations.

This project uses the very simple kinematic model that doesn't consider most of the details that would make it real, but within certain limits (speed, and steering angles), it can be very close to it and be considered good enough.

To make it very simple, it might be seen as 4 interconnected wheels, so the only parameter that is car related, is the distance between those wheels (wheelbase and axle width), that will be represented by the parameter Lf (sim car has a Lf value of 2.67).

All the forces from the real world are not considered (weight, engine and brake response, and tire friction with the road).
So, the model will consider the car position (x and y), its orientation (psi) and its speed. Those four values are the car state.

Then, possible inputs to the system are acceleration, braking and steering. To make it simple, rally driving will not be considered, so, the brake pedal and the throttle pedal cannot be pressed at the same time, making it like a single command ranging from -1 (full brakes or full negative throttle) to 1 (full throttle), and it is called a. The other actuator, will be the steering and it is called delta.

Update equations:

	  fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	  
	  fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	  
	  fg[2 + psi_start + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
	  
	  fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
	  
	  fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)) ;
	  
	  fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);

#### Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Ideally, the further we know the future and the more often we calculate it, the better. But all this comes with a price that is a high processing load. On the other hand, the further we move into the future, new information tends to add a very small value to our knowledge.

So the aim here will be to find a good balance between both. That balance will depende mostly on the processing power of the computer running the simulation. Time-wise, it could be good to have a vision of the next few seconds to allow anticipation (More than that will be useless). Then the computer limitations will dictate the balance, that in my case was around 1 second. Having a time horizon T of 1 second, we can set delta_t to 100 miliseconds and the number of points to 10 (T = N*dt = 10*0.1 = 1). Increasing the horizon, or increasing the steps, didn't produce good results on my computer. Tweeking the weights helped achieving a better result, but never as stable in long runs as the 1 second horizon and 0.1 step.

#### A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

In order to have usable values, it is wise to transform the waypoints coordinates to the car coodinates.
The transformation is done this way (main.cpp line 106 to 113):

		  for (uint i = 0; i < ptsx.size(); i++){
		  
		    //shift car reference to 90 degrees
			
			double shift_x = ptsx[i] - px;
			
			double shift_y = ptsy[i] - py;
			
			
			ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
			
			ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
			
		  }
		  

#### The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

The time between a decision and when it is implemented is never 0. Trying to explain in a funny way, if you do not antecipate when you need to park your car in your house driveway, you will end up most likely parking the car at your neighbour's. That will be called latency.
The approach to workaround the latency is to anticipate where the car would most likely be after that latency time and predict as if the car would already be there.
For this project the latenty as been established as being 100 milliseconds, so we will jump 0.1 secs ahead in time to predict the trajectory of the car.

		  double delayed_x = v * delay_t;
		  
		  double delayed_y = 0;
		  
		  double delayed_psi = - v * steer_value / Lf * delay_t;
		  
		  double delayed_v = v + throttle_value * delay_t;
		  
		  double delayed_cte = cte + v * sin(epsi) * delay_t;
		  
		  double delayed_epsi = epsi - v * steer_value / Lf * delay_t;

### Simulation

#### No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

The car manages to complete one or more laps without leaving the track. The car reached a maximum speed of 93 mph (around 150 kph).




