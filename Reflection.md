# Reflection

## Implementation

### The Model
> Student describes their model in detail. This includes the state, actuators and update equations.

First I've copy the solution code in CarND-MPC-Quizzes, try to fit the cost function, and then remove the part not important. But the car will shaking in the road because it will try to decrease the `cte` value, so I've add `delta * v` part like really driving (steer smoothly on high speed). After tuning some remaining part, the car can drive over the race, but the speed is not so cool. So I've add some features like races drift: steering hard when brake (minus `delta * a` in cost function), and then the car can drive over the race.

### Timestep Length and Elapsed Duration (N & dt)
> Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

First I've choose the `N` and `dt` value like CarND-MPC-Quizzes with 25 and 0.1, but I couldn't turn the car to finish the race for a long time.
When I tuning the variables, I found that increase `dt` can reduce the impact of latency, and decrease `dt` can make the prediction more smoothly.
At last, I found that `N * dt` means the future way that MPC Model are looking for, if this value is too big, `Ipopt` will calculate a good solution for the future, but too bad for the recent steps. Finally I decrease `N` and `dt` to make the polyfit path and MPC future path be just like one cross.


### Polynomial Fitting and MPC Preprocessing
> A polynomial is fitted to waypoints.  
> If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

First I push the waypoints directly into polyfit and MPC Model, but the derivative is hard to calculate, and the `next_x_vals` and `next_y_vals` are in the vehicle's coordinate system. So I've transform the waypoints into vehicle's coordinate like Project Kidnapped-Vehicle-Project.

### Model Predictive Control with Latency
> The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

First I've calculate `N_latency` that means the latency will delay `N_latency` steps, and then increase the latency to observe what will happen in the simulator: repeat the last command.
And then I've push the last command `steering_angle` and `throttle` into MPC model, set the initial vars, `xl` and `xu` into Ipopt (MPC.cpp: Line 201) to simulate the car's behavior in the latency, and ignore the first `N_latency` step's cost function because it's a static value and will make algorithm more faster. At last, use the next step after `N_latency` in the solution.

## Some Reference

- https://www.coin-or.org/CppAD/Doc/ipopt_solve.htm
- https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm
