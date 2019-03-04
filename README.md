# Model Predictive Control

## Model
- A Kinematic Model is used to represent the car.

### State
- The state of the model has six variables in them:

| Variable | Description                                                               |
|:--------:|:--------------------------------------------------------------------------|
|   x      | Position along the x-axis in the global coordinate system                 |
|   y      | Position along the y-axis in the global coordinate system                 |
|   ψ      | Orientation angle wrt to the x-axis                                       |
|   v      | Velocity of the car                                                       |
|   cte    | Cross Track Error - the difference in actual y-axis position and expected |
|   eψ     | Orientation error - the difference in actual orientation and expected     |

### Actuators
- There are two actuators for this model.
- Each of these actuators have their own constraints.

| Actuator | Constraints | Description                                                    |
|:--------:|:-----------:|:--------------------------------------------------------------:|
|   δ      | [-25°, 25°] | This is the steering angle. It changes the orientation of the car. Its maximum value can only be 25° clockwise or anti-clockwise.|
|   a      | [-1, 1]     | This is acceleration and deceleration. It alters the velocity of the car. It can range between -1 and 1 with negative values denoting deceleration or braking. |

### Update equations
The update equations for each of the state variables are as follows:

#### x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(ψ<sub>t</sub>) ∗ dt
#### y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(ψ<sub>t</sub>) ∗ dt
#### ψ<sub>t+1</sub> = ψ<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> * δ<sub>t</sub> ∗ dt
#### v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> ∗ dt
#### cte<sub>t+1</sub> = f(x<sub>t</sub>) − y<sub>t</sub> + v<sub>t</sub> ∗ sin(eψ<sub>t</sub>) ∗ dt
#### eψ<sub>t+1</sub> = ψ<sub>t</sub> - ψdes<sub>t</sub> + (v<sub>t</sub> /  L<sub>f</sub>  *  δ<sub>t</sub> * dt)


## Prediction

### Timestep length and elapsed duration
- The prediction part includes calculating actuator values for a specific number of time steps upto a few seconds.
- These are determined by hyper parameters `N` and `dt`.
- `N` is the number of time steps and `dt` is the length of each time step in seconds.
- `N` * `dt` = `elapsed duration` is the duration up to which we predict actuator values.
- Choosing a large `dt` leads to a discretization problem where the actuator values might be very large leading to sudden changes.

![Discretization Error](disc.gif)

- Choosing a very small `dt` will make the actuator values so small that the car won't change orientation or velocity fast enough and will stray from the road.

![Very small dt](smalldt.gif)

### Fitting the polynomial
- We fit a 3rd degree polynomial for the waypoints provided. This serves as the reference trajectory.
- The waypoints are provided with coordinates in the global system.
- They are transformed into the car's coordinate system before fitting the polynomial.
- This transformation helps avoid calculations down the line. Since the polynomial is fitted to the transformed waypoints, the predicted points also are in the car's coordinate system.

### Latency in transmission
- There's a 100ms latency in the actuators taking effect on the car.
- The time step duration is set to 100ms. This way, it becomes easy to model in this latency.
- To account for this latency we consider the `delta0` and `a0` values to be two time steps before the current one.
- This is because it takes 100ms for the change in actuators to happen, and we need another 100ms to observe the state change of the car due to the change in actuators.
