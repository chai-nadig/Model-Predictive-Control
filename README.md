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
- The update equations for each of the state variables are as follows:

> x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(ψ<sub>t</sub>) ∗ dt

> y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(ψ<sub>t</sub>) ∗ dt

> ψ<sub>t+1</sub> = ψ<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> * δ<sub>t</sub> q ∗ dt

> v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> ∗ dt

> cte<sub>t+1</sub> = f(x<sub>t</sub>) − y<sub>t</sub> + v<sub>t</sub> ∗ sin(eψ<sub>t</sub>) ∗ dt

> eψ<sub>t+1</sub> = ψ<sub>t</sub> - ψdes<sub>t</sub> + (v<sub>t</sub> /  L<sub>f</sub>  *  δ<sub>t</sub> * dt)eψ
