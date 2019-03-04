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

### 
