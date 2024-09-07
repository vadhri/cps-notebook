This folder contains model simulations for cyber physical systems. 

**Discrete time simulation model**

The variables contain the state of the system with the following. 

|Variable|Description|Represention in model|
|-|-|-|
|v1|Velocity the of the system considered to be constant|Contant block|
|v2|Angular adjustment of the steer|Signal pulse active between 4-6 seconds on a 10seconds pulse|
|x1|Horizontal position|state block with feedback|
|x2|Vertical position|''|
|x3|Angle between Orientation of the vehicle with horizontal axis|''|

**Constant time simulation model**

The variables are the same as above but the simuation is done using continous time plant.