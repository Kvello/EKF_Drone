## About
This package implements a statistics node that calculates the measurement covariances. Remember to keep the drone still while this node runs. Uppon completion the node outputs the covariance values to a file ("statistics.csv").
To run this package:

```{bash}
ros2 run stat stat --ros_args -r ns__:=/drone
```
