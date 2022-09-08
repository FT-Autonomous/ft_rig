This node listens to ackermann drive messages and sends them to the rig.
It listens to the speed parameter, not the acceleration parameter.

```
ros2 run ft_rig ft_rig_node
```

You may have to tweak the `pinion_home` variable in the `SteeringMotor` class.

I am working on an easier way to align the pinion and record the home position.
