This node listens to ackermann drive messages and sends them to the rig.
It listens to the speed parameter, not the acceleration parameter.

```
ros2 run ft_rig ft_rig_node
```

You may have to tweak the `pinion_home` variable in the `SteeringMotor` class.

I am working on an easier way to align the pinion and record the home position.

Currently, the process for aligning the pinion is as follows.

- Ensure that the TOF distance sensor is taped correctly to the place where the axles are such that it has a direct line of sight to the pinion.
- Exectue `python3 -m ft_rig.command_motor interactive`. Type in `left` to steer left and type in `right` to steer right. At this step ensure that the motor wires are not reversed.
- Move the wheels such that they are straight, then execute `python3 -m ft_rig.print_pinion_distance`.
- Copy this value and modify the `pinion_home` variable to `float(<VALUE>)` (it being a float is very important).
