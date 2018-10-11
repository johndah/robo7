# gate_controller

Opens and closes the gate ("gripper") based on a <std_msgs/Bool> message on the
<gate_closed> topic.

For example:gate_closed, true closes the gate

requires:
```
rosrun arduino_servo_control servo_control
```

## Connections

| Servo     | Pin     | Code    |
| --------- |:-------:| -------:|
| Left      | 9       | servo_0 |
| Right     | 10      | servo_1 |
