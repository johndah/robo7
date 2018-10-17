# gate_controller

The gate controller listen to service calls on ""/gate_controller/pickup_at"
with a twist message of where to pick up the object. The twist message is in
relation to the robot position

requires:
```
rosrun arduino_servo_control servo_control
```

## Connections

| Servo     | Pin     | Code    |
| --------- |:-------:| -------:|
| Left      | 9       | servo_0 |
| Right     | 10      | servo_1 |
