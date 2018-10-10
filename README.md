# robo7

This project will include all software for the autonomous robot.

## Motor
$$
angular Velocity = encoderCount \div CPR  \times 2\pi \times freqency
$$

```
CPR (counts per revolution) = 897.96
frequency = 121
```

range of motor input is -100 ~ 100

- left motor serial number: 469741
positive: forward

- right motor serial number: 469412
negative: forward

encoder output range -30 ~ 30
therefore, estimated angular velocity -25.387 ~ 25.387

- wheel distance : 23.8 - 2.07 = 21.73 cm

- wheel diameter : 9.76 cm
## Connect to NUCs
### Mobile NUC
```
- ssh ras17@192.168.1.202
```
### Stationary NUC
```
- ssh ras27@192.168.1.207
```


## Dead reckoning
