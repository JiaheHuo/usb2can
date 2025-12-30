# 指令:

./motor_test --config ../config/motors.yaml --test sine

```

```

在不连接电机但有imu的时候，运行dryrun：

```
./motor_test --dry-run --imu-only
--imu-port /dev/ttyUSB0 --imu-baud 921600
--rate 500 --telemetry-hz 50
```
