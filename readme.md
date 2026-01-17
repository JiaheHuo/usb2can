# 操作指令合集

# Build & Authorize

```构建与编译
cd /home/jhuo/robstride_usb2can_ctrl
rm -rf build
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```

```
sudo chmod +777 /dev/ttyUSB0
```

# DryRun

在不连接电机但有imu的时候，运行dryrun：

```
./motor_test --dry-run --imu-only --imu-enable --imu-port /dev/ttyUSB0 --imu-baud 921600 --rate 1000 --telemetry-hz 10
```

检查ankle的ik和fk

```
./motor_test --dry-run --test fkik --rate 1000 --telemetry-hz 2 --amp 0.2 --freq 0.5
```

# HardWare

*DAMPING MODE TO CHECK TELEMETRY FUNCTION*

```
 ./motor_test --config ../config/config.yaml --test damping --rate 1000 --telemetry-hz 10 --kd 2.0
```

*HOLD MODE TO CHECK KP AND KD SETTING*

```
./motor_test --config ../config/config.yaml --test hold --rate 1000 --telemetry-hz 10
```



POLICY TEST

```
./motor_test --config ../config/config.yaml --test policy --rate 1000 --telemetry-hz 50
```


EMERGENCY TEST BY ADD THE FOLLOWING CODE IN FUNC "checkMotoeLimit_"

```
if (lowlevel_cnt_ > 2000) jointPos_rad_(policy_joint_idx_[0]) = 999.f;
```
