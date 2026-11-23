# Autoware Hunter 实车调试问题总结

本文档用于记录当前 Autoware Universe 移植到 Hunter 小车平台过程中遇到的主要问题、排查过程和解决方法。当前阶段重点解决了两个问题：

1. Auto 按键周期性闪烁。
2. 车辆无法正常进入自动驾驶控制链路。

## 1. Auto 按键周期性闪烁

### 1.1 问题现象

启动 Autoware 后，RViz 左侧 Autoware Control 面板中的 `Auto` 按键会时不时闪烁，无法稳定保持可用状态。

诊断链路中反复出现：

```text
/autoware/modes/autonomous WARN
/autoware/localization WARN
/autoware/localization/sensor_fusion_status WARN
```

进一步查看 EKF 诊断后，主要报错为：

```text
twist topic is delay
mahalanobis distance of twist topic is large
```

关键诊断字段为：

```text
twist_queue_size: 0
twist_is_passed_delay_gate: False
twist_delay_time: nan
twist_is_passed_mahalanobis_gate: False
twist_mahalanobis_distance: nan
```

这些信息说明，EKF 在某些处理周期内没有拿到新的 twist 数据，因此 `sensor_fusion_status` 被置为 WARN，最终导致 Auto 按键闪烁。

### 1.2 数据链路

本系统中 twist 数据链路如下：

```text
/vehicle/status/velocity_status
-> /sensing/vehicle_velocity_converter/twist_with_covariance
-> /localization/twist_estimator/twist_with_covariance
-> EKF localizer
-> /autoware/localization/sensor_fusion_status
```

其中：

```text
/vehicle/status/velocity_status
```

来自 Hunter 底盘状态反馈。

```text
/sensing/vehicle_velocity_converter/twist_with_covariance
```

由 Autoware 的 vehicle velocity converter 将底盘速度状态转换得到。

```text
/localization/twist_estimator/twist_with_covariance
```

由 `gyro_odometer` 融合底盘速度和 IMU 角速度得到，作为 EKF 的 twist 输入。

### 1.3 排查结果

实际测得的话题频率大致如下：

```text
/vehicle/status/velocity_status
约 50 Hz，稳定
```

```text
/sensing/vehicle_velocity_converter/twist_with_covariance
约 50 Hz，稳定
```

```text
/sensing/imu/imu_data
约 100 Hz，但存在一定抖动
```

```text
/localization/twist_estimator/twist_with_covariance
约 31-32 Hz，且存在输出抖动
```

原始 EKF 参数为：

```yaml
predict_frequency: 50.0
tf_rate: 50.0
```

这意味着 EKF 以 50 Hz 的频率处理数据，而实际进入 EKF 的 twist 输入只有约 31-32 Hz，并且存在最大约 80 ms 左右的间隔抖动。

因此，当 EKF 某些周期没有收到新的 twist 数据时，就会出现：

```text
twist_queue_size: 0
```

从而触发：

```text
sensor_fusion_status WARN
```

### 1.4 解决方法

修改 EKF 参数文件：

```text
/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/localization/ekf_localizer.param.yaml
```

将 EKF 频率调整为：

```yaml
predict_frequency: 20.0
tf_rate: 20.0
```

由于当前安装目录中的该参数文件是 symlink 到源码文件，因此修改后不需要重新编译，但需要重启 Autoware 让参数重新加载。

重启后，Auto 按键能够保持常亮，`sensor_fusion_status` 不再因为 twist 队列为空而周期性 WARN。

### 1.5 对控制精度的影响

降低 EKF 频率可以解决当前 Auto 闪烁问题，但它属于调试阶段的稳定化方案，而不是最终最优方案。

EKF 的作用是融合：

```text
NDT pose
底盘速度
IMU 角速度
```

并输出稳定的车辆定位结果，例如：

```text
/localization/pose_twist_fusion_filter/pose
/localization/pose_twist_fusion_filter/twist_with_covariance
/localization/kinematic_state
map -> base_link
```

当 EKF 从 50 Hz 降低到 20 Hz 后，定位输出频率也会降低。对于当前低速实车调试、小车平台和封闭场地实验，20 Hz 通常可以接受。但如果后续车辆速度提高，或者轨迹跟踪对定位延迟和定位频率更敏感，可能会影响控制精度和响应速度。

长期更合理的方案是：

```text
1. 优化 /localization/twist_estimator/twist_with_covariance 的输出稳定性。
2. 检查 IMU 时间戳和发布抖动。
3. 优化 gyro_odometer 的融合输出逻辑。
4. 在 twist 输入稳定后，将 EKF 频率逐步恢复到 30 Hz 或 50 Hz。
5. 通过实车轨迹跟踪误差验证最终参数。
```

## 2. 车辆无法进入自动驾驶控制链路

### 2.1 问题现象

此前 Autoware 虽然能够启动，传感器、定位、规划和控制模块也逐步接入，但车辆无法稳定进入自动驾驶状态，或者 Auto 按键不可用。

主要表现包括：

```text
车辆无法初始化定位
Auto 按键不可用或闪烁
车辆模型在 RViz 中闪烁
TF 树存在冲突
底盘无法正确接收 Autoware 控制命令
```

### 2.2 控制链路确认

最终确认 Autoware 到 Hunter 底盘的控制链路为：

```text
/control/command/control_cmd
-> hunter_autoware_bridge
-> /cmd_vel
-> hunter_base
-> Hunter 底盘
```

Hunter 底盘状态反馈链路为：

```text
Hunter 底盘
-> /hunter_status
-> hunter_autoware_bridge
-> /vehicle/status/velocity_status
-> /vehicle/status/steering_status
-> /vehicle/status/gear_status
-> /vehicle/status/control_mode
```

这说明，只要 Autoware 的控制命令能够正常发布，`hunter_autoware_bridge` 能够正确转换，`hunter_base` 能够接收 `/cmd_vel`，底盘控制链路就可以打通。

### 2.3 TF 冲突问题

#### 问题原因

Hunter 底盘侧原本可能发布：

```text
odom -> base_link
```

而 Autoware 定位模块会发布：

```text
map -> base_link
```

如果两个模块同时发布 `base_link` 的父级 TF，就会导致 TF 树冲突。典型报错包括：

```text
Could not find a connection between 'map' and 'base_link'
Tf has two or more unconnected trees
```

RViz 中也会出现车辆模型闪烁、定位不稳定等现象。

#### 解决方法

让 Autoware 定位模块统一负责发布：

```text
map -> base_link
```

底盘侧不要再发布 `odom -> base_link`。

具体做法是在 Hunter 底盘 launch 或节点中增加并使用：

```text
publish_odom_tf
```

参数，在 Autoware 启动底盘时设置为：

```text
publish_odom_tf:=false
```

这样可以避免 `base_link` 被多个父级 TF 同时控制。

### 2.4 静止状态误判问题

#### 问题现象

定位初始化时曾出现：

```text
The vehicle is not stopped.
```

这表示 Autoware 判断车辆当前没有完全静止，因此拒绝初始化定位。

#### 问题原因

Hunter 底盘反馈速度在静止时仍可能存在小幅噪声，例如：

```text
0.03 m/s
-0.03 m/s
```

虽然车辆实际没有运动，但 Autoware 会根据该速度反馈认为车辆仍在移动。

#### 解决方法

在 Hunter-Autoware bridge 中对低速噪声增加 deadband，将小于阈值的速度置为 0。

示例逻辑：

```python
raw_v = msg.linear_velocity
v = 0.0 if abs(raw_v) < 0.05 else raw_v
```

这样 Autoware 能正确识别车辆处于静止状态，定位初始化可以正常完成。

### 2.5 Launch 参数问题

此前 `hunter_base.launch.py` 中的 `robot_model` 参数需要确认是否正确声明和传递。若 launch 文件中没有声明该参数，可能出现：

```text
launch configuration 'robot_model' does not exist
```

解决方向是确保：

```text
1. hunter_base.launch.py 中声明 robot_model。
2. Autoware 的 vehicle interface 启动 Hunter 底盘时正确传入 robot_model。
3. 使用的车型参数与实际 Hunter 底盘一致。
```

例如：

```text
robot_model:=hunter_se
```

## 3. 当前系统状态

目前已经完成的主要工作包括：

```text
1. Autoware 主系统可以启动。
2. Livox MID-360 点云可以进入 Autoware 感知和定位链路。
3. GINS2000/组合导航相关话题已经接入传感器链路。
4. Hunter 底盘控制链路已经打通。
5. Hunter 底盘状态可以反馈给 Autoware。
6. TF 树冲突问题已处理。
7. 静止状态误判问题已处理。
8. Auto 按键周期性闪烁问题已通过 EKF 频率匹配暂时解决。
```

当前重点修改的 EKF 参数为：

```yaml
predict_frequency: 20.0
tf_rate: 20.0
```

## 4. 后续建议

当前方案可以支持低速实车调试，但后续若要提升自动驾驶稳定性和控制精度，建议继续优化以下内容：

```text
1. 复查 IMU 时间戳是否稳定。
2. 复查 /sensing/imu/imu_data 的发布抖动。
3. 优化 gyro_odometer 的输出逻辑。
4. 尽量让 /localization/twist_estimator/twist_with_covariance 稳定达到 50 Hz。
5. 在 twist 输入稳定后，将 EKF 从 20 Hz 逐步恢复到 30 Hz 或 50 Hz。
6. 对比不同 EKF 频率下的轨迹跟踪误差。
7. 在低速、中速场景分别测试控制稳定性。
8. 正式测试前恢复必要的定位质量检查，避免过度屏蔽诊断项。
```

特别需要注意的是，降低 EKF 频率解决的是当前调试阶段的 Auto 状态稳定问题。它不应被视为最终性能最优解。最终系统应尽量保证传感器、底盘、IMU、twist estimator 和 EKF 之间的频率与时间戳都稳定对齐。
