# fj_experiment

帧间突变实验包：在 Autoware planning_simulator 的「预测输出 → 规划输入」之间插入两个节点。

```
map_based_prediction ──/perception/object_recognition/objects_raw──► fault_injector
      ──/fj/objects_injected──► existence_filter ──/perception/object_recognition/objects──► planning
```

| 可执行文件 | 作用 | 需要 ROS |
| --- | --- | --- |
| fault_injector | 成段漏检 / 独立漏检 / 位置抖动 / 虚警 | 是 |
| existence_filter | M0 直通、M1 连续 N 帧删除、M2 改进点一、M3 改进点一+二 | 是 |
| runner | teach 录制场景；run 批量运行并记录 | 是 |
| calibrate | 离线标定 CUSUM 阈值 h | 否 |
| evaluate | 离线计算指标、出图 | 否 |

所有实验参数在 `config/experiments.yaml`，完整步骤见实验方案文档。

```bash
mkdir -p ~/fj_ws/src && cp -r fj_experiment ~/fj_ws/src/
source ~/autoware/install/setup.bash
cd ~/fj_ws && colcon build --symlink-install --packages-select fj_experiment
source ~/fj_ws/install/setup.bash
```
