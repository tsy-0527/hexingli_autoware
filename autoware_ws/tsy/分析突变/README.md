# bag_frame_analysis.py 使用说明

## 1. 安装（Windows / Ubuntu 均可，无需 ROS）
    pip install rosbags numpy matplotlib

## 2. 运行
    python bag_frame_analysis.py <bag 目录或 .db3 文件> -o out

- bag 目录：ros2 bag record 生成的文件夹（含 metadata.yaml），.db3 和 .mcap 都支持
- 只想分析某一段：加 --start 30 --end 120（单位：秒，相对 bag 开头）
- 在装有 Autoware 的电脑上运行时，建议加 --msg-root ~/autoware/install，
  让脚本用你本机 Autoware 的消息定义（脚本内置了常用定义，版本不同时可能解析失败）
- 话题自动识别；如需指定：--objects /perception/object_recognition/objects  --paths /planning/scenario_planning/trajectory

## 3. 上传
把 out 文件夹压缩后上传即可（通常几百 KB），里面有：
- summary.json：话题清单、感知抖动/闪烁统计（含给玩具仿真用的 sigma_l、rho、p_miss、p_recover）、
  各层规划输出的帧间差异、绕行侧翻转、突变归因（感知触发 / 规划器自身）、求解耗时
- objects_*.csv、frames_*.csv：逐障碍物、逐帧数据
- timeline.png、jitter.png：曲线图

## 4. 分析口径
- 静态障碍物按位置聚类（不依赖跟踪 ID，可发现 ID 切换），横向 = 自车经过时航向的法向
- 闪烁：障碍物首次到最后一次出现之间、自车 60 m 内的帧中统计漏检
- 帧间差异：相邻两帧轨迹按空间对齐，比较自车前方 5 m（近端）和 25 m（中段）内的最大横向偏差；排除轨迹末端 3 m
- 突变归因：偏差超过 0.3 m（可用 --spike 调整）的帧，若前 0.3 s 内有障碍物出现/消失/位置跳变，记为感知触发，否则记为规划器自身
- 若同时录了行为层路径（path_with_lane_id、path），会分别统计，可比较突变出现在哪一层
