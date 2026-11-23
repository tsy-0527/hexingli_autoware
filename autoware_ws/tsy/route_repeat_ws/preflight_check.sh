#!/bin/bash
# 实车起飞前自检脚本
# 在 Autoware 已启动、NDT 雷达定位已运行的情况下执行
# 全部通过才允许录制/回放

# 先加载 ROS 环境，再启用未定义变量检查。
# 某些 ROS setup 脚本会读取尚未定义的环境变量；如果这里先 set -u，
# 脚本可能在第一条 source 命令处直接退出。
source /opt/ros/humble/setup.bash 2>/dev/null || {
    echo "无法加载 /opt/ros/humble/setup.bash"
    exit 1
}
source ~/autoware/install/setup.bash 2>/dev/null || {
    echo "无法加载 ~/autoware/install/setup.bash"
    exit 1
}
source ~/tsy/route_repeat_ws/install/setup.bash 2>/dev/null || {
    echo "无法加载 ~/tsy/route_repeat_ws/install/setup.bash"
    exit 1
}
set -u

# ========== 配置 ==========
MAP_DIR="${MAP_DIR:-$HOME/autoware_map/map_task1}"
PARAMS_YAML="$HOME/tsy/route_repeat_ws/src/route_teach_repeat/config/params.yaml"
POSE_TOPIC="/localization/pose_estimator/pose"
POSE_MSG_TYPE="pose_stamped"
MIN_POSE_HZ=5                                           # NDT pose 频率下限

# 读取当前 route_recorder 配置，避免自检检查的 topic 和录制器实际订阅的 topic 不一致。
yaml_value() {
    sed -n "s/^[[:space:]]*$1:[[:space:]]*//p" "$PARAMS_YAML" \
        | head -1 | tr -d '"' | sed 's/[[:space:]]*#.*$//'
}

if [ -f "$PARAMS_YAML" ]; then
    POSE_TOPIC="$(yaml_value pose_topic)"
    POSE_MSG_TYPE="$(yaml_value pose_msg_type)"
fi

PASS=0
FAIL=0
WARN=0

pass()  { echo -e "  \033[32m[PASS]\033[0m $1"; PASS=$((PASS+1)); }
fail()  { echo -e "  \033[31m[FAIL]\033[0m $1"; FAIL=$((FAIL+1)); }
warn()  { echo -e "  \033[33m[WARN]\033[0m $1"; WARN=$((WARN+1)); }
info()  { echo -e "  \033[36m[INFO]\033[0m $1"; }
section() { echo -e "\n\033[1m▸ $1\033[0m"; }

# ========== 检查 1: ROS 环境 ==========
section "1. ROS 环境"

if [ -z "${ROS_DISTRO:-}" ]; then
    fail "ROS_DISTRO 未设置,是否 source /opt/ros/humble/setup.bash?"
    exit 1
else
    pass "ROS_DISTRO = $ROS_DISTRO"
fi

if ! command -v ros2 &> /dev/null; then
    fail "ros2 命令未找到"
    exit 1
fi

# ========== 检查 2: 地图文件 ==========
section "2. 地图文件"

if [ ! -d "$MAP_DIR" ]; then
    fail "地图目录不存在: $MAP_DIR"
    fail "请设 MAP_DIR 环境变量,或修改脚本里的 MAP_DIR"
else
    pass "地图目录存在: $MAP_DIR"
    
    if [ ! -f "$MAP_DIR/lanelet2_map.osm" ]; then
        fail "找不到 lanelet2_map.osm"
    else
        LL_SIZE=$(du -h "$MAP_DIR/lanelet2_map.osm" | cut -f1)
        pass "Lanelet2 地图: $LL_SIZE"
    fi
    
    if [ ! -f "$MAP_DIR/pointcloud_map.pcd" ] && [ ! -f "$MAP_DIR/pointcloud_map/pointcloud_map.pcd" ]; then
        warn "找不到 pointcloud_map.pcd (NDT 雷达定位需要点云地图)"
    else
        pass "点云地图存在"
    fi
fi

# ========== 检查 3: Autoware 节点 ==========
section "3. Autoware 节点"

NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    fail "没有任何 ROS 节点在跑! 先启动 Autoware"
    exit 1
fi

NODE_COUNT=$(echo "$NODES" | wc -l)
if [ "$NODE_COUNT" -lt 20 ]; then
    fail "只有 $NODE_COUNT 个节点,Autoware 可能没启动完全 (正常应有 100+)"
else
    pass "共 $NODE_COUNT 个节点在运行"
fi

# 关键节点检查
for key in "mission_planner" "trajectory_follower" "behavior_path_planner"; do
    if echo "$NODES" | grep -q "$key"; then
        pass "关键节点存在: $key"
    else
        fail "关键节点缺失: $key"
    fi
done

# ========== 检查 4: 定位源 (NDT 雷达) ==========
section "4. 定位源"

TOPICS=$(ros2 topic list 2>/dev/null)
TOPIC_TYPE=$(ros2 topic type "$POSE_TOPIC" 2>/dev/null | head -1)
INIT_STATE=$(timeout 3 ros2 topic echo /api/localization/initialization_state --once 2>/dev/null \
    | awk '/state:/ {print $2; exit}')

if echo "$NODES" | grep -q "/localization/pose_estimator/ndt_scan_matcher"; then
    pass "NDT 雷达定位节点存在"
else
    fail "NDT 雷达定位节点不存在: /localization/pose_estimator/ndt_scan_matcher"
fi

if echo "$TOPICS" | grep -Fxq "$POSE_TOPIC"; then
    pass "雷达定位 topic 存在: $POSE_TOPIC"
else
    fail "雷达定位 topic 不存在: $POSE_TOPIC"
fi

if [ "$POSE_MSG_TYPE" = "pose_stamped" ] && [ "$TOPIC_TYPE" = "geometry_msgs/msg/PoseStamped" ]; then
    pass "定位消息类型正确: $TOPIC_TYPE"
elif [ "$POSE_MSG_TYPE" = "pose_with_covariance" ] && \
    [ "$TOPIC_TYPE" = "geometry_msgs/msg/PoseWithCovarianceStamped" ]; then
    pass "定位消息类型正确: $TOPIC_TYPE"
else
    fail "定位消息类型不匹配: 配置=$POSE_MSG_TYPE, topic=$TOPIC_TYPE"
fi

case "$INIT_STATE" in
    1) info "NDT 定位状态 = UNINITIALIZED，请先在 RViz 使用 2D Pose Estimate 初始化" ;;
    2) warn "NDT 定位状态 = INITIALIZING，请等待匹配收敛" ;;
    3) pass "NDT 定位状态 = INITIALIZED" ;;
    *) warn "无法读取 NDT 定位初始化状态" ;;
esac

# 检查 pose topic 频率
info "检查 $POSE_TOPIC 频率..."
HZ_OUTPUT=$(timeout 5 ros2 topic hz "$POSE_TOPIC" 2>&1 | awk '/average rate:/ {print $3; exit}')
if [ -z "$HZ_OUTPUT" ]; then
    if [ "$INIT_STATE" = "1" ]; then
        fail "$POSE_TOPIC 没有数据: NDT 尚未初始化，请先在 RViz 设置初始位姿"
    else
        fail "$POSE_TOPIC 没有数据! 定位源可能没接通"
    fi
else
    if awk -v rate="$HZ_OUTPUT" -v min="$MIN_POSE_HZ" 'BEGIN {exit !(rate >= min)}'; then
        pass "$POSE_TOPIC 频率 = ${HZ_OUTPUT} Hz"
    else
        warn "$POSE_TOPIC 频率过低 = ${HZ_OUTPUT} Hz (期望 >= $MIN_POSE_HZ)"
    fi
fi

# 检查 NDT 诊断输出；没有数据通常表示尚未完成初始定位或点云地图未匹配。
if echo "$TOPICS" | grep -Fxq "/localization/pose_estimator/transform_probability"; then
    info "NDT 匹配质量 topic 存在，录制前请确认 transform_probability 稳定"
else
    warn "没有 NDT 匹配质量 topic，请确认 ndt_scan_matcher 已正常运行"
fi

# ========== 检查 5: 地图投影信息 ==========
section "5. 地图投影信息"

PROJ_INFO=$(timeout 3 ros2 topic echo /map/map_projector_info --once 2>/dev/null)
if [ -z "$PROJ_INFO" ]; then
    fail "/map/map_projector_info 没有数据,地图可能没加载"
else
    PROJ_TYPE=$(echo "$PROJ_INFO" | awk '/projector_type:/ {print $2; exit}')
    ORIG_LAT=$(echo "$PROJ_INFO" | awk '/latitude:/ {print $2; exit}')
    ORIG_LON=$(echo "$PROJ_INFO" | awk '/longitude:/ {print $2; exit}')
    
    pass "投影类型: $PROJ_TYPE"
    info "  原点 lat=$ORIG_LAT, lon=$ORIG_LON"
    
    # 检查 params.yaml 里的配置是否匹配
    if [ -f "$PARAMS_YAML" ]; then
        YAML_PROJ=$(grep "projector_type:" "$PARAMS_YAML" | awk '{print $2}' | tr -d '"')
        YAML_LAT=$(grep "map_origin_lat:" "$PARAMS_YAML" | awk '{print $2}')
        YAML_LON=$(grep "map_origin_lon:" "$PARAMS_YAML" | awk '{print $2}')
        YAML_MAP=$(grep "map_path:" "$PARAMS_YAML" | awk '{print $2}' | tr -d '"')
        
        info "  params.yaml: projector=$YAML_PROJ lat=$YAML_LAT lon=$YAML_LON"
        info "  params.yaml map_path: $YAML_MAP"
        
        # Autoware 的 LocalCartesianUTM 对应 route_recorder 的 UtmProjector，
        # params.yaml 中应填写 projector_type: "utm"。
        if [ "$PROJ_TYPE" = "LocalCartesianUTM" ] && [ "$YAML_PROJ" != "utm" ]; then
            fail "投影类型不一致: Autoware=$PROJ_TYPE, route_repeat=$YAML_PROJ"
            fail "请将 params.yaml 的 projector_type 改为 utm"
        elif [ "$PROJ_TYPE" = "LocalCartesian" ] && [ "$YAML_PROJ" != "local_cartesian" ]; then
            fail "投影类型不一致: Autoware=$PROJ_TYPE, route_repeat=$YAML_PROJ"
        else
            pass "投影类型配置一致"
        fi

        if [ -n "$YAML_LAT" ] && [ -n "$YAML_LON" ] && [ -n "$ORIG_LAT" ] && [ -n "$ORIG_LON" ] && \
            awk -v a="$YAML_LAT" -v b="$ORIG_LAT" 'BEGIN {exit !(a-b < 1e-6 && b-a < 1e-6)}' && \
            awk -v a="$YAML_LON" -v b="$ORIG_LON" 'BEGIN {exit !(a-b < 1e-6 && b-a < 1e-6)}'; then
            pass "params.yaml 原点和 Autoware 一致"
        else
            fail "原点不一致! Autoware=($ORIG_LAT,$ORIG_LON) params=($YAML_LAT,$YAML_LON)"
            fail "这是最常见的车跑飞原因,必须修!"
        fi
        
        if [ "$YAML_MAP" != "$MAP_DIR/lanelet2_map.osm" ]; then
            warn "params.yaml 的 map_path 和当前 MAP_DIR 不完全一致"
            warn "  MAP_DIR: $MAP_DIR/lanelet2_map.osm"
            warn "  YAML   : $YAML_MAP"
        fi
    fi
fi

# ========== 检查 6: 车辆状态 ==========
section "6. 车辆状态"

VEH_STATUS=$(timeout 2 ros2 topic echo /vehicle/status/velocity_status --once 2>/dev/null)
if [ -z "$VEH_STATUS" ]; then
    warn "读不到车辆速度反馈,车辆接口可能未连接"
else
    LON_VEL=$(echo "$VEH_STATUS" | grep "longitudinal_velocity:" | awk '{print $2}')
    pass "车辆当前速度 = $LON_VEL m/s"
fi

# 检查 operation mode
OP_MODE=$(timeout 2 ros2 topic echo /api/operation_mode/state --once 2>/dev/null | grep "mode:" | head -1 | awk '{print $2}')
if [ -n "$OP_MODE" ]; then
    case "$OP_MODE" in
        1) pass "operation_mode = STOP" ;;
        2) info "operation_mode = AUTONOMOUS (车已在自动)" ;;
        3) pass "operation_mode = LOCAL (手动/遥控)" ;;
        4) pass "operation_mode = REMOTE" ;;
        *) info "operation_mode = $OP_MODE" ;;
    esac
fi

# ========== 检查 7: 关键 service ==========
section "7. Route 服务"

SVCS=$(ros2 service list 2>/dev/null)
for svc in \
    "/planning/mission_planning/mission_planner/set_lanelet_route" \
    "/planning/mission_planning/mission_planner/clear_route" \
    "/api/operation_mode/change_to_autonomous"; do
    if echo "$SVCS" | grep -q "^${svc}$"; then
        pass "service 可用: $svc"
    else
        fail "service 缺失: $svc"
    fi
done

# ========== 检查 8: params.yaml 内容审查 ==========
section "8. params.yaml 审查"

PARAMS_YAML=~/tsy/route_repeat_ws/src/route_teach_repeat/config/params.yaml
if [ -f "$PARAMS_YAML" ]; then
    if grep -q 'pose_topic:.*localization/pose_estimator/pose' "$PARAMS_YAML"; then
        pass "pose_topic = NDT 雷达定位"
    else
        warn "pose_topic 可能不对,当前: $(grep pose_topic $PARAMS_YAML)"
    fi

    if grep -q 'pose_msg_type:.*pose_stamped' "$PARAMS_YAML"; then
        pass "pose_msg_type = pose_stamped"
    else
        fail "pose_msg_type 应为 pose_stamped: $(grep pose_msg_type "$PARAMS_YAML")"
    fi
    
    if grep -q "projector_type:.*utm" "$PARAMS_YAML"; then
        pass "projector_type = utm"
    else
        fail "projector_type 应为 utm: $(grep projector_type "$PARAMS_YAML")"
    fi
    
    MATCH_DIST=$(grep "match_distance:" "$PARAMS_YAML" | awk '{print $2}')
    info "  match_distance = $MATCH_DIST"
fi

# ========== 检查 9: 感知/避障 ==========
section "9. 感知与避障"

if ros2 topic list 2>/dev/null | grep -q "/perception/obstacle"; then
    pass "感知 topic 存在"
else
    warn "没有 /perception/obstacle*,避障可能不生效!实车强烈建议接感知"
fi

# ========== 总结 ==========
echo ""
echo "=============================================================="
echo -e "  \033[32mPASS: $PASS\033[0m    \033[33mWARN: $WARN\033[0m    \033[31mFAIL: $FAIL\033[0m"
echo "=============================================================="

if [ "$FAIL" -gt 0 ]; then
    echo -e "\n\033[31m存在 FAIL 项,不建议继续! 请修复后重试\033[0m"
    exit 1
elif [ "$WARN" -gt 0 ]; then
    echo -e "\n\033[33m存在 WARN 项,建议仔细核对后再继续\033[0m"
    exit 0
else
    echo -e "\n\033[32m全部检查通过,可以开始录制/回放\033[0m"
    exit 0
fi
