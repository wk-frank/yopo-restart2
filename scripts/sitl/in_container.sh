#!/usr/bin/env bash
# 容器内入口（由 smoke.sh / up.sh 以 nice 15 调起）。
# YOPO_SITL_N=1（默认）：单个 PX4（gz_x500，instance 0）+ S1 任务。与票 S1 相同。
# YOPO_SITL_N=2 或 3：instance 1..N，uXRCE namespace px4_1..px4_N，同一个 DDS 域、同一个 XRCE Agent。
#   YOPO_SITL_MISSION=0 只把仿真留着；=1（默认）再按 YOPO_SITL_MISSION_MODE 飞。
#   YOPO_SITL_MISSION_MODE=hover（默认）|goto|swap。n=1 且 hover 时不把该变量传给单机脚本，
#   单机仍是 S1（2.5 m、平移、降落）。goto/swap 才是票 S3。
# PX4 参数：全新空状态目录。默认只用官方 airframe 默认值。
# YOPO_POSITIONING=rtk 时 source positioning-rtk.env（仅白名单定位/估计参数，经官方 PX4_PARAM_* 在启动时注入）。
set -eo pipefail
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /opt/ros_ws/install/setup.bash
BUILD=/opt/PX4-Autopilot/build/px4_sitl_default
# shellcheck disable=SC1091
source "$BUILD/rootfs/gz_env.sh"
set -u
OUT=/out
mkdir -p "$OUT/px4-state"
N="${YOPO_SITL_N:-1}"
MISSION="${YOPO_SITL_MISSION:-1}"
MISSION_MODE="${YOPO_SITL_MISSION_MODE:-hover}"
POSITIONING="${YOPO_POSITIONING:-default}"
echo "$POSITIONING" > "$OUT/positioning.txt"
echo "$MISSION_MODE" > "$OUT/mission_mode.txt"
if [[ "$POSITIONING" == "rtk" ]]; then
  # shellcheck disable=SC1091
  set -a
  # shellcheck disable=SC1091
  . /repo/scripts/sitl/positioning-rtk.env
  set +a
elif [[ "$POSITIONING" != "default" ]]; then
  echo "未知 YOPO_POSITIONING=$POSITIONING（只要 default 或 rtk）" >&2
  exit 2
fi
export HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=default
export PYTHONPATH=/repo/core/src${PYTHONPATH:+:$PYTHONPATH}
export YOPO_POSITIONING="$POSITIONING"

# trap 在 EXIT/TERM/INT 时调用。shellcheck 仍把函数体标成不可达。
# shellcheck disable=SC2317
cleanup() {
  local pid
  if [[ -f /root/.gz/rendering/ogre2.log ]]; then cp /root/.gz/rendering/ogre2.log "$OUT/ogre2.log"; fi
  while IFS= read -r pid; do
    if [[ -n "$pid" ]]; then
      kill "$pid" 2>/dev/null || true
    fi
  done < <(jobs -pr || true)
  pkill -f 'gz sim' 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT TERM INT

case "$N" in
  ''|*[!0-9]*) echo "未知 YOPO_SITL_N=$N" >&2; exit 2 ;;
esac
case "$MISSION_MODE" in
  hover|goto|swap) ;;
  *) echo "未知 YOPO_SITL_MISSION_MODE=$MISSION_MODE（只要 hover、goto 或 swap）" >&2; exit 2 ;;
esac
PLANNER="${YOPO_SITL_PLANNER:-offboard}"
case "$PLANNER" in
  offboard|yopo-l0) ;;
  *) echo "未知 YOPO_SITL_PLANNER=$PLANNER（只要 offboard 或 yopo-l0）" >&2; exit 2 ;;
esac
SCENE_WORLD="${YOPO_SITL_SCENE_WORLD:-0}"
case "$SCENE_WORLD" in
  0|1) ;;
  *) echo "未知 YOPO_SITL_SCENE_WORLD=$SCENE_WORLD（只要 0 或 1）" >&2; exit 2 ;;
esac
default_yopo_scene() {
  if [[ "${YOPO_L0_POLICY:-upstream}" == "explore" ]]; then
    if [[ "$N" -eq 6 ]]; then
      printf '%s\n' "scenes/explore-6-forest-sitl80.json"
      return 0
    elif [[ "$N" -eq 3 ]]; then
      printf '%s\n' "scenes/explore-3-forest-sitl80.json"
      return 0
    fi
  fi
  if [[ "$N" -eq 1 ]]; then
    printf '%s
' "scenes/g2-pillar.json"
  elif [[ "$N" -eq 6 ]]; then
    printf '%s
' "scenes/formation-6-open-sitl80.json"
  else
    printf '%s
' "scenes/swap-3-open.json"
  fi
}
if [[ "$PLANNER" == "yopo-l0" ]]; then
  # 只在 YOPO 实飞注入。参数都在 PX4 的 SITL 组，不改 COM_ / NAV_ / BAT_ 安全默认值。
  # shellcheck disable=SC1091
  set -a
  # shellcheck disable=SC1091
  . /repo/scripts/sitl/sitl-battery.env
  set +a
  if [[ "$SCENE_WORLD" == "1" ]]; then
    scene_for_world="${YOPO_L0_SCENE:-$(default_yopo_scene)}"
    base_world_args=()
    if [[ -n "${PX4_GZ_WORLDS:-}" && -f "${PX4_GZ_WORLDS}/default.sdf" ]]; then
      base_world_args=(--base-sdf "${PX4_GZ_WORLDS}/default.sdf")
    fi
    PYTHONPATH="/repo:/repo/core/src${PYTHONPATH:+:$PYTHONPATH}"       python3 /repo/scripts/sitl/world_sdf.py         --scene "$scene_for_world"         "${base_world_args[@]}"         --out-dir "$OUT/worlds"         --summary "$OUT/world_sdf.json" >/dev/null
    export PX4_GZ_WORLDS="$OUT/worlds"
    echo "yopo scene world enabled: $OUT/worlds/default.sdf"
    if [[ "${YOPO_LIVE_DEPTH_SOURCE:-auto}" == "gazebo" ]]; then
      PYTHONPATH="/repo:/repo/core/src${PYTHONPATH:+:$PYTHONPATH}" python3 /repo/scripts/sitl/prepare_native_cameras.py \
        --scene "$scene_for_world" --world "$OUT/worlds/default.sdf" \
        --base-model /opt/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf \
        --out "$OUT/native_cameras.json"
      # Start EGL rendering explicitly; PX4 attaches to this existing world.
      export EGL_PLATFORM=surfaceless
      if [[ "${YOPO_SITL_GAZEBO_GPU:-software}" == "software" ]]; then
        export LIBGL_ALWAYS_SOFTWARE=1
      else
        unset LIBGL_ALWAYS_SOFTWARE
      fi
      gz sim -r -s --headless-rendering "$OUT/worlds/default.sdf" > "$OUT/gazebo-server.log" 2>&1 &
      for _ in $(seq 1 180); do
        gz service -l 2>/dev/null | grep -q '/world/default/scene/info' && break
        sleep 1
      done
    fi
  fi
fi
python3 /repo/scripts/sitl/fleet.py layout --n "$N" --mission "$MISSION_MODE" --format json >/dev/null

run_yopo_l0() {
  # 只把场景、机数、种子和定位模式传给规划桥。定位白名单仍由上面的 YOPO_POSITIONING 分支注入。
  PYTHONPATH="/repo:/repo/core/src${PYTHONPATH:+:$PYTHONPATH}" \
    python3 /repo/scripts/sitl/yopo_l0_live.py \
      --out "$OUT/result.json" \
      --scene "${YOPO_L0_SCENE:-scenes/g2-pillar.json}" \
      --n "$N" \
      --seed "${YOPO_L0_SEED:-0}" \
      --policy "${YOPO_L0_POLICY:-upstream}" \
      --wall-timeout "${YOPO_SITL_WALL_TIMEOUT:-900}" \
      --positioning "$POSITIONING"
}

wait_px4() {
  local log=$1 pid=$2
  local _
  for _ in $(seq 1 1800); do
    grep -q 'Startup script returned successfully' "$log" 2>/dev/null && return 0
    kill -0 "$pid" 2>/dev/null || return 3
    sleep 0.1
  done
  echo "PX4 启动超时，见 $log" >&2
  return 4
}

dump_fail() {
  local log
  shopt -s nullglob
  for log in "$OUT"/px4*.log; do
    echo "-- $(basename "$log") --"
    grep -E 'Preflight Fail|Arming denied|arming denied|Failsafe|failsafe|ERROR' "$log" | sort | uniq -c | sort -rn | head -10 || true
  done
}

finish_mission_rc() {
  local rc=$1
  if [[ $rc -ne 0 && -f "$OUT/result.json" ]]; then
    if python3 -c 'import json,sys; sys.exit(0 if json.load(open("/out/result.json")).get("ok") else 1)'; then
      echo "mission ok, python rc=$rc (teardown); treating as success"
      rc=0
    fi
  fi
  dump_fail
  exit "$rc"
}

if [[ "$N" -eq 1 ]]; then
  MicroXRCEAgent udp4 -p 8888 > "$OUT/agent.log" 2>&1 &
  ( cd "$OUT/px4-state" && exec "$BUILD/bin/px4" -d "$BUILD/etc" ) > "$OUT/px4.log" 2>&1 &
  PX4_PID=$!
  for _ in $(seq 1 1200); do
    grep -q 'Startup script returned successfully' "$OUT/px4.log" 2>/dev/null && break
    kill -0 "$PX4_PID" 2>/dev/null || { echo "PX4 进程提前退出，见 px4.log" >&2; exit 3; }
    sleep 0.1
  done
  echo "px4_startup_wall_s $SECONDS"
  # 只读：记下与默认值不同的参数，作为没改参数的证据。
  "$BUILD/bin/px4-param" --instance 0 show -c > "$OUT/px4-params-changed.txt" 2>&1 || true
  if [[ "$MISSION" == 0 ]]; then
    date -Is > "$OUT/ready"
    echo "sim ready n=1 mission=0; waiting"
    wait
    exit 0
  fi
  set +e
  if [[ "$PLANNER" == "yopo-l0" ]]; then
    run_yopo_l0
    rc=$?
  elif [[ "$MISSION_MODE" == "hover" ]]; then
    # 与票 S1 相同的参数：不传 --mission。清掉模式变量，避免容器环境里的 hover 把单机改成 1.5 m 降落。
    env -u YOPO_SITL_MISSION_MODE python3 /repo/scripts/sitl/offboard_mission.py --out "$OUT/result.json" --wall-timeout "${YOPO_SITL_WALL_TIMEOUT:-900}"
    rc=$?
  elif [[ "$MISSION_MODE" == "goto" ]]; then
    python3 /repo/scripts/sitl/offboard_mission.py --out "$OUT/result.json" --wall-timeout "${YOPO_SITL_WALL_TIMEOUT:-900}" \
      --mission goto --goto-target-enu 6,0,0
    rc=$?
  else
    echo "swap 需要 3 架（YOPO_SITL_N=3 YOPO_SITL_MISSION_MODE=swap）" >&2
    rc=2
  fi
  set -e
  finish_mission_rc "$rc"
fi

case "$N" in
  2|3|6) ;;
  *)
    echo "YOPO_SITL_N=$N 超出支持范围（1、2、3 或 6）" >&2
    exit 2
    ;;
esac

python3 /repo/scripts/sitl/fleet.py --n "$N" --format shell > "$OUT/layout.txt"
if [[ "$PLANNER" == "yopo-l0" && "$N" -gt 1 ]]; then
  # S2 沿 y 轴 0/3/6 m 生成。swap-3-open 的起点在半径 8 m 的圆上，两者不是同一套点。
  # 实飞以场景卡为准写 PX4_GZ_MODEL_POSE。桥接用同一水平偏置把本地里程计抬到世界系。
  python3 /repo/scripts/sitl/fleet.py yopo-poses \
    --scene "${YOPO_L0_SCENE:-$(default_yopo_scene)}" \
    --n "$N" \
    --layout "$OUT/layout.txt" > "$OUT/layout.yopo.txt"
  mv "$OUT/layout.yopo.txt" "$OUT/layout.txt"
  echo "yopo scene-card poses: $(grep '^pose=' "$OUT/layout.txt" | tr '\n' ' ')"
fi
XRCE_PORT="$(awk -F= '/^xrce_port=/{print $2; exit}' "$OUT/layout.txt")"
DDS_DOMAIN_ID="$(awk -F= '/^dds_domain_id=/{print $2; exit}' "$OUT/layout.txt")"
if [[ -z "${XRCE_PORT}" || -z "${DDS_DOMAIN_ID}" ]]; then
  echo "布局缺少 xrce_port 或 dds_domain_id" >&2
  exit 2
fi
if [[ -n "${ROS_DOMAIN_ID:-}" && "$ROS_DOMAIN_ID" != "$DDS_DOMAIN_ID" ]]; then
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID 与布局 dds_domain_id=$DDS_DOMAIN_ID 不是同一个 DDS 域" >&2
  exit 2
fi
echo "$DDS_DOMAIN_ID" > "$OUT/dds_domain_id.txt"
MicroXRCEAgent udp4 -p "$XRCE_PORT" > "$OUT/agent.log" 2>&1 &

inst="" pose="" standalone=""
start_vehicle() {
  [[ -n "$inst" ]] || return 0
  local dir="$OUT/px4-state/$inst" pid
  mkdir -p "$dir"
  echo "start instance=$inst pose=${pose:-<model default>} standalone=$standalone"
  (
    cd "$dir"
    if [[ "$standalone" == 1 ]]; then
      export PX4_GZ_STANDALONE=1
    fi
    if [[ -n "$pose" ]]; then
      export PX4_GZ_MODEL_POSE="$pose"
    fi
    # 不设 PX4_UXRCE_DDS_NS：instance>0 时由 PX4 rcS 加 namespace px4_<instance>。
    # -i 放在 rootfs 路径前面：px4_getopt 遇到非选项会停。
    exec "$BUILD/bin/px4" -d -i "$inst" "$BUILD/etc"
  ) > "$OUT/px4-${inst}.log" 2>&1 &
  pid=$!
  if ! wait_px4 "$OUT/px4-${inst}.log" "$pid"; then
    echo "PX4 instance $inst 提前退出，见 px4-${inst}.log" >&2
    exit 3
  fi
  "$BUILD/bin/px4-param" --instance "$inst" show -c > "$OUT/px4-${inst}-params-changed.txt" 2>&1 || true
  inst="" pose="" standalone=""
}

while IFS= read -r line || [[ -n "${line:-}" ]]; do
  case "$line" in
    ---) start_vehicle ;;
    n=*|dds_domain_id=*|xrce_port=*|mav_sys_id=*|namespace=*|model=*|gcs_port=*) ;;
    instance=*) inst="${line#*=}" ;;
    pose=*) pose="${line#*=}" ;;
    standalone=*) standalone="${line#*=}" ;;
    '') ;;
    *) echo "无法识别的布局行：$line" >&2; exit 2 ;;
  esac
done < "$OUT/layout.txt"
start_vehicle

echo "px4_startup_wall_s $SECONDS"
date -Is > "$OUT/ready"
if [[ "$MISSION" == 0 ]]; then
  echo "sim ready n=$N domain=$DDS_DOMAIN_ID mission=0 mode=$MISSION_MODE; waiting"
  wait
  exit 0
fi
set +e
if [[ "$PLANNER" == "yopo-l0" ]]; then
  run_yopo_l0
  rc=$?
else
  python3 /repo/scripts/sitl/fleet.py run --out "$OUT/result.json" --n "$N" \
    --mission "$MISSION_MODE" --wall-timeout "${YOPO_SITL_WALL_TIMEOUT:-900}"
  rc=$?
fi
set -e
finish_mission_rc "$rc"
