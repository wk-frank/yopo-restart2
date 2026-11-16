#!/usr/bin/env bash
# 票 S1：headless 单机 PX4 SITL 冒烟（report-only，不进 scenes/suite.txt）。
# 票 S2：--n 3 让三架各自起飞到 1.5 m 悬停点再降落，按机记录估计/真值误差。
# 票 S3：--mission hover|goto|swap。默认 hover，因此 --n 1 仍是 S1、--n 3 仍是 S2 悬停。
#   scripts/sitl/smoke.sh [--positioning default|rtk] [--n 1|2|3|6] [--mission hover|goto|swap]
#   scripts/sitl/smoke.sh [--positioning default|rtk] [--n 1|2|3|6] [--mission hover|goto|swap] --bg
# --n 默认 1，走 S1：起飞到 2.5 m → 悬停 10 s → 飞到悬停点 +(5, 3, 0) m → NAV_LAND。
# --n 2|3 且 --mission hover：若 up.sh --n 已经拉起同一趟仿真，就在那个容器里补跑任务；否则一次性起仿真并飞完。
# --mission goto：每机在 1.5 m 上飞向悬停点 +(6, 0, 0)。--mission swap：三机错高 1.5/2.2/2.9 m 对飞换位。
# 判据未冻结。ok 只表示每机都落地并上锁，不表示误差达标。
set -euo pipefail
# shellcheck disable=SC1091 source=common.sh
. "$(dirname "$0")/common.sh"
mkdir -p "$SITL_STATE/runs"
PIDF="$SITL_STATE/smoke.pid"
BG=0
N=1
POSITIONING="${YOPO_POSITIONING:-default}"
MISSION_MODE="${YOPO_SITL_MISSION_MODE:-hover}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bg) BG=1; shift ;;
    --n) N="${2:-}"; shift 2 ;;
    --n=*) N="${1#*=}"; shift ;;
    --positioning) POSITIONING="${2:-}"; shift 2 ;;
    --positioning=*) POSITIONING="${1#*=}"; shift ;;
    --mission) MISSION_MODE="${2:-}"; shift 2 ;;
    --mission=*) MISSION_MODE="${1#*=}"; shift ;;
    -h|--help) echo "用法：$0 [--positioning default|rtk] [--n 1|2|3|6] [--mission hover|goto|swap] [--bg]"; exit 0 ;;
    *) echo "用法：$0 [--positioning default|rtk] [--n 1|2|3|6] [--mission hover|goto|swap] [--bg]" >&2; exit 2 ;;
  esac
done
if [[ "$POSITIONING" != "default" && "$POSITIONING" != "rtk" ]]; then
  echo "未知 --positioning=$POSITIONING（只要 default 或 rtk）" >&2
  exit 2
fi
case "$MISSION_MODE" in
  hover|goto|swap) ;;
  *) echo "未知 --mission=$MISSION_MODE（只要 hover、goto 或 swap）" >&2; exit 2 ;;
esac
python3 "$SITL_DIR/fleet.py" layout --n "$N" --mission "$MISSION_MODE" --format json >/dev/null
export YOPO_POSITIONING="$POSITIONING"
export YOPO_SITL_MISSION_MODE="$MISSION_MODE"

if [[ "$BG" == 1 ]]; then
  if pid_alive "$PIDF"; then echo "running pid=$(cat "$PIDF")"; exit 0; fi
  LOG="$SITL_STATE/runs/smoke-$(date +%Y%m%dT%H%M%S)-${POSITIONING}-n${N}-${MISSION_MODE}.log"
  setsid nohup "$0" --positioning "$POSITIONING" --n "$N" --mission "$MISSION_MODE" >"$LOG" 2>&1 < /dev/null &
  echo $! > "$PIDF"
  echo "started pid=$! pidfile=$PIDF log=$LOG positioning=$POSITIONING n=$N mission=$MISSION_MODE"
  exit 0
fi

if ! image_exists; then
  echo "镜像 $SITL_IMAGE 不存在：先跑 scripts/sitl/up.sh（或 --bg）" >&2
  exit 2
fi

NAME="$(sitl_container_name "$N")"
others="$(sitl_running_names | grep -v "^${NAME}$" || true)"
if [[ -n "$others" ]]; then
  echo "已有仿真容器在跑（一次只起一趟）：$others" >&2
  exit 3
fi

summarize_run() {
  local run=$1 rc=$2 t0=$3
  local t1
  t1=$(date +%s.%N)
  if "${DOCKER[@]}" ps -q -f "name=^${NAME}$" | grep -q .; then
    "${DOCKER[@]}" exec "$NAME" chown -R "$(id -u):$(id -g)" /out >/dev/null 2>&1 || true
  else
    "${DOCKER[@]}" run --rm -v "$run:/out" "$SITL_IMAGE" chown -R "$(id -u):$(id -g)" /out >/dev/null 2>&1 || true
  fi
  python3 "$SITL_DIR/summarize.py" "$run" --total-wall "$(python3 -c "print($t1 - $t0)")" --image "$SITL_IMAGE" | tee "$run/summary.md"
  echo "run_dir $run rc=$rc"
}

if [[ "$N" == "1" ]]; then
  if "${DOCKER[@]}" ps -q -f "name=^${NAME}$" | grep -q .; then
    echo "已有一个 $NAME 在跑（只允许 1 个 PX4 实例）" >&2
    exit 3
  fi
  RUN="$SITL_STATE/runs/$(date +%Y%m%dT%H%M%S)"
  mkdir -p "$RUN"
  # ROS_DOMAIN_ID=0：XRCE Agent 不读 ROS 环境变量，PX4 实体落在 UXRCE_DDS_DOM_ID 的默认域 0；
  # 不改这个参数，ROS 侧跟着用 0。容器走自己的 bridge 网络命名空间，不和宿主 ROS 混。
  t0=$(date +%s.%N)
  set +e
  # n=1 且默认 hover 不把 YOPO_SITL_MISSION_MODE 送进容器，单机脚本保持 S1。
  mission_env=()
  if [[ "$MISSION_MODE" != "hover" ]]; then
    mission_env=(-e "YOPO_SITL_MISSION_MODE=$MISSION_MODE")
  fi
  "${DOCKER[@]}" run --rm --name "$NAME" \
    -e ROS_DOMAIN_ID=0 -e YOPO_SITL_WALL_TIMEOUT="${YOPO_SITL_WALL_TIMEOUT:-900}" \
    -e YOPO_POSITIONING="${YOPO_POSITIONING:-default}" \
    "${mission_env[@]}" \
    -v "$REPO_ROOT:/repo:ro" -v "$RUN:/out" --shm-size 1g \
    "$SITL_IMAGE" nice -n "$SITL_NICE" bash /repo/scripts/sitl/in_container.sh 2>&1 | tee "$RUN/console.log"
  rc=${PIPESTATUS[0]}
  set -e
  summarize_run "$RUN" "$rc" "$t0"
  exit "$rc"
fi

# n>1。up.sh 拉起的容器在跑就补任务；否则一次性起仿真并飞完。
RUN=""
if "${DOCKER[@]}" ps -q -f "name=^${NAME}$" | grep -q .; then
  stack="$SITL_STATE/stack-n${N}.txt"
  if [[ ! -f "$stack" ]]; then
    echo "容器 $NAME 在跑，但没有 $stack，不知道输出目录" >&2
    exit 3
  fi
  RUN="$(cat "$stack")"
  echo "等待 $NAME ready（$RUN/ready）"
  ready=0
  for _ in $(seq 1 900); do
    if [[ -f "$RUN/ready" ]]; then ready=1; break; fi
    if ! "${DOCKER[@]}" ps -q -f "name=^${NAME}$" | grep -q .; then
      echo "容器已退出，见 $RUN" >&2
      exit 3
    fi
    sleep 1
  done
  if [[ "$ready" != 1 ]]; then
    echo "900 s 内没有 ready，见 $RUN/px4-*.log" >&2
    exit 3
  fi
  got="$(cat "$RUN/positioning.txt" 2>/dev/null || true)"
  if [[ -n "$got" && "$got" != "$POSITIONING" ]]; then
    echo "仿真已用 positioning=$got 拉起，与本次 --positioning=$POSITIONING 不一致。先 scripts/sitl/up.sh --n $N --down" >&2
    exit 2
  fi
  t0=$(date +%s.%N)
  set +e
  "${DOCKER[@]}" exec \
    -e YOPO_SITL_N="$N" -e YOPO_POSITIONING="$POSITIONING" \
    -e YOPO_SITL_MISSION_MODE="$MISSION_MODE" \
    -e YOPO_SITL_WALL_TIMEOUT="${YOPO_SITL_WALL_TIMEOUT:-900}" \
    "$NAME" nice -n "$SITL_NICE" bash /repo/scripts/sitl/exec_mission.sh 2>&1 | tee -a "$RUN/console.log"
  rc=${PIPESTATUS[0]}
  set -e
  summarize_run "$RUN" "$rc" "$t0"
  exit "$rc"
fi

RUN="$SITL_STATE/runs/$(date +%Y%m%dT%H%M%S)-n${N}"
mkdir -p "$RUN"
# 上一趟已经退出的同名容器会占着名字。
"${DOCKER[@]}" rm -f "$NAME" >/dev/null 2>&1 || true
t0=$(date +%s.%N)
set +e
"${DOCKER[@]}" run --rm --name "$NAME" \
  -e ROS_DOMAIN_ID=0 -e YOPO_SITL_N="$N" -e YOPO_SITL_MISSION=1 \
  -e YOPO_SITL_MISSION_MODE="$MISSION_MODE" \
  -e YOPO_SITL_WALL_TIMEOUT="${YOPO_SITL_WALL_TIMEOUT:-900}" \
  -e YOPO_POSITIONING="${YOPO_POSITIONING:-default}" \
  -v "$REPO_ROOT:/repo:ro" -v "$RUN:/out" --shm-size "$([[ "$N" -ge 6 ]] && echo 4g || echo 2g)" \
  "$SITL_IMAGE" nice -n "$SITL_NICE" bash /repo/scripts/sitl/in_container.sh 2>&1 | tee "$RUN/console.log"
rc=${PIPESTATUS[0]}
set -e
summarize_run "$RUN" "$rc" "$t0"
exit "$rc"
