#!/usr/bin/env bash
# 票 S1：一键构建单机 PX4 SITL 镜像（幂等）。
# 票 S2：--n 3 在镜像就绪后拉起三架 x500（headless，不起飞）。同一 DDS 域。
#   scripts/sitl/up.sh                 前台构建；镜像已存在直接退出 0
#   scripts/sitl/up.sh --bg            setsid nohup 后台构建，打印 PID 文件与日志路径后立即返回
#   scripts/sitl/up.sh --status        只报镜像状态：ready / building(PID) / missing
#   scripts/sitl/up.sh --n 3           镜像没有就先构建，然后拉起 3 架
#   scripts/sitl/up.sh --n 3 --status
#   scripts/sitl/up.sh --n 3 --down    停掉这一趟仿真
#   scripts/sitl/up.sh --n 3 --positioning rtk
# 构建用 nice 15、PX4 编译并行 4。脚本不改 PX4 安全参数；定位只走 S1b 白名单。
set -euo pipefail
# shellcheck source=common.sh
. "$(dirname "$0")/common.sh"
mkdir -p "$SITL_STATE"
PIDF="$SITL_STATE/build.pid"
LOG="$SITL_STATE/build-$SITL_HASH.log"

N=""
BG=0
STATUS=0
DOWN=0
POSITIONING="${YOPO_POSITIONING:-default}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bg) BG=1; shift ;;
    --status) STATUS=1; shift ;;
    --down) DOWN=1; shift ;;
    --n) N="${2:-}"; shift 2 ;;
    --n=*) N="${1#*=}"; shift ;;
    --positioning) POSITIONING="${2:-}"; shift 2 ;;
    --positioning=*) POSITIONING="${1#*=}"; shift ;;
    -h|--help)
      echo "用法：$0 [--bg|--status] [--n 1|2|3|6] [--positioning default|rtk] [--down]"
      exit 0 ;;
    *) echo "用法：$0 [--bg|--status] [--n 1|2|3|6] [--positioning default|rtk] [--down]" >&2; exit 2 ;;
  esac
done
if [[ "$POSITIONING" != "default" && "$POSITIONING" != "rtk" ]]; then
  echo "未知 --positioning=$POSITIONING（只要 default 或 rtk）" >&2
  exit 2
fi

image_status() {
  if image_exists; then echo "ready $SITL_IMAGE"
  elif pid_alive "$PIDF"; then echo "building pid=$(cat "$PIDF") log=$LOG"
  else echo "missing $SITL_IMAGE"; fi
}

build_image() {
  if image_exists; then echo "ready $SITL_IMAGE"; return 0; fi
  t0=$(date +%s)
  echo "build $SITL_IMAGE start $(date -Is)"
  DOCKER_BUILDKIT=1 nice -n "$SITL_NICE" "${DOCKER[@]}" build \
    --build-arg JOBS="$SITL_JOBS" --build-arg NICE="$SITL_NICE" \
    -t "$SITL_IMAGE" -f "$SITL_DIR/Dockerfile" "$SITL_DIR"
  t1=$(date +%s)
  size=$("${DOCKER[@]}" image inspect -f '{{.Size}}' "$SITL_IMAGE")
  printf 'build_seconds\t%s\nimage\t%s\nimage_bytes\t%s\n' "$((t1 - t0))" "$SITL_IMAGE" "$size" \
    | tee "$SITL_STATE/build-$SITL_HASH.tsv"
  echo "build done $(date -Is)"
}

require_n() {
  python3 "$SITL_DIR/fleet.py" --n "$N" --format json >/dev/null
}

stack_file() { printf '%s\n' "$SITL_STATE/stack-n${1}.txt"; }

start_stack() {
  local n=$1 pos=$2 name run others
  name="$(sitl_container_name "$n")"
  if "${DOCKER[@]}" ps -q -f "name=^${name}$" | grep -q .; then
    echo "running name=$name run_dir=$(cat "$(stack_file "$n")" 2>/dev/null || true)"
    return 0
  fi
  # 上一趟已经退出的同名容器会占着名字。
  "${DOCKER[@]}" rm -f "$name" >/dev/null 2>&1 || true
  others="$(sitl_running_names | grep -v "^${name}$" || true)"
  if [[ -n "$others" ]]; then
    echo "已有仿真容器在跑（一次只起一趟）：$others" >&2
    return 3
  fi
  run="$SITL_STATE/runs/$(date +%Y%m%dT%H%M%S)-n${n}"
  mkdir -p "$run"
  printf '%s\n' "$run" > "$(stack_file "$n")"
  local shm=1g
  [[ "$n" -gt 1 ]] && shm=2g
  [[ "$n" -ge 6 ]] && shm=4g
  "${DOCKER[@]}" run -d --name "$name" \
    -e ROS_DOMAIN_ID=0 -e YOPO_SITL_N="$n" -e YOPO_SITL_MISSION=0 \
    -e YOPO_POSITIONING="$pos" \
    -v "$REPO_ROOT:/repo:ro" -v "$run:/out" --shm-size "$shm" \
    "$SITL_IMAGE" nice -n "$SITL_NICE" bash /repo/scripts/sitl/in_container.sh >/dev/null
  echo "started name=$name run_dir=$run n=$n positioning=$pos domain=0"
}

if [[ -n "$N" ]]; then
  require_n
fi

if [[ "$STATUS" == 1 && -z "$N" ]]; then
  image_status
  exit 0
fi
if [[ "$STATUS" == 1 ]]; then
  name="$(sitl_container_name "$N")"
  img="$(image_status)"
  if "${DOCKER[@]}" ps -q -f "name=^${name}$" | grep -q .; then
    echo "$img sim=running name=$name run_dir=$(cat "$(stack_file "$N")" 2>/dev/null || true)"
  else
    echo "$img sim=stopped name=$name"
  fi
  exit 0
fi

if [[ "$DOWN" == 1 ]]; then
  [[ -n "$N" ]] || { echo "--down 要和 --n 一起用" >&2; exit 2; }
  name="$(sitl_container_name "$N")"
  "${DOCKER[@]}" rm -f "$name" >/dev/null
  rm -f "$(stack_file "$N")"
  echo "stopped $name"
  exit 0
fi

if [[ "$BG" == 1 && -z "$N" ]]; then
  if image_exists; then echo "ready $SITL_IMAGE"; exit 0; fi
  if pid_alive "$PIDF"; then echo "building pid=$(cat "$PIDF") log=$LOG"; exit 0; fi
  setsid nohup "$0" >"$LOG" 2>&1 < /dev/null &
  echo $! > "$PIDF"
  echo "started pid=$! pidfile=$PIDF log=$LOG"
  exit 0
fi
if [[ "$BG" == 1 ]]; then
  name="$(sitl_container_name "$N")"
  if image_exists && "${DOCKER[@]}" ps -q -f "name=^${name}$" | grep -q .; then
    echo "running name=$name run_dir=$(cat "$(stack_file "$N")" 2>/dev/null || true)"
    exit 0
  fi
  pidf="$SITL_STATE/up-n${N}.pid"
  if pid_alive "$pidf"; then echo "starting pid=$(cat "$pidf") log=$LOG"; exit 0; fi
  setsid nohup "$0" --n "$N" --positioning "$POSITIONING" >"$LOG" 2>&1 < /dev/null &
  echo $! > "$pidf"
  echo $! > "$PIDF"
  echo "started pid=$! pidfile=$pidf log=$LOG n=$N positioning=$POSITIONING"
  exit 0
fi

if [[ -z "$N" ]]; then
  if image_exists; then echo "ready $SITL_IMAGE"; exit 0; fi
  build_image
  exit 0
fi

build_image
start_stack "$N" "$POSITIONING"
