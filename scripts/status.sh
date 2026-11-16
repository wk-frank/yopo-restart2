#!/usr/bin/env bash
# 票 C2：只读状态快照。
set -u

REPO_EXPLICIT=0
[[ -v YOPO_REPO_DIR ]] && REPO_EXPLICIT=1

YOPO_OPS_DIR=${YOPO_OPS_DIR:-/workspace/yopo-ops}
WT_YOPO=${WT_YOPO:-/home/box/opt/worktrunk/bin/wt-yopo-restart}
YOPO_REPO_DIR=${YOPO_REPO_DIR:-/workspace/yopo-restart}
GH=${GH:-gh}
SCRIPT_ROOT=$(cd -- "$(dirname -- "$0")/.." 2>/dev/null && pwd)

parse_repo_slug() {
  local url=${1:-}
  url=${url%/}
  url=${url%.git}
  if [[ "$url" =~ ^(https?|ssh)://([^/@]+@)?[^/:]+(:[0-9]+)?/([^/:]+/[^/]+)$ ]]; then
    printf '%s\n' "${BASH_REMATCH[4]}"
    return 0
  fi
  if [[ "$url" =~ ^([^/@]+@)?[^/:]+:([^/:]+/[^/]+)$ ]]; then
    printf '%s\n' "${BASH_REMATCH[2]}"
    return 0
  fi
  return 1
}

resolve_gh_repo() {
  if [[ -n "${YOPO_GH_REPO:-}" ]]; then
    printf '%s\n' "$YOPO_GH_REPO"
    return 0
  fi
  local dirs=("$YOPO_REPO_DIR")
  if [[ "$REPO_EXPLICIT" -eq 0 ]]; then
    dirs+=(".")
    [[ -n "${SCRIPT_ROOT:-}" ]] && dirs+=("$SCRIPT_ROOT")
  fi
  local dir url slug
  for dir in "${dirs[@]}"; do
    [[ -d "$dir" ]] || continue
    if url=$(GIT_OPTIONAL_LOCKS=0 git -C "$dir" remote get-url origin 2>/dev/null); then
      if slug=$(parse_repo_slug "$url"); then
        printf '%s\n' "$slug"
        return 0
      fi
    fi
  done
  printf '%s\n' 'unununnnn/yopo-restart'
}

GH_REPO=$(resolve_gh_repo)

printf '生成时间：%s\n' "$(TZ=America/Argentina/San_Juan date '+%F %T UTC-3')"

printf '\n## 开放 PR\n'
printf '| 号 | 标题 | head sha | CI |\n'
printf '|---|---|---|---|\n'
if command -v "$GH" >/dev/null 2>&1; then
  if pr_json=$("$GH" pr list -R "$GH_REPO" --state open --json number,title,headRefOid 2>/dev/null); then
    if pr_lines=$(printf '%s\n' "$pr_json" | PYTHONDONTWRITEBYTECODE=1 python3 -c '
import json
import sys

try:
    rows = json.load(sys.stdin)
    if not isinstance(rows, list):
        raise ValueError()
    lines = []
    for row in rows:
        number = row["number"]
        title = str(row["title"]).replace("\r", " ").replace("\n", " ").replace("|", "\\|")
        sha = str(row["headRefOid"])[:7]
        lines.append(f"| #{number} | {title} | {sha} | 无 |")
except (KeyError, TypeError, ValueError):
    sys.exit(1)
sys.stdout.write("\n".join(lines))
'); then
      if [[ -n "$pr_lines" ]]; then
        printf '%s\n' "$pr_lines"
      else
        printf '（无）\n'
      fi
    else
      printf '（gh 输出不可解析）\n'
    fi
  else
    printf '（gh 查询失败）\n'
  fi
else
  printf '（gh 不可用）\n'
fi

printf '\n## 队列与结果\n'
for name in queue.txt results.tsv; do
  printf '### %s\n' "$name"
  if [[ -f "$YOPO_OPS_DIR/$name" ]]; then
    if ! tail -n 8 -- "$YOPO_OPS_DIR/$name" 2>/dev/null; then
      printf '（读取失败）\n'
    fi
  else
    printf '（无）\n'
  fi
  printf '\n'
done

printf '## 进程\n'
pid_count=0
while IFS= read -r -d '' pid_file; do
  pid_count=$((pid_count + 1))
  pid=$(head -n 1 -- "$pid_file" 2>/dev/null)
  if [[ "$pid" =~ ^[[:space:]]*([1-9][0-9]*)[[:space:]]*$ ]]; then
    pid=${BASH_REMATCH[1]}
    if kill -0 "$pid" 2>/dev/null; then
      state=活
    else
      state=死
    fi
  else
    pid=无效
    state=死
  fi

  log_file=${pid_file%.pid}.log
  if [[ -f "$log_file" ]]; then
    if last_line=$(tail -n 1 -- "$log_file" 2>/dev/null); then
      [[ -n "$last_line" ]] || last_line='（空）'
    else
      last_line=无日志
    fi
  else
    last_line=无日志
  fi
  printf '%s | PID：%s | %s | 日志：%s\n' "$pid_file" "$pid" "$state" "$last_line"
done < <(find "$YOPO_OPS_DIR" -maxdepth 3 -type f -name '*.pid' -print0 2>/dev/null)
if [[ "$pid_count" -eq 0 ]]; then
  printf '（无）\n'
fi

printf '\n## 工作树\n'
if command -v "$WT_YOPO" >/dev/null 2>&1; then
  if wt_output=$("$WT_YOPO" -C "$YOPO_REPO_DIR" list 2>/dev/null); then
    if [[ -n "$wt_output" ]]; then
      printf '%s\n' "$wt_output"
    else
      printf '（无）\n'
    fi
  else
    printf '（wt-yopo-restart 查询失败）\n'
  fi
else
  printf '（wt-yopo-restart 不可用）\n'
fi
