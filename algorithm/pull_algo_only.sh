#!/bin/bash
# 只拉算法源码/配置（可阅读、可调用），跳过语音包与大资源
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
KEY="$ROOT/.pull_key"
HOST="wheeltec@192.168.0.105"
export RSYNC_RSH="ssh -i $KEY -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"
PROGRESS="$ROOT/progress.json"
LOG="$ROOT/pull.log"
START_TS=$(date +%s)

# 仅包含这些扩展；目录先放行，最后排除其余文件
INC=(
  --include '*/'
  --include '*.py'
  --include '*.cpp'
  --include '*.cc'
  --include '*.cxx'
  --include '*.c'
  --include '*.h'
  --include '*.hpp'
  --include '*.hh'
  --include '*.launch'
  --include '*.launch.py'
  --include '*.yaml'
  --include '*.yml'
  --include '*.xml'
  --include '*.msg'
  --include '*.srv'
  --include '*.action'
  --include '*.idl'
  --include '*.xacro'
  --include '*.urdf'
  --include '*.md'
  --include '*.txt'
  --include '*.json'
  --include '*.cfg'
  --include '*.rviz'
  --include '*.lua'
  --include '*.sh'
  --include 'CMakeLists.txt'
  --include 'package.xml'
  --include 'setup.py'
  --include 'setup.cfg'
  --include 'COLCON_IGNORE'
  --exclude '*'
)

# 整包跳过（语音/资源型，不是调用算法所需）
SKIP_DIR=(
  --exclude 'wheeltec_mic/'
  --exclude 'wheeltec_mic_aiui/'
  --exclude 'tts_make_ros2/'
  --exclude 'wheeltec_bodyreader/'
  --exclude 'OrbbecSDK_ROS2-main/orbbec_camera/SDK/lib/'
  --exclude 'data/'
)

STEPS=(
  "1/6 wheeltec_ros2|~/wheeltec_ros2/src/|$ROOT/wheeltec_ros2/src/"
  "2/6 autoware|~/autoware/src/|$ROOT/autoware/src/"
  "3/6 wheeltec_arm|~/wheeltec_arm/src/|$ROOT/wheeltec_arm/src/"
  "4/6 wheeltec_stepper_arm|~/wheeltec_stepper_arm/|$ROOT/wheeltec_stepper_arm/"
  "5/6 wheeltec_test|~/wheeltec_test/|$ROOT/wheeltec_test/"
  "6/6 jetracer+jetcam|~/jetracer/|$ROOT/jetracer/"
)

TOTAL=${#STEPS[@]}

json_escape() { python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$1"; }

write_progress() {
  local idx="$1" name="$2" status="$3" detail="$4" pct="$5" done_flag="${6:-false}"
  local now elapsed eta local_bytes speed
  now=$(date '+%Y-%m-%d %H:%M:%S')
  elapsed=$(( $(date +%s) - START_TS ))
  local_bytes=$(du -sk "$ROOT" --exclude='.pull_key' 2>/dev/null | awk '{print $1*1024}')
  # mac du has no --exclude; count algo dirs only
  local_bytes=$(du -sk "$ROOT/wheeltec_ros2" "$ROOT/autoware" "$ROOT/wheeltec_arm" "$ROOT/wheeltec_stepper_arm" "$ROOT/wheeltec_test" "$ROOT/jetracer" "$ROOT/jetcam" "$ROOT/home_scripts" 2>/dev/null | awk '{s+=$1} END{print s*1024}')
  [ -z "$local_bytes" ] && local_bytes=0
  if [ "$pct" -gt 0 ] && [ "$pct" -lt 100 ] && [ "$elapsed" -gt 0 ]; then
    eta=$(( elapsed * (100 - pct) / pct ))
  else
    eta=0
  fi
  speed=$(( elapsed > 0 ? local_bytes / elapsed : 0 ))
  cat > "$PROGRESS.tmp" <<EOF
{
  "updated_at": "$now",
  "status": $(json_escape "$status"),
  "step_index": $idx,
  "total_steps": $TOTAL,
  "step_name": $(json_escape "$name"),
  "detail": $(json_escape "$detail"),
  "percent": $pct,
  "elapsed_sec": $elapsed,
  "eta_sec": $eta,
  "local_bytes": ${local_bytes:-0},
  "speed_bps": $speed,
  "done": $done_flag
}
EOF
  mv "$PROGRESS.tmp" "$PROGRESS"
  printf "\r[%3d%%] %s — %s" "$pct" "$name" "$detail"
}

echo "清理旧的大资源残留（只保留目录结构，重新拉源码）…"
# 清掉已下的二进制残留，避免混在一起
rm -rf "$ROOT/wheeltec_ros2/src" "$ROOT/autoware/src" "$ROOT/wheeltec_arm/src" \
       "$ROOT/robot_voice" "$ROOT/jetracer" "$ROOT/jetcam" \
       "$ROOT/wheeltec_stepper_arm" "$ROOT/wheeltec_test" "$ROOT/misc"
mkdir -p "$ROOT"/{wheeltec_ros2/src,autoware/src,wheeltec_arm/src,wheeltec_stepper_arm,wheeltec_test,jetracer,jetcam,home_scripts}
: > "$LOG"

write_progress 0 "准备" "running" "仅同步算法源码/配置（跳过语音与大资源）" 1

idx=0
for item in "${STEPS[@]}"; do
  idx=$((idx + 1))
  name="${item%%|*}"
  rest="${item#*|}"
  remote="${rest%%|*}"
  localp="${rest#*|}"
  pct=$(( (idx - 1) * 100 / TOTAL ))
  write_progress "$idx" "$name" "running" "拉取 $remote" "$pct"
  echo -e "\n=== $name $(date) ===" >> "$LOG"
  mkdir -p "$localp"
  # SKIP_DIR only meaningful for ros2 workspace; harmless elsewhere
  set +e
  rsync -a "${SKIP_DIR[@]}" "${INC[@]}" "$HOST:$remote" "$localp" >> "$LOG" 2>&1
  rc=$?
  set -e
  size=$(du -sh "$localp" 2>/dev/null | awk '{print $1}')
  if [ $rc -ne 0 ]; then
    write_progress "$idx" "$name" "error" "失败 rc=$rc，已有 $size" "$pct"
    echo ""
    continue
  fi
  end=$(( idx * 100 / TOTAL ))
  write_progress "$idx" "$name" "running" "完成 $size" "$end"
  echo ""
done

# jetcam separate (step 6 only did jetracer path) — quick
write_progress 6 "jetcam" "running" "拉取 jetcam 源码" 92
rsync -a "${INC[@]}" "$HOST:~/jetcam/" "$ROOT/jetcam/" >> "$LOG" 2>&1 || true

write_progress 6 "home_scripts" "running" "拉取调用脚本" 95
rsync -a \
  "$HOST:~/prepare-ab-navigation.py" \
  "$HOST:~/ab-navigation.launch.py" \
  "$HOST:~/network-config.py" \
  "$HOST:~/cyclone-network-config.py" \
  "$HOST:~/ip_serial_sender.c" \
  "$HOST:~/gps_waypoints.yaml" \
  "$HOST:~/cyclonedds.xml" \
  "$ROOT/home_scripts/" >> "$LOG" 2>&1 || true

# 包名清单
ssh -i "$KEY" -o IdentitiesOnly=yes "$HOST" 'ls ~/wheeltec_ros2/src | sort' > "$ROOT/_remote_ros2_pkgs.txt" 2>>"$LOG" || true
ssh -i "$KEY" -o IdentitiesOnly=yes "$HOST" 'ls ~/wheeltec_arm/src 2>/dev/null | sort' > "$ROOT/_remote_arm_pkgs.txt" 2>>"$LOG" || true

TOTAL_SIZE=$(du -sh "$ROOT/wheeltec_ros2" "$ROOT/autoware" "$ROOT/wheeltec_arm" "$ROOT/wheeltec_stepper_arm" "$ROOT/wheeltec_test" "$ROOT/jetracer" "$ROOT/jetcam" "$ROOT/home_scripts" 2>/dev/null | awk '{print} END{}')
SUM=$(du -sh "$ROOT/wheeltec_ros2" "$ROOT/autoware" "$ROOT/wheeltec_arm" "$ROOT/home_scripts" "$ROOT/jetracer" "$ROOT/jetcam" "$ROOT/wheeltec_stepper_arm" "$ROOT/wheeltec_test" 2>/dev/null | awk '{s+=$1} END{print}')
BYTES=$(du -sk "$ROOT/wheeltec_ros2" "$ROOT/autoware" "$ROOT/wheeltec_arm" "$ROOT/home_scripts" "$ROOT/jetracer" "$ROOT/jetcam" "$ROOT/wheeltec_stepper_arm" "$ROOT/wheeltec_test" 2>/dev/null | awk '{s+=$1} END{print s*1024}')
elapsed=$(( $(date +%s) - START_TS ))
write_progress "$TOTAL" "全部完成" "done" "算法源码已就绪，用时 ${elapsed}s" 100 true
echo ""
echo "ALL_PULL_DONE"
du -sh "$ROOT/wheeltec_ros2" "$ROOT/autoware" "$ROOT/wheeltec_arm" "$ROOT/jetracer" "$ROOT/jetcam" "$ROOT/wheeltec_stepper_arm" "$ROOT/wheeltec_test" "$ROOT/home_scripts" 2>/dev/null
find "$ROOT/wheeltec_ros2/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l
