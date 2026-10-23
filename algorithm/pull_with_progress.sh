#!/bin/bash
# Pull algorithm sources from Wheeltec with live progress.json + console bar
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
KEY="$ROOT/.pull_key"
HOST="wheeltec@192.168.0.105"
export RSYNC_RSH="ssh -i $KEY -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"

PROGRESS="$ROOT/progress.json"
LOG="$ROOT/pull.log"
MONITOR_PID=""

EX=(
  --exclude build/ --exclude install/ --exclude log/ --exclude .Trash-1000/
  --exclude __pycache__/ --exclude '*.pyc' --exclude .git/
  --exclude '*.so' --exclude '*.so.*' --exclude '*.a' --exclude '*.dylib'
  --exclude '*.pt' --exclude '*.pth' --exclude '*.onnx' --exclude '*.engine' --exclude '*.trt'
  --exclude '*.jet' --exclude '*.pcm' --exclude '*.wav' --exclude '*.mp3'
  --exclude '*.bag' --exclude '*.mcap' --exclude '*.db3'
  --exclude '*.STL' --exclude '*.stl' --exclude '*.dae' --exclude '*.obj' --exclude '*.FBX' --exclude '*.fbx'
  --exclude '*.png' --exclude '*.jpg' --exclude '*.jpeg' --exclude '*.gif' --exclude '*.mp4'
  --exclude '*/SDK/lib/*' --exclude '*/tmp/*' --exclude '*/AIUI/assets/*'
  --exclude '*.deb' --exclude '*.whl' --exclude '*.zip' --exclude '*.tar' --exclude '*.tar.gz'
)

STEPS=(
  "1/8 wheeltec_ros2|~/wheeltec_ros2/src/|$ROOT/wheeltec_ros2/src/"
  "2/8 autoware|~/autoware/src/|$ROOT/autoware/src/"
  "3/8 wheeltec_arm|~/wheeltec_arm/src/|$ROOT/wheeltec_arm/src/"
  "4/8 robot_voice|~/robot_voice/|$ROOT/robot_voice/"
  "5/8 wheeltec_stepper_arm|~/wheeltec_stepper_arm/|$ROOT/wheeltec_stepper_arm/"
  "6/8 wheeltec_test|~/wheeltec_test/|$ROOT/wheeltec_test/"
  "7/8 jetracer|~/jetracer/|$ROOT/jetracer/"
  "8/8 jetcam|~/jetcam/|$ROOT/jetcam/"
)

TOTAL_STEPS=${#STEPS[@]}
START_TS=$(date +%s)

json_escape() {
  python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$1"
}

write_progress() {
  local step_idx="$1" step_name="$2" status="$3" detail="$4" pct="$5"
  local now elapsed eta_sec speed local_bytes
  now=$(date '+%Y-%m-%d %H:%M:%S')
  elapsed=$(( $(date +%s) - START_TS ))
  local_bytes=$(du -sk "$ROOT" 2>/dev/null | awk '{print $1*1024}')
  if [ "$pct" -gt 0 ] && [ "$elapsed" -gt 0 ] && [ "$pct" -lt 100 ]; then
    eta_sec=$(( elapsed * (100 - pct) / pct ))
  elif [ "$pct" -ge 100 ]; then
    eta_sec=0
  else
    eta_sec=-1
  fi
  if [ "$elapsed" -gt 0 ]; then
    speed=$(( local_bytes / elapsed ))
  else
    speed=0
  fi
  cat > "$PROGRESS.tmp" <<EOF
{
  "updated_at": "$now",
  "status": $(json_escape "$status"),
  "step_index": $step_idx,
  "total_steps": $TOTAL_STEPS,
  "step_name": $(json_escape "$step_name"),
  "detail": $(json_escape "$detail"),
  "percent": $pct,
  "elapsed_sec": $elapsed,
  "eta_sec": $eta_sec,
  "local_bytes": $local_bytes,
  "speed_bps": $speed,
  "done": false
}
EOF
  mv "$PROGRESS.tmp" "$PROGRESS"
}

bar() {
  local pct="$1" width=36
  local filled=$(( pct * width / 100 ))
  local empty=$(( width - filled ))
  local out="["
  local i
  for ((i=0; i<filled; i++)); do out+="#"; done
  for ((i=0; i<empty; i++)); do out+="-"; done
  out+="] ${pct}%"
  printf "\r%s  %s" "$out" "$2"
}

stop_monitor() {
  if [ -n "${MONITOR_PID}" ] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    kill "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
  MONITOR_PID=""
}

start_monitor() {
  local step_idx="$1" step_name="$2" base_pct="$3" end_pct="$4" localp="$5"
  stop_monitor
  (
    local tick=0 last_bytes=0
    while true; do
      tick=$((tick + 1))
      bytes=$(du -sk "$ROOT" 2>/dev/null | awk '{print $1*1024}')
      step_bytes=$(du -sk "$localp" 2>/dev/null | awk '{print $1*1024}')
      # slowly advance within step band while files keep changing
      span=$(( end_pct - base_pct ))
      if [ "$span" -lt 1 ]; then span=1; fi
      if [ "$bytes" -gt "$last_bytes" ]; then
        last_bytes=$bytes
        # map growth into mid-step progress, capped below end_pct
        intra=$(( tick < span ? tick : span - 1 ))
      else
        intra=$(( tick / 3 ))
        if [ "$intra" -ge "$span" ]; then intra=$(( span - 1 )); fi
      fi
      if [ "$intra" -lt 1 ]; then intra=1; fi
      pct=$(( base_pct + intra ))
      if [ "$pct" -ge "$end_pct" ]; then pct=$(( end_pct - 1 )); fi
      if [ "$pct" -lt "$base_pct" ]; then pct=$base_pct; fi
      detail="同步中… 本步约 $(du -sh "$localp" 2>/dev/null | awk '{print $1}') | 合计 $(du -sh "$ROOT" 2>/dev/null | awk '{print $1}')"
      write_progress "$step_idx" "$step_name" "running" "$detail" "$pct"
      bar "$pct" "$step_name 同步中…                 "
      sleep 2
    done
  ) &
  MONITOR_PID=$!
}

trap 'stop_monitor' EXIT

mkdir -p "$ROOT"/{wheeltec_ros2/src,wheeltec_arm/src,autoware/src,robot_voice,wheeltec_stepper_arm,wheeltec_test,jetracer,jetcam,home_scripts,misc}
: > "$LOG"
write_progress 0 "准备中" "running" "开始同步算法源码（已排除大体积二进制）" 1
echo "开始拉取 → 进度页: http://127.0.0.1:8790/progress.html"
echo "日志: $LOG"

idx=0
for item in "${STEPS[@]}"; do
  idx=$((idx + 1))
  name="${item%%|*}"
  rest="${item#*|}"
  remote="${rest%%|*}"
  localp="${rest#*|}"
  base_pct=$(( (idx - 1) * 100 / TOTAL_STEPS ))
  end_pct=$(( idx * 100 / TOTAL_STEPS ))
  mkdir -p "$localp"
  write_progress "$idx" "$name" "running" "正在同步 $remote" "$base_pct"
  bar "$base_pct" "$name 开始…                 "
  echo "" >> "$LOG"
  echo "=== $name === $(date)" >> "$LOG"

  start_monitor "$idx" "$name" "$base_pct" "$end_pct" "$localp"

  set +e
  rsync -a --progress "${EX[@]}" \
    "$HOST:$remote" "$localp" >> "$LOG" 2>&1
  rc=$?
  set -e

  stop_monitor

  size=$(du -sh "$localp" 2>/dev/null | awk '{print $1}')
  if [ $rc -ne 0 ]; then
    write_progress "$idx" "$name" "error" "rsync 退出码 $rc，已同步约 $size（见 pull.log）" "$base_pct"
    bar "$base_pct" "$name 失败(rc=$rc) 继续…   "
    echo ""
    continue
  fi
  write_progress "$idx" "$name" "running" "完成 $name（$size）" "$end_pct"
  bar "$end_pct" "$name 完成 ($size)              "
  echo ""
done

write_progress "$TOTAL_STEPS" "home_scripts" "running" "同步家目录脚本" 95
rsync -a \
  "$HOST:~/prepare-ab-navigation.py" \
  "$HOST:~/ab-navigation.launch.py" \
  "$HOST:~/network-config.py" \
  "$HOST:~/cyclone-network-config.py" \
  "$HOST:~/ip_serial_sender.c" \
  "$HOST:~/gps_waypoints.yaml" \
  "$HOST:~/cyclonedds.xml" \
  "$ROOT/home_scripts/" >> "$LOG" 2>&1 || true

rsync -a "${EX[@]}" --max-size=20m "$HOST:~/torch2trt/" "$ROOT/misc/torch2trt/" >> "$LOG" 2>&1 || true
rsync -a "${EX[@]}" --max-size=20m "$HOST:~/Orin_NX_AP_STA/" "$ROOT/misc/Orin_NX_AP_STA/" >> "$LOG" 2>&1 || true

ssh -i "$KEY" -o IdentitiesOnly=yes "$HOST" 'ls ~/wheeltec_ros2/src | sort' > "$ROOT/_remote_ros2_pkgs.txt" 2>>"$LOG" || true
ssh -i "$KEY" -o IdentitiesOnly=yes "$HOST" 'ls ~/wheeltec_arm/src 2>/dev/null | sort' > "$ROOT/_remote_arm_pkgs.txt" 2>>"$LOG" || true

TOTAL=$(du -sh "$ROOT" | awk '{print $1}')
elapsed=$(( $(date +%s) - START_TS ))
now=$(date '+%Y-%m-%d %H:%M:%S')
local_bytes=$(du -sk "$ROOT" 2>/dev/null | awk '{print $1*1024}')
cat > "$PROGRESS" <<EOF
{
  "updated_at": "$now",
  "status": "done",
  "step_index": $TOTAL_STEPS,
  "total_steps": $TOTAL_STEPS,
  "step_name": "全部完成",
  "detail": "本地合计 $TOTAL，用时 ${elapsed}s",
  "percent": 100,
  "elapsed_sec": $elapsed,
  "eta_sec": 0,
  "local_bytes": $local_bytes,
  "speed_bps": $(( elapsed > 0 ? local_bytes / elapsed : 0 )),
  "done": true
}
EOF
bar 100 "全部完成 合计 $TOTAL               "
echo ""
echo "ALL_PULL_DONE $TOTAL"
