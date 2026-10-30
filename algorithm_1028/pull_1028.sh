#!/bin/bash
# Pull the latest Wheeltec sources into this directory. Skips build output and large binaries.
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
KEY="$HOME/.ssh/wheeltec_algorithm_1028"
HOST="wheeltec@192.168.0.105"
unset SSH_ASKPASS SSH_ASKPASS_REQUIRE
export RSYNC_RSH="ssh -i $KEY -o IdentitiesOnly=yes -o PreferredAuthentications=publickey -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15"
LOG="$ROOT/pull.log"

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
  --exclude .pull_key --exclude .pull_key.pub --exclude pull.log --exclude pull_1028.sh
)

STEPS=(
  "wheeltec_ros2|~/wheeltec_ros2/src/|$ROOT/wheeltec_ros2/src/"
  "wheeltec_arm|~/wheeltec_arm/src/|$ROOT/wheeltec_arm/src/"
  "wheeltec_stepper_arm|~/wheeltec_stepper_arm/|$ROOT/wheeltec_stepper_arm/"
  "wheeltec_test|~/wheeltec_test/|$ROOT/wheeltec_test/"
  "jetracer|~/jetracer/|$ROOT/jetracer/"
  "jetcam|~/jetcam/|$ROOT/jetcam/"
)

mkdir -p "$ROOT"/{wheeltec_ros2/src,wheeltec_arm/src,autoware/src,robot_voice,wheeltec_stepper_arm,wheeltec_test,jetracer,jetcam,home_scripts,misc}
: > "$LOG"

for item in "${STEPS[@]}"; do
  name="${item%%|*}"
  rest="${item#*|}"
  remote="${rest%%|*}"
  localp="${rest#*|}"
  mkdir -p "$localp"
  echo "=== $name === $(date)" | tee -a "$LOG"
  rsync -a "${EX[@]}" "$HOST:$remote" "$localp" >> "$LOG" 2>&1
  echo "rc=$? size=$(du -sh "$localp" | awk '{print $1}')" | tee -a "$LOG"
done

echo "=== home_scripts === $(date)" | tee -a "$LOG"
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
echo "ALL_PULL_DONE $(du -sh "$ROOT" | awk '{print $1}')" | tee -a "$LOG"
