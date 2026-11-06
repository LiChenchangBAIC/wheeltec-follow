const http = require("http");
const fs = require("fs");
const path = require("path");
const express = require("express");
const { WebSocketServer } = require("ws");
const { Client } = require("ssh2");

const PORT = Number(process.env.PORT) || 8765;
const ROOT = __dirname;
const BRIDGE = fs.readFileSync(path.join(ROOT, "bridge.py"));

// UWB 只读：车上的读取程序已经把快照写好，这里不往小车写任何文件。
const UWB_SNAPSHOT = process.env.UWB_SNAPSHOT || "/home/wheeltec/uwb_uart/latest.json";
const UWB_READ_INTERVAL = Number(process.env.UWB_READ_INTERVAL) || 0.3;
const UWB_MOUNT_YAW = {
  right: Number(process.env.UWB_YAW_RIGHT_DEG || 0),
  left: Number(process.env.UWB_YAW_LEFT_DEG || 0),
};

const app = express();
app.use((req, res, next) => {
  if (req.path.startsWith("/node_modules")) {
    res.status(404).end();
    return;
  }
  next();
});
app.use((req, res, next) => {
  let pathname = req.path;
  try {
    pathname = decodeURIComponent(pathname);
  } catch (_) {}
  if (/^\/[。．.\s]+$/.test(pathname)) {
    res.redirect(302, "/");
    return;
  }
  next();
});
app.get("/", (_req, res) => {
  res.sendFile(path.join(ROOT, "voice.html"));
});
app.use(express.static(ROOT, { index: false, dotfiles: "ignore" }));

const server = http.createServer(app);
const liveWss = new WebSocketServer({ noServer: true, perMessageDeflate: false });
const termWss = new WebSocketServer({ noServer: true, perMessageDeflate: false });

server.on("upgrade", (request, socket, head) => {
  let pathname = "";
  try {
    pathname = new URL(request.url, "http://localhost").pathname;
  } catch (_) {
    socket.destroy();
    return;
  }
  const target = pathname === "/live" ? liveWss : pathname === "/term" ? termWss : null;
  if (!target) {
    socket.destroy();
    return;
  }
  target.handleUpgrade(request, socket, head, (ws) => {
    target.emit("connection", ws, request);
  });
});

function send(ws, payload) {
  if (ws.readyState === ws.OPEN) ws.send(JSON.stringify(payload));
}

function connectSsh(opts) {
  return new Promise((resolve, reject) => {
    const ssh = new Client();
    const timer = setTimeout(() => {
      ssh.end();
      reject(new Error("连接小车超时"));
    }, 12000);
    ssh
      .on("ready", () => {
        clearTimeout(timer);
        resolve(ssh);
      })
      .on("error", (error) => {
        clearTimeout(timer);
        reject(error);
      })
      .connect({
        host: opts.host,
        port: opts.port,
        username: opts.username,
        password: opts.password,
        readyTimeout: 10000,
        keepaliveInterval: 10000,
      });
  });
}

const CAR_UNITS = [
  ["robot-voice-support.service", "底盘、超声波和雷达"],
  ["robot-voice-control.service", "语音"],
  ["robot-follow-scan.service", "跟随用雷达"],
  ["robot-voice-drive.service", "语音行驶"],
];

function execCollect(ssh, command, input, onLine) {
  return new Promise((resolve, reject) => {
    ssh.exec(command, (error, stream) => {
      if (error) {
        reject(error);
        return;
      }
      let stdout = "";
      let stderr = "";
      let pending = "";
      const timer = setTimeout(() => {
        try { stream.close(); } catch (_) {}
      }, 70000);
      stream.on("data", (chunk) => {
        const text = chunk.toString("utf8");
        stdout += text;
        if (!onLine) return;
        pending += text;
        let index;
        while ((index = pending.indexOf("\n")) >= 0) {
          const line = pending.slice(0, index).replace(/\r/g, "").trim();
          pending = pending.slice(index + 1);
          if (line) onLine(line);
        }
      });
      stream.stderr.on("data", (chunk) => {
        stderr += chunk.toString("utf8");
      });
      stream.on("close", (code) => {
        clearTimeout(timer);
        if (onLine && pending.trim()) onLine(pending.replace(/\r/g, "").trim());
        resolve({ code, stdout, stderr });
      });
      stream.end(input);
    });
  });
}

async function startCarUnits(ssh, password, say) {
  const names = CAR_UNITS.map(([unit]) => unit).join(" ");
  const status = await execCollect(
    ssh,
    "bash -s",
    `set +e\nfor unit in ${names}; do\n  if systemctl is-active --quiet "$unit"; then echo "ON $unit"; else echo "OFF $unit"; fi\ndone\n`
  );
  if (status.code !== 0 && !status.stdout.trim()) {
    say("没能查看底盘和语音是否在运行");
    return { code: status.code || 1 };
  }
  const off = [];
  status.stdout.split("\n").forEach((line) => {
    const match = /^(ON|OFF) (\S+)$/.exec(line.trim());
    if (!match) return;
    const known = CAR_UNITS.find(([unit]) => unit === match[2]);
    if (!known) return;
    if (match[1] === "ON") say(`${known[1]}已在运行，不会重新启动`);
    else off.push(known);
  });
  if (!off.length) return { code: 0 };
  const drive = off.find(([unit]) => unit === "robot-voice-drive.service");
  if (drive) say("语音行驶当前没在跑。把它启动后，小车可能按语音或定位移动");
  const start = await execCollect(
    ssh,
    `sudo -S -p '' systemctl start ${off.map(([unit]) => unit).join(" ")}`,
    `${password}\n`
  );
  const error = start.stderr
    .split("\n")
    .map((line) => line.trim())
    .filter((line) => line && !/password/i.test(line))
    .join(" ");
  if (start.code !== 0) {
    say(error || "有的系统服务没能启动");
    return { code: start.code || 1 };
  }
  off.forEach(([, label]) => say(`${label}已启动`));
  return { code: 0 };
}

async function stopCarUnits(ssh, password, say) {
  const names = CAR_UNITS.map(([unit]) => unit).join(" ");
  const status = await execCollect(
    ssh,
    "bash -s",
    `set +e\nfor unit in ${names}; do\n  if systemctl is-active --quiet "$unit"; then echo "ON $unit"; else echo "OFF $unit"; fi\ndone\n`
  );
  if (status.code !== 0 && !status.stdout.trim()) {
    say("没能查看底盘和语音是否在运行");
    return { code: status.code || 1 };
  }
  const on = [];
  status.stdout.split("\n").forEach((line) => {
    const match = /^(ON|OFF) (\S+)$/.exec(line.trim());
    if (!match || match[1] !== "ON") return;
    const known = CAR_UNITS.find(([unit]) => unit === match[2]);
    if (known) on.push(known);
  });
  if (!on.length) {
    say("底盘、语音和跟随雷达已经停着");
    return { code: 0 };
  }
  say("正在停止底盘、语音、跟随雷达和语音行驶。小车会不再按这些服务移动");
  const stop = await execCollect(
    ssh,
    `sudo -S -p '' systemctl stop ${on.map(([unit]) => unit).join(" ")}`,
    `${password}\n`
  );
  const error = stop.stderr
    .split("\n")
    .map((line) => line.trim())
    .filter((line) => line && !/password/i.test(line))
    .join(" ");
  if (stop.code !== 0) {
    say(error || "有的系统服务没能停下");
    return { code: stop.code || 1 };
  }
  on.forEach(([, label]) => say(`${label}已停止`));
  return { code: 0 };
}

async function startVision(ssh, say) {
  return execCollect(ssh, "bash -s", `
set +e
mkdir -p /tmp/baic-hmi
if pgrep -f '[/]astra_camera_node( |$)' >/dev/null; then
  echo "相机和目标检测已在运行"
  exit 0
fi
echo "正在启动相机和目标检测"
nohup /home/wheeltec/robot_voice/start-color-tracker.sh >/tmp/baic-hmi/vision.log 2>&1 &
echo $! >/tmp/baic-hmi/vision.pid
ready=0
for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25; do
  if pgrep -f '[/]astra_camera_node( |$)' >/dev/null; then ready=1; break; fi
  sleep 1
done
if [ "$ready" != 1 ]; then
  echo "相机没有启动成功"
  tail -n 12 /tmp/baic-hmi/vision.log
  exit 1
fi
echo "相机和目标检测已启动"
`, say);
}

async function stopVision(ssh, say) {
  return execCollect(ssh, "bash -s", `
set +e
echo "正在停止相机和目标检测"
if [ -f /tmp/baic-hmi/vision.pid ]; then
  pid=$(cat /tmp/baic-hmi/vision.pid)
  if [ -n "$pid" ]; then kill -TERM "$pid" 2>/dev/null || true; fi
fi
pkill -TERM -f '[v]oice-vision.launch.py' || true
sleep 1
pkill -TERM -f '[/]astra_camera_node( |$)' || true
pkill -TERM -f '[r]obot-bag-tracker.py' || true
sleep 1
if pgrep -f '[/]astra_camera_node( |$)' >/dev/null || pgrep -f '[r]obot-bag-tracker.py' >/dev/null; then
  pkill -KILL -f '[/]astra_camera_node( |$)' || true
  pkill -KILL -f '[r]obot-bag-tracker.py' || true
  sleep 1
fi
if pgrep -f '[/]astra_camera_node( |$)' >/dev/null || pgrep -f '[r]obot-bag-tracker.py' >/dev/null; then
  echo "相机或目标检测没有停下来"
  exit 1
fi
echo "相机和目标检测已停止"
`, say);
}

async function startMap(ssh, say) {
  return execCollect(ssh, "bash -s", `
set +e
mkdir -p /tmp/baic-hmi
if pgrep -x rtabmap >/dev/null; then
  echo "建图已在运行"
  exit 0
fi
echo "正在启动建图"
session="/home/wheeltec/slam_sessions/rtab_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$session" || exit 1
printf '%s\\n' "$session" > /home/wheeltec/robot_voice/RTAB_CURRENT_SESSION
nohup bash -lc "source /opt/ros/humble/setup.bash; source /home/wheeltec/wheeltec_ros2/install/setup.bash; export ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///home/wheeltec/robot_voice/cyclonedds.xml; exec ros2 launch /home/wheeltec/robot_voice/rtab_mapping_only.launch.py database_path:=$session/rtabmap.db" > "$session/launch.log" 2>&1 &
echo $! >/tmp/baic-hmi/rtab.pid
sleep 3
if ! pgrep -f 'rtab_mapping_only.launch.py' >/dev/null; then
  echo "建图没有启动成功"
  tail -n 12 "$session/launch.log"
  exit 1
fi
echo "建图已启动。地图要等相机画面稳定后才会显示"
`, say);
}

async function stopMap(ssh, say) {
  return execCollect(ssh, "bash -s", `
set +e
echo "正在停止建图"
if [ -f /tmp/baic-hmi/rtab.pid ]; then
  pid=$(cat /tmp/baic-hmi/rtab.pid)
  if [ -n "$pid" ]; then kill -TERM "$pid" 2>/dev/null || true; fi
fi
pkill -TERM -f '[r]tab_mapping_only.launch.py' || true
sleep 2
if pgrep -x rtabmap >/dev/null; then
  pkill -TERM -x rtabmap || true
  sleep 1
fi
if pgrep -x rtabmap >/dev/null || pgrep -f '[r]tab_mapping_only.launch.py' >/dev/null; then
  pkill -KILL -x rtabmap || true
  pkill -KILL -f '[r]tab_mapping_only.launch.py' || true
  sleep 1
fi
if pgrep -x rtabmap >/dev/null || pgrep -f '[r]tab_mapping_only.launch.py' >/dev/null; then
  echo "建图没有停下来"
  exit 1
fi
echo "建图已停止"
`, say);
}

function armWatch(bridge, target, on = true) {
  if (!bridge) return;
  try {
    bridge.write(`${JSON.stringify({ op: on ? "watch" : "unwatch", target })}\n`);
  } catch (_) {}
}

function execInput(ssh, command, input) {
  return new Promise((resolve, reject) => {
    ssh.exec(command, (error, stream) => {
      if (error) {
        reject(error);
        return;
      }
      let stderr = "";
      stream.on("data", () => {});
      stream.stderr.on("data", (chunk) => {
        stderr += chunk.toString("utf8");
      });
      stream.on("close", (code) => resolve({ code, stderr }));
      stream.end(input);
    });
  });
}

function startBridge(ssh) {
  const command = [
    "bash -lc '",
    "set -e;",
    "source /opt/ros/humble/setup.bash;",
    "if [ -f \"$HOME/wheeltec_ros2/install/setup.bash\" ]; then",
    "  source \"$HOME/wheeltec_ros2/install/setup.bash\";",
    "fi;",
    "export ROS_DOMAIN_ID=31;",
    "export ROS_LOCALHOST_ONLY=0;",
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;",
    "export CYCLONEDDS_URI=file:///home/wheeltec/robot_voice/cyclonedds.xml;",
    "exec python3 -u /tmp/baic-hmi/bridge.py'",
  ].join(" ");
  return new Promise((resolve, reject) => {
    ssh.exec(command, (error, stream) => {
      if (error) {
        reject(error);
        return;
      }
      resolve(stream);
    });
  });
}

function pickUwbFrame(snapshot) {
  const ports = snapshot && snapshot.ports ? snapshot.ports : null;
  if (!ports) return { ok: false, reason: "快照里没有端口数据" };
  let best = null;
  Object.keys(ports).forEach((name) => {
    const port = ports[name] || {};
    const frame = port.latest;
    if (!frame || typeof frame !== "object") return;
    const age = typeof port.position_age_s === "number" ? port.position_age_s : Infinity;
    if (age > 2.0) return;
    if (!best || age < best.age) {
      best = { age, port: name, side: port.physical_side || name, frame };
    }
  });
  if (!best) return { ok: false, reason: "UWB 无有效数据（模块未连接或数据过期）" };
  return { ok: true, ...best };
}

function uwbToWorld(frame, side, pose) {
  const dist = Number(frame.distance_m);
  const azimuth = Number(frame.azimuth_deg);
  if (!Number.isFinite(dist) || !Number.isFinite(azimuth)) return null;
  const offset = UWB_MOUNT_YAW[side] || 0;
  const bearing = ((azimuth + offset) * Math.PI) / 180;
  const shaped = { dist, azimuth, side, tag: frame.tag_id ?? null, anchor: frame.anchor_id ?? null };
  if (!pose) return { ...shaped, x: null, y: null };
  const heading = pose.yaw + bearing;
  return {
    ...shaped,
    x: Number((pose.x + dist * Math.cos(heading)).toFixed(3)),
    y: Number((pose.y + dist * Math.sin(heading)).toFixed(3)),
  };
}

function uwbReaderScript() {
  return [
    "import json, time",
    `path = ${JSON.stringify(UWB_SNAPSHOT)}`,
    "while True:",
    "    try:",
    "        with open(path) as handle:",
    "            data = json.load(handle)",
    "    except Exception:",
    "        data = {}",
    "    print(json.dumps(data, separators=(',', ':')), flush=True)",
    `    time.sleep(${UWB_READ_INTERVAL})`,
  ].join("\n");
}

const VOICE_LOG = "/home/wheeltec/robot_voice/voice-stable-live.log";
const VOICE_COMMANDS = {
  forward: "前进",
  backward: "后退",
  left: "左转",
  right: "右转",
  turnaround: "掉头",
  spin: "转圈",
  stop: "停止",
  follow: "跟随",
  who: "你是谁",
  wake: "唤醒",
};

function voiceCommandScript(text) {
  return `
import json, re, sys, datetime
sys.path.insert(0, "/home/wheeltec/robot_voice")
from voice_commands import parse, fuzzy_match
text = ${JSON.stringify(text)}
from voice_commands import ALIASES
normalized = re.sub(r"[\s，。！？,.!?]", "", text.lower())
command = parse(normalized) or fuzzy_match(normalized)
if not command:
    for phrase, name in sorted(ALIASES.items(), key=lambda item: len(item[0]), reverse=True):
        if phrase and phrase in normalized:
            command = name
            break
motion = {"forward", "backward", "left", "right", "turnaround", "spin", "stop", "follow"}
if command not in motion and command not in {"who", "wake"}:
    print(json.dumps({"ok": False, "reason": "小车不认识这句话"}, ensure_ascii=False))
    raise SystemExit(0)
if command in {"who", "wake"}:
    print(json.dumps({"ok": True, "command": command, "applied": False, "reason": "听到了，正在播放小车的回答"}, ensure_ascii=False))
    raise SystemExit(0)
now = datetime.datetime.now(datetime.timezone.utc).isoformat()
if command == "stop":
    item = {"utc": now, "stop_applied": True, "captured_to_apply_s": 0.05, "execute": False}
elif command == "follow":
    item = {"utc": now, "visual_follow_requested": True, "execute": False, "captured_to_apply_s": 0.05}
else:
    item = {"utc": now, "motion_requested": command, "execute": False}
with open(${JSON.stringify(VOICE_LOG)}, "a", encoding="utf-8") as handle:
    handle.write(json.dumps(item, ensure_ascii=False) + "\\n")
    handle.flush()
print(json.dumps({"ok": True, "command": command, "applied": True}, ensure_ascii=False))
`;
}

function submitVoiceCommand(ssh, text) {
  return new Promise((resolve) => {
    ssh.exec("python3 -u -", (error, stream) => {
      if (error) {
        resolve({ ok: false, reason: "没能把指令送到小车" });
        return;
      }
      let out = "";
      stream.on("data", (chunk) => { out += chunk.toString("utf8"); });
      stream.stderr.on("data", () => {});
      stream.on("close", () => {
        const line = out.trim().split("\n").filter(Boolean).pop();
        try {
          resolve(JSON.parse(line));
        } catch (_) {
          resolve({ ok: false, reason: "小车没有返回指令结果" });
        }
      });
      stream.end(voiceCommandScript(text));
    });
  });
}

const SPOKEN = {
  who: {
    file: "/home/wheeltec/tts_cache/bei_bei.wav",
    text: "我是贝贝，北汽出品，随时跟随你",
  },
  wake: {
    file: "/home/wheeltec/tts_cache/wo_zai_ne.wav",
    text: "我在呢",
  },
};

function speechWatchScript() {
  return `
import json, os, time
path = ${JSON.stringify(VOICE_LOG)}
with open(path, "rb") as handle:
    handle.seek(0, os.SEEK_END)
    buf = b""
    while True:
        chunk = handle.read(8192)
        if not chunk:
            time.sleep(0.2)
            try:
                if handle.tell() > os.path.getsize(path):
                    handle.seek(0, os.SEEK_END)
                    buf = b""
            except Exception:
                pass
            continue
        buf += chunk.replace(b"\\x00", b"")
        while b"\\n" in buf:
            line, buf = buf.split(b"\\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                item = json.loads(line)
            except Exception:
                continue
            kind = "who" if item.get("who_requested") else "wake" if item.get("wake_ack") else ""
            if kind:
                print(json.dumps({"kind": kind}, ensure_ascii=False), flush=True)
        if len(buf) > 200000:
            buf = buf[-4000:]
`;
}

function startSpeechWatch(ssh, onKind) {
  const payload = Buffer.from(speechWatchScript(), "utf8").toString("base64");
  const command = `python3 -u -c "exec(__import__('base64').b64decode('${payload}').decode())"`;
  return new Promise((resolve, reject) => {
    ssh.exec(command, (error, stream) => {
      if (error) {
        reject(error);
        return;
      }
      let pending = "";
      stream.on("data", (chunk) => {
        pending += chunk.toString("utf8");
        let index;
        while ((index = pending.indexOf("\n")) >= 0) {
          const line = pending.slice(0, index).trim();
          pending = pending.slice(index + 1);
          if (!line) continue;
          try {
            const item = JSON.parse(line);
            if (item.kind) onKind(item.kind);
          } catch (_) {}
        }
      });
      stream.stderr.on("data", () => {});
      resolve(stream);
    });
  });
}

function voskScript() {
  return `
import json, sys
from vosk import Model, KaldiRecognizer
model = Model("/home/wheeltec/robot_voice/models/vosk-model-small-cn-0.22")
rec = KaldiRecognizer(model, 16000)
print("READY", flush=True)
while True:
    data = sys.stdin.buffer.read(3200)
    if not data:
        break
    final = rec.AcceptWaveform(data)
    try:
        payload = json.loads(rec.Result() if final else rec.PartialResult())
    except Exception:
        continue
    text = (payload.get("text") or payload.get("partial") or "").replace(" ", "")
    if not text:
        continue
    print(("F " if final else "P ") + text, flush=True)
`;
}

function startVosk(ssh, onLine) {
  const payload = Buffer.from(voskScript(), "utf8").toString("base64");
  const command = `python3 -u -c "exec(__import__('base64').b64decode('${payload}').decode())"`;
  return new Promise((resolve, reject) => {
    ssh.exec(command, (error, stream) => {
      if (error) {
        reject(error);
        return;
      }
      let pending = "";
      stream.on("data", (chunk) => {
        pending += chunk.toString("utf8");
        let index;
        while ((index = pending.indexOf("\n")) >= 0) {
          const line = pending.slice(0, index).trim();
          pending = pending.slice(index + 1);
          if (line) onLine(line, stream);
        }
      });
      stream.stderr.on("data", () => {});
      resolve(stream);
    });
  });
}

function startUwbStream(ssh, getPose, emit) {
  const payload = Buffer.from(uwbReaderScript(), "utf8").toString("base64");
  const command = `python3 -u -c "exec(__import__('base64').b64decode('${payload}').decode())"`;
  return new Promise((resolve, reject) => {
    ssh.exec(command, (error, stream) => {
      if (error) {
        reject(error);
        return;
      }
      let pending = "";
      stream.on("data", (chunk) => {
        pending += chunk.toString("utf8");
        let index;
        while ((index = pending.indexOf("\n")) >= 0) {
          const line = pending.slice(0, index).trim();
          pending = pending.slice(index + 1);
          if (!line) continue;
          let snapshot;
          try {
            snapshot = JSON.parse(line);
          } catch (_) {
            continue;
          }
          const picked = pickUwbFrame(snapshot);
          if (!picked.ok) {
            emit({ type: "uwb", ok: false, reason: picked.reason });
            continue;
          }
          const placed = uwbToWorld(picked.frame, picked.side, getPose());
          if (!placed) {
            emit({ type: "uwb", ok: false, reason: "UWB 帧缺少距离或角度" });
            continue;
          }
          emit({ type: "uwb", ok: true, age: picked.age, ...placed });
        }
      });
      stream.stderr.on("data", () => {});
      resolve(stream);
    });
  });
}

liveWss.on("connection", (ws) => {
  let ssh = null;
  let bridge = null;
  let buffer = "";
  let stderr = "";
  let ready = false;
  let closed = false;
  let password = "";
  const launching = { units: false, vision: false, map: false };
  let lastPose = null;
  let uwbStream = null;
  let speechWatch = null;
  const clipCache = {};
  const lastSpeak = {};
  let voskIn = null;
  let voskReady = false;
  let lastPartial = "";

  const cleanup = () => {
    if (closed) return;
    closed = true;
    if (uwbStream) {
      try { uwbStream.close(); } catch (_) {}
    }
    uwbStream = null;
    if (speechWatch) {
      try { speechWatch.close(); } catch (_) {}
    }
    speechWatch = null;
    if (voskIn) {
      try { voskIn.close(); } catch (_) {}
    }
    voskIn = null;
    voskReady = false;
    try { bridge?.close(); } catch (_) {}
    try { ssh?.end(); } catch (_) {}
    bridge = null;
    ssh = null;
  };

  const fail = (message) => {
    send(ws, { type: "login", ok: false, message });
    cleanup();
  };

  const deliverClip = (kind) => {
    const spec = SPOKEN[kind];
    if (!spec || !ssh || closed) return;
    const now = Date.now();
    if (now - (lastSpeak[kind] || 0) < 3000) return;
    lastSpeak[kind] = now;
    const emitClip = (wav) => {
      if (closed || ws.readyState !== ws.OPEN) return;
      send(ws, { type: "speak", kind, text: spec.text, wav });
    };
    if (clipCache[kind]) {
      emitClip(clipCache[kind]);
      return;
    }
    execCollect(ssh, `base64 -w 0 ${spec.file}`).then((result) => {
      const wav = result.stdout.replace(/\s/g, "");
      if (!wav) return;
      clipCache[kind] = wav;
      emitClip(wav);
    }).catch(() => {});
  };

  const onBridgeLine = (line) => {
    if (!line) return;
    let msg;
    try {
      msg = JSON.parse(line);
    } catch (_) {
      return;
    }
    if (msg.type === "ready" && !ready) {
      ready = true;
      send(ws, { type: "login", ok: true });
      if (!uwbStream) {
        startUwbStream(ssh, () => lastPose, (payload) => send(ws, payload))
          .then((stream) => { uwbStream = stream; })
          .catch(() => send(ws, { type: "uwb", ok: false, reason: "UWB 读取没能启动" }));
      }
      if (!speechWatch) {
        startSpeechWatch(ssh, (kind) => deliverClip(kind))
          .then((stream) => {
            speechWatch = stream;
            send(ws, { type: "audio", ok: true });
            stream.on("close", () => {
              if (speechWatch === stream) speechWatch = null;
            });
          })
          .catch(() => send(ws, { type: "audio", ok: false, reason: "小车的回答没有接上" }));
      }
      return;
    }
    if (!ready) return;
    if (msg.type === "pose") lastPose = { x: msg.x, y: msg.y, yaw: msg.yaw };
    if (msg.type === "camera" && ws.bufferedAmount > 1500000) return;
    send(ws, msg);
  };

  ws.on("message", async (raw, isBinary) => {
    if (isBinary) {
      if (voskIn && voskReady) {
        try { voskIn.write(raw); } catch (_) {}
      }
      return;
    }
    let msg;
    try {
      msg = JSON.parse(String(raw));
    } catch (_) {
      send(ws, { type: "error", message: "无效的消息" });
      return;
    }

    if (msg.type === "login") {
      if (ssh) return;
      const host = String(msg.host || "").trim();
      const username = String(msg.username || "").trim();
      password = String(msg.password || "");
      const port = Number(msg.port) || 22;
      if (!host || !username || !password) {
        fail("请填写主机、用户名和密码");
        return;
      }
      try {
        ssh = await connectSsh({ host, port, username, password });
        const uploaded = await execInput(
          ssh,
          "mkdir -p /tmp/baic-hmi && cat > /tmp/baic-hmi/bridge.py",
          BRIDGE
        );
        if (uploaded.code !== 0) {
          fail(uploaded.stderr.trim() || "没能把桥接程序写到小车上");
          return;
        }
        bridge = await startBridge(ssh);
      } catch (error) {
        fail(error.message || "连接小车失败");
        return;
      }

      const readyTimer = setTimeout(() => {
        if (!ready) fail(stderr.trim() || "小车没有启动语音和数据桥");
      }, 15000);

      bridge.on("data", (chunk) => {
        buffer += chunk.toString("utf8");
        let index;
        while ((index = buffer.indexOf("\n")) >= 0) {
          const line = buffer.slice(0, index).trim();
          buffer = buffer.slice(index + 1);
          onBridgeLine(line);
          if (ready) clearTimeout(readyTimer);
        }
      });
      bridge.stderr.on("data", (chunk) => {
        stderr = (stderr + chunk.toString("utf8")).slice(-1200);
      });
      bridge.on("close", () => {
        clearTimeout(readyTimer);
        if (closed) return;
        if (!ready) {
          fail(stderr.trim() || "小车上的桥接程序退出了");
          return;
        }
        send(ws, { type: "closed", message: "小车连接已断开" });
        cleanup();
      });
      return;
    }

    if (msg.type === "launch") {
      const target = msg.target;
      const action = msg.action === "stop" ? "stop" : "start";
      const jobs = {
        units: {
          start: () => startCarUnits(ssh, password, (line) => send(ws, { type: "launch", target, action, line })),
          stop: () => stopCarUnits(ssh, password, (line) => send(ws, { type: "launch", target, action, line })),
        },
        vision: {
          start: () => startVision(ssh, (line) => send(ws, { type: "launch", target, action, line })),
          stop: () => stopVision(ssh, (line) => send(ws, { type: "launch", target, action, line })),
        },
        map: {
          start: () => startMap(ssh, (line) => send(ws, { type: "launch", target, action, line })),
          stop: () => stopMap(ssh, (line) => send(ws, { type: "launch", target, action, line })),
        },
      };
      if (!jobs[target]) {
        send(ws, { type: "launch", target: target || "", action, line: "没有这项服务", done: true, ok: false });
        return;
      }
      if (!ssh || !ready) {
        send(ws, { type: "launch", target, action, line: "请先登录小车", done: true, ok: false });
        return;
      }
      if (launching[target]) {
        send(ws, { type: "launch", target, action, line: action === "stop" ? "正在停止，请稍等" : "正在启动，请稍等", done: false });
        return;
      }
      launching[target] = true;
      let ok = false;
      jobs[target][action]()
        .then((result) => {
          ok = !result || result.code === 0;
          if (ok && target === "vision") armWatch(bridge, "camera", action === "start");
          if (ok && target === "map") armWatch(bridge, "map", action === "start");
          if (!ok) {
            send(ws, {
              type: "launch",
              target,
              action,
              line: action === "stop" ? "这项没有停下来" : "这项没有启动成功",
            });
          }
        })
        .catch((error) => send(ws, { type: "launch", target, action, line: error.message || (action === "stop" ? "停止失败" : "启动失败") }))
        .finally(() => {
          launching[target] = false;
          send(ws, { type: "launch", target, action, done: true, ok });
        });
      return;
    }

    if (msg.type === "hear" && ssh && ready) {
      const text = String(msg.text || "").trim().slice(0, 80);
      if (!text) return;
      submitVoiceCommand(ssh, text).then((result) => {
        const name = VOICE_COMMANDS[result.command] || result.command || "";
        send(ws, { type: "command", text, command: result.command || "", name, ok: !!result.ok, applied: !!result.applied, reason: result.reason || "" });
        if (result.command === "who" || result.command === "wake") deliverClip(result.command);
      });
      return;
    }

    if (msg.type === "car-audio" && ssh && ready) {
      if (!msg.on) {
        if (voskIn) {
          try { voskIn.close(); } catch (_) {}
        }
        voskIn = null;
        voskReady = false;
        lastPartial = "";
        return;
      }
      if (!voskIn) {
        startVosk(ssh, (line, stream) => {
          if (voskIn !== stream) return;
          if (line === "READY") {
            voskReady = true;
            send(ws, { type: "heard", ready: true });
            return;
          }
          const text = line.slice(2).replace(/\s/g, "");
          const unknown = !text || /^(\[unk\])+$/.test(text);
          if (line.startsWith("P ")) {
            if (unknown || text === lastPartial) return;
            lastPartial = text;
            send(ws, { type: "heard", text, final: false });
            return;
          }
          if (line.startsWith("F ")) {
            lastPartial = "";
            if (unknown) {
              send(ws, { type: "heard", unclear: true });
              return;
            }
            send(ws, { type: "heard", text, final: true });
            submitVoiceCommand(ssh, text).then((result) => {
              const name = VOICE_COMMANDS[result.command] || result.command || "";
              send(ws, { type: "command", text, command: result.command || "", name, ok: !!result.ok, applied: !!result.applied, reason: result.reason || "" });
              if (result.command === "who" || result.command === "wake") deliverClip(result.command);
            });
          }
        }).then((stream) => {
          voskIn = stream;
          stream.on("close", () => {
            if (voskIn === stream) {
              voskIn = null;
              voskReady = false;
            }
          });
        }).catch(() => send(ws, { type: "heard", ready: false, reason: "识别没有启动" }));
      }
      return;
    }

    if (msg.type === "logout") {
      send(ws, { type: "closed", message: "已断开" });
      cleanup();
    }
  });

  ws.on("close", cleanup);
  ws.on("error", cleanup);
});

termWss.on("connection", (ws) => {
  let ssh = null;
  let stream = null;
  let closed = false;

  const cleanup = () => {
    if (closed) return;
    closed = true;
    try { stream?.close(); } catch (_) {}
    try { ssh?.end(); } catch (_) {}
    stream = null;
    ssh = null;
  };

  ws.on("message", (raw) => {
    let msg;
    try {
      msg = JSON.parse(String(raw));
    } catch (_) {
      send(ws, { type: "error", message: "无效的消息" });
      return;
    }

    if (msg.type === "connect") {
      if (ssh) return;
      const host = String(msg.host || "").trim();
      const username = String(msg.username || "").trim();
      const password = String(msg.password || "");
      const port = Number(msg.port) || 22;
      if (!host || !username || !password) {
        send(ws, { type: "error", message: "请先登录" });
        return;
      }
      ssh = new Client();
      ssh
        .on("ready", () => {
          ssh.shell({ term: "vt100", cols: 100, rows: 24 }, (error, shell) => {
            if (error) {
              send(ws, { type: "error", message: error.message });
              cleanup();
              return;
            }
            stream = shell;
            send(ws, { type: "ready" });
            shell.on("data", (data) => send(ws, { type: "data", data: data.toString("utf8") }));
            shell.on("close", () => {
              send(ws, { type: "closed", message: "终端已结束" });
              cleanup();
            });
          });
        })
        .on("error", (error) => {
          send(ws, { type: "error", message: error.message });
          cleanup();
        })
        .on("close", () => {
          if (!closed) send(ws, { type: "closed", message: "终端连接已关闭" });
          cleanup();
        })
        .connect({
          host,
          port,
          username,
          password,
          readyTimeout: 10000,
          keepaliveInterval: 10000,
        });
      return;
    }

    if (msg.type === "data" && stream) stream.write(String(msg.data ?? ""));
    if (msg.type === "disconnect") {
      cleanup();
      send(ws, { type: "closed", message: "已断开" });
    }
  });

  ws.on("close", cleanup);
  ws.on("error", cleanup);
});

server.listen(PORT, "0.0.0.0", () => {
  console.log(`UI_Latest_20261104: http://127.0.0.1:${PORT}/`);
});
