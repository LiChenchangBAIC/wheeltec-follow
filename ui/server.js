const http = require("http");
const fs = require("fs");
const path = require("path");
const express = require("express");
const { WebSocketServer } = require("ws");
const { Client } = require("ssh2");

const PORT = Number(process.env.PORT) || 8765;
const ROOT = __dirname;
const BRIDGE = fs.readFileSync(path.join(ROOT, "bridge.py"));

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

async function startCarFunctions(ssh, password, say) {
  const names = CAR_UNITS.map(([unit]) => unit).join(" ");
  const status = await execCollect(
    ssh,
    "bash -s",
    `set +e\nfor unit in ${names}; do\n  if systemctl is-active --quiet "$unit"; then echo "ON $unit"; else echo "OFF $unit"; fi\ndone\n`
  );
  const off = [];
  status.stdout.split("\n").forEach((line) => {
    const match = /^(ON|OFF) (\S+)$/.exec(line.trim());
    if (!match) return;
    const known = CAR_UNITS.find(([unit]) => unit === match[2]);
    if (!known) return;
    if (match[1] === "ON") say(`${known[1]}已在运行，不会重新启动`);
    else off.push(known);
  });
  if (off.length) {
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
    } else {
      off.forEach(([, label]) => say(`${label}已启动`));
    }
  }

  const vision = await execCollect(ssh, "bash -s", `
set +e
mkdir -p /tmp/baic-hmi
if pgrep -f '[/]astra_camera_node( |$)' >/dev/null; then
  echo "相机和目标检测已在运行"
else
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
fi
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
  if (vision.code !== 0) say("有功能没能启动，上面是小车返回的原因");
  else say("启动完成");
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

liveWss.on("connection", (ws) => {
  let ssh = null;
  let bridge = null;
  let buffer = "";
  let stderr = "";
  let ready = false;
  let closed = false;
  let password = "";
  let launching = false;

  const cleanup = () => {
    if (closed) return;
    closed = true;
    try { bridge?.close(); } catch (_) {}
    try { ssh?.end(); } catch (_) {}
    bridge = null;
    ssh = null;
  };

  const fail = (message) => {
    send(ws, { type: "login", ok: false, message });
    cleanup();
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
      return;
    }
    if (!ready) return;
    if (msg.type === "camera" && ws.bufferedAmount > 1500000) return;
    send(ws, msg);
  };

  ws.on("message", async (raw) => {
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
      if (!ssh || !ready) {
        send(ws, { type: "launch", line: "请先登录小车", done: true });
        return;
      }
      if (launching) {
        send(ws, { type: "launch", line: "正在启动，请稍等", done: false });
        return;
      }
      launching = true;
      startCarFunctions(ssh, password, (line) => send(ws, { type: "launch", line }))
        .catch((error) => send(ws, { type: "launch", line: error.message || "启动失败" }))
        .finally(() => {
          launching = false;
          send(ws, { type: "launch", done: true });
        });
      return;
    }

    if ((msg.type === "hear" || msg.type === "say") && bridge && ready) {
      const text = String(msg.text || "").slice(0, 400);
      bridge.write(`${JSON.stringify({ op: msg.type, text })}\n`);
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
  console.log(`BAIC 车辆界面: http://127.0.0.1:${PORT}/`);
});
