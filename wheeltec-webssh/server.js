const http = require("http");
const os = require("os");
const path = require("path");
const { execFileSync } = require("child_process");
const express = require("express");
const { WebSocketServer } = require("ws");
const { Client } = require("ssh2");

const PORT = Number(process.env.PORT) || 8787;
const DEFAULT_HOST = process.env.WHEELTEC_HOST || "192.168.0.105";

function isVirtualInterface(name) {
  return /^(utun|awdl|llw|bridge|docker|veth|vmnet|vboxnet|gif|stf|ap\d)/i.test(name);
}

function isUsefulIPv4(address) {
  if (address.startsWith("169.254.")) return false;
  const [a, b] = address.split(".").map(Number);
  if (a === 198 && (b === 18 || b === 19)) return false;
  return true;
}

function localHostName() {
  try {
    const name = execFileSync("scutil", ["--get", "LocalHostName"], {
      encoding: "utf8",
    }).trim();
    if (/^[A-Za-z0-9-]+$/.test(name)) return name;
  } catch (_) {}
  const fallback = os.hostname().replace(/\.local$/i, "");
  return /^[A-Za-z0-9-]+$/.test(fallback) ? fallback : "";
}

function portalEntries(port) {
  const preferred = [];
  const fallback = [];
  const nets = os.networkInterfaces();
  for (const [name, list] of Object.entries(nets)) {
    for (const net of list || []) {
      const v4 = net.family === "IPv4" || net.family === 4;
      if (!v4 || net.internal || !isUsefulIPv4(net.address)) continue;
      const entry = {
        url: `http://${net.address}:${port}/`,
        label: "局域网地址",
      };
      if (isVirtualInterface(name)) fallback.push(entry);
      else preferred.push(entry);
    }
  }
  const entries = preferred.length ? preferred : fallback;
  const seen = new Set();
  const unique = entries.filter((entry) => {
    if (seen.has(entry.url)) return false;
    seen.add(entry.url);
    return true;
  });
  const hostName = localHostName();
  if (hostName) {
    unique.push({
      url: `http://${hostName}.local:${port}/`,
      label: "本机名称",
    });
  }
  return unique;
}

const app = express();
app.use(express.static(path.join(__dirname, "public")));
app.get("/api/defaults", (_req, res) => {
  res.json({
    host: DEFAULT_HOST,
    port: 22,
    username: "wheeltec",
  });
});
app.get("/api/portal", (_req, res) => {
  res.json({ urls: portalEntries(PORT) });
});

const server = http.createServer(app);
const wss = new WebSocketServer({ server, path: "/ssh" });

function send(ws, type, payload = {}) {
  if (ws.readyState === ws.OPEN) {
    ws.send(JSON.stringify({ type, ...payload }));
  }
}

wss.on("connection", (ws) => {
  let ssh = null;
  let stream = null;
  let closed = false;

  const cleanup = () => {
    if (closed) return;
    closed = true;
    try {
      stream?.close();
    } catch (_) {}
    try {
      ssh?.end();
    } catch (_) {}
    stream = null;
    ssh = null;
  };

  ws.on("message", (raw) => {
    let msg;
    try {
      msg = JSON.parse(String(raw));
    } catch {
      send(ws, "error", { message: "无效的消息格式" });
      return;
    }

    if (msg.type === "connect") {
      if (ssh) {
        send(ws, "error", { message: "已在连接中" });
        return;
      }

      const host = String(msg.host || DEFAULT_HOST).trim();
      const port = Number(msg.port) || 22;
      const username = String(msg.username || "").trim();
      const password = String(msg.password || "");
      const cols = Number(msg.cols) || 120;
      const rows = Number(msg.rows) || 36;

      if (!host || !username || !password) {
        send(ws, "error", { message: "请填写主机、用户名和密码" });
        return;
      }

      send(ws, "status", { message: `正在连接 ${username}@${host}:${port}…` });

      ssh = new Client();
      ssh
        .on("ready", () => {
          send(ws, "status", { message: "SSH 已就绪，打开终端…" });
          ssh.shell(
            { term: "xterm-256color", cols, rows },
            (err, shellStream) => {
              if (err) {
                send(ws, "error", { message: `打开 shell 失败: ${err.message}` });
                cleanup();
                return;
              }
              stream = shellStream;
              send(ws, "ready", { host, username });

              stream.on("data", (data) => {
                send(ws, "data", { data: data.toString("utf8") });
              });
              stream.stderr?.on("data", (data) => {
                send(ws, "data", { data: data.toString("utf8") });
              });
              stream.on("close", () => {
                send(ws, "closed", { message: "远程会话已结束" });
                cleanup();
              });
            }
          );
        })
        .on("error", (err) => {
          send(ws, "error", { message: `连接失败: ${err.message}` });
          cleanup();
        })
        .on("close", () => {
          send(ws, "closed", { message: "SSH 连接已关闭" });
          cleanup();
        })
        .connect({
          host,
          port,
          username,
          password,
          readyTimeout: 15000,
          tryKeyboard: false,
        });
      return;
    }

    if (msg.type === "data") {
      if (stream) stream.write(msg.data ?? "");
      return;
    }

    if (msg.type === "resize") {
      if (stream && typeof stream.setWindow === "function") {
        const cols = Number(msg.cols) || 80;
        const rows = Number(msg.rows) || 24;
        stream.setWindow(rows, cols, 0, 0);
      }
      return;
    }

    if (msg.type === "disconnect") {
      cleanup();
      send(ws, "closed", { message: "已断开" });
    }
  });

  ws.on("close", cleanup);
  ws.on("error", cleanup);
});

server.listen(PORT, "0.0.0.0", () => {
  console.log(`Wheeltec WebSSH 已启动: http://127.0.0.1:${PORT}`);
  const entries = portalEntries(PORT).filter((entry) => entry.label === "局域网地址");
  if (entries.length) {
    console.log("同一网络的其他电脑打开：");
    for (const entry of entries) console.log(`  ${entry.url}`);
  } else {
    console.log("未发现局域网地址，目前只能本机打开");
  }
  console.log(`默认目标设备: ${DEFAULT_HOST}`);
});
