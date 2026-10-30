const state = {
  speed: null,
  voltage: null,
  linked: false,
  loggedIn: false,
  fault: "",
  voice: "",
  reply: "",
};

const $ = (id) => document.getElementById(id);
const wave = $("wave");

for (let i = 0; i < 42; i += 1) {
  const bar = document.createElement("span");
  const h = 20 + Math.abs(Math.sin(i * 0.55)) * 80;
  bar.style.height = `${h}%`;
  bar.style.animationDelay = `${(i % 7) * 0.08}s`;
  wave.appendChild(bar);
}

let auth = null;
let starting = false;
let live = null;
let speaking = false;
let wantListen = false;
let recognition = null;
let speakToken = 0;
let hearToken = 0;
let spoken = "";
let lastSent = "";

function connected() {
  return state.loggedIn && state.linked;
}

function renderStatus() {
  const online = connected();
  $("speed").innerHTML = online && state.speed != null
    ? `${Number(state.speed).toFixed(2)} <small>m/s</small>`
    : "-- <small>m/s</small>";
  $("voltage").innerHTML = online && state.voltage != null
    ? `${Number(state.voltage).toFixed(1)} <small>V</small>`
    : "-- <small>V</small>";

  const link = $("lamp-link");
  link.className = `lamp ${online ? "ok" : "bad"}`;
  link.querySelector("b").textContent = online ? "已连接" : "未连接";

  const func = $("lamp-func");
  const funcBad = !online || state.fault;
  func.className = `lamp ${funcBad ? "bad" : "ok"}`;
  if (!online) func.querySelector("b").textContent = "无法确认";
  else if (state.fault) func.querySelector("b").textContent = `${state.fault}异常`;
  else func.querySelector("b").textContent = "功能正常";

  $("voice-line").textContent = `最近指令：${state.voice || "—"}`;
  $("reply-line").textContent = `回复：${state.reply || "—"}`;
  if (auth) $("prompt-label").textContent = `${auth.username}@${auth.host}:~$`;
  else $("prompt-label").textContent = "$";
  $("start-all").disabled = !online || starting;
  if (!starting) $("start-all").textContent = "启动全部";
}

const tts = { volume: 1, pitch: 1, rate: 1 };

const commands = [
  ["小车前进", "小车前进"],
  ["小车后退", "小车后退"],
  ["小车左转", "小车左转"],
  ["小车右转", "小车右转"],
  ["小车停", "小车停"],
  ["小车休眠", "小车休眠"],
  ["小车过来", "小车寻找声源"],
  ["小车去i点", "好的"],
  ["小车去j点", "好的"],
  ["小车去k点", "好的"],
  ["遇到障碍物", "遇到障碍物"],
  ["关闭雷达跟随", "好的"],
  ["小车雷达跟随", "好的"],
  ["跟着前面那个人", "好的"],
  ["你好小微", "你好，我在"],
  ["前进", "小车前进"],
  ["后退", "小车后退"],
  ["左转", "小车左转"],
  ["右转", "小车右转"],
  ["停下", "小车停"],
  ["停止", "小车停"],
  ["休眠", "小车休眠"],
  ["过来", "小车寻找声源"],
  ["障碍物", "遇到障碍物"],
  ["雷达跟随", "好的"],
  ["你好", "你好，我在"],
];

function normalize(text) {
  return text.toLowerCase().replace(/[\s，。！？、,.!?;；：:""''（）()\-]/g, "");
}

function replyFor(text) {
  const heard = normalize(text);
  for (const [key, reply] of commands) {
    if (heard.includes(normalize(key))) return reply;
  }
  const clean = text.replace(/[。！？\s]+$/g, "");
  return clean ? `收到，${clean}` : "我在";
}

function pickVoice() {
  const voices = window.speechSynthesis ? speechSynthesis.getVoices() : [];
  return (
    voices.find((item) => /xiaoyan|tingting|ting-ting|meijia/i.test(item.name)) ||
    voices.find((item) => /^zh/i.test(item.lang)) ||
    null
  );
}

function setWave() {
  wave.classList.toggle("off", !wantListen && !speaking);
  wave.classList.toggle("talk", speaking);
  wave.setAttribute("aria-pressed", wantListen ? "true" : "false");
  wave.setAttribute("aria-label", wantListen ? "停止说话" : "开始说话");
}

function speak(text) {
  const line = String(text || "").trim();
  state.reply = line;
  renderStatus();
  if (!line || line === spoken) return;
  spoken = line;
  if (!window.speechSynthesis) return;
  const token = ++speakToken;
  const utter = new SpeechSynthesisUtterance(line);
  utter.lang = "zh-CN";
  utter.volume = tts.volume;
  utter.pitch = tts.pitch;
  utter.rate = tts.rate;
  const voice = pickVoice();
  if (voice) utter.voice = voice;
  utter.onstart = () => {
    speaking = true;
    if (recognition && wantListen) {
      try { recognition.stop(); } catch (_) {}
    }
    setWave();
  };
  utter.onend = utter.onerror = () => {
    if (token !== speakToken) return;
    speaking = false;
    setWave();
    if (wantListen) startRecognition();
  };
  speechSynthesis.cancel();
  speechSynthesis.speak(utter);
}

function wsUrl(pathname) {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}${pathname}`;
}

function showFrame(id, b64, mime) {
  const binary = atob(b64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) bytes[i] = binary.charCodeAt(i);
  const url = URL.createObjectURL(new Blob([bytes], { type: mime }));
  const img = $(id);
  const previous = img.dataset.url;
  img.src = url;
  img.hidden = false;
  img.dataset.url = url;
  if (previous) URL.revokeObjectURL(previous);
}

function clearFrame(id) {
  const img = $(id);
  if (img.dataset.url) URL.revokeObjectURL(img.dataset.url);
  img.removeAttribute("src");
  img.hidden = true;
  delete img.dataset.url;
}

const world = {
  map: null,
  pose: null,
  beams: [],
  goal: null,
  sheet: null,
  layout: null,
  trail: [],
};

function cellColor(value) {
  if (value === 255) return [0x1c, 0x1c, 0x1c, 255];
  if (value < 50) {
    const shade = Math.round(0x3a - (value / 49) * (0x3a - 0x2e));
    return [shade, shade, shade, 255];
  }
  const mix = (value - 50) / 50;
  return [
    Math.round(0xf4 + (0xff - 0xf4) * mix),
    Math.round(0xf1 + (0x5a - 0xf1) * mix),
    Math.round(0xec + (0x32 - 0xec) * mix),
    255,
  ];
}

function paintSheet(map) {
  const sheet = document.createElement("canvas");
  sheet.width = map.w;
  sheet.height = map.h;
  const context = sheet.getContext("2d");
  const image = context.createImageData(map.w, map.h);
  for (let row = 0; row < map.h; row += 1) {
    const dest = map.h - 1 - row;
    for (let col = 0; col < map.w; col += 1) {
      const [r, g, b, a] = cellColor(map.cells[row * map.w + col]);
      const index = (dest * map.w + col) * 4;
      image.data[index] = r;
      image.data[index + 1] = g;
      image.data[index + 2] = b;
      image.data[index + 3] = a;
    }
  }
  context.putImageData(image, 0, 0);
  return sheet;
}

function worldToCanvas(x, y, map, layout) {
  const dx = x - map.ox;
  const dy = y - map.oy;
  const cos = Math.cos(map.oyaw);
  const sin = Math.sin(map.oyaw);
  const localX = cos * dx + sin * dy;
  const localY = -sin * dx + cos * dy;
  return [
    layout.left + (localX / map.res) * layout.scale,
    layout.top + (map.h - localY / map.res) * layout.scale,
  ];
}

function canvasToWorld(px, py, map, layout) {
  const localX = ((px - layout.left) / layout.scale) * map.res;
  const localY = (map.h - (py - layout.top) / layout.scale) * map.res;
  const cos = Math.cos(map.oyaw);
  const sin = Math.sin(map.oyaw);
  return {
    x: map.ox + cos * localX - sin * localY,
    y: map.oy + sin * localX + cos * localY,
    col: Math.floor(localX / map.res),
    row: Math.floor(localY / map.res),
  };
}

function drawMap() {
  const canvas = $("slam-view");
  const map = world.map;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!map || width < 2 || height < 2) return;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, canvas.width, canvas.height);
  const pad = 14 * ratio;
  const scale = Math.min((canvas.width - pad * 2) / map.w, (canvas.height - pad * 2) / map.h);
  const drawW = map.w * scale;
  const drawH = map.h * scale;
  const layout = {
    left: (canvas.width - drawW) / 2,
    top: (canvas.height - drawH) / 2,
    scale,
    drawW,
    drawH,
  };
  world.layout = layout;
  context.imageSmoothingEnabled = false;
  context.drawImage(world.sheet, layout.left, layout.top, drawW, drawH);

  if (world.trail.length > 1) {
    context.beginPath();
    world.trail.forEach((point, index) => {
      const [tx, ty] = worldToCanvas(point.x, point.y, map, layout);
      if (index === 0) context.moveTo(tx, ty);
      else context.lineTo(tx, ty);
    });
    context.strokeStyle = "#f4f1ec";
    context.lineWidth = 2 * ratio;
    context.lineJoin = "round";
    context.stroke();
  }

  if (world.pose) {
    world.beams.forEach((beam) => {
      if (beam.r < 0.02 || beam.r > 0.8) return;
      const heading = world.pose.yaw + beam.yaw;
      const originX = world.pose.x + Math.cos(world.pose.yaw) * beam.x - Math.sin(world.pose.yaw) * beam.y;
      const originY = world.pose.y + Math.sin(world.pose.yaw) * beam.x + Math.cos(world.pose.yaw) * beam.y;
      const [sx, sy] = worldToCanvas(originX, originY, map, layout);
      const radius = (beam.r / map.res) * scale;
      context.save();
      context.translate(sx, sy);
      context.rotate(-heading);
      context.beginPath();
      context.moveTo(0, 0);
      context.arc(0, 0, radius, -0.26, 0.26);
      context.closePath();
      context.fillStyle = "rgba(255, 90, 50, 0.35)";
      context.fill();
      context.restore();
    });

    const [cx, cy] = worldToCanvas(world.pose.x, world.pose.y, map, layout);
    const length = (0.4 / map.res) * scale;
    const breadth = (0.3 / map.res) * scale;
    context.beginPath();
    context.arc(cx, cy, Math.max(length * 0.75, 9 * ratio), 0, Math.PI * 2);
    context.strokeStyle = "#f4f1ec";
    context.lineWidth = 2 * ratio;
    context.stroke();
    context.save();
    context.translate(cx, cy);
    context.rotate(-world.pose.yaw);
    context.beginPath();
    context.moveTo(length / 2, 0);
    context.lineTo(-length / 2, breadth / 2);
    context.lineTo(-length / 2, -breadth / 2);
    context.closePath();
    context.fillStyle = "#ff5a32";
    context.fill();
    context.restore();
    context.fillStyle = "#f4f1ec";
    context.font = `${12 * ratio}px sans-serif`;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText("A", cx, cy - length * 0.85);
  }

  if (world.goal) {
    const [gx, gy] = worldToCanvas(world.goal.x, world.goal.y, map, layout);
    context.beginPath();
    context.arc(gx, gy, 9 * ratio, 0, Math.PI * 2);
    context.strokeStyle = "#ff5a32";
    context.lineWidth = 1.6 * ratio;
    context.stroke();
    context.fillStyle = "#ff5a32";
    context.font = `${11 * ratio}px sans-serif`;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText("B", gx, gy);
  }
}

function rememberMap(msg) {
  const binary = atob(msg.grid);
  const cells = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) cells[i] = binary.charCodeAt(i);
  const map = {
    w: msg.w,
    h: msg.h,
    res: msg.res,
    ox: msg.ox,
    oy: msg.oy,
    oyaw: msg.oyaw || 0,
    cells,
  };
  if (cells.length !== map.w * map.h) return;
  world.map = map;
  world.sheet = paintSheet(map);
  if (world.goal) {
    const dx = world.goal.x - map.ox;
    const dy = world.goal.y - map.oy;
    const cos = Math.cos(map.oyaw);
    const sin = Math.sin(map.oyaw);
    const col = Math.floor((cos * dx + sin * dy) / map.res);
    const row = Math.floor((-sin * dx + cos * dy) / map.res);
    const value = col < 0 || row < 0 || col >= map.w || row >= map.h
      ? 100
      : cells[row * map.w + col];
    if (value !== 255 && value >= 50) world.goal = null;
  }
  const canvas = $("slam-view");
  canvas.hidden = false;
  requestAnimationFrame(drawMap);
}

function clearMap() {
  world.map = null;
  world.pose = null;
  world.beams = [];
  world.goal = null;
  world.sheet = null;
  world.layout = null;
  world.trail = [];
  latestHost = null;
  hostHistory.length = 0;
  const canvas = $("slam-view");
  canvas.hidden = true;
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, canvas.width, canvas.height);
}

function sendLive(payload) {
  if (!live || live.readyState !== WebSocket.OPEN) return false;
  live.send(JSON.stringify(payload));
  return true;
}

const hostHistory = [];
let latestHost = null;
let debugOn = false;

function formatUptime(seconds) {
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  const secs = seconds % 60;
  return `${hours}:${String(minutes).padStart(2, "0")}:${String(secs).padStart(2, "0")}`;
}

function drawSeries(canvas, values) {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (width < 2 || height < 2) return;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);
  const fontSize = Math.max(10, Math.min(13, height * 0.12));
  const axis = fontSize * 2.6;
  const top = fontSize * 0.85;
  const bottom = height - fontSize * 0.85;
  const plotH = Math.max(1, bottom - top);
  context.font = `${fontSize}px "PingFang SC", "Noto Sans SC", sans-serif`;
  context.textAlign = "right";
  context.textBaseline = "middle";
  context.lineWidth = 1;
  [0, 25, 50, 75, 100].forEach((mark) => {
    const y = bottom - (mark / 100) * plotH;
    context.fillStyle = "rgba(244, 241, 236, 0.62)";
    context.fillText(String(mark), axis - 5, y);
    context.strokeStyle = "rgba(255,255,255,0.08)";
    context.beginPath();
    context.moveTo(axis, y);
    context.lineTo(width - 4, y);
    context.stroke();
  });
  context.beginPath();
  context.strokeStyle = "#ff5a32";
  context.lineWidth = 1.6;
  context.lineJoin = "round";
  const slot = (width - axis - 4) / 59;
  let started = false;
  values.forEach((value, index) => {
    if (value == null) {
      started = false;
      return;
    }
    const x = axis + index * slot;
    const y = bottom - (Math.max(0, Math.min(100, value)) / 100) * plotH;
    if (!started) context.moveTo(x, y);
    else context.lineTo(x, y);
    started = true;
  });
  context.stroke();
}

function renderDebug() {
  const host = latestHost;
  const online = connected() && host;
  $("cpu-now").innerHTML = online && host.cpu != null
    ? `${Number(host.cpu).toFixed(1)} <small>%</small>`
    : "-- <small>%</small>";
  $("mem-now").innerHTML = online
    ? `${host.mem_used} <small>MiB</small>`
    : "-- <small>MiB</small>";
  $("cpu-sub").textContent = online
    ? `整机 · ${host.cores} 核 · 负载 ${host.load.map((item) => Number(item).toFixed(2)).join(" ")}`
    : "登录后显示车上的占用";
  $("mem-sub").textContent = online ? `${host.mem_used} / ${host.mem_total} MiB` : "—";
  const pose = world.pose;
  const map = world.map;
  const facts = [
    ["连接", connected() ? "已连接" : "未连接"],
    ["车速", connected() && state.speed != null ? `${Number(state.speed).toFixed(2)} m/s` : "—"],
    ["电压", connected() && state.voltage != null ? `${Number(state.voltage).toFixed(1)} V` : "—"],
    ["位姿", pose ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}` : "—"],
    ["朝向", pose ? `${(pose.yaw * 180 / Math.PI).toFixed(0)}°` : "—"],
    ["地图", map ? `${map.w}×${map.h} · ${Number(map.res).toFixed(2)} m` : "—"],
    ["温度", host && host.temp != null ? `${Number(host.temp).toFixed(1)} °C` : "—"],
    ["运行", host ? formatUptime(host.uptime) : "—"],
  ];
  $("debug-facts").innerHTML = facts.map(([key, value]) => `<dt>${key}</dt><dd>${value}</dd>`).join("");
  const procs = host && Array.isArray(host.procs) ? host.procs : [];
  $("debug-procs").innerHTML = procs.length
    ? procs.map((proc) => `<li><span>${proc.name}</span><span>${proc.cpu == null ? "—" : `${Number(proc.cpu).toFixed(1)}%`} · ${proc.rss} MiB</span></li>`).join("")
    : "<li><span>还没有进程采样</span><span></span></li>";
  if (!debugOn) return;
  drawSeries($("cpu-chart"), hostHistory.map((item) => item.cpu));
  drawSeries($("mem-chart"), hostHistory.map((item) => item.mem));
}

function setDebug(open) {
  debugOn = open;
  $("drive-board").hidden = open;
  $("debug-board").hidden = !open;
  $("debug-open").classList.toggle("on", open);
  $("debug-open").setAttribute("aria-pressed", open ? "true" : "false");
  if (open) requestAnimationFrame(renderDebug);
}

function onLiveMessage(msg) {
  if (msg.type === "status") {
    state.linked = true;
    state.speed = msg.speed;
    state.voltage = msg.voltage;
    state.fault = msg.fault || "";
    renderStatus();
    if (debugOn) renderDebug();
    return;
  }
  if (msg.type === "host") {
    latestHost = msg;
    const mem = msg.mem_total ? (msg.mem_used / msg.mem_total) * 100 : null;
    hostHistory.push({ cpu: msg.cpu, mem });
    if (hostHistory.length > 60) hostHistory.shift();
    renderDebug();
    return;
  }
  if (msg.type === "voice") {
    if (msg.text && msg.text !== lastSent) {
      state.voice = msg.text;
      renderStatus();
    }
    return;
  }
  if (msg.type === "feedback") {
    hearToken += 1;
    speak(msg.text);
    return;
  }
  if (msg.type === "camera" && msg.jpeg) {
    showFrame("camera-view", msg.jpeg, "image/jpeg");
    return;
  }
  if (msg.type === "map" && msg.grid) {
    rememberMap(msg);
    return;
  }
  if (msg.type === "pose") {
    world.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
    world.beams = Array.isArray(msg.beams) ? msg.beams : [];
    const last = world.trail[world.trail.length - 1];
    if (!last || Math.hypot(msg.x - last.x, msg.y - last.y) >= 0.04) {
      world.trail.push({ x: msg.x, y: msg.y });
      if (world.trail.length > 120) world.trail.shift();
    }
    if (world.map) drawMap();
    return;
  }
  if (msg.type === "launch") {
    const session = sessions[active];
    if (msg.line) {
      session.lines.push(msg.line);
      if (session.lines.length > 300) session.lines.splice(0, session.lines.length - 300);
    }
    if (msg.done) starting = false;
    renderTabs();
    renderStatus();
    return;
  }
  if (msg.type === "closed") {
    dropLink(msg.message || "小车连接已断开");
  }
}

function dropLink(message) {
  starting = false;
  state.loggedIn = false;
  state.linked = false;
  state.speed = null;
  state.voltage = null;
  state.fault = "";
  auth = null;
  if (live) {
    live.onclose = null;
    try { live.close(); } catch (_) {}
  }
  live = null;
  sessions.forEach((session) => {
    try { session.ws?.close(); } catch (_) {}
    session.ws = null;
    session.ready = false;
  });
  clearFrame("camera-view");
  clearMap();
  $("user-name").textContent = "未登录";
  $("logout").hidden = true;
  if (message) {
    $("login-status").className = "login-status";
    $("login-status").textContent = message;
  }
  renderStatus();
  renderDebug();
  renderTabs();
}

function submitUtterance(text) {
  const line = text.trim();
  if (!line) return;
  spoken = "";
  state.voice = line;
  renderStatus();
  const reply = replyFor(line);
  if (!sendLive({ type: "hear", text: line })) {
    speak("未连接小车");
    return;
  }
  lastSent = line;
  const token = ++hearToken;
  window.setTimeout(() => {
    if (token !== hearToken) return;
    sendLive({ type: "say", text: reply });
    speak(reply);
  }, 1600);
}

function setupRecognition() {
  const Ctor = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!Ctor) return;
  recognition = new Ctor();
  recognition.lang = "zh-CN";
  recognition.continuous = true;
  recognition.interimResults = true;
  recognition.onresult = (event) => {
    let text = "";
    let finalResult = false;
    for (let i = event.resultIndex; i < event.results.length; i += 1) {
      text += event.results[i][0].transcript;
      if (event.results[i].isFinal) finalResult = true;
    }
    const line = text.trim();
    if (!line) return;
    if (!finalResult) {
      state.voice = line;
      renderStatus();
      return;
    }
    submitUtterance(line);
  };
  recognition.onerror = (event) => {
    if (event.error === "not-allowed" || event.error === "service-not-allowed") {
      wantListen = false;
      state.reply = "没有拿到麦克风";
      renderStatus();
      setWave();
    }
  };
  recognition.onend = () => {
    if (wantListen && !speaking) startRecognition();
  };
}

function startRecognition() {
  if (!recognition || speaking) return;
  try { recognition.start(); } catch (_) {}
}

wave.addEventListener("click", () => {
  if (!recognition) {
    state.reply = "这台浏览器不能识别语音，可以在下面输入";
    renderStatus();
    return;
  }
  wantListen = !wantListen;
  setWave();
  if (wantListen) startRecognition();
  else {
    try { recognition.stop(); } catch (_) {}
  }
});

$("say-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const input = $("say-input");
  const text = input.value.trim();
  if (!text) return;
  input.value = "";
  submitUtterance(text);
});

if (window.speechSynthesis) speechSynthesis.onvoiceschanged = () => pickVoice();
setupRecognition();

const sessions = [];
let active = 0;
let seq = 1;

function makeSession() {
  const session = {
    id: seq,
    lines: ["登录后，这里是小车上的终端。"],
    pending: "",
    ws: null,
    ready: false,
  };
  seq += 1;
  return session;
}

function renderTabs() {
  $("tabs").innerHTML = sessions
    .map(
      (session, index) =>
        `<button class="tab${index === active ? " on" : ""}" type="button" data-index="${index}">${session.id}</button>`
    )
    .join("");
  const session = sessions[active];
  const shown = session.lines.concat(session.pending ? [session.pending] : []);
  const log = $("log");
  log.textContent = shown.join("\n");
  log.scrollTop = log.scrollHeight;
}

function pushOutput(session, text) {
  const clean = text
    .replace(/\x1b\[[0-?]*[ -/]*[@-~]/g, "")
    .replace(/\x1b\][^\x07]*\x07/g, "");
  session.pending += clean;
  const parts = session.pending.split("\n");
  session.pending = parts.pop() || "";
  parts.forEach((part) => {
    const pieces = part.split("\r").filter((piece) => piece.length > 0);
    session.lines.push(pieces.length ? pieces[pieces.length - 1] : "");
  });
  if (session.lines.length > 300) session.lines.splice(0, session.lines.length - 300);
  if (sessions[active] === session) renderTabs();
}

function connectShell(session) {
  if (!auth || session.ws) return;
  const ws = new WebSocket(wsUrl("/term"));
  session.ws = ws;
  ws.onopen = () => {
    ws.send(JSON.stringify({
      type: "connect",
      host: auth.host,
      username: auth.username,
      password: auth.password,
    }));
  };
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "ready") {
      session.ready = true;
      return;
    }
    if (msg.type === "data") pushOutput(session, msg.data);
    if (msg.type === "error" || msg.type === "closed") {
      session.ready = false;
      pushOutput(session, `\n${msg.message || "终端已断开"}\n`);
    }
  };
  ws.onclose = () => {
    session.ready = false;
    session.ws = null;
  };
}

function runCommand(raw) {
  const session = sessions[active];
  if (!auth) {
    session.lines.push("请先点击右上角头像登录。");
    renderTabs();
    return;
  }
  if (!session.ws || session.ws.readyState !== WebSocket.OPEN || !session.ready) {
    session.lines.push("终端还没准备好。");
    renderTabs();
    return;
  }
  session.ws.send(JSON.stringify({ type: "data", data: `${raw}\n` }));
}

$("tabs").addEventListener("click", (event) => {
  const tab = event.target.closest(".tab");
  if (!tab) return;
  active = Number(tab.dataset.index);
  renderTabs();
  $("prompt").focus();
});

$("new-tab").addEventListener("click", () => {
  const session = makeSession();
  sessions.push(session);
  active = sessions.length - 1;
  if (auth) connectShell(session);
  renderTabs();
  $("tabs").scrollLeft = $("tabs").scrollWidth;
  $("prompt").focus();
});

$("prompt-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const input = $("prompt");
  runCommand(input.value);
  input.value = "";
});

$("slam-view").addEventListener("click", (event) => {
  const map = world.map;
  const layout = world.layout;
  if (!map || !layout) return;
  const canvas = $("slam-view");
  const rect = canvas.getBoundingClientRect();
  const px = (event.clientX - rect.left) * (canvas.width / rect.width);
  const py = (event.clientY - rect.top) * (canvas.height / rect.height);
  if (px < layout.left || py < layout.top || px > layout.left + layout.drawW || py > layout.top + layout.drawH) return;
  const hit = canvasToWorld(px, py, map, layout);
  if (hit.col < 0 || hit.row < 0 || hit.col >= map.w || hit.row >= map.h) return;
  const value = map.cells[hit.row * map.w + hit.col];
  if (value !== 255 && value >= 50) return;
  world.goal = { x: hit.x, y: hit.y, yaw: world.pose ? world.pose.yaw : 0 };
  drawMap();
});

new ResizeObserver(() => {
  if (world.map) drawMap();
}).observe($("slam-view").parentElement);

$("term").addEventListener("click", (event) => {
  if (event.target === $("prompt")) return;
  $("prompt").focus();
});

sessions.push(makeSession(), makeSession(), makeSession());
renderTabs();
renderStatus();
setWave();

function openLogin() {
  $("login-layer").hidden = false;
  $("login-pass").focus();
}

function closeLogin() {
  $("login-layer").hidden = true;
}

$("user-btn").addEventListener("click", openLogin);

$("debug-open").addEventListener("click", () => setDebug(!debugOn));
renderDebug();

new ResizeObserver(() => {
  if (debugOn) renderDebug();
}).observe($("debug-board"));

$("start-all").addEventListener("click", () => {
  if (!connected() || !live || starting) return;
  starting = true;
  $("start-all").disabled = true;
  $("start-all").textContent = "启动中";
  const session = sessions[active];
  session.lines.push("正在按小车现有服务启动还没运行的功能。已在运行的不会重新启动。");
  renderTabs();
  live.send(JSON.stringify({ type: "launch" }));
});
$("login-close").addEventListener("click", closeLogin);
$("login-layer").addEventListener("click", (event) => {
  if (event.target === $("login-layer")) closeLogin();
});

$("login-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const host = $("login-host").value.trim();
  const username = $("login-user").value.trim();
  const password = $("login-pass").value;
  const status = $("login-status");
  if (!host || !username || !password) {
    status.className = "login-status";
    status.textContent = "请填写主机、用户名和密码。";
    return;
  }
  if (location.protocol === "file:") {
    status.className = "login-status";
    status.textContent = "请通过本机服务打开这个页面。";
    return;
  }
  status.className = "login-status";
  status.textContent = "正在连接小车…";
  const ws = new WebSocket(wsUrl("/live"));
  let settled = false;
  const timer = window.setTimeout(() => {
    if (settled) return;
    settled = true;
    try { ws.close(); } catch (_) {}
    status.className = "login-status";
    status.textContent = "连接超时。";
  }, 30000);
  ws.onopen = () => {
    ws.send(JSON.stringify({ type: "login", host, username, password }));
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "login") {
      window.clearTimeout(timer);
      settled = true;
      if (!msg.ok) {
        status.className = "login-status";
        status.textContent = msg.message || "登录失败。";
        try { ws.close(); } catch (_) {}
        return;
      }
      auth = { host, username, password };
      live = ws;
      state.loggedIn = true;
      state.linked = true;
      $("user-name").textContent = username;
      status.className = "login-status ok";
      status.textContent = "已连上小车。可以关闭这个窗口。";
      $("logout").hidden = false;
      sessions.forEach(connectShell);
      renderStatus();
      return;
    }
    onLiveMessage(msg);
  };
  ws.onerror = () => {
    if (settled) return;
    settled = true;
    window.clearTimeout(timer);
    status.className = "login-status";
    status.textContent = "连不上本机服务。";
  };
  ws.onclose = () => {
    if (!settled) {
      settled = true;
      window.clearTimeout(timer);
      status.className = "login-status";
      status.textContent = "连接中断。";
      return;
    }
    if (live === ws) dropLink("小车连接已断开");
  };
});

$("logout").addEventListener("click", () => {
  if (live && live.readyState === WebSocket.OPEN) {
    try { live.send(JSON.stringify({ type: "logout" })); } catch (_) {}
  }
  $("login-pass").value = "";
  dropLink("已退出。");
});
