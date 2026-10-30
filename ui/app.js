const state = {
  speed: 0.42,
  voltage: 24.6,
  linked: false,
  loggedIn: false,
  fault: "",
  voice: "跟着前面那个人",
  lockId: 3,
  moving: false,
  robot: { x: 30, y: 76 },
  goal: { x: 74, y: 24 },
  objects: [
    { id: 3, label: "人", x: 34, y: 18, w: 18, h: 62 },
    { id: 1, label: "椅子", x: 58, y: 38, w: 22, h: 36 },
  ],
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

function connected() {
  return state.loggedIn && state.linked;
}

function renderStatus() {
  const online = connected();
  $("speed").innerHTML = online
    ? `${state.speed.toFixed(2)} <small>m/s</small>`
    : "-- <small>m/s</small>";
  $("voltage").innerHTML = online
    ? `${state.voltage.toFixed(1)} <small>V</small>`
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
  $("go").disabled = !online || state.moving;

  wave.classList.toggle("off", !online);
  $("voice-line").textContent = `最近指令：${state.voice || "—"}`;
}

function renderLock() {
  const locked = state.objects.find((item) => item.id === state.lockId);
  $("lock-line").textContent = locked
    ? `已锁定：${locked.label} #${locked.id}`
    : "未锁定";
  $("unlock").disabled = !connected() || !locked;
  $("boxes").innerHTML = state.objects
    .map(
      (item) => `<button class="box${item.id === state.lockId ? " locked" : ""}" type="button" data-id="${item.id}" style="left:${item.x}%;top:${item.y}%;width:${item.w}%;height:${item.h}%"><b>${item.label} #${item.id}</b></button>`
    )
    .join("");
}

$("boxes").addEventListener("click", (event) => {
  const button = event.target.closest(".box");
  if (!button || !connected()) return;
  state.lockId = Number(button.dataset.id);
  renderLock();
});

$("unlock").addEventListener("click", () => {
  state.lockId = null;
  state.speed = 0;
  renderLock();
  renderStatus();
});

const camera = $("camera");
const ctx = camera.getContext("2d");

function drawCamera() {
  const w = camera.width;
  const h = camera.height;
  ctx.fillStyle = "#1a1c20";
  ctx.fillRect(0, 0, w, h);
  ctx.fillStyle = "#2a2e33";
  ctx.beginPath();
  ctx.moveTo(w * 0.18, h * 0.28);
  ctx.lineTo(w * 0.82, h * 0.28);
  ctx.lineTo(w, h);
  ctx.lineTo(0, h);
  ctx.fill();
  ctx.fillStyle = "#3a4046";
  for (let i = 0; i < 6; i += 1) {
    ctx.fillRect(w * (0.28 + i * 0.08), h * 0.22, 2, h * 0.12);
  }
  ctx.fillStyle = "#c8ccd0";
  ctx.fillRect(w * 0.08, h * 0.16, w * 0.14, 3);
  ctx.fillRect(w * 0.72, h * 0.16, w * 0.14, 3);
  const person = state.objects.find((item) => item.id === 3);
  const chair = state.objects.find((item) => item.id === 1);
  if (person) {
    ctx.fillStyle = "#24282d";
    const px = w * ((person.x + person.w * 0.5) / 100);
    ctx.fillRect(px - 7, h * 0.34, 14, 28);
    ctx.beginPath();
    ctx.arc(px, h * 0.3, 6, 0, Math.PI * 2);
    ctx.fill();
  }
  if (chair) {
    ctx.strokeStyle = "#8d9399";
    ctx.strokeRect(w * 0.62, h * 0.48, 22, 16);
  }
}

function renderMap() {
  const { robot, goal } = state;
  const bend = { x: robot.x, y: Math.min(robot.y, goal.y) - 8 };
  $("route").setAttribute("d", `M ${robot.x} ${robot.y} L ${bend.x} ${goal.y} L ${goal.x} ${goal.y}`);
  $("sonar").innerHTML = [0, 1, 2]
    .map((i) => {
      const a = (-2.4 + i * 0.45);
      const x2 = robot.x + Math.cos(a) * 7;
      const y2 = robot.y + Math.sin(a) * 7;
      return `<line x1="${robot.x}" y1="${robot.y}" x2="${x2.toFixed(1)}" y2="${y2.toFixed(1)}"></line>`;
    })
    .join("");
  $("robot").innerHTML = `<polygon points="${robot.x},${robot.y - 3.2} ${robot.x - 2.4},${robot.y + 2.6} ${robot.x + 2.4},${robot.y + 2.6}"></polygon>`;
  $("goal").innerHTML = `<circle cx="${goal.x}" cy="${goal.y}" r="1.5"></circle><path d="M ${goal.x} ${goal.y - 6} v 4"></path><text x="${goal.x}" y="${goal.y + 7}">B</text>`;
}

const map = $("map");
map.addEventListener("click", (event) => {
  if (!connected() || state.moving) return;
  const point = map.createSVGPoint();
  point.x = event.clientX;
  point.y = event.clientY;
  const mapped = point.matrixTransform(map.getScreenCTM().inverse());
  state.goal.x = Math.min(92, Math.max(8, mapped.x));
  state.goal.y = Math.min(92, Math.max(8, mapped.y));
  renderMap();
});

$("go").addEventListener("click", () => {
  if (!connected() || state.moving) return;
  state.moving = true;
  $("go").disabled = true;
  const from = { ...state.robot };
  const to = { ...state.goal };
  const started = performance.now();
  const duration = 2800;
  const step = (now) => {
    const t = Math.min(1, (now - started) / duration);
    state.robot.x = from.x + (to.x - from.x) * t;
    state.robot.y = from.y + (to.y - from.y) * t;
    state.speed = t < 1 ? 0.42 : 0;
    renderMap();
    renderStatus();
    if (t < 1) requestAnimationFrame(step);
    else {
      state.moving = false;
      $("go").disabled = false;
    }
  };
  requestAnimationFrame(step);
});

const sessions = [];
let active = 0;
let seq = 1;

function makeSession() {
  const session = {
    id: seq,
    lines: seq === 1
      ? ["仿真模式，尚未连接小车。", "输入 help 查看测试信号。"]
      : ["新会话。输入 help 查看测试信号。"],
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
  const log = $("log");
  log.textContent = sessions[active].lines.join("\n");
  const pin = () => {
    log.scrollTop = log.scrollHeight;
  };
  pin();
  requestAnimationFrame(pin);
}

function print(text) {
  sessions[active].lines.push(text);
  renderTabs();
}

function helpText() {
  return [
    "sim speed 0.80        车速，单位 m/s",
    "sim voltage 23.2      电压，单位 V",
    "sim link on|off       连接状态",
    "sim func on           功能恢复正常",
    "sim func off 相机     某一项异常，灯变红",
    "sim voice 去门口等我   最近一条语音指令",
    "sim lose              锁定目标消失",
    "sim detect            检测框重新出现",
  ].join("\n");
}

function runCommand(raw) {
  const line = raw.trim();
  print(`wheeltec@wheeltec:~$ ${line}`);
  if (!line) return;
  const [cmd, sub, ...rest] = line.split(/\s+/);
  if (cmd === "help") {
    print(helpText());
    return;
  }
  if (cmd !== "sim") {
    print("仿真环境没有执行这条命令。输入 help。");
    return;
  }
  if (sub === "speed") {
    const value = Number(rest[0]);
    if (!Number.isFinite(value)) return print("用法：sim speed 0.42");
    state.speed = value;
  } else if (sub === "voltage") {
    const value = Number(rest[0]);
    if (!Number.isFinite(value)) return print("用法：sim voltage 24.6");
    state.voltage = value;
  } else if (sub === "link") {
    if (rest[0] !== "on" && rest[0] !== "off") return print("用法：sim link on 或 sim link off");
    if (!state.loggedIn) return print("请先点击右上角头像登录。");
    state.linked = rest[0] === "on";
  } else if (sub === "func") {
    if (rest[0] === "on") state.fault = "";
    else if (rest[0] === "off") state.fault = rest[1] || "相机";
    else return print("用法：sim func on 或 sim func off 相机");
  } else if (sub === "voice") {
    const text = rest.join(" ");
    if (!text) return print("用法：sim voice 跟着前面那个人");
    state.voice = text;
  } else if (sub === "lose") {
    state.objects = state.objects.filter((item) => item.id !== state.lockId);
    state.lockId = null;
    state.speed = 0;
  } else if (sub === "detect") {
    state.objects = [
      { id: 3, label: "人", x: 34, y: 18, w: 18, h: 62 },
      { id: 1, label: "椅子", x: 58, y: 38, w: 22, h: 36 },
    ];
  } else {
    print("未知的仿真信号。输入 help。");
    return;
  }
  renderStatus();
  renderLock();
  drawCamera();
  print("已写入仿真信号。");
}

$("tabs").addEventListener("click", (event) => {
  const tab = event.target.closest(".tab");
  if (!tab) return;
  active = Number(tab.dataset.index);
  renderTabs();
  $("prompt").focus();
});

$("new-tab").addEventListener("click", () => {
  sessions.push(makeSession());
  active = sessions.length - 1;
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

$("term").addEventListener("click", () => $("prompt").focus());

sessions.push(makeSession(), makeSession(), makeSession());
renderTabs();
renderStatus();
renderLock();
drawCamera();
renderMap();

function openLogin() {
  $("login-layer").hidden = false;
  $("login-user").focus();
}

function closeLogin() {
  $("login-layer").hidden = true;
}

$("user-btn").addEventListener("click", openLogin);
$("login-close").addEventListener("click", closeLogin);
$("login-layer").addEventListener("click", (event) => {
  if (event.target === $("login-layer")) closeLogin();
});

$("login-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const name = $("login-user").value.trim();
  const password = $("login-pass").value;
  const status = $("login-status");
  if (!name || !password) {
    status.className = "login-status";
    status.textContent = "请填写用户名和密码。";
    return;
  }
  state.loggedIn = true;
  state.linked = true;
  $("user-name").textContent = name;
  status.className = "login-status ok";
  status.textContent = "已登录。可以关闭这个窗口。";
  $("logout").hidden = false;
  renderStatus();
  renderLock();
});

$("logout").addEventListener("click", () => {
  state.loggedIn = false;
  state.linked = false;
  state.moving = false;
  $("user-name").textContent = "未登录";
  $("login-pass").value = "";
  $("login-status").className = "login-status";
  $("login-status").textContent = "已退出。";
  $("logout").hidden = true;
  renderStatus();
  renderLock();
});

setInterval(() => {
  if (!connected() || state.moving) return;
  const shown = Math.max(0, state.speed + (Math.random() - 0.5) * 0.03);
  $("speed").innerHTML = `${shown.toFixed(2)} <small>m/s</small>`;
}, 900);
