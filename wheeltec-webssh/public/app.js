(() => {
  const STORAGE_KEY = "wheeltec-webssh-prefs";

  const loginPanel = document.getElementById("loginPanel");
  const termPanel = document.getElementById("termPanel");
  const loginForm = document.getElementById("loginForm");
  const formStatus = document.getElementById("formStatus");
  const connectBtn = document.getElementById("connectBtn");
  const disconnectBtn = document.getElementById("disconnectBtn");
  const sessionMeta = document.getElementById("sessionMeta");
  const sessionLabel = document.getElementById("sessionLabel");
  const tabBar = document.getElementById("tabBar");
  const newTabBtn = document.getElementById("newTabBtn");
  const termHost = document.getElementById("termHost");
  const hostInput = document.getElementById("host");
  const portInput = document.getElementById("port");
  const usernameInput = document.getElementById("username");
  const passwordInput = document.getElementById("password");
  const rememberInput = document.getElementById("remember");

  const MAX_TABS = 6;
  const sessions = [];
  let activeId = null;
  let credentials = null;
  let seq = 0;

  function setStatus(message, isError = false) {
    formStatus.textContent = message || "";
    formStatus.classList.toggle("error", Boolean(isError));
  }

  function loadPrefs() {
    try {
      return JSON.parse(localStorage.getItem(STORAGE_KEY) || "{}");
    } catch {
      return {};
    }
  }

  function savePrefs() {
    if (!rememberInput.checked) {
      localStorage.removeItem(STORAGE_KEY);
      return;
    }
    localStorage.setItem(
      STORAGE_KEY,
      JSON.stringify({
        host: hostInput.value.trim(),
        port: Number(portInput.value) || 22,
        username: usernameInput.value.trim(),
        remember: true,
      })
    );
  }

  function copyText(text) {
    if (navigator.clipboard && window.isSecureContext) {
      return navigator.clipboard.writeText(text);
    }
    const field = document.createElement("textarea");
    field.value = text;
    field.setAttribute("readonly", "");
    field.style.position = "fixed";
    field.style.left = "-9999px";
    document.body.appendChild(field);
    field.focus();
    field.select();
    field.setSelectionRange(0, field.value.length);
    const ok = document.execCommand("copy");
    field.remove();
    if (!ok) return Promise.reject(new Error("copy failed"));
    return Promise.resolve();
  }

  async function loadPortal() {
    const hint = document.getElementById("lanHint");
    const list = document.getElementById("lanLinks");
    try {
      const res = await fetch("/api/portal");
      if (!res.ok) throw new Error("portal");
      const data = await res.json();
      const urls = Array.isArray(data.urls) ? data.urls : [];
      if (!urls.length) {
        hint.textContent = "没有发现局域网地址。请确认这台电脑已连上 Wi-Fi 或网线。";
        return;
      }
      hint.textContent = "其他电脑用浏览器打开下面的地址即可登录。点一下复制。";
      list.replaceChildren();
      for (const item of urls) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "lan-link";
        const code = document.createElement("code");
        code.textContent = item.url;
        const note = document.createElement("small");
        note.textContent = item.label === "本机名称" ? "名称" : "复制";
        button.append(code, note);
        button.addEventListener("click", async () => {
          try {
            await copyText(item.url);
            hint.textContent = "已复制。把这个地址发给同一网络里的电脑，用浏览器打开。";
          } catch {
            hint.textContent = "请选中上面的地址，手动复制后发给同一网络里的电脑。";
          }
        });
        list.appendChild(button);
      }
    } catch {
      hint.textContent = "读不到门户地址。请查看运行服务的终端里打印的链接。";
    }
  }

  async function applyDefaults() {
    const prefs = loadPrefs();
    let defaults = { host: "192.168.0.105", port: 22, username: "wheeltec" };
    try {
      const res = await fetch("/api/defaults");
      if (res.ok) defaults = await res.json();
    } catch (_) {}

    hostInput.value = prefs.host || defaults.host || "192.168.0.105";
    portInput.value = prefs.port || defaults.port || 22;
    usernameInput.value = prefs.username || defaults.username || "wheeltec";
    rememberInput.checked = Boolean(prefs.remember);
  }

  function renderTabs() {
    tabBar.replaceChildren();
    for (const session of sessions) {
      const tab = document.createElement("button");
      tab.type = "button";
      tab.className = "tab" + (session.id === activeId ? " active" : "");
      const label = document.createElement("span");
      label.textContent = session.title;
      const close = document.createElement("span");
      close.className = "tab-close";
      close.textContent = "×";
      close.setAttribute("aria-label", `关闭${session.title}`);
      close.addEventListener("click", (event) => {
        event.stopPropagation();
        closeSession(session.id);
      });
      tab.append(label, close);
      tab.addEventListener("click", () => activate(session.id));
      tabBar.appendChild(tab);
    }
    newTabBtn.disabled = sessions.length >= MAX_TABS;
  }

  function activate(id) {
    activeId = id;
    for (const session of sessions) {
      session.pane.hidden = session.id !== id;
    }
    renderTabs();
    const session = sessions.find((item) => item.id === id);
    if (!session || termPanel.hidden) return;
    requestAnimationFrame(() => {
      session.fitAddon.fit();
      session.term.focus();
      if (session.connected && session.socket && session.socket.readyState === WebSocket.OPEN) {
        session.socket.send(
          JSON.stringify({
            type: "resize",
            cols: session.term.cols,
            rows: session.term.rows,
          })
        );
      }
    });
  }

  function showWorkspace(host, username) {
    loginPanel.hidden = true;
    termPanel.hidden = false;
    sessionMeta.hidden = false;
    sessionLabel.textContent = `${username}@${host}`;
  }

  function showLogin() {
    termPanel.hidden = true;
    sessionMeta.hidden = true;
    loginPanel.hidden = false;
    connectBtn.disabled = false;
    passwordInput.focus();
  }

  function endSocket(session) {
    const socket = session.socket;
    session.socket = null;
    session.connected = false;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({ type: "disconnect" }));
      socket.close();
    }
  }

  function closeSession(id) {
    const index = sessions.findIndex((item) => item.id === id);
    if (index < 0) return;
    const session = sessions[index];
    endSocket(session);
    session.term.dispose();
    session.pane.remove();
    sessions.splice(index, 1);
    if (!sessions.length) {
      activeId = null;
      renderTabs();
      showLogin();
      return;
    }
    if (activeId === id) activate(sessions[Math.max(0, index - 1)].id);
    else renderTabs();
  }

  function disconnectAll() {
    credentials = null;
    const openSessions = sessions.slice();
    sessions.length = 0;
    activeId = null;
    for (const session of openSessions) {
      endSocket(session);
      session.term.dispose();
      session.pane.remove();
    }
    renderTabs();
    showLogin();
    setStatus("已断开连接");
  }

  function createSession() {
    const id = ++seq;
    const pane = document.createElement("div");
    pane.className = "term-pane";
    pane.hidden = true;
    const mount = document.createElement("div");
    mount.className = "terminal";
    pane.appendChild(mount);
    termHost.appendChild(pane);

    const term = new Terminal({
      cursorBlink: true,
      fontFamily: '"IBM Plex Mono", Menlo, monospace',
      fontSize: 14,
      lineHeight: 1.2,
      theme: {
        background: "#0b100e",
        foreground: "#e7efe8",
        cursor: "#c4f542",
        selectionBackground: "rgba(61, 214, 198, 0.35)",
        black: "#0b100e",
        green: "#c4f542",
        cyan: "#3dd6c6",
        yellow: "#f0c34a",
        red: "#ff6b5a",
      },
    });
    const fitAddon = new FitAddon.FitAddon();
    term.loadAddon(fitAddon);
    term.open(mount);

    const session = {
      id,
      title: `终端 ${id}`,
      term,
      fitAddon,
      pane,
      socket: null,
      connected: false,
    };
    term.onData((data) => {
      if (session.connected && session.socket && session.socket.readyState === WebSocket.OPEN) {
        session.socket.send(JSON.stringify({ type: "data", data }));
      }
    });
    sessions.push(session);
    renderTabs();
    return session;
  }

  function openSocket(session) {
    const { host, port, username, password } = credentials;
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const socket = new WebSocket(`${proto}//${location.host}/ssh`);
    session.socket = socket;

    socket.addEventListener("open", () => {
      if (!termPanel.hidden && activeId === session.id) session.fitAddon.fit();
      socket.send(
        JSON.stringify({
          type: "connect",
          host,
          port,
          username,
          password,
          cols: session.term.cols || 120,
          rows: session.term.rows || 36,
        })
      );
    });

    socket.addEventListener("message", (event) => {
      if (!sessions.includes(session)) return;
      let msg;
      try {
        msg = JSON.parse(event.data);
      } catch {
        return;
      }

      if (msg.type === "status") {
        if (!loginPanel.hidden) setStatus(msg.message || "");
        return;
      }

      if (msg.type === "ready") {
        session.connected = true;
        connectBtn.disabled = false;
        setStatus("");
        showWorkspace(msg.host || host, msg.username || username);
        if (activeId == null || activeId === session.id) activate(session.id);
        return;
      }

      if (msg.type === "data") {
        session.term.write(msg.data || "");
        return;
      }

      if (msg.type === "error") {
        session.connected = false;
        if (loginPanel.hidden) {
          session.term.writeln(`\r\n\x1b[31m${msg.message || "连接出错"}\x1b[0m`);
          return;
        }
        setStatus(msg.message || "连接出错", true);
        connectBtn.disabled = false;
        closeSession(session.id);
        return;
      }

      if (msg.type === "closed") {
        session.connected = false;
        if (loginPanel.hidden) {
          session.term.writeln(`\r\n\x1b[33m${msg.message || "连接已关闭"}\x1b[0m`);
        } else {
          setStatus(msg.message || "连接已关闭");
          connectBtn.disabled = false;
        }
      }
    });

    socket.addEventListener("close", () => {
      session.connected = false;
      if (!loginPanel.hidden) connectBtn.disabled = false;
    });

    socket.addEventListener("error", () => {
      session.connected = false;
      if (!sessions.includes(session)) return;
      if (loginPanel.hidden) {
        session.term.writeln("\r\n\x1b[31m无法连接 WebSSH 服务\x1b[0m");
        return;
      }
      setStatus(
        "无法连接 WebSSH 服务。请确认运行服务的电脑已执行 npm start，且本机可以访问它的 8787 端口。",
        true
      );
      connectBtn.disabled = false;
      closeSession(session.id);
    });
  }

  function connect() {
    const host = hostInput.value.trim();
    const port = Number(portInput.value) || 22;
    const username = usernameInput.value.trim();
    const password = passwordInput.value;

    if (!host || !username || !password) {
      setStatus("请填写完整登录信息", true);
      return;
    }

    savePrefs();
    credentials = { host, port, username, password };
    setStatus("正在建立连接…");
    connectBtn.disabled = true;
    const session = createSession();
    activeId = session.id;
    renderTabs();
    openSocket(session);
  }

  function addTerminal() {
    if (!credentials || sessions.length >= MAX_TABS) return;
    const session = createSession();
    activate(session.id);
    session.term.writeln("\x1b[36m正在连接…\x1b[0m");
    openSocket(session);
  }

  window.addEventListener("resize", () => {
    const session = sessions.find((item) => item.id === activeId);
    if (!session || termPanel.hidden || !session.connected) return;
    session.fitAddon.fit();
    if (session.socket && session.socket.readyState === WebSocket.OPEN) {
      session.socket.send(
        JSON.stringify({
          type: "resize",
          cols: session.term.cols,
          rows: session.term.rows,
        })
      );
    }
  });

  loginForm.addEventListener("submit", (event) => {
    event.preventDefault();
    connect();
  });

  newTabBtn.addEventListener("click", addTerminal);
  disconnectBtn.addEventListener("click", disconnectAll);

  applyDefaults();
  loadPortal();
})();
