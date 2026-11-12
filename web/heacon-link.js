(function () {
  // 离线环境：掐掉页面框架对外部图标 CDN 的请求，避免它无限重试拖垮轮询。
  const BLOCKED = ["api.simplesvg.com", "api.unisvg.com", "iconify.design"];
  try {
    const nativeFetch = window.fetch;
    if (nativeFetch) {
      window.fetch = function (input, init) {
        const url = typeof input === "string" ? input : (input && input.url) || "";
        if (BLOCKED.some(function (host) { return url.indexOf(host) >= 0; })) {
          return Promise.reject(new TypeError("offline: blocked " + url));
        }
        return nativeFetch.apply(this, arguments);
      };
    }
  } catch (e) {}
  try {
    const nativeOpen = XMLHttpRequest.prototype.open;
    XMLHttpRequest.prototype.open = function (method, url) {
      const href = String(url || "");
      if (BLOCKED.some(function (host) { return href.indexOf(host) >= 0; })) {
        this.__heaconBlocked = true;
      }
      return nativeOpen.apply(this, arguments);
    };
    const nativeSend = XMLHttpRequest.prototype.send;
    XMLHttpRequest.prototype.send = function () {
      if (this.__heaconBlocked) {
        this.dispatchEvent(new Event("error"));
        return;
      }
      return nativeSend.apply(this, arguments);
    };
  } catch (e) {}
})();

(function () {
  if (window.__heaconLinked) return;
  window.__heaconLinked = true;
  if (shouldUseHttpsDoor()) {
    location.replace("https://" + location.hostname + ":8766/" + location.search);
    return;
  }

  const CAM_MS = 400;
  const POLL_MS = 700;
  const STATE_MISS_WEAK = 3;
  const STATE_MISS_LOCK = 25;
  let liveTimer = 0;
  let pollTimer = 0;
  let heaconPoints = null;
  let heaconFrame = { width: 1280, height: 720 };
  let positionReady = false;
  let linkedRun = false;
  let robotStatus = null;
  let armLink = {
    connected: false,
    session: false,
    live: false,
    phase: "offline",
    ready: false,
    moving: false,
    fsm: null,
  };
  let unlocking = false;
  let disconnecting = false;
  let hintHide = 0;
  let stateMisses = 0;
  let camBusy = false;
  let lastCamUrl = "";
  let liveWatch = null;
  let sawSession = false;
  let lockRequested = false;
  let lastFitKey = "";
  let positionWait = null;
  // 这些状态在 IIFE 初始化阶段（startPoll）就会用到，必须先声明，避免暂时性死区。
  let armGateBusy = false;
  let lastLinkActAt = 0;
  let linkActing = false;
  const NET_TIMEOUT_MS = 3500;
  // 页面刚打开时先静默观察一段时间，避免用残留状态把机械臂锁掉。
  const SESSION_GRACE_MS = 6000;
  const pageOpenedAt = Date.now();
  let sawLiveThisPage = false;

  const original = {
    confirmPlan: window.confirmPlan,
    startScan: window.startScan,
    cancelScan: window.cancelScan,
    drawPoints: window.drawPoints,
    svgPoint: window.svgPoint,
    movePoint: window.movePoint,
    endDrag: window.endDrag,
    startServiceFromModal: window.startServiceFromModal,
    nudgeSpeed: window.nudgeSpeed,
    setLevelGrade: window.setLevelGrade,
    closeAdjust: window.closeAdjust,
    pauseService: window.pauseService,
    resumeService: window.resumeService,
    finishService: window.finishService,
    resetService: window.resetService,
    startRun: window.startRun,
    confirmRoute: window.confirmRoute,
    currentShow: window.currentShow,
    paintRunView: window.paintRunView,
    renderRunRoute: window.renderRunRoute,
    archiveService: window.archiveService,
    logEvent: window.logEvent,
    showToast: window.showToast,
  };

  injectStyle();
  ensureToast();
  wrapPage();
  startPoll();
  resumeRunningUi();
  installPageFullscreen();
  installTabletHome();
  installArmGate();
  window.primeHeaconVoice = primeVoice;

  function injectStyle() {
    const style = document.createElement("style");
    style.textContent = [
      ".body-stage.is-live{background:#1c1a18}",
      ".body-stage.is-live .body-stage-figure,.body-stage-figure.heacon-hidden{display:none!important}",
      ".heacon-live-tag{position:absolute;top:10px;left:10px;z-index:5;font-size:11px;font-weight:600;color:#fff;background:#ff6a1a;border-radius:999px;padding:3px 8px;letter-spacing:.04em}",
      ".heacon-live-note{position:absolute;inset:0;z-index:6;display:none;align-items:center;justify-content:center;color:#fff;background:rgba(28,26,24,.42);font-size:15px;font-weight:600;text-align:center;padding:24px;pointer-events:none}",
      ".heacon-live-frame.is-wait .heacon-live-note,.heacon-live-frame.is-stale .heacon-live-note{display:flex}",
      ".body-stage--position.is-live>.relative:not(.heacon-live-frame){position:absolute;left:50%;bottom:56px;transform:translateX(-50%);z-index:3}",
      ".position-layout:has(.is-live),.confirm-layout:has(.is-live){align-items:stretch}",
      ".confirm-layout:has(.is-live){grid-template-columns:minmax(0,1fr) 300px}",
      ".body-stage.body-stage--position.is-live,.body-stage.body-stage--confirm.is-live{",
      "  position:relative;",
      "  display:flex;",
      "  flex-direction:column;",
      "  align-items:center;",
      "  justify-content:center;",
      "  overflow:hidden;",
      "  aspect-ratio:auto!important;",
      "  width:100%!important;",
      "  height:100%!important;",
      "  max-height:none!important;",
      "  min-height:0!important;",
      "  flex:1 1 auto!important;",
      "  padding:0!important;",
      "  background:#1c1a18;",
      "}",
      ".body-stage--confirm.is-live .body-map{",
      "  width:100%!important;",
      "  height:100%!important;",
      "  max-width:none!important;",
      "  max-height:none!important;",
      "  aspect-ratio:auto!important;",
      "  margin:0!important;",
      "  display:flex;",
      "  align-items:center;",
      "  justify-content:center;",
      "  background:#1c1a18;",
      "  transform:none!important;",
      "}",
      ".heacon-live-frame{",
      "  position:relative;",
      "  flex:0 0 auto;",
      "  align-self:center;",
      "  margin:0 auto;",
      "  overflow:hidden;",
      "  background:#1c1a18;",
      "  width:100%;",
      "  height:100%;",
      "  min-width:160px;",
      "  min-height:160px;",
      "}",
      ".heacon-live-spin{",
      "  position:absolute;",
      "  transform:rotate(90deg);",
      "  transform-origin:center center;",
      "}",
      ".heacon-live-spin .heacon-live-cam,",
      ".heacon-live-spin #bodyView{",
      "  position:absolute!important;",
      "  inset:0!important;",
      "  left:0!important;",
      "  top:0!important;",
      "  width:100%!important;",
      "  height:100%!important;",
      "  margin:0!important;",
      "  max-width:none!important;",
      "  max-height:none!important;",
      "  object-fit:contain!important;",
      "  background:transparent;",
      "  transform:none!important;",
      "}",
      ".heacon-live-spin .heacon-live-cam{pointer-events:none;z-index:1}",
      ".heacon-live-spin #bodyView{z-index:2;pointer-events:auto}",
      ".heacon-live-frame.is-aim::before,.heacon-live-frame.is-aim::after{",
      "  content:'';",
      "  position:absolute;",
      "  z-index:4;",
      "  pointer-events:none;",
      "  background:#00dc00;",
      "  box-shadow:0 0 0 1.5px #111;",
      "}",
      ".heacon-live-frame.is-aim::before{left:0;right:0;top:50%;height:1px;margin-top:-0.5px}",
      ".heacon-live-frame.is-aim::after{top:0;bottom:0;left:50%;width:1px;margin-left:-0.5px}",
      ".body-map.is-live>img:not(.heacon-live-cam){display:none!important}",
      "#heaconToast{position:fixed;left:50%;bottom:28px;transform:translateX(-50%);background:#3d3834;color:#fff;padding:10px 16px;border-radius:999px;font-size:13px;z-index:80;opacity:0;pointer-events:none;transition:opacity .2s}",
      "#heaconToast.show{opacity:1}",
      "#heaconEar{position:fixed;left:50%;top:14px;transform:translateX(-50%);display:none;align-items:center;gap:8px;background:#3d3834;color:#fff;padding:8px 16px;border-radius:999px;font-size:13px;font-weight:600;z-index:85;box-shadow:0 8px 20px rgba(61,56,52,.18)}",
      "#heaconEar.is-on{display:flex}",
      "#heaconEar.is-hot{background:#ff6a1a;box-shadow:0 8px 20px rgba(255,106,26,.28)}",
      "html{--sat:env(safe-area-inset-top,0px);--sab:env(safe-area-inset-bottom,0px)}",
      "html.heacon-tablet,html.heacon-tablet body,html.heacon-fs,html.heacon-fs body{",
      "  position:fixed!important;",
      "  top:0!important;",
      "  left:0!important;",
      "  right:0!important;",
      "  bottom:0!important;",
      "  width:100vw!important;",
      "  height:100lvh!important;",
      "  min-height:100lvh!important;",
      "  max-height:none!important;",
      "  margin:0!important;",
      "  padding:0!important;",
      "  overflow:hidden!important;",
      "}",
      "html.heacon-tablet body,html.heacon-fs body{",
      "  display:flex!important;",
      "  align-items:stretch!important;",
      "  justify-content:flex-start!important;",
      "}",
      "html.heacon-tablet .app-shell,html.heacon-fs .app-shell{",
      "  position:fixed!important;",
      "  top:0!important;",
      "  left:0!important;",
      "  right:0!important;",
      "  bottom:0!important;",
      "  width:100vw!important;",
      "  height:100lvh!important;",
      "  max-width:none!important;",
      "  max-height:none!important;",
      "  margin:0!important;",
      "  padding:0!important;",
      "}",
      "html.heacon-tablet .header-glass{",
      "  display:flex!important;",
      "  align-items:center!important;",
      "  height:calc(56px + var(--sat))!important;",
      "  min-height:calc(56px + var(--sat))!important;",
      "  padding:var(--sat) 24px 0!important;",
      "  box-sizing:border-box!important;",
      "}",
      "html.heacon-tablet .sidebar-glass{",
      "  padding-top:calc(12px + var(--sat))!important;",
      "  padding-bottom:8px!important;",
      "}",
      "html.heacon-tablet #content{padding:8px 16px 8px!important}",
      "html.heacon-tablet .screen[data-screen=position]>.flex.justify-between.items-end,",
      "html.heacon-tablet .screen[data-screen=confirm]>.flex.justify-between.items-end{",
      "  margin:0 0 6px!important;",
      "}",
      "html.heacon-tablet .screen[data-screen=position]>.flex.justify-between.items-end p,",
      "html.heacon-tablet .screen[data-screen=confirm]>.flex.justify-between.items-end p{display:none!important}",
      "html.heacon-tablet .screen[data-screen=run]{justify-content:flex-start!important}",
      "html.heacon-tablet .body-stage.is-live{align-items:center!important;justify-content:center!important}",
      ".header-glass{overflow:visible}",
      ".header-glass .flex.items-center{flex-wrap:nowrap}",
      "#heaconDeviceChip{appearance:none;border:0;white-space:nowrap;flex:0 0 auto;max-width:none;min-height:40px;cursor:pointer;user-select:none;touch-action:manipulation;-webkit-tap-highlight-color:transparent}",
      "#heaconDeviceChip iconify-icon{pointer-events:none}",
      "#heaconDeviceChip.is-off{background:#ffe3e0!important;color:#cf1322!important}",
      "#heaconDeviceChip.is-wait{background:#fff7e6!important;color:#d48806!important}",
      "#heaconDeviceChip.is-ok{background:#e8f8e0!important;color:#237804!important}",
      "#heaconLinkHint{display:none;align-items:center;justify-content:center;gap:8px;height:32px;font-size:13px;font-weight:600}",
      "#heaconLinkHint.is-on{display:flex}",
      "#heaconLinkHint.is-wait{background:#fff7e6;color:#d48806}",
      "#heaconLinkHint.is-retry{background:#fff1f0;color:#cf1322}",
      "#heaconLinkHint.is-ok{background:#f6ffed;color:#389e0d}",
      "#heaconLinkHint i{width:6px;height:6px;border-radius:50%;background:currentColor;opacity:.35;animation:heacon-dot 1s infinite}",
      "#heaconLinkHint i:nth-child(2){animation-delay:.15s}",
      "#heaconLinkHint i:nth-child(3){animation-delay:.3s}",
      "@keyframes heacon-dot{0%,80%,100%{opacity:.25}40%{opacity:1}}",
      "#heaconLinkModal{z-index:200!important}",
      "#heaconLinkModal.flex{display:flex!important}",
      "#heaconLinkGo{min-height:52px;touch-action:manipulation;-webkit-tap-highlight-color:transparent}",
      "#heaconLinkGo iconify-icon{pointer-events:none}",
      "#heaconUnlockMask{position:fixed;inset:0;z-index:120;display:none;align-items:center;justify-content:center;background:rgba(61,56,52,.28);padding:24px;pointer-events:none}",
      "#heaconUnlockMask.is-on{display:flex;pointer-events:auto}",
      "#heaconUnlockMask .heacon-unlock-card{width:min(420px,92vw);background:#fff;border-radius:20px;padding:28px 24px 20px;box-shadow:0 18px 40px rgba(61,56,52,.16)}",
      "#heaconUnlockMask h3{margin:0 0 8px;font-size:20px;font-weight:700;color:#3d3834}",
      "#heaconUnlockMask p{margin:0 0 22px;font-size:14px;line-height:1.6;color:#8a847e}",
      "#heaconUnlockMask .heacon-unlock-actions{display:flex;gap:10px;justify-content:flex-end}",
      "#heaconUnlockMask .heacon-unlock-skip{border:0;background:#f6f3f0;color:#5c564f;border-radius:999px;padding:10px 16px;font-size:14px;font-weight:600}",
      "#heaconUnlockMask .heacon-unlock-go{border:0;background:#ff6a1a;color:#fff;border-radius:999px;padding:10px 18px;font-size:14px;font-weight:700}",
      "#heaconUnlockMask .heacon-unlock-go:disabled{opacity:.55}",
      "#heaconHomeInstall{position:fixed;right:16px;bottom:16px;z-index:90;display:none;align-items:center;gap:10px;max-width:min(420px,70vw);background:#3d3834;color:#fff;padding:12px 14px;border-radius:16px;box-shadow:0 10px 28px rgba(61,56,52,.22);font-size:13px;line-height:1.4}",
      "html.heacon-tablet #heaconHomeInstall.is-on{display:flex}",
      "#heaconHomeInstall button{flex:0 0 auto;border:0;background:#ff6a1a;color:#fff;border-radius:999px;padding:8px 14px;font-weight:700;font-size:13px}",
      "#heaconHomeInstall .heacon-home-x{flex:0 0 auto;border:0;background:transparent;color:#fff;font-size:18px;line-height:1;padding:0 4px}",
    ].join("");
    document.head.appendChild(style);
  }

  function isHeaconPadApp() {
    return /HeaconPad/i.test(navigator.userAgent || "");
  }

  function isTabletRemote() {
    if (isLocalHost()) return false;
    if (isHeaconPadApp()) return true;
    return !!(window.matchMedia && window.matchMedia("(pointer: coarse)").matches);
  }

  function wantPageFullscreen() {
    return true;
  }

  function isPageFullscreen() {
    return !!(
      isHeaconPadApp() ||
      document.fullscreenElement ||
      document.webkitFullscreenElement ||
      (window.matchMedia &&
        (window.matchMedia("(display-mode: fullscreen)").matches ||
          window.matchMedia("(display-mode: standalone)").matches)) ||
      window.navigator.standalone
    );
  }

  function enterPageFullscreen() {
    if (isPageFullscreen()) {
      document.documentElement.classList.add("heacon-fs");
      return;
    }
    const root = document.documentElement;
    const req = root.requestFullscreen || root.webkitRequestFullscreen || root.webkitRequestFullScreen;
    if (req) {
      try {
        const ret = req.call(root, { navigationUI: "hide" });
        if (ret && ret.catch) ret.catch(function () {});
      } catch (err) {}
    }
    if (isTabletRemote()) refitSoon();
  }

  function viewportBox() {
    const vv = window.visualViewport;
    const vvW = vv ? vv.width + (vv.offsetLeft || 0) : 0;
    const vvH = vv ? vv.height + (vv.offsetTop || 0) : 0;
    return {
      w: Math.max(window.innerWidth || 0, document.documentElement.clientWidth || 0, vvW, 1),
      h: Math.max(window.innerHeight || 0, document.documentElement.clientHeight || 0, vvH, 1),
    };
  }

  function readSafeTop() {
    const probe = document.createElement("div");
    probe.style.cssText =
      "position:fixed;left:0;top:0;width:0;height:env(safe-area-inset-top,0px);visibility:hidden;pointer-events:none";
    document.body.appendChild(probe);
    const px = probe.offsetHeight || 0;
    probe.remove();
    const vv = window.visualViewport;
    const fromView = vv && vv.offsetTop ? Math.round(vv.offsetTop) : 0;
    return Math.max(px, fromView);
  }

  function fitPageToViewport() {
    const root = document.documentElement;
    root.classList.toggle("heacon-fs", isPageFullscreen());
    if (!isTabletRemote()) {
      root.classList.remove("heacon-tablet");
      lastFitKey = "";
      return;
    }
    const box = viewportBox();
    const w = Math.max(1, Math.round(box.w));
    const h = Math.max(1, Math.round(box.h));
    const sat = readSafeTop();
    const key = w + "x" + h + "x" + sat + "x" + (isPageFullscreen() ? "1" : "0");
    if (key === lastFitKey && root.classList.contains("heacon-tablet")) return;
    lastFitKey = key;
    root.style.setProperty("--app-w", w + "px");
    root.style.setProperty("--app-h", h + "px");
    root.style.setProperty("--sat", sat + "px");
    root.classList.add("heacon-tablet");
    root.classList.toggle("heacon-fs", isPageFullscreen());
    const theme = document.querySelector('meta[name="theme-color"]');
    if (theme) theme.setAttribute("content", "#ffffff");
    [root, document.body].forEach(function (el) {
      if (!el) return;
      el.style.position = "fixed";
      el.style.left = "0";
      el.style.top = "0";
      el.style.right = "0";
      el.style.bottom = "0";
      el.style.width = "100vw";
      el.style.height = "100lvh";
      el.style.margin = "0";
      el.style.padding = "0";
    });
    document.body.style.alignItems = "stretch";
    document.body.style.justifyContent = "flex-start";
    const shell = document.querySelector(".app-shell");
    if (shell) {
      shell.style.position = "fixed";
      shell.style.top = "0";
      shell.style.left = "0";
      shell.style.right = "0";
      shell.style.bottom = "0";
      shell.style.width = "100vw";
      shell.style.height = "100lvh";
      shell.style.margin = "0";
    }
    window.scrollTo(0, 0);
    requestAnimationFrame(function () {
      document.querySelectorAll(".heacon-live-frame").forEach(sizeLiveFrame);
      if (typeof window.syncBrandWidth === "function") window.syncBrandWidth();
    });
  }

  function refitSoon() {
    fitPageToViewport();
    [50, 180, 400].forEach(function (ms) {
      setTimeout(fitPageToViewport, ms);
    });
  }

  function installPageFullscreen() {
    if (!document.querySelector('meta[name="mobile-web-app-capable"]')) {
      [
        ["mobile-web-app-capable", "yes"],
        ["apple-mobile-web-app-capable", "yes"],
        ["apple-mobile-web-app-status-bar-style", "black-translucent"],
        ["theme-color", "#ffffff"],
      ].forEach(function (pair) {
        const el = document.createElement("meta");
        el.setAttribute("name", pair[0]);
        el.setAttribute("content", pair[1]);
        document.head.appendChild(el);
      });
    }
    const mark = function () {
      document.documentElement.classList.toggle("heacon-fs", isPageFullscreen());
      if (isTabletRemote()) refitSoon();
    };
    document.addEventListener("fullscreenchange", mark);
    document.addEventListener("webkitfullscreenchange", mark);
    window.addEventListener("resize", fitPageToViewport);
    window.addEventListener("orientationchange", refitSoon);
    if (window.visualViewport) {
      window.visualViewport.addEventListener("resize", fitPageToViewport);
      window.visualViewport.addEventListener("scroll", fitPageToViewport);
    }
    if (isTabletRemote()) fitPageToViewport();
    enterPageFullscreen();
    [80, 240, 800].forEach(function (ms) {
      setTimeout(enterPageFullscreen, ms);
    });
  }

  function installTabletHome() {
    if (!document.querySelector('link[rel="manifest"]')) {
      const link = document.createElement("link");
      link.rel = "manifest";
      link.href = "/manifest.webmanifest";
      document.head.appendChild(link);
    }
    if (location.protocol === "https:" && "serviceWorker" in navigator) {
      navigator.serviceWorker.register("/sw.js").catch(function () {});
    }
    if (isHeaconPadApp() || !isTabletRemote() || isPageFullscreen()) return;
    if (sessionStorage.getItem("heaconHomeHide") === "1") return;
    let deferred = null;
    const box = document.createElement("div");
    box.id = "heaconHomeInstall";
    box.innerHTML =
      '<div>把 HEACON 装到平板桌面。以后从图标打开就是全屏，没有网址栏。</div>' +
      '<button type="button" id="heaconHomeBtn">安装到桌面</button>' +
      '<button type="button" class="heacon-home-x" id="heaconHomeX" aria-label="关闭">×</button>';
    document.body.appendChild(box);
    const show = function () {
      if (!isPageFullscreen()) box.classList.add("is-on");
    };
    window.addEventListener("beforeinstallprompt", function (event) {
      event.preventDefault();
      deferred = event;
      show();
    });
    document.getElementById("heaconHomeBtn").onclick = function () {
      if (deferred) {
        deferred.prompt();
        deferred.userChoice.finally(function () {
          box.classList.remove("is-on");
        });
        return;
      }
      showToast("请点浏览器右上角 ⋮，再选「添加到主屏幕」");
    };
    document.getElementById("heaconHomeX").onclick = function () {
      sessionStorage.setItem("heaconHomeHide", "1");
      box.classList.remove("is-on");
    };
    show();
    setTimeout(function () {
      if (isPageFullscreen()) box.classList.remove("is-on");
    }, 400);
  }

  function ensureToast() {
    if (!document.getElementById("heaconToast")) {
      const el = document.createElement("div");
      el.id = "heaconToast";
      document.body.appendChild(el);
    }
    window.showToast = function (text) {
      if (typeof original.showToast === "function") original.showToast(text);
      const el = document.getElementById("heaconToast");
      if (!el || !text) return;
      el.textContent = text;
      el.classList.add("show");
      clearTimeout(el._hide);
      el._hide = setTimeout(function () {
        el.classList.remove("show");
      }, 2600);
    };
  }

  function applyArmState(state) {
    if (!state || typeof state !== "object") return;
    if (state.session == null && state.phase == null && state.connected == null) return;
    const wasBusy = armBusy();
    const wasLive = !!armLink.live;
    const phase = state.phase || "offline";
    const live = !!(state.live === true || (state.session && state.connected));
    armLink = {
      connected: !!state.connected,
      session: !!state.session,
      live: live,
      phase: phase,
      ready: !!(state.ready === true || state.fsm === 33 || state.fsm === 25 || state.fsm === 27),
      moving: !!(state.fsm === 25 || state.fsm === 27),
      fsm: state.fsm != null ? state.fsm : null,
    };
    if (armLink.session || armLink.live) {
      sawSession = true;
      sawLiveThisPage = true;
    }
    if (live && wasBusy) {
      hideLinkModal();
      showToast("设备已连接");
    }
    if (live && !wasLive) goScanPoseIfNeeded();
  }

  function armLive() {
    return !!armLink.live;
  }

  function armSession() {
    return (
      armLive() ||
      !!armLink.session ||
      armLink.phase === "recovering" ||
      armLink.phase === "signal"
    );
  }

  function armBusy() {
    return armLink.phase === "connecting" || armLink.phase === "unlocking" || armLink.phase === "retrying";
  }

  function deviceChip() {
    return document.getElementById("heaconDeviceChip");
  }

  function bindDeviceChip(el) {
    if (!el || el.dataset.heaconBound === "1") return;
    el.dataset.heaconBound = "1";
    el.addEventListener("click", function (ev) {
      if (ev) ev.stopPropagation();
      showSessionModal();
    });
  }

  function paintDeviceChip() {
    const el = deviceChip();
    if (!el) return;
    let icon = el.querySelector("iconify-icon");
    if (!icon) {
      icon = document.createElement("iconify-icon");
      icon.setAttribute("width", "16");
      el.insertBefore(icon, el.firstChild);
    }
    let lab = document.getElementById("heaconDeviceLabel");
    if (!lab || lab.parentElement !== el) {
      Array.from(el.childNodes).forEach(function (node) {
        if (node.nodeType === 3) node.remove();
      });
      lab = document.createElement("span");
      lab.id = "heaconDeviceLabel";
      el.appendChild(lab);
    }
    el.classList.remove("is-off", "is-lock", "is-wait", "is-ok");
    let label = "设备已离线";
    let kind = "is-off";
    let mark = "solar:shield-cross-linear";
    if (armLink.phase === "unlocking") {
      label = "正在解锁…";
      kind = "is-wait";
      mark = "solar:shield-up-linear";
    } else if (armLink.phase === "recovering") {
      label = "连接不稳，正在恢复…";
      kind = "is-wait";
      mark = "solar:refresh-linear";
    } else if (armLink.phase === "signal") {
      label = "信号弱，正在重试…";
      kind = "is-wait";
      mark = "solar:refresh-linear";
    } else if (armBusy()) {
      label = "正在连接…";
      kind = "is-wait";
      mark = "solar:refresh-linear";
    } else if (armLive()) {
      label = "设备已连接";
      kind = "is-ok";
      mark = "solar:shield-check-linear";
    }
    el.classList.add(kind);
    icon.setAttribute("icon", mark);
    lab.textContent = label;
    el.title = label;
    bindDeviceChip(el);
    paintLinkHint();
  }

  function ensureLinkHint() {
    if (document.getElementById("heaconLinkHint")) return;
    const header = document.querySelector(".header-glass");
    if (!header || !header.parentElement) return;
    const el = document.createElement("div");
    el.id = "heaconLinkHint";
    header.insertAdjacentElement("afterend", el);
  }

  function paintLinkHint() {
    ensureLinkHint();
    const el = document.getElementById("heaconLinkHint");
    if (!el) return;
    const dots = '<i></i><i></i><i></i>';
    if (armLink.phase === "unlocking") {
      el.innerHTML = dots + "<span>正在解锁…</span>";
      el.className = "is-on is-wait";
      el.dataset.wasLive = "";
      return;
    }
    if (armLink.phase === "recovering") {
      el.innerHTML = dots + "<span>连接不稳，正在恢复…</span>";
      el.className = "is-on is-retry";
      return;
    }
    if (armLink.phase === "signal") {
      el.innerHTML = dots + "<span>信号弱，正在重试…</span>";
      el.className = "is-on is-retry";
      return;
    }
    if (armLink.phase === "retrying") {
      el.innerHTML = dots + "<span>还没连上，正在重试…</span>";
      el.className = "is-on is-retry";
      el.dataset.wasLive = "";
      return;
    }
    if (armLink.phase === "connecting") {
      el.innerHTML = dots + "<span>正在连接机械臂…</span>";
      el.className = "is-on is-wait";
      el.dataset.wasLive = "";
      return;
    }
    if (armLive()) {
      if (el.dataset.wasLive !== "1") {
        el.innerHTML = "<span>已连接</span>";
        el.className = "is-on is-ok";
        clearTimeout(hintHide);
        hintHide = setTimeout(function () {
          el.className = "";
        }, 1500);
      }
      el.dataset.wasLive = "1";
      return;
    }
    el.dataset.wasLive = "";
    el.className = "";
  }

  function hideLinkModal() {
    const modal = document.getElementById("heaconLinkModal");
    if (modal) {
      modal.classList.add("hidden");
      modal.classList.remove("flex");
    }
    const mask = document.getElementById("heaconUnlockMask");
    if (mask) mask.remove();
  }

  function paintLinkModal() {
    const title = document.getElementById("heaconLinkTitle");
    const body = document.getElementById("heaconLinkBody");
    const skip = document.getElementById("heaconLinkSkip");
    const go = document.getElementById("heaconLinkGo");
    if (!title || !body || !skip || !go) return;
    go.disabled = false;
    if (disconnecting) {
      title.textContent = "正在断开…";
      body.textContent = "断开后会锁定机械臂。";
      skip.textContent = "关闭";
      go.textContent = "正在断开…";
      return;
    }
    if (armBusy() && !armSession()) {
      title.textContent = armLink.phase === "unlocking" ? "正在解锁…" : "正在连接机械臂…";
      body.textContent = "还在连接，可再点一次。";
      skip.textContent = "关闭";
      go.textContent = "继续连接";
      return;
    }
    if (armSession()) {
      title.textContent = "断开连接";
      body.textContent = "断开后会锁定机械臂。页面仍可模拟操作，但不会写入真实记录。";
      skip.textContent = "暂不断开";
      go.textContent = "断开连接";
    } else {
      title.textContent = "连接机械臂";
      body.textContent = "连接后机械臂才能真正动作，并写入操作记录。";
      skip.textContent = "暂不连接";
      go.textContent = "连接机械臂";
    }
  }

  function showSessionModal() {
    const leftover = document.getElementById("heaconUnlockMask");
    if (leftover) leftover.remove();
    paintLinkModal();
    const modal = document.getElementById("heaconLinkModal");
    if (modal) {
      if (typeof closeOverlays === "function") closeOverlays();
      modal.classList.remove("hidden");
      modal.classList.add("flex");
      return;
    }
    if (typeof window.showHeaconLink === "function") window.showHeaconLink();
  }

  window.paintHeaconLink = paintLinkModal;
  window.goHeaconLink = async function () {
    const now = Date.now();
    // 一次点击会被页面内联 onclick、捕获阶段监听和本文件冒泡监听重复触发。
    // 这里统一防抖 + 互斥，保证一次操作只发一个请求。
    if (now - lastLinkActAt < 2500) return;
    lastLinkActAt = now;
    if (linkActing || disconnecting) {
      paintLinkModal();
      return;
    }
    linkActing = true;
    try {
      // 以服务端真实状态为准，避免 UI 状态过期导致“点一下先断开再连上”。
      let truth = null;
      try {
        truth = await get("/api/state");
      } catch (err) {
        truth = null;
      }
      if (truth) applyArmState(truth);
      const reallyOn = !!(truth && (truth.session === true || truth.live === true));
      if (reallyOn) {
        linkActing = false;
        await startDisconnect();
        return;
      }
      linkActing = false;
      await startConnect();
    } finally {
      linkActing = false;
    }
  };

  function bindLinkModal() {
    const go = document.getElementById("heaconLinkGo");
    if (!go || go.dataset.heaconBound === "1") return;
    go.dataset.heaconBound = "1";
    go.addEventListener("click", function (ev) {
      ev.preventDefault();
      ev.stopPropagation();
      window.goHeaconLink();
    });
  }

  function ensureUnlockModal() {
    const leftover = document.getElementById("heaconUnlockMask");
    if (leftover) leftover.remove();
    bindLinkModal();
    paintLinkModal();
  }

  async function startConnect() {
    if (armLive()) {
      hideLinkModal();
      showToast("设备已连接");
      return;
    }
    if (linkActing) return;
    linkActing = true;
    armLink.phase = "connecting";
    paintDeviceChip();
    paintLinkModal();
    showToast("正在连接机械臂…");
    try {
      const data = await post("/api/session/connect", {});
      applyArmState(data);
      paintDeviceChip();
      paintLinkModal();
      if (!armLive()) {
        const waited = await waitArmReady(6000);
        if (waited && waited.ok) {
          applyArmState(await get("/api/state"));
          paintDeviceChip();
          paintLinkModal();
        }
      }
      if (armLive()) {
        hideLinkModal();
        showToast("设备已连接");
        goScanPoseIfNeeded();
        return;
      }
      showToast((data && data.message) || "正在连接机械臂…");
    } catch (err) {
      armLink.phase = "offline";
      paintDeviceChip();
      paintLinkModal();
      showToast(err.message || "没有开始连接");
    } finally {
      linkActing = false;
    }
  }

  async function startDisconnect() {
    if (disconnecting) {
      showToast("正在断开…");
      return;
    }
    disconnecting = true;
    unlocking = true;
    linkActing = true;
    paintLinkModal();
    showToast("正在断开连接…");
    try {
      const data = await post("/api/session/disconnect", {});
      applyArmState(data && data.ok === false ? await get("/api/state") : data);
      paintDeviceChip();
      paintLinkModal();
      if (data && data.ok === false) {
        showToast(data.message || "没有断开");
        return;
      }
      hideLinkModal();
      showToast("已断开连接");
    } catch (err) {
      showToast(err.message || "没有断开");
    } finally {
      disconnecting = false;
      unlocking = false;
      linkActing = false;
      paintDeviceChip();
      paintLinkModal();
    }
  }

  function installArmGate() {
    ensureUnlockModal();
    ensureLinkHint();
    paintDeviceChip();
    syncArmGate();
  }

  async function syncArmGate() {
    if (armGateBusy) return;
    armGateBusy = true;
    try {
      applyArmState(await get("/api/state"));
      stateMisses = 0;
      lockRequested = false;
    } catch (err) {
      stateMisses += 1;
      if (armSession() && stateMisses >= STATE_MISS_WEAK) {
        armLink.phase = "signal";
      }
      // 网络抖动只提示，不自动断开。
      // 之前这里会自动 post session/disconnect，导致平板 WiFi 稍差就把机械臂锁掉，
      // 表现就是操作员看到的“反复解锁上锁”。断开必须由操作员明确点击触发。
      if (stateMisses >= STATE_MISS_LOCK && armSession()) {
        armLink.phase = "signal";
      }
    } finally {
      paintDeviceChip();
      paintLinkModal();
      armGateBusy = false;
    }
  }

  function wrapPage() {
    const pageStepTo = window.stepTo;
    if (typeof pageStepTo === "function") {
      window.stepTo = function (id) {
        const ok = pageStepTo.apply(this, arguments);
        if (id === "position" && ok !== false) {
          heaconPoints = null;
          clearRouteOverlay();
          showLive("position");
          goScanPoseIfNeeded();
        }
        return ok;
      };
    }
    window.confirmPlan = function () {
      heaconPoints = null;
      clearRouteOverlay();
      original.confirmPlan();
      showLive("position");
      goScanPoseIfNeeded();
    };
    window.startScan = function () {
      if (typeof canEnter === "function" && !canEnter("scan")) {
        showToast(lockedMessage("scan"));
        return;
      }
      if (!armLive()) {
        mockScan();
        return;
      }
      realScan();
    };
    window.cancelScan = function () {
      scanBusy = false;
      scanSeq += 1;
      original.cancelScan();
      showLive("position");
    };
    window.drawPoints = function () {
      if (heaconPoints) drawCameraPoints();
      else if (pageCurrent() === "position") clearRouteOverlay();
      else original.drawPoints();
    };
    window.svgPoint = function (event) {
      if (!heaconPoints) return original.svgPoint(event);
      return cameraPoint(event);
    };
    window.movePoint = function (el, x, y) {
      if (!heaconPoints) return original.movePoint(el, x, y);
      moveCameraPoint(el, x, y);
    };
    window.endDrag = function () {
      if (!heaconPoints) return original.endDrag();
      finishCameraDrag();
    };
    window.confirmRoute = function () {
      if (typeof flowState !== "undefined") flowState.routeConfirmed = true;
      if (!armLive()) {
        startMockService();
        return;
      }
      if (original.confirmRoute) original.confirmRoute();
    };
    window.startServiceFromModal = function () {
      primeVoice();
      startLinkedService();
    };
    window.nudgeSpeed = function (delta) {
      original.nudgeSpeed(delta);
      if (armLive()) post("/api/service/speed", { speed: pageSpeed() }).catch(function () {});
    };
    window.setLevelGrade = function (grade) {
      original.setLevelGrade(grade);
      if (!armLive()) return;
      const bind = pageBind();
      const next = bind && bind.serviceParams ? bind.serviceParams.levelGrade : Number(grade);
      post("/api/service/level", { grade: next }).catch(function () {});
    };
    window.closeAdjust = function (save) {
      original.closeAdjust(save);
      if (save && armLive()) {
        const bind = pageBind();
        const next = bind && bind.serviceParams ? bind.serviceParams.levelGrade : 2;
        post("/api/service/level", { grade: next }).catch(function () {});
      }
    };
    window.pauseService = function () {
      stopVoiceListen();
      original.pauseService();
      if (!armLive()) return;
      post("/api/service/stop", {}).then(function () {
        post("/api/position", {});
      });
    };
    window.resumeService = function () {
      if (!armLive()) {
        if (original.resumeService) original.resumeService();
        return;
      }
      startLinkedService(false);
    };
    window.finishService = function (endedEarly) {
      stopVoiceListen();
      if (armLive()) {
        post("/api/service/stop", {}).then(function () {
          post("/api/position", {});
        });
      }
      original.finishService(endedEarly);
    };
    window.archiveService = function () {
      if (!armLive()) {
        showToast("离线模拟，未写入真实记录");
        if (original.resetService) original.resetService();
        return;
      }
      post("/api/position", {}).catch(function () {});
      if (original.archiveService) original.archiveService();
    };
    window.logEvent = function (name, payload) {
      if (!armLive() && (name === "service_archive" || name === "service_start")) return;
      if (original.logEvent) original.logEvent(name, payload);
    };
    window.resetService = function () {
      heaconPoints = null;
      linkedRun = false;
      robotStatus = null;
      positionReady = false;
      stopVoiceListen();
      stopLive();
      clearRouteOverlay();
      original.resetService();
    };
    window.startRun = function () {
      primeVoice();
      if (!armLive()) {
        linkedRun = false;
        robotStatus = null;
        original.startRun();
        return;
      }
      linkedRun = true;
      original.startRun();
      if (window.liveTimer) {
        clearInterval(window.liveTimer);
        window.liveTimer = setInterval(function () {
          if (typeof updateTimer === "function") updateTimer();
          if (typeof updateLiveReadings === "function") updateLiveReadings();
          if (window.paintRunView) window.paintRunView();
          if (window.renderRunRoute) window.renderRunRoute();
        }, 1000);
      }
    };
    window.currentShow = function () {
      if (!linkedRun || !robotStatus) return original.currentShow();
      const bind = pageBind();
      const pts = (bind && bind.pointSets && bind.pointSets[bind.selected]) || [];
      let idx = typeof robotStatus.index === "number" ? robotStatus.index : -1;
      if (idx < 0) {
        idx = pts.findIndex(function (item) {
          return item[0] === robotStatus.point;
        });
      }
      if (idx < 0) idx = 0;
      const working = robotStatus.working === true || robotStatus.phase === "手法";
      const dir = robotStatus.direction < 0 ? -1 : 1;
      const name = (pts[idx] && pts[idx][0]) || robotStatus.point || "—";
      const tech = working
        ? robotStatus.gesture || robotStatus.technique || (typeof pointTechnique === "function" ? pointTechnique(name, dir) : "")
        : typeof travelTechnique === "function"
          ? travelTechnique()
          : "走线";
      return {
        step: {
          type: working ? "hold" : "travel",
          index: idx,
          name: name,
          technique: tech,
          dir: dir,
          seconds: working ? 25 : 0,
        },
        remain: working && robotStatus.pointRemain != null ? robotStatus.pointRemain : 0,
        round: robotStatus.round || 1,
        done: new Set(robotStatus.visited || []),
        working: working,
      };
    };
    window.paintRunView = function () {
      if (!linkedRun || !robotStatus) return original.paintRunView();
      const now = window.currentShow();
      const step = now.step;
      const bind = pageBind();
      const pts = (bind && bind.pointSets && bind.pointSets[bind.selected]) || [];
      const currentPoint = document.getElementById("currentPoint");
      const currentRoute = document.getElementById("currentRoute");
      const currentStepRemain = document.getElementById("currentStepRemain");
      if (currentPoint) currentPoint.textContent = step.name || "—";
      if (currentRoute) {
        currentRoute.textContent =
          "路线 " +
          String((step.index || 0) + 1).padStart(2, "0") +
          " / " +
          pts.length +
          " · " +
          step.technique +
          (step.type === "travel" ? " · 前往中" : "");
      }
      if (currentStepRemain) {
        currentStepRemain.textContent =
          step.type === "hold"
            ? "这一轮手法还剩 " + formatDuration(now.remain)
            : "前往穴位中";
      }
    };
    window.renderRunRoute = function () {
      if (!linkedRun || !robotStatus) return original.renderRunRoute();
      const now = window.currentShow();
      const idx = now.step.index;
      const working = now.working === true;
      const bind = pageBind();
      const pts = (bind && bind.pointSets && bind.pointSets[bind.selected]) || [];
      const rows = pts.map(function (item, i) {
        const active = working && i === idx;
        const done = now.done.has(i);
        const tech =
          typeof pointTechnique === "function" ? pointTechnique(item[0], now.step.dir || 1) : "";
        const status = active ? "进行中" : done ? "本轮已完成" : "待执行";
        const timeText = active ? tech + " · 还剩 " + formatDuration(now.remain) : tech;
        const numClass = active
          ? "bg-[var(--brand)] text-white"
          : done
            ? "bg-[#ffe8d4] text-[var(--brand)]"
            : "bg-[#fff5ec] text-[var(--muted)]";
        return (
          '<div class="run-route-item"><div class="run-route-marker"><span class="run-route-num ' +
          numClass +
          '">' +
          String(i + 1).padStart(2, "0") +
          '</span><span class="run-route-line" aria-hidden="true"></span></div><div class="flex-1 min-w-0 self-center"><div class="flex items-center justify-between gap-2"><span class="text-sm font-medium text-[var(--ink)]">' +
          item[0] +
          '</span><span class="text-xs ' +
          (active ? "text-[var(--brand)] font-medium" : "text-[var(--muted)]") +
          '">' +
          status +
          '</span></div><div class="text-xs text-[var(--muted)] mt-0.5">' +
          timeText +
          "</div></div></div>"
        );
      });
      const list = document.getElementById("runRouteList");
      if (list) list.innerHTML = rows.join("");
    };
    const origShow = window.show;
    window.show = function (id) {
      origShow(id);
      if (id === "position") showLive("position");
      if (id === "confirm") showLive("confirm");
      if (id === "run" || id === "start") stopLive();
      if (id === "run") {
        if (listenOnThisDevice()) startVoiceListen();
        else showTabletVoiceHint();
      } else stopVoiceListen();
    };
  }

  let scanBusy = false;
  let scanSeq = 0;

  function completeScan(fromCamera, offline) {
    clearInterval(scanTimer);
    $("#scanBar").style.width = "100%";
    $("#scanPercent").textContent = "100%";
    $("#scanStatus").textContent = offline
      ? "已识别穴位（离线模拟，不写记录）"
      : "已识别穴位，正在叠到实时画面";
    flowState.scanComplete = true;
    const seq = scanSeq;
    setTimeout(function () {
      if (seq !== scanSeq) return;
      scanBusy = false;
      stepTo("confirm");
      showLive("confirm");
      if (fromCamera && heaconPoints) drawCameraPoints();
      else if (original.drawPoints) original.drawPoints();
    }, 350);
  }

  function clearRouteOverlay() {
    const layer = document.getElementById("pointLayer");
    if (layer) layer.innerHTML = "";
    const line = document.getElementById("routeLine");
    if (line) line.setAttribute("points", "");
  }

  async function realScan() {
    heaconPoints = null;
    clearRouteOverlay();
    const seq = ++scanSeq;
    scanBusy = true;
    show("scan");
    let n = 8;
    clearInterval(scanTimer);
    clearTimeout(scanTransitionTimer);
    $("#scanBar").style.width = n + "%";
    $("#scanPercent").textContent = n + "%";
    $("#scanStatus").textContent = "正在识别背部轮廓和穴位";
    scanTimer = setInterval(function () {
      n = Math.min(88, n + 3);
      $("#scanBar").style.width = n + "%";
      $("#scanPercent").textContent = n + "%";
    }, 400);
    try {
      for (let i = 0; i < 8 && scanSeq === seq; i += 1) {
        const bind = pageBind();
        const plan = (bind && bind.selected) || (typeof selected !== "undefined" ? selected : "p1");
        const data = await post("/api/service/preview", { plan: plan });
        if (scanSeq !== seq) return;
        if (data && data.ok && data.points && data.points.length) {
          heaconPoints = data.points.map(function (pt) {
            return { name: pt.name, u: pt.u, v: pt.v };
          });
          heaconFrame = { width: data.width || 1280, height: data.height || 720 };
          completeScan(true, false);
          return;
        }
        await sleep(1200);
      }
      if (scanSeq !== seq) return;
      heaconPoints = null;
      completeScan(false, false);
    } catch (err) {
      if (scanSeq !== seq) return;
      heaconPoints = null;
      completeScan(false, false);
    } finally {
      if (scanSeq === seq) clearInterval(scanTimer);
    }
  }

  function goScanPoseIfNeeded() {
    if (pageCurrent() && pageCurrent() !== "position") return;
    ensurePosition().catch(function (err) {
      showToast(err.message || "就位失败");
    });
  }

  async function ensurePosition() {
    if (positionWait) return positionWait;
    positionWait = ensurePositionBody().finally(function () {
      positionWait = null;
    });
    return positionWait;
  }

  async function ensurePositionBody() {
    const ready = await waitArmReady(15000);
    if (!ready.ok) {
      throw new Error(ready.message || "没有连上机械臂");
    }
    showToast("机械臂正在回到就位位置");
    const now = await get("/api/position");
    if (!(now && now.positioning)) {
      const started = await post("/api/position", {});
      if (started && started.ok === false) {
        throw new Error(started.message || "就位失败");
      }
    }
    const deadline = Date.now() + 160000;
    while (Date.now() < deadline) {
      const status = await get("/api/position");
      if (status && status.ready) {
        positionReady = true;
        showToast(status.message || "机械臂已就位");
        return status;
      }
      if (status && status.positioning === false && !status.ready && status.message) {
        if (status.message !== "机械臂还没有就位") {
          throw new Error(status.message);
        }
      }
      await sleep(800);
    }
    throw new Error("就位超时，请查看现场");
  }

  async function waitArmReady(ms) {
    const deadline = Date.now() + (ms || 15000);
    let last = "正在连接机械臂";
    while (Date.now() < deadline) {
      try {
        const state = await get("/api/state");
        if (
          state &&
          (state.live === true ||
            (state.connected &&
              (state.ready === true || state.fsm === 33 || state.fsm === 25 || state.fsm === 27)))
        ) {
          return { ok: true };
        }
        if (state && state.connected) {
          last = state.message || "机械臂正在上电待机";
        } else {
          last = (state && state.message) || "正在连接机械臂";
        }
        if (typeof showToast === "function") showToast(last);
      } catch (err) {
        last = (err && err.message) || "正在连接机械臂";
      }
      await sleep(700);
    }
    return { ok: false, message: last };
  }

  async function mockScan() {
    heaconPoints = null;
    clearRouteOverlay();
    const seq = ++scanSeq;
    scanBusy = true;
    show("scan");
    let n = 12;
    clearInterval(scanTimer);
    $("#scanBar").style.width = n + "%";
    $("#scanPercent").textContent = n + "%";
    $("#scanStatus").textContent = "正在识别背部轮廓和穴位";
    scanTimer = setInterval(function () {
      n = Math.min(88, n + 8);
      $("#scanBar").style.width = n + "%";
      $("#scanPercent").textContent = n + "%";
    }, 280);
    try {
      const bind = pageBind();
      const plan = (bind && bind.selected) || (typeof selected !== "undefined" ? selected : "p1");
      for (let i = 0; i < 8 && scanSeq === seq; i += 1) {
        const data = await post("/api/service/preview", { plan: plan });
        if (scanSeq !== seq) return;
        if (data && data.ok && data.points && data.points.length) {
          heaconPoints = data.points.map(function (pt) {
            return { name: pt.name, u: pt.u, v: pt.v };
          });
          heaconFrame = { width: data.width || 1280, height: data.height || 720 };
          completeScan(true, true);
          return;
        }
        await sleep(800);
      }
      if (scanSeq !== seq) return;
      heaconPoints = null;
      completeScan(false, true);
    } catch (err) {
      if (scanSeq !== seq) return;
      heaconPoints = null;
      completeScan(false, true);
    } finally {
      if (scanSeq === seq) clearInterval(scanTimer);
    }
  }

  function startMockService() {
    primeVoice();
    if (typeof flowState !== "undefined") {
      flowState.routeConfirmed = true;
      flowState.checksComplete = true;
      flowState.serviceStarted = true;
    }
    linkedRun = false;
    robotStatus = null;
    if (typeof closeStartModal === "function") closeStartModal();
    showToast("离线模拟，机械臂不动作，也不会写入真实记录");
    if (typeof stepTo === "function") stepTo("run");
    else if (original.startRun) original.startRun();
  }

  async function startLinkedService(fresh) {
    if (!armLive()) {
      startMockService();
      return;
    }
    primeVoice();
    try {
      const ready = await waitArmReady(15000);
      if (!ready.ok) {
        showToast(ready.message || "没有连上机械臂");
        return;
      }
      const bind = pageBind();
      const payload = {
        plan: bind ? bind.selected : "p1",
        points: heaconPoints,
        width: heaconFrame.width,
        height: heaconFrame.height,
        fresh: fresh !== false,
        level: bind && bind.serviceParams ? bind.serviceParams.levelGrade : 1,
        duration: Number(bind && bind.serviceParams ? bind.serviceParams.duration : 35) * 60,
        speed: pageSpeed(),
      };
      const data = await post("/api/service/start", payload);
      if (!data || !data.ok) {
        showToast((data && data.message) || "机械臂没有开始");
        return;
      }
      flowState.checksComplete = true;
      flowState.serviceStarted = true;
      linkedRun = true;
      closeStartModal();
      showToast(data.message || "机械臂开始按路线工作");
      stepTo("run");
    } catch (err) {
      showToast(err.message || "机械臂没有开始");
    }
  }

  function showLive(which) {
    const stage =
      which === "confirm"
        ? document.querySelector(".body-stage--confirm")
        : document.querySelector(".body-stage--position");
    if (!stage) return;
    stage.classList.add("is-live");
    const wrap = which === "confirm" ? document.querySelector(".body-map") : stage;
    if (!wrap) return;
    if (which === "confirm") wrap.classList.add("is-live");
    let frame = wrap.querySelector(".heacon-live-frame");
    if (!frame) {
      frame = document.createElement("div");
      frame.className = "heacon-live-frame is-wait";
      wrap.insertBefore(frame, wrap.firstChild);
    }
    let spin = frame.querySelector(".heacon-live-spin");
    if (!spin) {
      spin = document.createElement("div");
      spin.className = "heacon-live-spin";
      frame.appendChild(spin);
    }
    let img = frame.querySelector(".heacon-live-cam") || wrap.querySelector(".heacon-live-cam");
    if (!img) {
      img = document.createElement("img");
      img.className = "heacon-live-cam";
      img.alt = "机械臂摄像头实时画面";
    }
    if (img.parentElement !== spin) spin.insertBefore(img, spin.firstChild);
    wrap.querySelectorAll(".body-stage-figure").forEach(function (el) {
      el.classList.add("heacon-hidden");
    });
    if (!frame.querySelector(".heacon-live-tag")) {
      const tag = document.createElement("div");
      tag.className = "heacon-live-tag";
      tag.textContent = "实时画面";
      frame.appendChild(tag);
    }
    if (!frame.querySelector(".heacon-live-note")) {
      const note = document.createElement("div");
      note.className = "heacon-live-note";
      note.textContent = "正在连接摄像头…";
      frame.appendChild(note);
    }
    const svg = document.getElementById("bodyView");
    // 画面 img 已经放进被 rotate(90deg) 的 .heacon-live-spin，
    // 叠加层必须一起进去，否则穴位留在未旋转的坐标系里，整体错位。
    // 原来只在 confirm 页搬，人体定位页不搬，所以那页的穴位必然偏。
    if (svg) {
      svg.classList.add("is-live");
      if (svg.parentElement !== spin) spin.appendChild(svg);
    }
    if (which === "position") {
      heaconPoints = null;
      clearRouteOverlay();
    }
    document.querySelectorAll(".heacon-live-frame").forEach(function (el) {
      el.classList.toggle("is-aim", which === "position" && el === frame);
    });
    sizeLiveFrame(frame);
    watchLiveFrame(wrap);
    tickLive();
    if (!liveTimer) liveTimer = setInterval(tickLive, CAM_MS);
    if (!frame._sized) {
      frame._sized = true;
      window.addEventListener("resize", function () {
        document.querySelectorAll(".heacon-live-frame").forEach(sizeLiveFrame);
      });
    }
  }

  function markLiveStatus(kind, text) {
    document.querySelectorAll(".heacon-live-frame").forEach(function (frame) {
      frame.classList.toggle("is-wait", kind === "wait");
      frame.classList.toggle("is-stale", kind === "stale");
      const tag = frame.querySelector(".heacon-live-tag");
      if (tag) tag.textContent = text;
      const note = frame.querySelector(".heacon-live-note");
      if (note) note.textContent = kind === "live" ? "" : text;
    });
  }

  function watchLiveFrame(wrap) {
    if (!window.ResizeObserver) return;
    if (liveWatch) liveWatch.disconnect();
    liveWatch = new ResizeObserver(function () {
      document.querySelectorAll(".heacon-live-frame").forEach(sizeLiveFrame);
    });
    liveWatch.observe(wrap);
    const stage = wrap.closest(".body-stage") || wrap;
    if (stage !== wrap) liveWatch.observe(stage);
  }

  function sizeLiveFrame(frame) {
    const parent = frame && frame.parentElement;
    if (!parent) return;
    let roomW = parent.clientWidth;
    let roomH = parent.clientHeight;
    if (roomW < 8 || roomH < 8) {
      const box = (parent.closest(".body-stage") || parent).getBoundingClientRect();
      roomW = box.width;
      roomH = box.height;
    }
    if (roomW < 8 || roomH < 8) {
      frame.style.width = "100%";
      frame.style.height = "100%";
      requestAnimationFrame(function () {
        if (frame.isConnected) sizeLiveFrame(frame);
      });
      return;
    }
    const srcW = heaconFrame.width || 1280;
    const srcH = heaconFrame.height || 720;
    const rotated = srcH / srcW;
    let height = roomH;
    let width = height * rotated;
    if (width > roomW) {
      width = roomW;
      height = width / rotated;
    }
    frame.style.width = Math.round(width) + "px";
    frame.style.height = Math.round(height) + "px";
    const boxW = frame.clientWidth || width;
    const boxH = frame.clientHeight || height;
    const spin = frame.querySelector(".heacon-live-spin");
    if (spin) {
      spin.style.width = boxH + "px";
      spin.style.height = boxW + "px";
      spin.style.left = (boxW - boxH) / 2 + "px";
      spin.style.top = (boxH - boxW) / 2 + "px";
    }
  }

  function stopLive() {
    clearInterval(liveTimer);
    liveTimer = 0;
    camBusy = false;
  }

  function liveScreen() {
    const pos = document.querySelector('.screen[data-screen="position"]');
    const conf = document.querySelector('.screen[data-screen="confirm"]');
    if (pos && !pos.classList.contains("hidden")) return "position";
    if (conf && !conf.classList.contains("hidden")) return "confirm";
    return "";
  }

  function keepLiveView() {
    const which = liveScreen();
    if (which) showLive(which);
  }

  function tickLive() {
    if (!liveScreen() || camBusy) return;
    const imgs = document.querySelectorAll(".heacon-live-cam");
    if (!imgs.length) return;
    camBusy = true;
    const ctrl = new AbortController();
    const timer = setTimeout(function () {
      ctrl.abort();
    }, 1600);
    fetch("/api/camera.jpg?t=" + Date.now(), { cache: "no-store", signal: ctrl.signal })
      .then(function (resp) {
        const type = (resp.headers.get("content-type") || "").toLowerCase();
        const state = (resp.headers.get("X-Heacon-Camera") || "").toLowerCase();
        if (!resp.ok || type.indexOf("image") < 0) {
          markLiveStatus("wait", "正在连接摄像头…");
          return null;
        }
        return resp.blob().then(function (blob) {
          return { blob: blob, state: state };
        });
      })
      .then(function (payload) {
        if (!payload || !payload.blob || payload.blob.size < 400) {
          if (payload) markLiveStatus("wait", "正在连接摄像头…");
          return;
        }
        const url = URL.createObjectURL(payload.blob);
        imgs.forEach(function (img) {
          img.onload = function () {
            if (img.naturalWidth > 8 && img.naturalHeight > 8) {
              heaconFrame = { width: img.naturalWidth, height: img.naturalHeight };
              document.querySelectorAll(".heacon-live-frame").forEach(sizeLiveFrame);
            }
          };
          img.src = url;
        });
        if (lastCamUrl && lastCamUrl !== url) URL.revokeObjectURL(lastCamUrl);
        lastCamUrl = url;
        if (payload.state === "stale") markLiveStatus("stale", "画面已暂停，正在重试");
        else if (payload.state === "missing") markLiveStatus("wait", "正在连接摄像头…");
        else markLiveStatus("live", "实时画面");
      })
      .catch(function () {
        markLiveStatus("wait", "正在连接摄像头…");
      })
      .then(function () {
        clearTimeout(timer);
        camBusy = false;
      });
  }

  function pointMark() {
    const scale = Math.min((heaconFrame.width || 1280) / 1280, (heaconFrame.height || 720) / 720);
    return {
      glow: Math.max(10, 28 * scale),
      dot: Math.max(6, 18 * scale),
      font: Math.max(11, 18 * scale),
      stroke: Math.max(2, 4 * scale),
      dx: 22 * scale,
      dy: -8 * scale,
    };
  }

  function drawCameraPoints() {
    const layer = $("#pointLayer");
    const svg = $("#bodyView");
    if (!layer || !svg || !heaconPoints) return;
    const fw = heaconFrame.width || 1280;
    const fh = heaconFrame.height || 720;
    svg.setAttribute("viewBox", "0 0 " + fw + " " + fh);
    // 图片是 object-fit:contain，叠加层必须用同样的缩放规则才能像素级对齐。
    // 早先留空（默认为 meet），在旋转容器里会被二次缩放，穴位整体滑出去。
    svg.setAttribute("preserveAspectRatio", "xMidYMid meet");
    svg.classList.add("is-live");
    const mark = pointMark();
    layer.innerHTML = heaconPoints
      .map(function (pt, i) {
        const x = pt.u;
        const y = pt.v;
        const tx = x + mark.dx;
        const ty = y + mark.dy;
        return (
          '<g class="acupoint" data-index="' +
          i +
          '" data-x="' +
          x +
          '" data-y="' +
          y +
          '" tabindex="0" role="button" aria-label="拖动' +
          pt.name +
          '"><circle class="point-glow" cx="' +
          x +
          '" cy="' +
          y +
          '" r="' +
          mark.glow +
          '" fill="#ff6a1a" opacity=".18"/><circle class="point-dot" cx="' +
          x +
          '" cy="' +
          y +
          '" r="' +
          mark.dot +
          '" fill="#ff6a1a" stroke="#fff" stroke-width="' +
          mark.stroke +
          '"/><text x="' +
          tx +
          '" y="' +
          ty +
          '" fill="#fff7ef" font-size="' +
          mark.font +
          '" font-weight="700" stroke="#8a3f1c" stroke-width="' +
          mark.stroke +
          '" paint-order="stroke" transform="rotate(-90 ' +
          tx +
          " " +
          ty +
          ')">' +
          pt.name +
          "</text></g>"
        );
      })
      .join("");
    updateRoute();
    layer.querySelectorAll(".acupoint").forEach(function (el) {
      el.addEventListener("pointerdown", startDrag);
      el.addEventListener("pointermove", dragPoint);
      el.addEventListener("pointerup", endDrag);
      el.addEventListener("pointercancel", endDrag);
    });
    const hint = $("#pointFeedback");
    if (hint) hint.textContent = "拖动穴位即可调整落点，开始后机械臂按新位置走";
  }

  function cameraPoint(event) {
    const svg = $("#bodyView");
    if (svg && svg.createSVGPoint && svg.getScreenCTM()) {
      const pt = svg.createSVGPoint();
      pt.x = event.clientX;
      pt.y = event.clientY;
      const loc = pt.matrixTransform(svg.getScreenCTM().inverse());
      return { x: loc.x, y: loc.y };
    }
    const rect = svg.getBoundingClientRect();
    const scaleX = heaconFrame.width / Math.max(rect.width, 1);
    const scaleY = heaconFrame.height / Math.max(rect.height, 1);
    return { x: (event.clientX - rect.left) * scaleX, y: (event.clientY - rect.top) * scaleY };
  }

  function moveCameraPoint(el, x, y) {
    const nx = Math.max(8, Math.min(heaconFrame.width - 8, x));
    const ny = Math.max(8, Math.min(heaconFrame.height - 8, y));
    el.dataset.x = nx;
    el.dataset.y = ny;
    el.querySelectorAll("circle").forEach(function (c) {
      c.setAttribute("cx", nx);
      c.setAttribute("cy", ny);
    });
    el.querySelectorAll("text").forEach(function (t) {
      t.setAttribute("x", nx + 28);
      t.setAttribute("y", ny - 10);
      t.setAttribute("transform", "rotate(-90 " + (nx + 28) + " " + (ny - 10) + ")");
    });
    updateRoute();
  }

  function finishCameraDrag() {
    if (!activeDrag || !heaconPoints) return;
    const el = activeDrag;
    const index = Number(el.dataset.index);
    const name = heaconPoints[index].name;
    heaconPoints[index] = {
      name: name,
      u: Math.round(Number(el.dataset.x) * 10) / 10,
      v: Math.round(Number(el.dataset.y) * 10) / 10,
    };
    logEvent("acupoint_adjust", { point: name, u: heaconPoints[index].u, v: heaconPoints[index].v, accepted: true });
    $("#pointFeedback").textContent = "已保留新位置，开始后机械臂按这个点走";
    el.dataset.dragging = "false";
    activeDrag = null;
    updateRoute();
  }

  function startPoll() {
    if (pollTimer) return;
    pollTimer = setInterval(function () {
      syncService();
      keepLiveView();
      syncArmGate();
    }, POLL_MS);
    keepLiveView();
    syncArmGate();
  }

  function pageBind() {
    return typeof window.heaconBind === "function" ? window.heaconBind() : null;
  }

  function pageSpeed() {
    const bind = pageBind();
    if (bind && bind.runSpeed >= 1) return bind.runSpeed;
    const label = document.getElementById("speedLabel");
    if (label) {
      const n = parseInt(label.textContent, 10);
      if (n >= 1 && n <= 5) return n;
    }
    return 1;
  }

  function pageCurrent() {
    const bind = pageBind();
    if (bind && bind.current) return bind.current;
    return typeof current === "string" ? current : "";
  }

  async function syncService() {
    if (!armLive()) return;
    const screen = pageCurrent();
    if (!["run", "pause"].includes(screen) || !linkedRun) return;
    try {
      const data = await get("/api/service/status");
      if (!data) return;
      robotStatus = data;
      const bind = pageBind();
      if (data.speed && bind) {
        bind.runSpeed = data.speed;
        if (typeof syncSpeedButtons === "function") syncSpeedButtons();
      }
      if (typeof data.remainSeconds === "number" && bind) {
        bind.timerVal = data.remainSeconds;
        if (typeof updateTimer === "function") updateTimer();
      }
      earOpen();
      if (window.paintRunView) window.paintRunView();
      if (window.renderRunRoute) window.renderRunRoute();
      const started = bind && bind.flowState && bind.flowState.serviceStarted;
      if (screen === "run" && data.running === false && started) {
        linkedRun = false;
        stopVoiceListen();
        if (data.message && typeof showToast === "function") showToast(data.message);
        post("/api/position", {}).catch(function () {});
        if (original.finishService) original.finishService(false);
      }
    } catch (err) {}
  }

  function netMessage(err) {
    const raw = String((err && err.message) || err || "");
    if (/failed to fetch|networkerror|load failed|network request failed/i.test(raw)) {
      return "电脑上的服务没开。请先在电脑打开 HEACON，并确认平板和电脑在同一网络";
    }
    return raw || "没有连上电脑";
  }

  async function fetchWithTimeout(path, options) {
    const ctrl = new AbortController();
    const timer = setTimeout(function () {
      ctrl.abort();
    }, NET_TIMEOUT_MS);
    try {
      const resp = await fetch(path, Object.assign({ signal: ctrl.signal }, options || {}));
      return resp;
    } finally {
      clearTimeout(timer);
    }
  }

  async function get(path) {
    try {
      const resp = await fetchWithTimeout(path, { cache: "no-store" });
      return readJson(resp);
    } catch (err) {
      throw new Error(netMessage(err));
    }
  }

  async function post(path, body) {
    try {
      const resp = await fetchWithTimeout(path, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body || {}),
      });
      return readJson(resp);
    } catch (err) {
      throw new Error(netMessage(err));
    }
  }

  async function readJson(resp) {
    const text = await resp.text();
    try {
      return JSON.parse(text);
    } catch (err) {
      return { ok: false, message: text || "接口没有返回数据" };
    }
  }

  function sleep(ms) {
    return new Promise(function (resolve) {
      setTimeout(resolve, ms);
    });
  }

  let voiceRec = null;
  let voiceWanted = false;
  let earUntil = 0;
  let lastVoiceAt = 0;
  let micStream = null;
  let primingVoice = false;
  let heardWake = false;
  let voicePlayer = null;
  let voiceOutCtx = null;
  let clipBuffers = {};
  let hearQuietUntil = 0;
  let voiceUnlocked = false;
  let hearCtx = null;
  let hearProc = null;
  let hearSource = null;
  let hearSending = false;
  let hearChunks = [];
  let hearTalking = false;
  let hearSpeechMs = 0;
  let hearSilentMs = 0;

  function isLocalHost() {
    const host = location.hostname;
    return host === "127.0.0.1" || host === "localhost" || host === "::1";
  }

  function shouldUseHttpsDoor() {
    return false;
  }

  function useLocalHear() {
    return location.protocol === "https:" && !isLocalHost();
  }

  function listenOnThisDevice() {
    if (useLocalHear()) return !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia);
    return isLocalHost() && !!(window.SpeechRecognition || window.webkitSpeechRecognition);
  }

  function showTabletVoiceHint() {
    voiceWanted = false;
    ensureEar();
    paintEar("remote");
    if (typeof showToast === "function") {
      showToast("请对着电脑说小希小希，平板只负责点");
    }
  }

  async function resumeRunningUi() {
    try {
      const data = await get("/api/service/status");
      if (!data || !data.running) return;
      robotStatus = data;
      linkedRun = true;
      const bind = pageBind();
      if (bind && bind.flowState) {
        bind.flowState.serviceStarted = true;
        bind.flowState.unlockedStage = 3;
        bind.flowState.routeConfirmed = true;
        bind.flowState.scanComplete = true;
        bind.flowState.planConfirmed = true;
      }
      if (typeof show === "function") show("run");
      if (window.paintRunView) window.paintRunView();
      if (window.renderRunRoute) window.renderRunRoute();
      primeVoice();
    } catch (err) {}
  }

  function primeVoice() {
    if (!listenOnThisDevice()) {
      showTabletVoiceHint();
      return;
    }
    voiceWanted = true;
    ensureEar();
    paintEar("idle");
    unlockVoiceOut();
    if (primingVoice) return;
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      startVoiceListen();
      return;
    }
    primingVoice = true;
    navigator.mediaDevices
      .getUserMedia({ audio: true })
      .then(function (stream) {
        primingVoice = false;
        if (micStream) {
          stream.getTracks().forEach(function (track) {
            track.stop();
          });
        } else {
          micStream = stream;
        }
        startVoiceListen();
      })
      .catch(function () {
        primingVoice = false;
        paintEar("need");
        if (typeof showToast === "function") {
          showToast(useLocalHear() ? "请允许平板的麦克风，才能说小希小希" : "请允许这台电脑的麦克风，才能说小希小希调力度");
        }
      });
  }

  function startVoiceListen() {
    if (!listenOnThisDevice()) {
      showTabletVoiceHint();
      return;
    }
    if (useLocalHear()) {
      startLocalHear();
      return;
    }
    voiceWanted = true;
    ensureEar();
    paintEar(earOpen() ? "hot" : "idle");
    const Rec = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!Rec) {
      paintEar("need");
      if (typeof showToast === "function") {
        showToast("请用 Chrome 打开控制台，才能听语音");
      }
      return;
    }
    if (voiceRec) return;
    const rec = new Rec();
    rec.lang = "zh-CN";
    rec.continuous = true;
    rec.interimResults = true;
    rec.maxAlternatives = 3;
    rec.onresult = function (event) {
      const heard = pickHeard(event);
      if (!heard.text) return;
      if (heard.text) paintEar(heardWake || isWake(heard.text) ? "hot" : "idle", heard.text);
      if (isWake(heard.text) && !heardWake) {
        openEar();
        playClip("ei");
        paintEar("hot", heard.text);
        if (voiceIntent(heard.text)) {
          window.setTimeout(function () {
            applyVoiceIntent(voiceIntent(heard.text), heard.text);
          }, 480);
        }
        return;
      }
      if (heardWake && (heard.final || voiceIntent(heard.text))) {
        handleVoiceCommand(heard.text);
      }
    };
    rec.onend = function () {
      voiceRec = null;
      if (voiceWanted && pageCurrent() === "run") {
        setTimeout(startVoiceListen, 200);
      }
    };
    rec.onerror = function (event) {
      const err = event && event.error;
      if (err === "not-allowed" || err === "service-not-allowed") {
        paintEar("need");
        if (typeof showToast === "function") {
          showToast("麦克风被拒绝了，请在地址栏允许麦克风后刷新");
        }
      }
    };
    voiceRec = rec;
    try {
      rec.start();
    } catch (err) {
      voiceRec = null;
    }
  }

  function startLocalHear() {
    voiceWanted = true;
    ensureEar();
    paintEar(earOpen() ? "hot" : "idle");
    if (hearProc || primingVoice) return;
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      paintEar("need");
      return;
    }
    primingVoice = true;
    navigator.mediaDevices
      .getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true } })
      .then(function (stream) {
        primingVoice = false;
        micStream = stream;
        const Ctx = window.AudioContext || window.webkitAudioContext;
        hearCtx = voiceOutCtx || hearCtx || new Ctx();
        voiceOutCtx = hearCtx;
        if (hearCtx.state === "suspended") hearCtx.resume();
        hearSource = hearCtx.createMediaStreamSource(stream);
        hearProc = (hearCtx.createScriptProcessor || hearCtx.createJavaScriptNode).call(hearCtx, 4096, 1, 1);
        hearProc.onaudioprocess = onHearAudio;
        const mute = hearCtx.createGain();
        mute.gain.value = 0;
        hearSource.connect(hearProc);
        hearProc.connect(mute);
        mute.connect(hearCtx.destination);
        preloadVoiceClips();
        paintEar("idle");
      })
      .catch(function () {
        primingVoice = false;
        paintEar("need");
        if (typeof showToast === "function") showToast("请允许平板的麦克风，才能说小希小希");
      });
  }

  function onHearAudio(event) {
    if (!voiceWanted || pageCurrent() !== "run") return;
    if (Date.now() < hearQuietUntil) return;
    const input = event.inputBuffer.getChannelData(0);
    const rate = event.inputBuffer.sampleRate;
    let sum = 0;
    for (let i = 0; i < input.length; i += 1) sum += input[i] * input[i];
    const rms = Math.sqrt(sum / input.length);
    const frameMs = (1000 * input.length) / rate;
    if (rms > 0.018) {
      hearTalking = true;
      hearSilentMs = 0;
      hearSpeechMs += frameMs;
      hearChunks.push(new Float32Array(input));
      if (hearSpeechMs > 4500) flushHear(rate);
    } else if (hearTalking) {
      hearSilentMs += frameMs;
      hearChunks.push(new Float32Array(input));
      if (hearSilentMs > 550) flushHear(rate);
    }
  }

  function flushHear(rate) {
    const chunks = hearChunks;
    hearChunks = [];
    hearTalking = false;
    hearSpeechMs = 0;
    hearSilentMs = 0;
    if (hearSending || !chunks.length) return;
    const pcm = downsampleTo16k(chunks, rate);
    if (pcm.length < 16000 * 0.35) return;
    hearSending = true;
    fetch("/api/voice/hear", {
      method: "POST",
      headers: { "Content-Type": "audio/wav" },
      body: encodeWav(pcm, 16000),
    })
      .then(function (resp) {
        return resp.json();
      })
      .then(function (data) {
        hearSending = false;
        const text = data && data.text ? String(data.text).replace(/\s+/g, "") : "";
        if (!text) return;
        paintEar(heardWake || isWake(text) ? "hot" : "idle", text);
        if (isWake(text) && !heardWake) {
          openEar();
          playClip("ei");
          if (voiceIntent(text)) {
            window.setTimeout(function () {
              applyVoiceIntent(voiceIntent(text), text);
            }, 480);
          }
          return;
        }
        if (heardWake) handleVoiceCommand(text);
      })
      .catch(function () {
        hearSending = false;
      });
  }

  function downsampleTo16k(chunks, rate) {
    let total = 0;
    for (let i = 0; i < chunks.length; i += 1) total += chunks[i].length;
    const merged = new Float32Array(total);
    let offset = 0;
    for (let i = 0; i < chunks.length; i += 1) {
      merged.set(chunks[i], offset);
      offset += chunks[i].length;
    }
    if (rate === 16000) return floatTo16(merged);
    const ratio = rate / 16000;
    const outLen = Math.max(1, Math.floor(merged.length / ratio));
    const out = new Float32Array(outLen);
    for (let i = 0; i < outLen; i += 1) {
      const start = Math.floor(i * ratio);
      const end = Math.min(merged.length, Math.floor((i + 1) * ratio));
      let sum = 0;
      let n = 0;
      for (let j = start; j < end; j += 1) {
        sum += merged[j];
        n += 1;
      }
      out[i] = n ? sum / n : 0;
    }
    return floatTo16(out);
  }

  function floatTo16(samples) {
    const pcm = new Int16Array(samples.length);
    for (let i = 0; i < samples.length; i += 1) {
      const x = Math.max(-1, Math.min(1, samples[i]));
      pcm[i] = x < 0 ? x * 0x8000 : x * 0x7fff;
    }
    return pcm;
  }

  function encodeWav(pcm, rate) {
    const bytes = pcm.byteLength;
    const buffer = new ArrayBuffer(44 + bytes);
    const view = new DataView(buffer);
    writeAscii(view, 0, "RIFF");
    view.setUint32(4, 36 + bytes, true);
    writeAscii(view, 8, "WAVE");
    writeAscii(view, 12, "fmt ");
    view.setUint32(16, 16, true);
    view.setUint16(20, 1, true);
    view.setUint16(22, 1, true);
    view.setUint32(24, rate, true);
    view.setUint32(28, rate * 2, true);
    view.setUint16(32, 2, true);
    view.setUint16(34, 16, true);
    writeAscii(view, 36, "data");
    view.setUint32(40, bytes, true);
    new Uint8Array(buffer, 44).set(new Uint8Array(pcm.buffer));
    return buffer;
  }

  function writeAscii(view, offset, text) {
    for (let i = 0; i < text.length; i += 1) view.setUint8(offset + i, text.charCodeAt(i));
  }

  function stopVoiceListen() {
    voiceWanted = false;
    earUntil = 0;
    heardWake = false;
    hideEar();
    hearChunks = [];
    hearTalking = false;
    hearSpeechMs = 0;
    hearSilentMs = 0;
    if (hearProc) {
      try {
        hearProc.disconnect();
      } catch (err) {}
      hearProc = null;
    }
    if (hearSource) {
      try {
        hearSource.disconnect();
      } catch (err) {}
      hearSource = null;
    }
    if (hearCtx) {
      hearCtx = null;
    }
    if (micStream) {
      micStream.getTracks().forEach(function (track) {
        track.stop();
      });
      micStream = null;
    }
    if (!voiceRec) return;
    const rec = voiceRec;
    voiceRec = null;
    try {
      rec.onend = null;
      rec.stop();
    } catch (err) {}
  }

  function pickHeard(event) {
    const texts = [];
    let anyFinal = false;
    for (let i = event.resultIndex; i < event.results.length; i += 1) {
      if (event.results[i].isFinal) anyFinal = true;
      for (let a = 0; a < event.results[i].length; a += 1) {
        const piece = String(event.results[i][a].transcript || "").replace(/\s+/g, "");
        if (piece && texts.indexOf(piece) < 0) texts.push(piece);
      }
    }
    const picked =
      texts.find(function (item) {
        return voiceIntent(item);
      }) ||
      texts.find(function (item) {
        return isWake(item);
      }) ||
      texts[0] ||
      "";
    return { text: picked, final: anyFinal };
  }

  function isWake(text) {
    const t = text || "";
    return /小希小希|小西小西|晓希晓希|小喜小喜|小溪小溪|小心小心|消息消息|小希|小西|晓希|小喜|小溪/.test(t);
  }

  function normalizeHeard(text) {
    return String(text || "")
      .replace(/\s+/g, "")
      .replace(/[，。！？、,.!?;；:：'"""''~～]/g, "")
      .replace(/小希小希|小西小西|晓希晓希|小喜小喜|小溪小溪|小心小心|消息消息/g, "")
      .replace(/小希|小西|晓希|小喜|小溪|小心|消息/g, "")
      .replace(/问度|温渡|稳度|温度度/g, "温度")
      .replace(/里度|利度|力道|力的/g, "力度")
      .replace(/题高|提搞|体高|提后/g, "提高")
      .replace(/将低|将底/g, "降低");
  }

  function voiceIntent(text) {
    const t = normalizeHeard(text);
    if (!t) return 0;
    if (/不要再?(加大|增加|调高|提高|升高)|别(再)?(加|热|重|烫)/.test(t)) return -1;
    if (/不要再?(减小|减少|降低|调低)|别(再)?(减|轻|冷)/.test(t)) return 1;
    const upHits = [
      /提高/,
      /调高/,
      /升高/,
      /加大/,
      /增加/,
      /加点/,
      /加一/,
      /加档/,
      /加力/,
      /加热/,
      /升温/,
      /高一点/,
      /大一点/,
      /热一点/,
      /重一点/,
      /强一点/,
      /深一点/,
      /高一些/,
      /大一些/,
      /热一些/,
      /重一些/,
      /高点/,
      /大点/,
      /热点/,
      /重点/,
      /强点/,
      /用力/,
      /再来[点一]/,
      /再[高大加热重强]/,
      /不够[热力高大重强]/,
      /太[轻小低冷弱浅]/,
      /往上/,
      /上调/,
      /力度大/,
      /温度高/,
      /变[高大热重强]/,
    ];
    const downHits = [
      /降低/,
      /调低/,
      /减小/,
      /减少/,
      /减点/,
      /减一/,
      /减档/,
      /减力/,
      /降温/,
      /低一点/,
      /小一点/,
      /轻一点/,
      /弱一点/,
      /缓一点/,
      /凉一点/,
      /冷一点/,
      /低一些/,
      /小一些/,
      /轻一些/,
      /弱一些/,
      /凉一些/,
      /冷一些/,
      /低点/,
      /轻点/,
      /弱点/,
      /凉点/,
      /冷点/,
      /少点/,
      /小点/,
      /太[重烫热高低大强猛]/,
      /太用力/,
      /受不了/,
      /往下/,
      /下调/,
      /力度小/,
      /温度低/,
      /变[小低冷凉轻弱]/,
      /柔和/,
      /轻柔/,
    ];
    let up = 0;
    let down = 0;
    upHits.forEach(function (re) {
      if (re.test(t)) up += 1;
    });
    downHits.forEach(function (re) {
      if (re.test(t)) down += 1;
    });
    if (/温|力|档|灸|按/.test(t)) {
      if (/[加增高升热烫重猛强深大]/.test(t)) up += 1;
      if (/[减降少低冷凉轻弱浅]/.test(t)) down += 1;
    }
    if (!up && !down) {
      if (/[加增高升提]/.test(t) || /热|烫|重|用力/.test(t)) up += 1;
      if (/[减降少]/.test(t) || /冷|凉|轻|柔/.test(t)) down += 1;
    }
    if (up === down) return 0;
    return up > down ? 1 : -1;
  }

  function openEar() {
    heardWake = true;
    earUntil = Date.now() + 10000;
    paintEar("hot");
  }

  function earOpen() {
    if (!earUntil || Date.now() > earUntil) {
      heardWake = false;
      if (voiceWanted && pageCurrent() === "run") paintEar("idle");
      else hideEar();
      return false;
    }
    return true;
  }

  function handleVoiceCommand(text) {
    if (!heardWake || !earOpen()) return;
    const intent = voiceIntent(text);
    if (!intent) {
      paintEar("hot", text);
      if (typeof showToast === "function") showToast("没听清，想加大还是减小都可以说");
      return;
    }
    applyVoiceIntent(intent, text);
  }

  function applyVoiceIntent(intent, text) {
    if (!intent) return;
    if (Date.now() - lastVoiceAt < 1200) return;
    lastVoiceAt = Date.now();
    heardWake = false;
    earUntil = 0;
    paintEar("hot", text);
    voiceStep(intent);
  }

  function voiceStep(delta) {
    post("/api/service/level", { delta: delta })
      .then(function (data) {
        const spoken = (data && data.spoken) || "已完成";
        if (data && typeof data.level === "number" && original.setLevelGrade) {
          original.setLevelGrade(data.level);
        }
        const clip =
          spoken.indexOf("最高") >= 0 ? "high" : spoken.indexOf("最低") >= 0 ? "low" : "done";
        playClip(clip);
        if (typeof showToast === "function") showToast(spoken);
        paintEar("idle");
      })
      .catch(function () {
        playClip("done");
        paintEar("idle");
      });
  }

  function clipPhrase(name) {
    if (name === "ei") return "诶";
    if (name === "high") return "已经是最高了";
    if (name === "low") return "已经是最低了";
    return "已完成";
  }

  function voiceCtx() {
    const Ctx = window.AudioContext || window.webkitAudioContext;
    if (!Ctx) return null;
    if (hearCtx) return hearCtx;
    if (!voiceOutCtx) voiceOutCtx = new Ctx();
    return voiceOutCtx;
  }

  function unlockVoiceOut() {
    const ctx = voiceCtx();
    if (ctx && ctx.state === "suspended") ctx.resume();
    preloadVoiceClips();
    if (voiceUnlocked || !ctx) return;
    voiceUnlocked = true;
    try {
      const blank = ctx.createBuffer(1, 1, ctx.sampleRate || 16000);
      const src = ctx.createBufferSource();
      src.buffer = blank;
      src.connect(ctx.destination);
      src.start(0);
    } catch (err) {}
  }

  function decodeClip(ctx, raw) {
    return new Promise(function (resolve, reject) {
      const copy = raw.slice(0);
      let settled = false;
      const ok = function (buf) {
        if (settled) return;
        settled = true;
        resolve(buf);
      };
      const fail = function (err) {
        if (settled) return;
        settled = true;
        reject(err || new Error("decode"));
      };
      try {
        const maybe = ctx.decodeAudioData(copy, ok, fail);
        if (maybe && typeof maybe.then === "function") maybe.then(ok, fail);
      } catch (err) {
        fail(err);
      }
    });
  }

  function preloadVoiceClips() {
    const ctx = voiceCtx();
    if (!ctx) return;
    ["ei", "done", "high", "low"].forEach(function (name) {
      if (clipBuffers[name]) return;
      fetch("/voice/" + name + ".mp3", { cache: "force-cache" })
        .then(function (resp) {
          if (!resp.ok) throw new Error("clip");
          return resp.arrayBuffer();
        })
        .then(function (raw) {
          return decodeClip(ctx, raw);
        })
        .then(function (decoded) {
          clipBuffers[name] = decoded;
        })
        .catch(function () {});
    });
  }

  function hushMic(on) {
    if (!micStream) return;
    micStream.getAudioTracks().forEach(function (track) {
      track.enabled = !on;
    });
  }

  function speakFallback(text) {
    try {
      if (typeof speak === "function") {
        speak(text);
        return;
      }
      if (!window.speechSynthesis) return;
      const utter = new SpeechSynthesisUtterance(text);
      utter.lang = "zh-CN";
      utter.rate = 0.92;
      window.speechSynthesis.cancel();
      window.speechSynthesis.speak(utter);
    } catch (err) {}
  }

  function playBuffer(buf, onDone) {
    const ctx = hearCtx || voiceOutCtx;
    if (!ctx || !buf) return false;
    if (ctx.state === "suspended") ctx.resume();
    const src = ctx.createBufferSource();
    src.buffer = buf;
    const gain = ctx.createGain();
    gain.gain.value = 2;
    src.connect(gain);
    gain.connect(ctx.destination);
    hearQuietUntil = Date.now() + Math.round((buf.duration + 0.3) * 1000);
    src.onended = onDone;
    src.start(0);
    return true;
  }

  function playClip(name) {
    unlockVoiceOut();
    const phrase = clipPhrase(name);
    hearQuietUntil = Date.now() + 1600;
    if (useLocalHear()) hushMic(true);

    function donePlay() {
      hearQuietUntil = Date.now() + 280;
      if (useLocalHear()) hushMic(false);
    }

    if (playBuffer(clipBuffers[name], donePlay)) return;

    const ctx = hearCtx || voiceOutCtx;
    if (ctx) {
      fetch("/voice/" + name + ".mp3")
        .then(function (resp) {
          if (!resp.ok) throw new Error("clip");
          return resp.arrayBuffer();
        })
        .then(function (raw) {
          return decodeClip(ctx, raw);
        })
        .then(function (decoded) {
          clipBuffers[name] = decoded;
          if (!playBuffer(decoded, donePlay)) {
            speakFallback(phrase);
            donePlay();
          }
        })
        .catch(function () {
          playFile(name, phrase, donePlay);
        });
      return;
    }
    playFile(name, phrase, donePlay);
  }

  function playFile(name, phrase, onDone) {
    try {
      if (voicePlayer) {
        voicePlayer.pause();
        voicePlayer = null;
      }
      const audio = new Audio("/voice/" + name + ".mp3");
      audio.setAttribute("playsinline", "true");
      audio.volume = 1;
      voicePlayer = audio;
      audio.onended = onDone;
      const playing = audio.play();
      if (playing && playing.catch) {
        playing.catch(function () {
          speakFallback(phrase);
          onDone();
        });
      }
    } catch (err) {
      speakFallback(phrase);
      onDone();
    }
  }

  function ensureEar() {
    if (document.getElementById("heaconEar")) return;
    const el = document.createElement("div");
    el.id = "heaconEar";
    document.body.appendChild(el);
  }

  function paintEar(mode, heard) {
    ensureEar();
    const el = document.getElementById("heaconEar");
    if (!el) return;
    el.classList.add("is-on");
    el.classList.toggle("is-hot", mode === "hot");
    if (mode === "need") {
      el.textContent = "未打开麦克风，请允许后再说小希小希";
    } else if (mode === "remote") {
      el.textContent = "平板请打开这台电脑的 8765 页面；语音可对着电脑说小希小希";
    } else if (mode === "hot") {
      el.textContent = heard ? "听到了：" + heard : "诶，说加大或减小就行";
    } else {
      el.textContent = "先说「小希小希」，等我应一声再讲要求";
    }
    const bar = document.getElementById("voiceAssistBar");
    const btn = document.getElementById("voiceAssistBtn");
    if (bar) bar.textContent = el.textContent;
    if (btn) {
      btn.textContent = mode === "need" ? "开启语音" : mode === "remote" ? "对着电脑说" : "语音已开";
    }
  }

  function hideEar() {
    const el = document.getElementById("heaconEar");
    if (!el) return;
    el.classList.remove("is-on", "is-hot");
  }

  document.addEventListener(
    "pointerdown",
    function (ev) {
      const hit = ev.target && ev.target.closest
        ? ev.target.closest("#heaconDeviceChip, #heaconLinkModal")
        : null;
      unlockVoiceOut();
      if (!hit) enterPageFullscreen();
    },
    true
  );
})();
