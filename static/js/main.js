const LOG_POLL_INTERVAL_MS = 1000;
const VIDEO_VIEWPORT_GAP_PX = 12;

function updateVideoPanelHeightLimit() {
  const panel = document.getElementById("video-panel");
  if (!panel || document.fullscreenElement) return;

  const panelTop = panel.getBoundingClientRect().top;
  const availableHeight = Math.max(160, window.innerHeight - panelTop - VIDEO_VIEWPORT_GAP_PX);
  document.documentElement.style.setProperty("--video-panel-max-height", `${availableHeight}px`);
}

function toggleFullscreen() {
  const container = document.querySelector(".video-container");
  if (!container) return;
  if (!container.requestFullscreen) return;

  if (!document.fullscreenElement) {
    container
      .requestFullscreen()
      .then(() => document.body.classList.add("fullscreen-mode"))
      .catch((error) => console.warn(`fullscreen failed: ${error.message}`));
    return;
  }

  document
    .exitFullscreen()
    .then(() => document.body.classList.remove("fullscreen-mode"))
    .catch((error) => console.warn(`exit fullscreen failed: ${error.message}`));
}

function bindControls() {
  const fullscreenButton = document.getElementById("fullscreenBtn");
  if (!document.querySelector(".video-container")?.requestFullscreen) {
    fullscreenButton.hidden = true;
  }
  fullscreenButton?.addEventListener("click", toggleFullscreen);
  document.getElementById("multiLineChart")?.addEventListener("change", updateCharts);
  document.getElementById("chartSelectControls")?.addEventListener("change", (event) => {
    if (event.target.matches("input[type='checkbox']")) updateCharts();
  });
  document.getElementById("applyMainRange")?.addEventListener("click", updateMainRange);

  document.addEventListener("fullscreenchange", () => {
    const active = Boolean(document.fullscreenElement);
    document.body.classList.toggle("fullscreen-mode", active);
    fullscreenButton.title = active ? "退出全屏" : "全屏";
    fullscreenButton.setAttribute("aria-label", fullscreenButton.title);
    fullscreenButton.setAttribute("aria-pressed", String(active));
    const icon = fullscreenButton.querySelector("use");
    const path = icon.getAttribute("href").split("#")[0];
    icon.setAttribute("href", `${path}#${active ? "minimize" : "maximize"}`);
    if (!document.fullscreenElement) updateVideoPanelHeightLimit();
  });

  window.addEventListener("resize", updateVideoPanelHeightLimit);
  window.addEventListener("orientationchange", updateVideoPanelHeightLimit);
}

function updateVideoState() {
  const videos = ["video-stream", "omni0-stream", "omni1-stream"]
    .map((id) => document.getElementById(id))
    .filter(Boolean);
  const ready = videos.some((video) => video.naturalWidth > 0);
  const status = document.getElementById("video-status");
  videos.forEach((video) => {
    video.parentElement.classList.toggle("has-video", video.naturalWidth > 0);
  });
  status.classList.toggle("ok", ready);
  const text = ready ? "画面已接入" : "等待画面";
  if (status.textContent !== text) status.textContent = text;
}

document.addEventListener("DOMContentLoaded", () => {
  bindControls();
  updateVideoPanelHeightLimit();
  initCharts();
  // An MJPEG response stays open, so load may not fire after its first frame.
  updateVideoState();
  ["video-stream", "omni0-stream", "omni1-stream"].forEach((id) => {
    document.getElementById(id)?.addEventListener("error", updateVideoState);
  });
  setInterval(updateVideoState, 500);
  startDataLoop();
  fetchAndDisplayJsonWithTree("json-log", "/log");
  setInterval(() => fetchAndDisplayJsonWithTree("json-log", "/log"), LOG_POLL_INTERVAL_MS);
});
