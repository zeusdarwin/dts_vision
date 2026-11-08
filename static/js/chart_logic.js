const DATA_POLL_INTERVAL_MS = 250;
const CHART_RENDER_INTERVAL_MS = 100;
const DEFAULT_MAX_POINTS = 100;

const chartMap = {
  yaw: { label: "Yaw", color: "#ff8500" },
  pitch: { label: "Pitch", color: "#00a9f4" },
  target_yaw: { label: "Target Yaw", color: "#ff3d6e" },
  target_pitch: { label: "Target Pitch", color: "#00bd78" },
  gimbal_yaw: { label: "Gimbal Yaw", color: "#8d4dff" },
  gimbal_pitch: { label: "Gimbal Pitch", color: "#00c8c8" },
  control_v_yaw: { label: "Control V Yaw", color: "#6acb00" },
  control_v_pitch: { label: "Control V Pitch", color: "#ff9d00" },
  control_a_yaw: { label: "Control A Yaw", color: "#ff572f" },
  control_a_pitch: { label: "Control A Pitch", color: "#d43cff" },
  fly_time: { label: "Fly Time", color: "#00a88f" },
  target_v_yaw: { label: "Target V Yaw", color: "#e52aa5" },
  target_v_roll: { label: "Target V Roll", color: "#9acb00" },
  rune_a_diff: { label: "Rune A Diff", color: "#e9a400" },
  rune_w_diff: { label: "Rune W Diff", color: "#00a85a" },
  yaw_diff: { label: "Yaw Diff", color: "#007fc4" },
  pitch_diff: { label: "Pitch Diff", color: "#f05b00" },
  enable_yaw_diff: { label: "Enable Yaw Diff", color: "#d68a00" },
  enable_pitch_diff: { label: "Enable Pitch Diff", color: "#3ca800" }
};

let mainChart = null;
let dataTimer = null;
let dataFetchController = null;
let latestDataSignature = "";
let latestChartData = null;
let renderQueued = false;
let lastRenderAt = 0;
let selectedKeys = [];
let individualCharts = {};
let individualRanges = {};

function chartOptions() {
  return {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    normalized: true,
    interaction: {
      mode: "nearest",
      axis: "x",
      intersect: false,
    },
    elements: {
      line: {
        borderWidth: 3,
        borderCapStyle: "round",
        borderJoinStyle: "round",
        tension: 0.08,
        spanGaps: true,
      },
      point: {
        radius: 1.2,
        hoverRadius: 5,
        hoverBorderWidth: 2,
      },
    },
    plugins: {
      decimation: {
        enabled: true,
        algorithm: "min-max",
      },
      legend: {
        labels: {
          color: "#294b5c",
          boxWidth: 7,
          boxHeight: 7,
          usePointStyle: true,
          pointStyle: "circle",
          font: { size: 12 },
        },
      },
      tooltip: {
        enabled: true,
        backgroundColor: "#163f55",
        titleColor: "#ffffff",
        bodyColor: "#ffffff",
        padding: 10,
        cornerRadius: 5,
        intersect: false,
        callbacks: {
          title: () => "",
          label: (context) => `${context.dataset.label}: ${Number(context.parsed.y).toFixed(3)}`,
        },
      },
    },
    scales: {
      x: {
        display: false,
        grid: { display: false },
      },
      y: {
        ticks: {
          color: "#355f72",
          font: { size: 11 },
          maxTicksLimit: 6,
        },
        border: { display: false },
        grid: { color: "#c6e1eb", drawTicks: false },
      },
    },
  };
}

function setStatus(id, ok) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.toggle("ok", ok);
  el.classList.toggle("error", !ok);
  const label = el.dataset.label;
  if (label) {
    const text = `${label} · ${ok ? "已连接" : "未连接"}`;
    if (el.textContent !== text) el.textContent = text;
  }
}

function updateChartEmptyState() {
  const empty = document.getElementById("chart-empty");
  if (!empty) return;
  const showMulti = document.getElementById("multiLineChart")?.checked;
  empty.hidden = Boolean(showMulti && selectedKeys.length && latestChartData?.time?.length);
  empty.textContent = !selectedKeys.length ? "未选择曲线" : !showMulti ? "总图已关闭" : "等待曲线数据";
}

function createSeriesSwatch(color) {
  const swatch = document.createElement("span");
  swatch.className = "series-swatch";
  swatch.style.setProperty("--series-color", color);
  swatch.setAttribute("aria-hidden", "true");
  return swatch;
}

function getMaxPoints() {
  const input = document.getElementById("mainMaxPts");
  const parsed = Number.parseInt(input?.value, 10);
  return Number.isFinite(parsed) ? Math.min(Math.max(parsed, 10), 1000) : DEFAULT_MAX_POINTS;
}

function getSelectedKeys() {
  return Array.from(document.querySelectorAll("#chartSelectControls input[type='checkbox']:checked"))
    .map((input) => input.dataset.key)
    .filter(Boolean);
}

function setAutoScale(chart, values, range) {
  if (range?.enabled) {
    chart.options.scales.y.min = range.min;
    chart.options.scales.y.max = range.max;
    return;
  }

  const clean = values.filter((value) => Number.isFinite(value));
  if (clean.length === 0) {
    chart.options.scales.y.min = undefined;
    chart.options.scales.y.max = undefined;
    return;
  }

  let min = clean[0];
  let max = clean[0];
  for (const value of clean) {
    if (value < min) min = value;
    if (value > max) max = value;
  }

  const padding = (max - min) * 0.1 || 1;
  chart.options.scales.y.min = min - padding;
  chart.options.scales.y.max = max + padding;
}

function createDataset(key) {
  const meta = chartMap[key] || { label: key, color: "#637e93" };
  return {
    label: meta.label,
    data: [],
    borderColor: meta.color,
    backgroundColor: meta.color,
    fill: false,
  };
}

function createChart(canvas, datasets) {
  return new Chart(canvas.getContext("2d"), {
    type: "line",
    data: {
      labels: [],
      datasets,
    },
    options: chartOptions(),
  });
}

function initCharts() {
  if (!window.Chart) {
    setStatus("data-status", false);
    document.getElementById("chart-empty").textContent = "图表加载失败";
    console.warn("Chart.js is not available");
    return;
  }

  const mainCanvas = document.getElementById("mainChart");
  for (const input of document.querySelectorAll("#chartSelectControls input")) {
    input.after(createSeriesSwatch(chartMap[input.dataset.key].color));
  }
  mainChart = createChart(mainCanvas, []);
  updateCharts();
}

function updateMainRange() {
  if (!mainChart) return;
  mainChart._maxPoints = getMaxPoints();
  latestDataSignature = "";
  fetchDataAndQueueRender();
}

function updateCharts() {
  if (!mainChart) return;

  selectedKeys = getSelectedKeys();
  document.getElementById("series-count").textContent = `${selectedKeys.length} 项`;
  updateChartEmptyState();
  const showMulti = document.getElementById("multiLineChart")?.checked ?? true;
  const container = document.getElementById("individualCharts");

  mainChart.data.datasets = showMulti ? selectedKeys.map(createDataset) : [];
  mainChart.update("none");

  for (const chart of Object.values(individualCharts)) {
    chart.destroy();
  }
  individualCharts = {};
  container.replaceChildren();

  if (selectedKeys.length === 0) {
    const empty = document.createElement("div");
    empty.className = "empty-state";
    empty.textContent = "未选择曲线";
    container.appendChild(empty);
  }

  for (const key of selectedKeys) {
    const meta = chartMap[key] || { label: key };
    const box = document.createElement("section");
    box.className = "chart-box";

    const title = document.createElement("h4");
    title.textContent = meta.label;
    title.prepend(createSeriesSwatch(meta.color));
    box.appendChild(title);

    const controls = document.createElement("div");
    controls.className = "range-controls";
    controls.innerHTML = `
      <label><input type="checkbox" class="childEnable" /> 固定范围</label>
      <label>min <input type="number" class="childMin" value="0" step="0.1" aria-label="${meta.label} 最小值" /></label>
      <label>max <input type="number" class="childMax" value="1" step="0.1" aria-label="${meta.label} 最大值" /></label>
      <button type="button" class="applyRange">应用</button>
    `;
    box.appendChild(controls);

    const savedRange = individualRanges[key];
    if (savedRange) {
      controls.querySelector(".childEnable").checked = savedRange.enabled;
      controls.querySelector(".childMin").value = savedRange.min ?? 0;
      controls.querySelector(".childMax").value = savedRange.max ?? 1;
    }

    const canvasWrap = document.createElement("div");
    canvasWrap.className = "chart-canvas-wrap";
    const canvas = document.createElement("canvas");
    canvas.setAttribute("role", "img");
    canvas.setAttribute("aria-label", `${meta.label} 曲线`);
    canvasWrap.appendChild(canvas);
    box.appendChild(canvasWrap);
    container.appendChild(box);

    const chart = createChart(canvas, [createDataset(key)]);
    individualCharts[key] = chart;

    controls.querySelector(".applyRange").addEventListener("click", () => {
      const enabled = controls.querySelector(".childEnable").checked;
      const min = Number.parseFloat(controls.querySelector(".childMin").value);
      const max = Number.parseFloat(controls.querySelector(".childMax").value);
      individualRanges[key] = {
        enabled,
        min: Number.isFinite(min) ? min : undefined,
        max: Number.isFinite(max) ? max : undefined,
      };
      latestDataSignature = "";
      fetchDataAndQueueRender();
    });
  }

  latestDataSignature = "";
  fetchDataAndQueueRender();
}

function dataSignature(json, keys) {
  const time = Array.isArray(json.time) ? json.time : [];
  const lastTime = time.length ? time[time.length - 1] : "";
  const lengths = keys.map((key) => `${key}:${Array.isArray(json[key]) ? json[key].length : 0}`).join("|");
  return `${time.length}:${lastTime}:${lengths}`;
}

function updateChartData(json) {
  const labels = Array.isArray(json.time) ? json.time : [];
  if (labels.length === 0) return;

  if (document.getElementById("multiLineChart")?.checked) {
    mainChart.data.labels = labels;
    const allValues = [];
    mainChart.data.datasets.forEach((dataset, index) => {
      const key = selectedKeys[index];
      const values = Array.isArray(json[key]) ? json[key] : [];
      dataset.data = values;
      allValues.push(...values);
    });
    setAutoScale(mainChart, allValues);
    mainChart.update("none");
  }

  for (const [key, chart] of Object.entries(individualCharts)) {
    const values = Array.isArray(json[key]) ? json[key] : [];
    chart.data.labels = labels;
    chart.data.datasets[0].data = values;
    setAutoScale(chart, values, individualRanges[key]);
    chart.update("none");
  }
}

function queueChartRender() {
  if (renderQueued) return;
  renderQueued = true;
  requestAnimationFrame(renderChartFrame);
}

function renderChartFrame(timestamp) {
  if (!latestChartData) {
    renderQueued = false;
    return;
  }

  const elapsed = timestamp - lastRenderAt;
  if (elapsed < CHART_RENDER_INTERVAL_MS) {
    setTimeout(() => requestAnimationFrame(renderChartFrame), CHART_RENDER_INTERVAL_MS - elapsed);
    return;
  }

  lastRenderAt = timestamp;
  renderQueued = false;
  updateChartData(latestChartData);
}

async function fetchDataAndQueueRender() {
  if (!mainChart) return;
  if (dataFetchController) return;

  const controller = new AbortController();
  dataFetchController = controller;
  const maxPoints = getMaxPoints();
  try {
    const response = await fetch(`/data?max_points=${maxPoints}`, {
      cache: "no-store",
      signal: controller.signal,
    });
    if (!response.ok) throw new Error(response.statusText);

    const json = await response.json();
    const time = Array.isArray(json.time) ? json.time : [];
    if (time.length === 0) {
      latestChartData = null;
      latestDataSignature = "";
      updateChartEmptyState();
      setStatus("data-status", true);
      return;
    }

    const signature = dataSignature(json, selectedKeys);
    if (signature === latestDataSignature) {
      setStatus("data-status", true);
      return;
    }
    latestDataSignature = signature;
    latestChartData = json;
    updateChartEmptyState();
    queueChartRender();

    setStatus("data-status", true);
  } catch (error) {
    if (error.name !== "AbortError") {
      setStatus("data-status", false);
      console.warn("data fetch failed:", error.message);
    }
  } finally {
    if (dataFetchController === controller) {
      dataFetchController = null;
    }
  }
}

function startDataLoop() {
  if (dataTimer) return;
  const tick = async () => {
    await fetchDataAndQueueRender();
    dataTimer = setTimeout(tick, DATA_POLL_INTERVAL_MS);
  };
  dataTimer = setTimeout(tick, 0);
}
