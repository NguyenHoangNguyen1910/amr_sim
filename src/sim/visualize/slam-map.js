"use strict";

(() => {
  const get = (id) => document.getElementById(id);
  const panel = get("mapping-tools");
  const drawer = get("map-tools");
  const canvas = get("slam-map");
  const context = canvas.getContext("2d");
  const tile = document.createElement("canvas");
  const feedback = get("slam-feedback");
  let revision = -1;
  let session = "";
  let grid = null;
  let overlay = null;
  let sensorLayers = {};
  let active = false;
  let generation = 0;
  let driving = null;
  let commandPending = false;

  async function api(path, method = "GET", body) {
    const response = await fetch(path, {
      method,
      cache: "no-store",
      ...(body === undefined ? {} : {
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.detail || result.error || `HTTP ${response.status}`);
    return result;
  }

  function paintGrid() {
    if (!grid) return;
    const { width, height, cells } = grid;
    tile.width = width;
    tile.height = height;
    const tileContext = tile.getContext("2d");
    const pixels = tileContext.createImageData(width, height);
    for (let row = 0; row < height; row += 1) {
      const source = row * width;
      const target = (height - 1 - row) * width * 4;
      for (let column = 0; column < width; column += 1) {
        const value = cells[source + column];
        const offset = target + column * 4;
        const shade = Math.round(240 - 2.14 * value);
        pixels.data[offset] = value < 0 ? 69 : shade;
        pixels.data[offset + 1] = value < 0 ? 83 : shade;
        pixels.data[offset + 2] = value < 0 ? 100 : shade;
        pixels.data[offset + 3] = 255;
      }
    }
    tileContext.putImageData(pixels, 0, 0);
  }

  function draw() {
    context.fillStyle = "#455364";
    context.fillRect(0, 0, canvas.width, canvas.height);
    if (!grid) return;
    const {width, height} = grid;
    const scale = Math.min((canvas.width - 24) / width, (canvas.height - 24) / height);
    const left = (canvas.width - width * scale) / 2;
    const top = (canvas.height - height * scale) / 2;
    const px = (x) => left + (x - grid.x) * scale / grid.resolution;
    const py = (y) => top + height * scale - (y - grid.y) * scale / grid.resolution;
    context.imageSmoothingEnabled = false;
    context.drawImage(tile, left, top, width * scale, height * scale);

    mapSensorLayers.draw(context, sensorLayers, px, py, true);
    const pose = overlay?.pose;
    if (pose) {
      context.save();
      context.translate(px(pose.x), py(pose.y));
      context.rotate(-pose.yaw);
      context.beginPath();
      context.moveTo(11, 0);
      context.lineTo(-7, 6);
      context.lineTo(-4, 0);
      context.lineTo(-7, -6);
      context.closePath();
      context.fillStyle = "#ffd569";
      context.strokeStyle = "#303544";
      context.lineWidth = 1.25;
      context.fill();
      context.stroke();
      context.restore();
    }
  }

  async function poll(run) {
    if (run !== generation || panel.hidden || drawer.hidden) return;
    try {
      const query = mapSensorLayers.query();
      const result = await api(`/api/slam/map?after=${revision}&previous=${session}${query ? "&" + query : ""}`);
      if (run !== generation || drawer.hidden) return;
      if (result.session !== session || result.revision !== revision) {
        grid = result.grid;
        paintGrid();
      }
      overlay = result.overlay ?? null;
      sensorLayers = result.sensor_layers ?? {lidar: overlay?.scan};
      draw();
      active = result.active;
      revision = result.revision;
      session = result.session ?? "";
      get("slam-start").disabled = !result.online || active;
      get("slam-stop").disabled = !active;
      get("slam-save").disabled = !grid;
      feedback.textContent = !result.online
        ? "Chưa kết nối Cartographer."
        : active
          ? (grid ? "Đang dựng bản đồ. Màu xám xanh là vùng chưa biết." : "Chưa biết · đang chờ lượt quét đầu tiên.")
          : (grid ? "Đã dừng dựng bản đồ. Có thể tải lưới đã quét." : "Chưa biết · bắt đầu để dựng bản đồ.");
    } catch (error) {
      feedback.textContent = error.message;
    }
    setTimeout(() => poll(run), 400);
  }

  mapSensorLayers.controls.addEventListener("change", () => {
    if (!panel.hidden) draw();
  });

  get("map-show-slam").addEventListener("click", () => {
    get("map-show-slam").setAttribute("aria-pressed", "true");
    get("map-show-layout").setAttribute("aria-pressed", "false");
    panel.hidden = false;
    get("static-map-tools").hidden = true;
    get("map-title").textContent = "Dựng bản đồ 2D bằng SLAM";
    draw();
    poll(++generation);
  });
  get("map-show-layout").addEventListener("click", () => {
    get("map-show-slam").setAttribute("aria-pressed", "false");
    get("map-show-layout").setAttribute("aria-pressed", "true");
    stopDriving();
    generation += 1;
    panel.hidden = true;
    get("static-map-tools").hidden = false;
    get("map-title").textContent = "Bản đồ điều hướng";
  });
  get("open-map-tools").addEventListener("click", () => {
    if (!panel.hidden) poll(++generation);
  });
  get("slam-start").disabled = true;
  for (const action of ["start", "stop"]) {
    const button = get(`slam-${action}`);
    button.addEventListener("click", async () => {
      button.disabled = true;
      try {
        if (action === "stop") stopDriving();
        const result = await api(`/api/slam/${action}`, "POST");
        if (action === "start") {
          session = result.session;
          revision = -1;
          grid = null;
          overlay = null;
          sensorLayers = {};
          draw();
        }
        poll(++generation);
      } catch (error) {
        button.disabled = false;
        feedback.textContent = error.message;
      }
    });
  }
  get("slam-save").addEventListener("click", () => {
    if (!grid) return;
    const url = URL.createObjectURL(new Blob([JSON.stringify(grid)], { type: "application/json" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = "amr-slam-map.json";
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });

  async function move(command) {
    try {
      await api("/api/amr/command", "POST", command);
    } catch (error) {
      feedback.textContent = error.message;
    }
  }
  function stopDriving() {
    if (driving === null) return;
    clearInterval(driving);
    driving = null;
    if (!commandPending) void move({ linear: 0, angular: 0 });
  }
  document.querySelectorAll(".slam-drive").forEach((button) => {
    button.addEventListener("pointerdown", (event) => {
      if (!active || event.button !== 0 || driving !== null) return;
      button.setPointerCapture(event.pointerId);
      const command = {
        linear: Number(button.dataset.linear || 0),
        angular: manualAngularSpeed(Number(button.dataset.angular || 0)),
      };
      const tick = async () => {
        if (commandPending) return;
        commandPending = true;
        await move(command);
        commandPending = false;
        if (driving === null) void move({ linear: 0, angular: 0 });
      };
      driving = setInterval(tick, 100);
      void tick();
    });
    for (const event of ["pointerup", "pointercancel", "lostpointercapture"]) {
      button.addEventListener(event, stopDriving);
    }
  });
  drawer.addEventListener("close", stopDriving);
  window.addEventListener("blur", stopDriving);
  draw();
})();
