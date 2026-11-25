"use strict";

(() => {
  const get = (id) => document.getElementById(id);
  const drawer = get("map-tools");
  const opener = get("open-map-tools");
  const canvas = get("operations-map");
  const context = canvas.getContext("2d");
  const feedback = get("map-feedback");
  const waypoints = [];
  let layout;
  let rosGrid;
  let nativeMap = false;
  let walls = [];
  let wallStart;
  let mode = "route";
  let view = { x: 0, y: 0, span: 6 };
  let maxSpan = 24;
  let robotPose;
  let navigation;
  let generation = 0;
  let graph;
  let graphChecked = false;
  let graphDirty = false;
  let edgeStart;
  const graphGoal = get("graph-goal");
  let docking;
  let dockFace;
  let relocateDraft;
  let relocateBusy = false;
  let obstacleSession;
  let obstacleActive = false;
  const obstacleButton = get("obstacle-demo-toggle");
  const placeObstacleButton = get("obstacle-place");
  const obstaclePick = get("obstacle-pick");
  const obstacleStatus = get("obstacle-placement-status");
  let placedObstacles = [];
  let obstacleLimit = 0;
  let placingObstacle = false;
  let obstacleBusy = false;
  const waitForm = get("obstacle-wait-tools");
  const waitInput = get("obstacle-wait-seconds");
  const waitStatus = get("obstacle-wait-status");
  let waitSettings;
  let waitEdited = false;
  let waitSaving = false;

  async function api(path, method = "GET", data) {
    const response = await fetch(path, {
      method,
      cache: "no-store",
      ...(data === undefined ? {} : {
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(data),
      }),
    });
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || `HTTP ${response.status}`);
    return body;
  }

  const scale = () => canvas.width / view.span;
  const verticalSpan = (span = view.span) => span * canvas.height / canvas.width;
  const clampSpan = (span) => Math.min(maxSpan, Math.max(2, span));
  const pixel = ({ x, y }) => ({
    x: canvas.width / 2 + (x - view.x) * scale(),
    y: canvas.height / 2 - (y - view.y) * scale(),
  });

  function fitMap() {
    if (!layout) return;
    const {min_x, max_x, min_y, max_y} = layout.bounds;
    view = {
      x: (min_x + max_x) / 2,
      y: (min_y + max_y) / 2,
      span: Math.max(max_x - min_x, (max_y - min_y) * canvas.width / canvas.height) * 1.04,
    };
    maxSpan = Math.max(24, view.span);
  }

  function line(points, color, width = 2) {
    if (points.length < 2) return;
    context.beginPath();
    points.map(pixel).forEach(({ x, y }, i) => {
      if (i === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.lineWidth = width;
    context.strokeStyle = color;
    context.stroke();
  }

  function dot(point, radius, color) {
    const { x, y } = pixel(point);
    context.fillStyle = color;
    context.beginPath();
    context.arc(x, y, radius, 0, Math.PI * 2);
    context.fill();
  }

  function drawGraph() {
    if (!graph) return;
    const points = new Map(graph.points.map((p) => [p.name, p]));
    for (const edge of graph.edges) {
      const source = points.get(edge.source);
      const destination = points.get(edge.destination);
      const angle = Math.atan2(destination.y - source.y, destination.x - source.x);
      const tip = {
        x: source.x * 0.38 + destination.x * 0.62,
        y: source.y * 0.38 + destination.y * 0.62,
      };
      line([source, destination], "#2375b0", 2.5);
      for (const side of [-0.55, 0.55]) {
        line([tip, {
          x: tip.x - 10 / scale() * Math.cos(angle + side),
          y: tip.y - 10 / scale() * Math.sin(angle + side),
        }], "#2375b0", 2.5);
      }
    }
    for (const point of graph.points) {
      const selected = point.name === edgeStart || point.name === graphGoal.value;
      dot(point, selected ? 8 : 6, "#173d60");
      dot(point, selected ? 6 : 4, selected ? "#ffc063" : "#87d2ff");
      const p = pixel(point);
      context.font = "bold 13px sans-serif";
      context.strokeStyle = "#102033";
      context.lineWidth = 3;
      context.strokeText(point.name, p.x + 9, p.y - 7);
      context.fillStyle = "#f5f8fd";
      context.fillText(point.name, p.x + 9, p.y - 7);
    }
  }

  function drawDock() {
    if (dockFace) dot(dockFace, 6, "#07836c");
    if (!docking?.face) return;
    const {face, pose, approach, charging} = docking;
    const color = charging ? "#258143" : "#087864";
    const x = 0.20 * Math.sin(face.yaw);
    const y = 0.20 * Math.cos(face.yaw);
    line([{x: face.x + x, y: face.y - y}, {x: face.x - x, y: face.y + y}], color, 5);
    context.save();
    context.setLineDash([4, 4]);
    line([face, approach], color, 1.5);
    context.restore();
    dot(pose, 4, color);
    dot(approach, 3, color);
    const corner = pixel({x: face.x + x, y: face.y - y});
    context.fillStyle = color;
    context.font = "bold 13px sans-serif";
    context.fillText("Dock", corner.x + 7, corner.y - 8);
  }

  function drawRelocate() {
    if (mode !== "relocate" || !relocateDraft) return;
    dot(relocateDraft, 7, "#ffd569");
    if (!Number.isFinite(relocateDraft.yaw)) return;
    const {x, y, yaw} = relocateDraft;
    const end = {x: x + 0.5 * Math.cos(yaw), y: y + 0.5 * Math.sin(yaw)};
    line([relocateDraft, end], "#ffd569", 3);
    for (const side of [-0.5, 0.5]) {
      line([end, {x: end.x - 0.13 * Math.cos(yaw + side),
        y: end.y - 0.13 * Math.sin(yaw + side)}], "#ffd569", 3);
    }
  }

  function draw() {
    context.fillStyle = "#172635";
    context.fillRect(0, 0, canvas.width, canvas.height);
    if (rosGrid) {
      const top = pixel({x: layout.bounds.min_x, y: layout.bounds.max_y});
      context.imageSmoothingEnabled = false;
      context.drawImage(rosGrid, top.x, top.y,
        layout.width * layout.resolution * scale(), layout.height * layout.resolution * scale());
    }
    const halfX = view.span / 2;
    const halfY = verticalSpan() / 2;
    const interval = view.span < 8 ? 0.5 : 1;
    context.beginPath();
    for (let x = Math.ceil((view.x - halfX) / interval) * interval; x <= view.x + halfX; x += interval) {
      const column = pixel({x, y: view.y}).x;
      context.moveTo(column, 0);
      context.lineTo(column, canvas.height);
    }
    for (let y = Math.ceil((view.y - halfY) / interval) * interval; y <= view.y + halfY; y += interval) {
      const row = pixel({x: view.x, y}).y;
      context.moveTo(0, row);
      context.lineTo(canvas.width, row);
    }
    context.lineWidth = 1;
    context.strokeStyle = "rgba(71, 111, 136, .20)";
    context.stroke();
    for (const obstacle of layout?.obstacles ?? []) {
      dot(obstacle, obstacle.radius * scale(), "#8997a7");
    }
    drawPlacedObstacles();
    for (const wall of walls) line(wall, "#fa756f", 4);
    if (wallStart) dot(wallStart, 5, "#fa756f");
    drawGraph();
    drawDock();
    drawRelocate();
    line(waypoints, "#1a78b2", 2.5);
    line(navigation?.global_path ?? [], "#0b7a65", 4);
    waypoints.forEach((point, i) => {
      dot(point, 6, "#2076ac");
      const { x, y } = pixel(point);
      context.fillStyle = "#fff";
      context.font = "13px sans-serif";
      context.fillText(String(i + 1), x + 9, y - 7);
    });
    const pose = robotPose;
    if (pose?.position) {
      const { x, y } = pose.position;
      const heading = {x: x + 0.30 * Math.cos(pose.yaw), y: y + 0.30 * Math.sin(pose.yaw)};
      line([pose.position, heading], "#492e12", 7);
      line([pose.position, heading], "#ffc655", 4);
      dot(pose.position, 9, "#492e12");
      dot(pose.position, 6.5, "#ffc655");
    }
  }

  function drawPlacedObstacles() {
    for (const {id, x, y, yaw, length, width} of placedObstacles) {
      const center = pixel({x, y});
      const halfLength = length * scale() / 2;
      const halfWidth = width * scale() / 2;
      context.save();
      context.translate(center.x, center.y);
      context.rotate(-yaw);
      context.fillStyle = "#ce712e";
      context.strokeStyle = "#623210";
      context.lineWidth = id === Number(obstaclePick.value) ? 3 : 1.5;
      context.fillRect(-halfLength, -halfWidth, 2 * halfLength, 2 * halfWidth);
      context.strokeRect(-halfLength, -halfWidth, 2 * halfLength, 2 * halfWidth);
      context.restore();
      context.fillStyle = "#4a260d";
      context.font = "bold 13px sans-serif";
      context.fillText(`#${id}`, center.x + 7, center.y - 9);
    }
  }

  function refreshControls() {
    const busy = ["PLANNING", "NAVIGATING", "RESETTING"].includes(navigation?.state);
    const graphMode = mode.startsWith("graph-");
    placeObstacleButton.disabled = !layout || obstacleBusy || placedObstacles.length >= obstacleLimit;
    placeObstacleButton.setAttribute("aria-pressed", String(placingObstacle));
    placeObstacleButton.textContent = placingObstacle ? "Hủy đặt vật cản" : "Đặt vật cản";
    get("obstacle-remove").disabled = obstacleBusy || !obstaclePick.value;
    get("map-zoom-in").disabled = !layout || view.span <= 2;
    get("map-zoom-out").disabled = !layout || view.span >= maxSpan;
    get("map-fit").disabled = !layout;
    const canPlan = graphMode ? graphGoal.value && !graphDirty : mode === "route" && waypoints.length;
    get("map-plan-route").disabled = !layout || !canPlan || busy;
    const graphPlan = navigation?.goal?.kind === "defined-route";
    const matchingPlan = graphMode
      ? graphPlan && !graphDirty && graphGoal.value === navigation.goal.name : mode === "route" && !graphPlan;
    get("map-start-route").disabled = navigation?.state !== "READY" || !matchingPlan;
    get("map-save-walls").disabled = !layout || busy || navigation?.state === "READY";
    get("map-route-mode").classList.toggle("active", mode === "route");
    get("map-wall-mode").classList.toggle("active", mode === "wall");
    for (const id of ["graph-add", "graph-connect", "graph-delete"]) {
      get(id).disabled = busy || !layout;
      get(id).classList.toggle("active", mode === id);
    }
    graphGoal.disabled = busy;
    get("graph-save").disabled = busy || !graphDirty;
    get("dock-place").disabled = !layout || busy;
    get("dock-place").classList.toggle("active", mode === "dock");
    get("dock-return").disabled = !docking?.pose || busy || docking.charging;
    get("dock-stop").hidden = !docking?.active;
    get("map-relocate").disabled = !layout || relocateBusy;
    get("map-relocate").classList.toggle("active", mode === "relocate");
    get("map-relocate-confirm").hidden = mode !== "relocate";
    get("map-relocate-confirm").disabled = relocateBusy || !Number.isFinite(relocateDraft?.yaw);
    get("map-relocate-cancel").hidden = mode !== "relocate";
    get("map-relocate-cancel").disabled = relocateBusy;
  }

  function syncGraphChoices() {
    const selected = graphGoal.value;
    graphGoal.replaceChildren(new Option("Chọn điểm", ""), ...graph.points.filter(
      (point) => point.type !== "waypoint"
    ).map((point) => new Option(point.name, point.name)));
    if ([...graphGoal.options].some((option) => option.value === selected)) graphGoal.value = selected;
  }

  function action(id, work) {
    get(id).addEventListener("click", async () => {
      try {
        await work();
      } catch (error) {
        feedback.textContent = error.message;
      }
      refreshControls();
      draw();
    });
  }

  function renderObstacleDemo({active}) {
    obstacleActive = active;
    obstacleButton.setAttribute("aria-pressed", String(active));
    obstacleButton.textContent = active ? "Ẩn vật cản thử" : "Thử vật cản động";
    get("obstacle-demo-status").textContent = active
      ? "Khối tím băng qua đường một lượt rồi dừng ở mép bên kia." : "";
  }

  function updateWaitControls() {
    const seconds = waitInput.valueAsNumber;
    const limit = get("obstacle-wait-limit");
    const recovery = waitSettings?.recovery_seconds;
    waitInput.disabled = !waitSettings || waitSaving;
    get("obstacle-wait-apply").disabled = waitInput.disabled || !waitInput.validity.valid
      || seconds === waitSettings.seconds;
    limit.hidden = recovery == null || !(seconds > recovery);
    if (!limit.hidden) {
      limit.textContent = `Reeman có thể vào chế độ khôi phục sau ${recovery.toLocaleString("vi-VN")} giây.`;
    }
  }

  function showObstacleWait(settings) {
    if (!settings || (waitSettings && settings.seconds === waitSettings.seconds
        && settings.recovery_seconds === waitSettings.recovery_seconds
        && settings.min_seconds === waitSettings.min_seconds && settings.max_seconds === waitSettings.max_seconds)) return;
    waitSettings = settings;
    waitForm.hidden = false;
    waitInput.min = settings.min_seconds;
    waitInput.max = settings.max_seconds;
    if (!waitEdited && !waitSaving) waitInput.value = settings.seconds;
    updateWaitControls();
  }

  function showPlacedObstacles(obstacles, selectedId = obstaclePick.value) {
    placedObstacles = obstacles;
    obstaclePick.replaceChildren(...obstacles.map(({id}) => new Option(`Vật cản #${id}`, id)));
    if (obstacles.some(({id}) => id === Number(selectedId))) obstaclePick.value = selectedId;
    get("placed-obstacle-tools").hidden = !obstacles.length;
    refreshControls();
    draw();
  }

  async function loadPlacedObstacles(session) {
    try {
      const result = await api("/api/obstacles/barriers");
      if (session !== generation) return;
      obstacleLimit = result.limit;
      showPlacedObstacles(result.obstacles);
    } catch (error) {
      if (session === generation) obstacleStatus.textContent = error.message;
    }
  }

  async function poll(session) {
    if (drawer.hidden || session !== generation) return;
    try {
      const {pose, navigation: status, dock, relocate_available: relocateAvailable,
        camera_obstacle_demo: cameraDemo, obstacle_wait: obstacleWait} = await api("/api/map/live");
      if (drawer.hidden || session !== generation) return;
      robotPose = pose;
      navigation = status;
      docking = dock;
      get("dock-tools").hidden = !docking;
      get("map-relocate").hidden = !relocateAvailable;
      get("obstacle-demo-tools").hidden = !cameraDemo;
      showObstacleWait(obstacleWait);
      if (cameraDemo && pose && obstacleSession !== session) {
        obstacleSession = session;
        loadPlacedObstacles(session);
        obstacleButton.disabled = true;
        api("/api/obstacles/demo").then(renderObstacleDemo).catch((error) => {
          get("obstacle-demo-status").textContent = error.message;
        }).finally(() => { obstacleButton.disabled = false; });
      }
      get("dock-status").textContent = docking?.message ?? "";
      const sequence = status.route_points?.join(" → ");
      get("map-mission-state").textContent = status.goal?.kind === "charging-dock" ? ""
        : [status.error || status.message, sequence].filter(Boolean).join(" · ");
      refreshControls();
      draw();
    } catch (error) {
      get("map-mission-state").textContent = error.message;
    }
    window.setTimeout(() => poll(session), 450);
  }

  async function loadMap(session) {
    let nextLayout;
    for (let attempt = 0; attempt < 8; attempt++) {
      try {
        nextLayout = await api("/api/map");
        break;
      } catch (error) {
        if (attempt === 7) throw error;
        feedback.textContent = "Đang chờ bản đồ từ ROS…";
        await new Promise((resolve) => window.setTimeout(resolve, 600));
        if (session !== generation) return;
      }
    }
    if (session !== generation) return;
    const needsFit = !layout || layout.revision !== nextLayout.revision;
    if (nextLayout.revision != null && nextLayout.revision !== layout?.revision) {
      const response = await fetch("/api/map/grid", {cache: "no-store"});
      if (!response.ok) throw new Error("Chưa nhận lưới bản đồ ROS.");
      const cells = new Uint8Array(await response.arrayBuffer());
      const {width, height} = nextLayout;
      if (cells.length !== width * height) throw new Error("Lưới bản đồ ROS vừa thay đổi.");
      rosGrid = document.createElement("canvas");
      rosGrid.width = width;
      rosGrid.height = height;
      const renderer = rosGrid.getContext("2d");
      const image = renderer.createImageData(width, height);
      for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
          const cell = cells[row * width + col];
          const i = ((height - row - 1) * width + col) * 4;
          const shade = cell ? 1 - (cell - 1) / 100 : 0;
          image.data[i] = cell ? 34 + shade * 191 : 69;
          image.data[i + 1] = cell ? 51 + shade * 186 : 83;
          image.data[i + 2] = cell ? 62 + shade * 181 : 100;
          image.data[i + 3] = 255;
        }
      }
      renderer.putImageData(image, 0, 0);
    }
    layout = nextLayout;
    if (needsFit) fitMap();
    const result = await api("/api/map/walls");
    walls = result.walls;
    nativeMap = result.editable === false;
    for (const id of ["map-wall-mode", "map-save-walls", "map-clear-walls"]) get(id).hidden = nativeMap;
    if (nativeMap) {
      get("map-route-mode").textContent = "Đích tự do";
      if (!graphChecked) {
        const response = await fetch("/api/navigation/graph", {cache: "no-store"});
        if (response.ok) {
          graph = await response.json();
          syncGraphChoices();
          get("defined-route-tools").hidden = false;
        }
        graphChecked = true;
      }
      if (graphGoal.value) mode = "graph-target";
    }
    wallStart = undefined;
    if (session !== generation) return;
    feedback.textContent = "Chọn đích trực tiếp trên bản đồ.";
    poll(session);
  }

  action("open-map-tools", async () => {
    drawer.hidden = false;
    document.body.classList.add("map-tools-open");
    opener.setAttribute("aria-expanded", "true");
    await loadMap(++generation);
  });
  get("close-map-tools").addEventListener("click", () => {
    drawer.hidden = true;
    if (placingObstacle) obstacleStatus.textContent = "";
    placingObstacle = false;
    document.body.classList.remove("map-tools-open");
    opener.setAttribute("aria-expanded", "false");
    generation += 1;
    drawer.dispatchEvent(new Event("close"));
    opener.focus();
  });
  action("map-route-mode", () => {
    mode = "route";
    dockFace = undefined;
    feedback.textContent = nativeMap ? "Chọn một đích tự do trên bản đồ."
      : "Chọn các waypoint theo thứ tự muốn robot đi qua.";
  });
  action("map-zoom-in", () => { view.span = clampSpan(view.span / 1.25); });
  action("map-zoom-out", () => { view.span = clampSpan(view.span * 1.25); });
  action("map-fit", fitMap);
  action("dock-place", () => {
    mode = "dock";
    dockFace = undefined;
    feedback.textContent = "Bấm vào tâm mặt trước của dock.";
  });
  action("dock-return", async () => {
    docking = await api("/api/dock/return", "POST", {});
    navigation = await api("/api/navigation/status");
    feedback.textContent = docking.message;
  });
  action("dock-stop", async () => {
    navigation = await api("/api/navigation/cancel", "POST", {});
    docking = await api("/api/dock");
    feedback.textContent = docking.message;
  });
  action("map-relocate", () => {
    mode = "relocate";
    dockFace = undefined;
    relocateDraft = undefined;
    feedback.textContent = "Bấm vào vị trí tâm robot trên bản đồ.";
  });
  action("map-relocate-confirm", async () => {
    relocateBusy = true;
    refreshControls();
    feedback.textContent = "Đang đặt lại định vị…";
    try {
      const {pose, message} = await api("/api/relocate", "POST", relocateDraft);
      robotPose = {position: {x: pose.x, y: pose.y}, yaw: pose.yaw};
      relocateDraft = undefined;
      mode = "route";
      feedback.textContent = message;
      if (obstacleLimit) loadPlacedObstacles(generation);
    } finally {
      relocateBusy = false;
    }
  });
  action("map-relocate-cancel", () => {
    mode = "route";
    relocateDraft = undefined;
    feedback.textContent = "Đã hủy đặt pose.";
  });
  action("obstacle-demo-toggle", async () => {
    obstacleButton.disabled = true;
    try {
      renderObstacleDemo(await api("/api/obstacles/demo", "POST", {active: !obstacleActive}));
    } finally {
      obstacleButton.disabled = false;
    }
  });
  action("obstacle-place", () => {
    placingObstacle = !placingObstacle;
    obstacleStatus.textContent = placingObstacle ? "Bấm vào vị trí đặt rào cam trên bản đồ." : "";
  });
  action("obstacle-remove", async () => {
    obstacleBusy = true;
    refreshControls();
    try {
      const result = await api("/api/obstacles/barriers", "POST", {action: "remove", id: Number(obstaclePick.value)});
      showPlacedObstacles(placedObstacles.filter(({id}) => id !== result.removed));
      obstacleStatus.textContent = `Đã gỡ vật cản #${result.removed}.`;
    } catch (error) {
      obstacleStatus.textContent = error.message;
    } finally {
      obstacleBusy = false;
    }
  });
  obstaclePick.addEventListener("change", draw);
  waitInput.addEventListener("input", () => {
    waitEdited = waitInput.valueAsNumber !== waitSettings?.seconds;
    waitStatus.textContent = "";
    updateWaitControls();
  });
  waitForm.addEventListener("submit", async (event) => {
    event.preventDefault();
    if (waitSaving || !waitSettings || !waitInput.reportValidity()) return;
    const seconds = waitInput.valueAsNumber;
    waitSaving = true;
    updateWaitControls();
    try {
      const settings = await api("/api/navigation/obstacle-wait", "POST", {seconds});
      waitEdited = false;
      waitInput.value = settings.seconds;
      showObstacleWait(settings);
      waitStatus.textContent = `Đã áp dụng ${settings.seconds.toLocaleString("vi-VN")} giây và lưu cài đặt.`;
    } catch (error) {
      waitStatus.textContent = error.message;
    } finally {
      waitSaving = false;
      updateWaitControls();
    }
  });
  for (const [id, hint] of [
    ["graph-add", "Bấm bản đồ để thêm các điểm theo số thứ tự."],
    ["graph-connect", "Chọn điểm nguồn rồi điểm đích. Nối lại cùng chiều để bỏ đoạn đó."],
    ["graph-delete", "Chọn điểm cần xóa cùng các đoạn nối của nó."],
  ]) {
    action(id, () => {
      mode = id;
      dockFace = undefined;
      edgeStart = undefined;
      graphGoal.value = "";
      feedback.textContent = hint;
    });
  }
  action("graph-save", async () => {
    graph = await api("/api/navigation/graph", "POST", graph);
    graphDirty = false;
    edgeStart = undefined;
    mode = "graph-target";
    syncGraphChoices();
    navigation = await api("/api/navigation/status");
    feedback.textContent = "Đã lưu. Chọn đích theo tuyến rồi bấm Lập tuyến.";
  });
  graphGoal.addEventListener("change", () => {
    mode = "graph-target";
    dockFace = undefined;
    edgeStart = undefined;
    feedback.textContent = graphDirty ? "Lưu tuyến trước khi lập đường."
      : "Bấm Lập tuyến để xem đường đi đến đích đã chọn.";
    refreshControls();
    draw();
  });
  action("map-wall-mode", () => {
    mode = "wall";
    feedback.textContent = "Chọn hai đầu mút cho mỗi tường, rồi bấm Áp dụng tường.";
  });
  action("map-undo", () => {
    if (mode === "route") waypoints.pop();
    else if (mode === "dock") dockFace = undefined;
    else if (mode === "relocate" && !relocateBusy) relocateDraft = undefined;
    else if (mode === "wall") {
      if (wallStart) wallStart = undefined;
      else walls.pop();
    } else edgeStart = undefined;
  });
  action("map-clear-walls", () => {
    walls = [];
    wallStart = undefined;
    feedback.textContent = "Bấm Áp dụng tường để cập nhật bộ điều hướng.";
  });
  action("map-save-walls", async () => {
    if (wallStart) throw new Error("Chọn đầu mút thứ hai trước khi áp dụng.");
    ({ walls } = await api("/api/map/walls", "PUT", { walls }));
    feedback.textContent = `Đã áp dụng ${walls.length} đoạn tường cho bộ điều hướng.`;
  });
  action("map-plan-route", async () => {
    if (mode.startsWith("graph-")) {
      navigation = await api("/api/navigation/graph/plan", "POST", {name: graphGoal.value});
      feedback.textContent = "Đường theo tuyến đã hiện màu xanh lá. Bấm Chạy tuyến để di chuyển.";
      return;
    }
    navigation = nativeMap
      ? await api("/api/navigation/plan", "POST", {goal: waypoints[0]})
      : await api("/api/navigation/route", "POST", {waypoints});
    feedback.textContent = `Đã lập tuyến qua ${waypoints.length} waypoint.`;
  });
  action("map-start-route", async () => {
    navigation = await api("/api/navigation/start", "POST");
    feedback.textContent = "Đang chạy tuyến đã lập.";
  });
  action("map-stop-route", async () => {
    navigation = await api("/api/navigation/cancel", "POST");
    feedback.textContent = "Đã dừng điều hướng.";
  });
  canvas.addEventListener("click", async (event) => {
    if (!layout) return;
    const bounds = canvas.getBoundingClientRect();
    const x = view.x + ((event.clientX - bounds.left) / bounds.width - 0.5) * view.span;
    const y = view.y - ((event.clientY - bounds.top) / bounds.height - 0.5) * verticalSpan();
    const { min_x, min_y, max_x, max_y } = layout.bounds;
    if (x < min_x || x >= max_x || y < min_y || y >= max_y) {
      feedback.textContent = "Điểm nằm ngoài bản đồ.";
      return;
    }
    const point = { x, y };
    if (placingObstacle) {
      if (obstacleBusy) return;
      obstacleBusy = true;
      refreshControls();
      try {
        const yaw = get("obstacle-direction").value === "vertical" ? Math.PI / 2 : 0;
        const result = await api("/api/obstacles/barriers", "POST", {action: "add", ...point, yaw});
        placingObstacle = false;
        const previousIds = new Set(placedObstacles.map(({id}) => id));
        const added = result.obstacles.find(({id}) => !previousIds.has(id));
        showPlacedObstacles(result.obstacles, added?.id);
        obstacleStatus.textContent = added ? `Đã đặt vật cản #${added.id}.` : "Đã đặt vật cản.";
      } catch (error) {
        obstacleStatus.textContent = error.message;
      } finally {
        obstacleBusy = false;
        refreshControls();
      }
      return;
    }
    if (mode === "route") {
      if (nativeMap) waypoints.splice(0, waypoints.length, point);
      else waypoints.push(point);
    }
    else if (mode === "relocate") {
      if (relocateBusy) return;
      if (!relocateDraft) {
        relocateDraft = point;
        feedback.textContent = "Bấm một điểm về phía đầu robot để chọn hướng.";
      } else {
        const dx = x - relocateDraft.x;
        const dy = y - relocateDraft.y;
        if (Math.hypot(dx, dy) < 0.12) {
          feedback.textContent = "Chọn điểm xa tâm robot hơn để xác định hướng.";
          return;
        }
        relocateDraft.yaw = Math.atan2(dy, dx);
        feedback.textContent = "Bấm Áp dụng pose để đặt lại định vị.";
      }
    } else if (mode === "dock") {
      if (docking?.active) return;
      if (!dockFace) {
        dockFace = point;
        feedback.textContent = "Bấm một điểm phía trước dock, về phía khoảng trống để robot quay đầu.";
      } else {
        const dx = x - dockFace.x;
        const dy = y - dockFace.y;
        if (Math.hypot(dx, dy) < 0.12) {
          feedback.textContent = "Chọn điểm xa mặt dock hơn để xác định hướng.";
          return;
        }
        try {
          docking = await api("/api/dock", "POST", {...dockFace, yaw: Math.atan2(dy, dx)});
          dockFace = undefined;
          mode = "route";
          feedback.textContent = "Đã lưu vị trí và hướng dock. Bấm Về dock sạc để chạy.";
        } catch (error) {
          feedback.textContent = error.message;
        }
      }
    } else if (mode.startsWith("graph-")) {
      if (!graph || navigation?.state === "NAVIGATING") return;
      const match = graph.points.find((item) => Math.hypot(item.x - x, item.y - y) * scale() < 13);
      if (mode === "graph-add" && !match) {
        const names = new Set(graph.points.map((item) => item.name));
        let number = 1;
        while (names.has(String(number))) number++;
        graph.points.push({name: String(number), ...point, yaw: 0, type: "delivery"});
        graphDirty = true;
      } else if (mode === "graph-delete" && match) {
        graph.points = graph.points.filter((item) => item !== match);
        graph.edges = graph.edges.filter(({ source, destination }) => (
          source !== match.name && destination !== match.name
        ));
        graphDirty = true;
      } else if (mode === "graph-connect" && match) {
        if (!edgeStart || edgeStart === match.name) {
          edgeStart = edgeStart === match.name ? undefined : match.name;
        } else {
          const index = graph.edges.findIndex(({ source, destination }) => (
            source === edgeStart && destination === match.name
          ));
          if (index < 0) graph.edges.push({source: edgeStart, destination: match.name});
          else graph.edges.splice(index, 1);
          edgeStart = undefined;
          graphDirty = true;
        }
      } else if (mode === "graph-target" && match && match.type !== "waypoint") {
        graphGoal.value = match.name;
      }
      syncGraphChoices();
    } else if (wallStart) {
      walls.push([wallStart, point]);
      wallStart = undefined;
    } else wallStart = point;
    refreshControls();
    draw();
  });
  canvas.addEventListener("wheel", (event) => {
    if (!layout) return;
    event.preventDefault();
    const span = clampSpan(view.span * Math.exp(Math.sign(event.deltaY) * 0.18));
    const bounds = canvas.getBoundingClientRect();
    view = {
      x: view.x + ((event.clientX - bounds.left) / bounds.width - 0.5) * (view.span - span),
      y: view.y - ((event.clientY - bounds.top) / bounds.height - 0.5) * verticalSpan(view.span - span),
      span,
    };
    refreshControls();
    draw();
  }, { passive: false });
  refreshControls();
  if (document.body.dataset.mapDefault === "open") {
    window.addEventListener("load", () => opener.click(), {once: true});
  }
})();
