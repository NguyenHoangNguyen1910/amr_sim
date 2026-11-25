class ReemanSensorPanel {
  constructor() {
    this.panel = document.querySelector('#sensor-panel details');
    this.scan = document.getElementById('reeman-scan');
    this.depth = document.getElementById('reeman-depth');
    this.scanStatus = document.getElementById('reeman-scan-status');
    this.imuStatus = document.getElementById('reeman-imu-status');
    this.depthStatus = document.getElementById('reeman-depth-status');
    this.cameraScanStatus = document.getElementById('reeman-camera-scan');
    this.scanSequence = null;
    this.depthSequence = null;
  }

  async poll() {
    try {
      if (!this.panel.open || document.hidden) return;
      const response = await fetch('/api/sensors', {cache: 'no-store'});
      if (!response.ok) throw new Error('Mất kết nối dữ liệu cảm biến');
      const sensors = await response.json();
      this.renderScan(sensors.reeman_lidar);
      this.renderImu(sensors.imu);
      this.renderCameraScan(sensors.camera_scan);
      await this.renderDepth(sensors.depth);
    } catch (error) {
      this.depthStatus.textContent = error.message;
    } finally {
      window.setTimeout(() => this.poll(), 500);
    }
  }

  fresh(sensor) {
    return sensor?.data && sensor.age_seconds != null && sensor.age_seconds < 2;
  }

  number(value, digits = 2) {
    return Number.isFinite(value) ? value.toFixed(digits) : '—';
  }

  renderScan(sensor) {
    const data = sensor?.data;
    this.scanStatus.textContent = this.fresh(sensor)
      ? `${data.count} tia · 240° · phạm vi ${data.range_max} m`
      : data ? 'Dữ liệu LiDAR đã cũ' : 'Đang chờ bản tin';
    if (!this.fresh(sensor) || data.sequence === this.scanSequence) return;
    this.scanSequence = data.sequence;
    const ctx = this.scan.getContext('2d');
    const {width, height} = this.scan;
    const x = width / 2;
    const y = height * .72;
    const radius = height * .65;
    const scale = radius / 5;
    ctx.clearRect(0, 0, width, height);
    ctx.strokeStyle = '#2c4d63';
    for (const range of [1, 3, 5]) {
      ctx.beginPath();
      ctx.arc(x, y, range * scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.fillStyle = '#59d9c8';
    for (const [index, distance] of data.ranges.entries()) {
      if (!Number.isFinite(distance) || distance < data.range_min || distance > 5) continue;
      const angle = data.angle_min + index * data.angle_step;
      ctx.fillRect(x - Math.sin(angle) * distance * scale,
        y - Math.cos(angle) * distance * scale, 2, 2);
    }
    ctx.fillStyle = '#eef5fa';
    ctx.fillRect(x - 2, y - 2, 4, 4);
  }

  renderImu(sensor) {
    const data = sensor?.data;
    this.imuStatus.textContent = this.fresh(sensor)
      ? `Yaw ${this.number(data.yaw == null ? null : data.yaw * 180 / Math.PI, 1)}° · ωz ${this.number(data.angular_velocity.z)} rad/s · az ${this.number(data.linear_acceleration.z)} m/s²`
      : data ? 'Dữ liệu IMU đã cũ' : 'Đang chờ bản tin';
  }

  renderCameraScan(sensor) {
    this.cameraScanStatus.hidden = !sensor;
    if (!sensor) return;
    const closest = sensor.data?.closest_m;
    this.cameraScanStatus.textContent = !this.fresh(sensor)
      ? 'Đang chờ /camera/scan'
      : Number.isFinite(closest) ? `/camera/scan · Điểm gần nhất ${this.number(closest)} m`
        : '/camera/scan · Chưa thấy vật cản trong tầm đo';
  }

  async renderDepth(sensor) {
    const data = sensor?.data;
    this.depthStatus.textContent = this.fresh(sensor)
      ? `${data.width} × ${data.height} · tâm ảnh ${this.number(data.center_m)} m`
      : data ? 'Ảnh depth đã cũ' : 'Đang chờ ảnh';
    if (!this.fresh(sensor) || data.sequence === this.depthSequence) return;
    const response = await fetch('/api/sensors/depth', {cache: 'no-store'});
    if (!response.ok) return;
    const width = Number(response.headers.get('X-Depth-Width'));
    const height = Number(response.headers.get('X-Depth-Height'));
    const step = Number(response.headers.get('X-Depth-Step'));
    const pixels = new DataView(await response.arrayBuffer());
    if (!width || !height || step < width * 4 || pixels.byteLength !== step * height) return;
    if (this.depth.width !== width) this.depth.width = width;
    if (this.depth.height !== height) this.depth.height = height;
    const ctx = this.depth.getContext('2d');
    const image = ctx.createImageData(width, height);
    for (let row = 0; row < height; row++) {
      for (let column = 0; column < width; column++) {
        const value = pixels.getFloat32(row * step + column * 4, true);
        const level = Number.isFinite(value) ? Math.max(0, 1 - value / 6) : 0;
        const index = (row * width + column) * 4;
        image.data[index] = level * 126;
        image.data[index + 1] = level * 220;
        image.data[index + 2] = level * 255;
        image.data[index + 3] = 255;
      }
    }
    ctx.putImageData(image, 0, 0);
    this.depthSequence = Number(response.headers.get('X-Depth-Sequence'));
  }
}

new ReemanSensorPanel().poll();

const turnSpeedInput = document.getElementById('turn-speed');
const customTurnSpeed = window.location.port === '8020';

function manualAngularSpeed(requestedAngular) {
  return customTurnSpeed
    ? Math.sign(requestedAngular) * turnSpeedInput.valueAsNumber
    : requestedAngular;
}

const navigationLabels = {
  IDLE: 'Chờ', READY: 'Sẵn sàng', PLANNING: 'Đang lập tuyến', NAVIGATING: 'Đang chạy',
  SUCCEEDED: 'Đã tới đích', BLOCKED: 'Bị chặn', CANCELLED: 'Đã dừng',
  RESETTING: 'Đang đặt lại', ERROR: 'Lỗi điều hướng',
};

class AMRViewer {
  constructor() {
    const host = window.location.hostname || 'localhost';
    const scheme = window.location.protocol === 'https:' ? 'wss' : 'ws';

    this.world = 'amr_world';
    this.gzUrl = `${scheme}://${host}:9002`;
    this.gz = null;
    this.objects = new Map();
    this.poseTargets = new Map();
    this.pressedKeys = new Set();
    this.command = {linear: 0, angular: 0};
    this.speedMps = 0.50;
    this.speedMinMps = 0.30;
    this.speedMaxMps = 0.90;
    this.speedStepMps = 0.10;
    this.speedRequestPending = false;
    this.commandHeartbeat = null;
    this.reconnectTimer = null;
    this.sceneLoaded = false;
    this.lastAnimationTime = performance.now();
    this.selectingGoal = false;
    this.selectedGoal = null;
    this.goalSelectionIsLocal = false;
    this.goalSelectionError = null;
    this.goalValidationSequence = 0;
    this.navigationState = 'IDLE';
    this.pathObject = null;
    this.trailObject = null;
    this.goalObject = null;

    this.connectionStatus = document.getElementById('connection-status');
    this.sensorStatus = document.getElementById('sensor-status');
    this.poseStatus = document.getElementById('pose-status');
    this.commandStatus = document.getElementById('command-status');
    this.speedDownButton = document.getElementById('speed-down-button');
    this.speedUpButton = document.getElementById('speed-up-button');
    this.speedValue = document.getElementById('speed-value');
    this.viewer = document.getElementById('viewer');
    this.wheelViewButton = document.getElementById('wheel-view');
    this.navigationStateElement = document.getElementById('navigation-state');
    this.navigationGoal = document.getElementById('navigation-goal');
    this.navigationDistance = document.getElementById('navigation-distance');
    this.selectGoalButton = document.getElementById('select-goal-button');
    this.goButton = document.getElementById('go-button');
    this.stopNavigationButton = document.getElementById('stop-navigation-button');

    this.scene = new GZ3D.Scene(new GZ3D.Shaders());
    this.scene.ambient.color.setHex(0x777777);
    this.scene.renderer.setClearColor(0xdde3e8, 1);
    this.scene.renderer.shadowMap.enabled = true;
    this.scene.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    const light = new THREE.DirectionalLight(0xffffff, 0.8);
    light.position.set(-7, -9, 12);
    light.castShadow = true;
    this.scene.add(light);
    this.scene.grid.visible = document.body.dataset.mapDefault !== 'open';
  }

  start() {
    this.viewer.appendChild(this.scene.renderer.domElement);
    this.resize();
    this.setCamera();
    this.viewerResize = new ResizeObserver(() => this.resize());
    this.viewerResize.observe(this.viewer);
    this.wheelViewButton.addEventListener('click', () => this.toggleWheelView());
    document.getElementById('scene-overview').addEventListener('click', () => this.setCamera());
    window.addEventListener('resize', () => this.resize());
    window.addEventListener('blur', () => {
      if (this.command.linear !== 0 || this.command.angular !== 0) {
        this.stopManual();
      }
    });
    window.addEventListener('beforeunload', () => {
      if (this.command.linear !== 0 || this.command.angular !== 0) {
        this.sendCommand(0, 0);
      }
    });

    this.bindManualControls();
    this.bindNavigationControls();
    this.loadStatus();
    this.connect(true);
    this.animate();
    this.pollTelemetry();
    window.setInterval(() => this.pollTelemetry(), 500);
    window.setInterval(() => this.pollNavigation(), 250);
  }

  async loadStatus() {
    try {
      const response = await fetch('/api/health', {cache: 'no-store'});
      if (!response.ok) throw new Error('health unavailable');
      const health = await response.json();
      if (health.navigation) this.updateNavigation(health.navigation);
    } catch (error) {
      this.setConnectionState(error.message, 'error');
    }
  }

  bindManualControls() {
    document.querySelectorAll('#controls [data-linear]').forEach((button) => {
      const start = (event) => {
        event.preventDefault();
        const requestedLinear = Number(button.dataset.linear);
        this.setCommand(
          requestedLinear === 0
            ? 0
            : Math.sign(requestedLinear) * this.manualLinearSpeed(),
          manualAngularSpeed(Number(button.dataset.angular))
        );
      };
      button.addEventListener('pointerdown', start);
      button.addEventListener('pointerup', () => this.stopManual());
      button.addEventListener('pointercancel', () => this.stopManual());
      button.addEventListener('pointerleave', () => this.stopManual());
    });
    const drivingKeys = new Set([
      'KeyW', 'ArrowUp', 'KeyS', 'ArrowDown', 'KeyA', 'ArrowLeft', 'KeyD', 'ArrowRight',
    ]);
    window.addEventListener('keydown', (event) => {
      if (event.target instanceof HTMLElement &&
          event.target.closest('input, textarea, select, [contenteditable]')) return;
      if (!drivingKeys.has(event.code) && event.code !== 'Space') return;
      event.preventDefault();
      if (event.code === 'Space') {
        this.pressedKeys.clear();
        this.stopManual();
        return;
      }
      this.pressedKeys.add(event.code);
      this.updateKeyboardCommand();
    });

    window.addEventListener('keyup', (event) => {
      if (!this.pressedKeys.has(event.code)) return;
      event.preventDefault();
      this.pressedKeys.delete(event.code);
      this.updateKeyboardCommand();
    });

    this.speedDownButton.addEventListener('click', () => this.adjustSpeed(-1));
    this.speedUpButton.addEventListener('click', () => this.adjustSpeed(1));
    this.updateSpeedControl(this.speedMps);
    if (customTurnSpeed) {
      this.speedValue.closest('.speed-control').hidden = true;
      document.getElementById('turn-speed-control').hidden = false;
      turnSpeedInput.disabled = false;
      const output = document.getElementById('turn-speed-value');
      turnSpeedInput.addEventListener('input', () => {
        output.value = `${turnSpeedInput.valueAsNumber.toFixed(2).replace('.', ',')} rad/s`;
        if (this.command.angular !== 0) this.stopManual();
      });
    }
  }

  manualLinearSpeed() {
    return this.speedMps;
  }

  updateSpeedControl(speedMps) {
    const value = Number(speedMps);
    if (!Number.isFinite(value)) return;
    this.speedMps = Math.max(
      this.speedMinMps,
      Math.min(this.speedMaxMps, Math.round(value * 100) / 100)
    );
    this.speedValue.textContent = `${this.speedMps.toFixed(2)} m/s`;
    this.speedDownButton.disabled = this.speedRequestPending
      || this.speedMps <= this.speedMinMps;
    this.speedUpButton.disabled = this.speedRequestPending
      || this.speedMps >= this.speedMaxMps;
  }

  async adjustSpeed(direction) {
    const next = Math.max(
      this.speedMinMps,
      Math.min(
        this.speedMaxMps,
        Math.round((this.speedMps + direction * this.speedStepMps) * 100) / 100
      )
    );
    if (next === this.speedMps) return;
    // A held manual command should not keep running while its speed cap
    // changes. Autonomous control has a zero browser command and is untouched.
    if (this.command.linear !== 0 || this.command.angular !== 0) {
      this.stopManual();
    }
    this.speedRequestPending = true;
    this.updateSpeedControl(this.speedMps);
    try {
      const result = await this.request('/api/amr/speed', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({speed_mps: next})
      });
      this.updateSpeedControl(result.speed_mps || next);
    } catch (error) {
      this.navigationStateElement.textContent = error.message;
    } finally {
      this.speedRequestPending = false;
      this.updateSpeedControl(this.speedMps);
    }
  }

  bindNavigationControls() {
    this.selectGoalButton.addEventListener('click', () => {
      this.selectingGoal = !this.selectingGoal;
      this.selectGoalButton.classList.toggle('active', this.selectingGoal);
      this.viewer.classList.toggle('selecting-goal', this.selectingGoal);
      if (this.selectingGoal) {
        this.goalValidationSequence += 1;
        this.goalSelectionIsLocal = true;
        this.selectedGoal = null;
        this.goalSelectionError = null;
        this.navigationGoal.textContent = 'B: —';
        this.navigationDistance.textContent = '—';
        this.clearNavigationOverlay();
        this.navigationStateElement.textContent = 'Chọn đích';
      } else {
        this.goalSelectionIsLocal = false;
      }
      this.updateNavigationButtons();
    });
    this.scene.renderer.domElement.addEventListener('click', (event) => {
      if (!this.selectingGoal) return;
      const point = this.groundPoint(event);
      if (!point) return;
      this.validateSelectedGoal(point);
    });
    this.goButton.addEventListener('click', () => this.go());
    this.stopNavigationButton.addEventListener('click', () => this.stopNavigation());
    document.getElementById('reset-navigation-button').addEventListener(
      'click', () => this.resetSimulation()
    );
  }

  async validateSelectedGoal(point) {
    const sequence = ++this.goalValidationSequence;
    this.navigationGoal.textContent = `B: ${point.x.toFixed(2)}, ${point.y.toFixed(2)}`;
    this.setGoalMarker(point, 0xffd447);
    this.navigationStateElement.textContent = 'Đang kiểm tra đích';
    try {
      const result = await this.request('/api/navigation/validate-goal', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({goal: point})
      });
      if (sequence !== this.goalValidationSequence) return;
      if (!result.valid) {
        this.selectedGoal = null;
        this.goalSelectionError = result.message;
        this.navigationStateElement.textContent = result.message;
        this.setGoalMarker(point, 0xdf5b57);
        this.updateNavigationButtons();
        return;
      }
      this.selectedGoal = point;
      this.goalSelectionError = null;
      this.selectingGoal = false;
      this.selectGoalButton.classList.remove('active');
      this.viewer.classList.remove('selecting-goal');
      this.navigationStateElement.textContent = 'Sẵn sàng chạy';
      this.updateNavigationButtons();
    } catch (error) {
      if (sequence !== this.goalValidationSequence) return;
      this.selectedGoal = null;
      this.goalSelectionError = error.message;
      this.navigationStateElement.textContent = error.message;
      this.setGoalMarker(point, 0xdf5b57);
      this.updateNavigationButtons();
    }
  }

  groundPoint(event) {
    const rect = this.scene.renderer.domElement.getBoundingClientRect();
    const vector = new THREE.Vector3(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
      0.5
    );
    vector.unproject(this.scene.camera);
    const direction = vector.sub(this.scene.camera.position).normalize();
    if (Math.abs(direction.z) < 1e-8) return null;
    const amount = -this.scene.camera.position.z / direction.z;
    if (amount <= 0) return null;
    const point = this.scene.camera.position.clone().add(
      direction.multiplyScalar(amount)
    );
    return {x: point.x, y: point.y};
  }

  async request(path, options = {}) {
    const response = await fetch(path, options);
    const result = await response.json();
    if (!response.ok || result.ok === false) {
      throw new Error(result.error || 'request failed');
    }
    return result;
  }

  async go() {
    if (!this.selectedGoal) return;
    const goal = {...this.selectedGoal};
    this.goButton.disabled = true;
    this.navigationStateElement.textContent = 'Đang lập tuyến';
    try {
      const planned = await this.request('/api/navigation/plan', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({goal})
      });
      this.goalSelectionIsLocal = false;
      this.updateNavigation(planned);
      const result = await this.request('/api/navigation/start', {method: 'POST'});
      this.updateNavigation(result);
    } catch (error) {
      this.navigationStateElement.textContent = error.message;
      this.setGoalMarker(this.selectedGoal, 0xdf5b57);
    }
    this.updateNavigationButtons();
  }

  async stopNavigation() {
    try {
      const result = await this.request('/api/navigation/cancel', {method: 'POST'});
      this.updateNavigation(result);
    } catch (error) {
      this.navigationStateElement.textContent = error.message;
    }
  }

  updateKeyboardCommand() {
    const speed = this.manualLinearSpeed();
    const held = (wasd, arrow) => this.pressedKeys.has(wasd) || this.pressedKeys.has(arrow);
    const linear = (held('KeyW', 'ArrowUp') ? speed : 0)
      + (held('KeyS', 'ArrowDown') ? -speed : 0);
    const angular = (held('KeyA', 'ArrowLeft') ? 1.5 : 0)
      + (held('KeyD', 'ArrowRight') ? -1.5 : 0);
    this.setCommand(linear, manualAngularSpeed(angular));
  }

  setCommand(linear, angular) {
    this.command = {linear, angular};
    this.commandStatus.textContent =
      `Tiến ${linear.toFixed(2)} · Quay ${angular.toFixed(2)}`;
    this.sendCommand(linear, angular);
    if (this.commandHeartbeat) window.clearInterval(this.commandHeartbeat);
    this.commandHeartbeat = null;
    if (linear !== 0 || angular !== 0) {
      this.commandHeartbeat = window.setInterval(
        () => this.sendCommand(this.command.linear, this.command.angular),
        100
      );
    }
  }

  stopManual() {
    this.pressedKeys.clear();
    this.setCommand(0, 0);
  }

  sendCommand(linear, angular) {
    fetch('/api/amr/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({linear, angular}),
      keepalive: true
    }).catch(() => this.setConnectionState('Mất kết nối điều khiển', 'error'));
  }

  async resetSimulation() {
    this.stopManual();
    try {
      const result = await this.request('/api/amr/reset', {method: 'POST'});
      if (!result.ok) throw new Error('reset failed');
      this.poseTargets.clear();
      this.selectedGoal = null;
      this.goalSelectionIsLocal = false;
      this.clearNavigationOverlay();
      this.navigationGoal.textContent = 'B: —';
      this.navigationDistance.textContent = '—';
      this.navigationStateElement.textContent = 'Chờ';
      this.navigationState = 'IDLE';
      this.updateNavigationButtons();
    } catch (error) {
      this.setConnectionState(`Reset failed: ${error.message}`, 'error');
    }
  }

  async pollTelemetry() {
    try {
      const response = await fetch('/api/amr/telemetry', {cache: 'no-store'});
      if (!response.ok) throw new Error('Chưa nhận dữ liệu cảm biến');
      const telemetry = await response.json();
      const freshness = telemetry.freshness || {};
      const sensorStates = Object.values(freshness);
      const freshCount = sensorStates
        .filter((item) => item.fresh).length;
      this.sensorStatus.textContent =
        `Cảm biến ${freshCount}/${sensorStates.length}`;
      const modelPose = telemetry.sensors && telemetry.sensors.model_pose;
      if (modelPose) {
        const x = Number(modelPose.position.x || 0);
        const y = Number(modelPose.position.y || 0);
        const yaw = Number(modelPose.yaw || 0) * 180 / Math.PI;
        this.poseStatus.textContent =
          `A: x ${x.toFixed(2)} · y ${y.toFixed(2)} · yaw ${yaw.toFixed(1)}°`;
      }
      this.updateNavigationButtons();
    } catch (error) {
      this.sensorStatus.textContent = error.message;
      this.updateNavigationButtons();
    }
  }

  async pollNavigation() {
    try {
      const response = await fetch('/api/navigation/status', {cache: 'no-store'});
      if (!response.ok) throw new Error('Chưa kết nối điều hướng');
      this.updateNavigation(await response.json());
    } catch (error) {
      this.navigationStateElement.textContent = error.message;
    }
  }

  updateNavigation(status) {
    if (status.speed_mps != null) this.updateSpeedControl(status.speed_mps);
    this.navigationState = status.state || 'IDLE';
    if (this.goalSelectionIsLocal) {
      this.navigationStateElement.textContent = this.goalSelectionError
        || (this.selectedGoal && !this.selectingGoal
          ? 'Sẵn sàng chạy'
          : 'Chọn đích');
    } else {
      this.navigationStateElement.textContent = status.error || navigationLabels[this.navigationState]
        || status.message || this.navigationState;
      this.navigationStateElement.title = status.message || '';
    }
    if (status.goal && !this.goalSelectionIsLocal) {
      this.selectedGoal = status.goal;
      this.navigationGoal.textContent =
        `B: ${status.goal.x.toFixed(2)}, ${status.goal.y.toFixed(2)}`;
      const color = this.navigationState === 'ERROR' ? 0xdf5b57 : 0xffd447;
      this.setGoalMarker(status.goal, color);
    }
    if (!this.goalSelectionIsLocal) {
      this.navigationDistance.textContent = status.distance_to_goal_m == null
        ? '—'
        : `${status.distance_to_goal_m.toFixed(2)} m`;
      this.setLine('pathObject', status.global_path || [], 0x3478c9, 0.035);
    }
    this.updateNavigationButtons();
  }

  updateNavigationButtons() {
    const active = ['PLANNING', 'READY', 'NAVIGATING', 'RESETTING']
      .includes(this.navigationState);
    // Sensor readiness is enforced by the planning API. Keeping it out of the
    // button state avoids a stale telemetry count permanently disabling Go.
    this.goButton.disabled = !this.selectedGoal || active;
    this.stopNavigationButton.disabled = !active;
    this.selectGoalButton.disabled = active;
  }

  setGoalMarker(point, color) {
    if (this.goalObject) this.scene.remove(this.goalObject);
    const geometry = new THREE.SphereGeometry(0.045, 16, 10);
    const material = new THREE.MeshBasicMaterial({color});
    this.goalObject = new THREE.Mesh(geometry, material);
    this.goalObject.position.set(point.x, point.y, 0.045);
    this.scene.add(this.goalObject);
  }

  setLine(property, points, color, z) {
    if (this[property]) this.scene.remove(this[property]);
    this[property] = null;
    if (points.length < 2) return;
    const geometry = new THREE.Geometry();
    points.forEach((point) => {
      geometry.vertices.push(new THREE.Vector3(point.x, point.y, z));
    });
    this[property] = new THREE.Line(
      geometry,
      new THREE.LineBasicMaterial({color, linewidth: 2})
    );
    this.scene.add(this[property]);
  }

  clearNavigationOverlay() {
    ['pathObject', 'trailObject', 'goalObject'].forEach((property) => {
      if (this[property]) this.scene.remove(this[property]);
      this[property] = null;
    });
  }

  connect(loadScene) {
    const gz = new Gazebo({url: this.gzUrl, key: ''});
    this.gz = gz;
    gz.on('connection', () => {
      if (this.gz !== gz) return;
      if (this.reconnectTimer) window.clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
      this.setConnectionState('Đã kết nối', 'connected');
    });
    gz.on('error', () => {
      if (this.gz !== gz) return;
      this.setConnectionState('Lỗi kết nối Gazebo', 'error');
      this.scheduleReconnect();
    });
    gz.on('close', () => {
      if (this.gz !== gz) return;
      this.setConnectionState('Đang kết nối lại', 'pending');
      this.scheduleReconnect();
    });
    if (loadScene) {
      gz.on('worlds', () => {
        if (this.gz !== gz) return;
        gz.once('scene', (scene) => this.loadScene(scene));
        gz.socket.send(buildMsg(['scene', this.world, '', '']));
      });
    }
    new Topic({
      gz,
      name: `/world/${this.world}/dynamic_pose/info`,
      messageType: 'gz.msgs.Pose_V',
      callback: (message) => {
        if (this.gz === gz) this.updatePoses(message);
      }
    });
  }

  scheduleReconnect() {
    if (this.reconnectTimer) return;
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect(!this.sceneLoaded);
    }, 1000);
  }

  loadScene(scene) {
    (scene.model || []).forEach((model) => {
      this.scene.add(this.createModel(model));
    });
    this.sceneLoaded = true;
    this.wheelViewButton.disabled = !this.objects.has('amr');
    this.setConnectionState(
      `Đã kết nối · ${(scene.model || []).length} mô hình`,
      'connected'
    );
  }

  createModel(model) {
    const object = new THREE.Object3D();
    object.name = model.name;
    this.remember(model.name, object);
    this.setPose(object, model.pose);
    (model.link || []).forEach((link) => {
      const linkObject = new THREE.Object3D();
      linkObject.name = link.name;
      this.remember(link.name, linkObject);
      this.remember(`${model.name}::${link.name}`, linkObject);
      this.setPose(linkObject, link.pose);
      (link.visual || []).forEach((visual) => {
        const visualObject = this.createVisual(visual);
        if (visualObject) linkObject.add(visualObject);
      });
      object.add(linkObject);
    });
    (model.model || []).forEach((nested) => {
      object.add(this.createModel(nested));
    });
    return object;
  }

  createVisual(visual) {
    const object = new THREE.Object3D();
    this.setPose(object, visual.pose);
    const geometry = visual.geometry || {};
    let mesh = null;
    if (geometry.box) {
      const size = geometry.box.size;
      mesh = this.scene.createBox(size.x, size.y, size.z);
    } else if (geometry.sphere) {
      mesh = this.scene.createSphere(geometry.sphere.radius);
    } else if (geometry.cylinder) {
      mesh = this.scene.createCylinder(
        geometry.cylinder.radius,
        geometry.cylinder.length
      );
    } else if (geometry.plane) {
      const plane = geometry.plane;
      mesh = this.scene.createPlane(
        plane.normal.x,
        plane.normal.y,
        plane.normal.z,
        plane.size.x,
        plane.size.y
      );
    } else if (geometry.mesh) {
      const meshInfo = geometry.mesh;
      const uri = meshInfo.filename || meshInfo.uri || '';
      const assetUrl = this.resolveMeshUri(uri);
      if (!assetUrl) return null;
      this.scene.loadMeshFromUri(
        assetUrl,
        meshInfo.submesh,
        meshInfo.center_submesh,
        (loaded) => {
          if (!loaded) return;
          const scale = meshInfo.scale;
          if (scale) loaded.scale.set(scale.x, scale.y, scale.z);
          this.applyMaterialTree(loaded, visual.material);
          object.add(loaded);
        }
      );
      return object;
    }
    if (!mesh) return null;
    this.applyMaterial(mesh, visual.material);
    mesh.castShadow = !geometry.plane;
    mesh.receiveShadow = true;
    object.add(mesh);
    return object;
  }

  resolveMeshUri(uri) {
    if (!uri) return null;
    if (/^https?:\/\//.test(uri)) return uri;
    const marker = '/models/';
    if (uri.startsWith('model://')) return `/assets/models/${uri.slice(8)}`;
    const markerIndex = uri.indexOf(marker);
    if (markerIndex >= 0) {
      return `/assets/models/${uri.slice(markerIndex + marker.length)}`;
    }
    return `/assets/models/${uri.replace(/^file:\/\//, '').replace(/^\/+/, '')}`;
  }

  applyMaterialTree(object, material) {
    if (object instanceof THREE.Mesh) this.applyMaterial(object, material);
    const descendants = [];
    if (object.getDescendants) object.getDescendants(descendants);
    descendants.forEach((child) => {
      if (child instanceof THREE.Mesh) this.applyMaterial(child, material);
    });
  }

  applyMaterial(mesh, material) {
    const color = (value) => {
      if (!value) return null;
      if (Array.isArray(value)) return value;
      if (typeof value.r === 'number') {
        return [value.r, value.g, value.b, value.a ?? 1];
      }
      return null;
    };
    const diffuse = color(material && material.diffuse);
    if (!diffuse) return;
    this.scene.setMaterial(mesh, {
      ambient: color(material.ambient),
      diffuse,
      specular: color(material.specular),
      opacity: diffuse[3]
    });
  }

  updatePoses(message) {
    (message.pose || []).forEach((pose) => {
      const object = this.objects.get(pose.name)
        || this.objects.get(this.rootName(pose.name));
      if (!object || !pose.position || !pose.orientation) return;
      this.poseTargets.set(object, {
        position: new THREE.Vector3(
          pose.position.x,
          pose.position.y,
          pose.position.z
        ),
        orientation: new THREE.Quaternion(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w
        )
      });
    });
  }

  remember(name, object) {
    if (name) this.objects.set(name, object);
  }

  rootName(name) {
    return String(name || '').split('::')[0];
  }

  setPose(object, pose) {
    if (pose) this.scene.setPose(object, pose.position, pose.orientation);
  }

  setCamera() {
    const {camera, controls} = this.scene;
    if (document.body.dataset.mapDefault === 'open') camera.position.set(-6.4, -8.2, 8.7);
    else camera.position.set(-3.6, -4.8, 3.6);
    controls.target.set(0, 0, .15);
    camera.lookAt(controls.target);
    controls.update();
    this.overviewCamera = null;
    this.wheelViewButton.setAttribute('aria-pressed', 'false');
    this.wheelViewButton.textContent = 'Bánh xe';
  }

  toggleWheelView() {
    const robot = this.objects.get('amr');
    if (!robot) return;
    const controls = this.scene.controls;
    const camera = this.scene.camera;
    if (this.overviewCamera) {
      camera.position.copy(this.overviewCamera.position);
      controls.target.copy(this.overviewCamera.target);
      this.overviewCamera = null;
    } else {
      this.overviewCamera = {position: camera.position.clone(), target: controls.target.clone()};
      const offset = new THREE.Vector3(0.12, -0.48, 0.12).applyQuaternion(robot.quaternion);
      camera.position.copy(robot.position).add(offset);
      controls.target.copy(robot.position).add(new THREE.Vector3(-0.02, 0, 0.035));
    }
    camera.lookAt(controls.target);
    controls.update();
    this.wheelViewButton.setAttribute('aria-pressed', String(Boolean(this.overviewCamera)));
    this.wheelViewButton.textContent = this.overviewCamera ? 'Rời góc bánh xe' : 'Bánh xe';
  }

  resize() {
    this.scene.setSize(this.viewer.clientWidth, this.viewer.clientHeight);
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    const now = performance.now();
    const elapsed = Math.min(100, now - this.lastAnimationTime);
    this.lastAnimationTime = now;
    const alpha = 1 - Math.exp(-elapsed / 40);
    this.poseTargets.forEach((target, object) => {
      object.position.lerp(target.position, alpha);
      object.quaternion.slerp(target.orientation, alpha);
    });
    this.scene.render();
  }

  setConnectionState(message, state) {
    this.connectionStatus.textContent = message;
    this.connectionStatus.className = `status ${state}`;
  }
}

window.addEventListener('load', () => new AMRViewer().start());
