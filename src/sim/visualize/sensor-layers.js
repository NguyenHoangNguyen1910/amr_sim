"use strict";

const mapSensorLayers = (() => {
  const controls = document.getElementById("map-sensor-layers");
  const inputs = Array.from(controls.querySelectorAll("input"));
  const colors = {lidar: "#25b9a6", laser: "#e08218", camera: "#b645e8"};

  controls.hidden = document.body.dataset.mapDefault !== "open";
  document.getElementById("slam-scan-key").hidden = !controls.hidden;

  function query() {
    if (controls.hidden) return "";
    return "layers=" + inputs.filter(input => input.checked).map(input => input.value).join(",");
  }

  function draw(context, layers, px, py, slam = false) {
    for (const {checked, value} of inputs) {
      const scan = layers[value];
      if (!checked || !scan) continue;

      const rays = slam && value === "lidar";
      const startX = px(scan.x);
      const startY = py(scan.y);
      const startAngle = scan.yaw + scan.angle_min;
      const size = rays ? 1.4 : value === "lidar" ? 2 : 3;
      context.fillStyle = colors[value];
      if (rays) {
        context.strokeStyle = "rgba(25, 208, 182, .07)";
        context.lineWidth = 0.65;
        context.beginPath();
      }

      scan.ranges.forEach((distance, beam) => {
        if (distance == null) return;
        const angle = startAngle + beam * scan.angle_step;
        const x = px(scan.x + distance * Math.cos(angle));
        const y = py(scan.y + distance * Math.sin(angle));
        if (rays) {
          context.moveTo(startX, startY);
          context.lineTo(x, y);
        }
        context.fillRect(x - size / 2, y - size / 2, size, size);
      });
      if (rays) context.stroke();
    }
  }

  return {controls, query, draw};
})();
