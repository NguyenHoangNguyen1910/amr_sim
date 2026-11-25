"""Serve a read-only preview of the Pi–ESP deployment on localhost."""

from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
from urllib.parse import urlsplit


VIEWER = Path(__file__).resolve().parents[1] / "sim" / "visualize"
NOTICE = """
<section id="pi-esp-readiness" style="position:absolute;z-index:10;left:50%;top:70px;
transform:translateX(-50%);width:min(500px,85vw);padding:16px 20px;
border:1px solid #4e647b;border-radius:8px;background:#172331;color:#e8edf2">
  <strong>Reeman · Pi–ESP · cổng 8020</strong>
  <p>Giao diện xem trước. Chưa kết nối Pi, ESP và driver cảm biến gốc.</p>
  <p>Lệnh điều khiển, odometry, cảm biến và SLAM sẽ khả dụng khi có backend thật.</p>
</section>
"""
IDLE = {"state": "IDLE", "message": "Chờ kết nối Pi–ESP"}


class Preview(SimpleHTTPRequestHandler):
    def send_payload(self, payload, status=200, content_type="application/json"):
        if isinstance(payload, dict):
            payload = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            page = (VIEWER / "index.html").read_text(encoding="utf-8")
            page = re.sub(r'<script src="[^"]+"></script>', "", page)
            page = page.replace("</body>", NOTICE + "</body>")
            page = page.replace('>Connecting</span>', '>Chờ backend Reeman</span>')
            page = page.replace('>A: x -2.00 · y -0.50 · yaw 0.0°</span>', '>Vị trí chưa có dữ liệu</span>')
            page = page.replace(">IDLE</strong>", ">Chờ kết nối</strong>")
            page = page.replace('<button ', '<button disabled ')
            self.send_payload(page.encode(), content_type="text/html; charset=utf-8")
        elif path == "/api/version":
            self.send_payload({"backend": "pi-esp-preview", "ready": False})
        elif path == "/api/health":
            self.send_payload({"ok": True, "ready": False, "navigation": IDLE})
        elif path.startswith("/api/"):
            self.send_payload({"error": "Chưa kết nối backend Reeman Pi–ESP"}, status=503)
        else:
            super().do_GET()

    def do_POST(self):
        self.send_payload({"error": "Chưa kết nối backend Reeman Pi–ESP"}, status=503)


if __name__ == "__main__":
    handler = partial(Preview, directory=str(VIEWER))
    with ThreadingHTTPServer(("127.0.0.1", 8020), handler) as server:
        server.serve_forever()
