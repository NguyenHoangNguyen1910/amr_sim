"""Expose the sandboxed ROS dashboard on a local TCP port."""

import select
import socket
import socketserver
import sys


class Relay(socketserver.BaseRequestHandler):
    def handle(self):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as upstream:
            try:
                upstream.connect(self.server.socket_path)
            except OSError:
                self.request.sendall(
                    b"HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n"
                    b"Content-Type: text/plain; charset=utf-8\r\nContent-Length: 21\r\n\r\n"
                    b"Starting Reeman ROS.\n"
                )
                return
            while True:
                readable, _, _ = select.select([self.request, upstream], [], [], 15)
                if not readable:
                    return
                for source in readable:
                    data = source.recv(65536)
                    if not data:
                        return
                    target = upstream if source is self.request else self.request
                    target.sendall(data)


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    bind_address = sys.argv[3] if len(sys.argv) > 3 else "127.0.0.1"
    with Server((bind_address, int(sys.argv[2])), Relay) as server:
        server.socket_path = sys.argv[1]
        server.serve_forever()
