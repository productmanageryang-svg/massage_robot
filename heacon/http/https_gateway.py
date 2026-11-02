"""HTTPS front door so the tablet can use its microphone. Proxies the console.

The browser only grants microphone access on a secure origin, so the tablet
talks to 8766 and this process relays to the plain HTTP console on 8765.
"""

from __future__ import annotations

import ssl
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import heacon.config as config
from heacon.voice import stt

UPSTREAM = "http://127.0.0.1:%d" % config.HTTP_PORT
HOST = config.HTTPS_HOST
PORT = config.HTTPS_PORT
CERT = config.TLS_CERT
KEY = config.TLS_KEY
PWA = config.WEB_DIR / "pwa"
SKIP = {"host", "content-length", "transfer-encoding", "connection", "keep-alive"}
PWA_FILES = {
    "/manifest.webmanifest": (PWA / "manifest.webmanifest", "application/manifest+json"),
    "/sw.js": (PWA / "sw.js", "text/javascript; charset=utf-8"),
    "/icon-192.png": (PWA / "icon-192.png", "image/png"),
    "/icon-512.png": (PWA / "icon-512.png", "image/png"),
}


class Gateway(ThreadingHTTPServer):
    """Accept stays free if a tablet TLS handshake hangs."""

    tls = None

    def get_request(self):
        sock, addr = self.socket.accept()
        sock.settimeout(8)
        try:
            ssl_sock = self.tls.wrap_socket(sock, server_side=True)
        except ssl.SSLError:
            try:
                sock.close()
            except OSError:
                pass
            raise
        ssl_sock.settimeout(30)
        return ssl_sock, addr


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in PWA_FILES:
            self._pwa(path, body=True)
            return
        if self._quiet_asset(path, body=True):
            return
        self._proxy()

    def do_HEAD(self):
        path = self.path.split("?", 1)[0]
        if path in PWA_FILES:
            self._pwa(path, body=False)
            return
        if self._quiet_asset(path, body=False):
            return
        self._proxy()

    def _quiet_asset(self, path, body=True):
        if path == "/favicon.ico":
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            return True
        if path.startswith("/iconify-offline/"):
            prefix = path.rsplit("/", 1)[-1].split(".", 1)[0] or "solar"
            data = json_bytes({"prefix": prefix, "lastModified": 1, "icons": {}})
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            if body:
                self.wfile.write(data)
            return True
        return False

    def _pwa(self, path, body=True):
        file, mime = PWA_FILES[path]
        data = file.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if body:
            self.wfile.write(data)

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/voice/hear":
            self._hear()
            return
        self._proxy()

    def _hear(self):
        length = int(self.headers.get("Content-Length", "0") or 0)
        blob = self.rfile.read(length) if length else b""
        try:
            text = stt.transcribe_wav(blob) if blob else ""
            payload = json_bytes({"ok": True, "text": text})
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
        except Exception as exc:
            payload = json_bytes({"ok": False, "text": "", "message": str(exc)})
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

    def _proxy(self):
        url = UPSTREAM + self.path
        length = int(self.headers.get("Content-Length", "0") or 0)
        body = self.rfile.read(length) if length and self.command == "POST" else None
        req = urllib.request.Request(url, data=body, method=self.command)
        req.add_header("Host", "127.0.0.1:8765")
        for key, value in self.headers.items():
            if key.lower() in SKIP or key.lower() == "host":
                continue
            req.add_header(key, value)
        try:
            with urllib.request.urlopen(req, timeout=20) as resp:
                data = resp.read()
                self.send_response(resp.status)
                for key, value in resp.headers.items():
                    if key.lower() not in SKIP:
                        self.send_header(key, value)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
        except urllib.error.HTTPError as exc:
            data = exc.read()
            self.send_response(exc.code)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except Exception as exc:
            msg = str(exc).encode("utf-8")
            self.send_response(502)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(msg)))
            self.end_headers()
            self.wfile.write(msg)

    def log_message(self, fmt, *args):
        path = str(args[0]) if args else ""
        if path.startswith("GET /api/"):
            return
        print(f"[heacon-https] {self.client_address[0]} {path}", flush=True)
        super().log_message(fmt, *args)


def json_bytes(payload):
    import json

    return json.dumps(payload, ensure_ascii=False).encode("utf-8")


def run():
    """Start the tablet door. Needs a certificate; see scripts/gen-cert.sh."""
    config.ensure_dirs()
    if not CERT.is_file() or not KEY.is_file():
        raise SystemExit(
            "缺少平板用的 HTTPS 证书：%s\n"
            "运行 scripts/gen-cert.sh 生成，或用 HEACON_RUNTIME_DIR 指向已有证书的目录。"
            % CERT
        )
    print("[heacon] https config", config.describe(), flush=True)
    print("loading voice model", flush=True)
    stt._load_vosk()
    print("voice model ready", flush=True)
    Gateway.tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    Gateway.tls.load_cert_chain(str(CERT), str(KEY))
    server = Gateway((HOST, PORT), Handler)
    print(f"HEACON 平板入口：https://0.0.0.0:{PORT}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")


if __name__ == "__main__":
    run()
