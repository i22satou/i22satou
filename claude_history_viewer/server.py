"""Claude Code 作業履歴ビューアのローカルサーバー(Python 標準ライブラリのみ)。

使い方:
    python server.py                 # http://127.0.0.1:8765 を開く
    python server.py --port 9000 --claude-dir ~/.claude

- 履歴ファイルを一定間隔で差分読み込みし、変化したセッションを Server-Sent Events(/api/events)で
  ブラウザへ送る。ブラウザ側は再読み込みせずに表示を更新する。
- 履歴には会話内容が含まれるため、既定では 127.0.0.1 だけで待ち受ける。
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from history_parser import HistoryStore

STATIC_DIR = Path(__file__).resolve().parent / "static"


class Broadcaster:
    """SSE の購読者ごとにキューを持ち、更新を配る。"""

    def __init__(self):
        self.clients: set[queue.Queue] = set()
        self.lock = threading.Lock()

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=100)
        with self.lock:
            self.clients.add(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.clients.discard(q)

    def publish(self, event: str, payload: dict):
        data = f"event: {event}\ndata: {json.dumps(payload, ensure_ascii=False)}\n\n".encode("utf-8")
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(data)
                except queue.Full:
                    self.clients.discard(q)  # 受け取れていない購読者は切る(ブラウザが自動で再接続する)


def watch_loop(store: HistoryStore, bus: Broadcaster, interval: float):
    """履歴ファイルとプロセス状態を監視し、表示内容が変わったセッションだけを配信する。"""
    last_sent: dict[str, str] = {}
    while True:
        try:
            store.scan()
            store.scan_live()
            changed = []
            for s in store.all_summaries():
                key = json.dumps(s, sort_keys=True, ensure_ascii=False)
                if last_sent.get(s["id"]) != key:
                    last_sent[s["id"]] = key
                    changed.append(s)
            if changed:
                bus.publish("update", {"sessions": changed, "serverTime": time.time()})
        except Exception as exc:  # 監視は止めない
            print(f"[watch] エラー: {exc!r}")
        time.sleep(interval)


def make_handler(store: HistoryStore, bus: Broadcaster):
    class Handler(BaseHTTPRequestHandler):
        server_version = "ClaudeHistoryViewer/1.0"

        def log_message(self, fmt, *args):  # アクセスログは出さない
            pass

        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

        def do_GET(self):
            url = urlparse(self.path)
            path = unquote(url.path)
            if path in ("/", "/index.html"):
                return self._static("index.html")
            if path.startswith("/static/"):
                return self._static(path[len("/static/"):])
            if path == "/api/state":
                return self._json(
                    {"sessions": store.all_summaries(), "serverTime": time.time(), "projectsDir": str(store.projects_dir)}
                )
            if path.startswith("/api/session/") and path.endswith("/log"):
                sid = path[len("/api/session/"): -len("/log")]
                limit = int(parse_qs(url.query).get("limit", ["400"])[0])
                return self._json({"id": sid, "events": store.session_log(sid, limit)})
            if path == "/api/events":
                return self._sse()
            self._send(404, b"not found", "text/plain; charset=utf-8")

        def _static(self, rel):
            target = (STATIC_DIR / rel).resolve()
            if STATIC_DIR not in target.parents or not target.is_file():
                return self._send(404, b"not found", "text/plain; charset=utf-8")
            ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
            if ctype.startswith("text/") or ctype in ("application/javascript",):
                ctype += "; charset=utf-8"
            self._send(200, target.read_bytes(), ctype)

        def _sse(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            q = bus.subscribe()
            try:
                self.wfile.write(b"retry: 3000\nevent: hello\ndata: {}\n\n")
                self.wfile.flush()
                while True:
                    try:
                        data = q.get(timeout=15)
                    except queue.Empty:
                        data = b": keep-alive\n\n"
                    self.wfile.write(data)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                bus.unsubscribe(q)

    return Handler


def main():
    ap = argparse.ArgumentParser(description="Claude Code 作業履歴ビューア")
    ap.add_argument("--host", default="127.0.0.1", help="待ち受けアドレス(既定: 127.0.0.1)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--claude-dir", default=str(Path.home() / ".claude"), help="Claude Code の設定ディレクトリ")
    ap.add_argument("--projects-dir", default=None, help="履歴 jsonl のディレクトリ(既定: <claude-dir>/projects)")
    ap.add_argument("--interval", type=float, default=1.5, help="監視間隔(秒)")
    args = ap.parse_args()

    store = HistoryStore(Path(args.claude_dir).expanduser(), Path(args.projects_dir).expanduser() if args.projects_dir else None)
    store.scan()
    store.scan_live()
    bus = Broadcaster()
    threading.Thread(target=watch_loop, args=(store, bus, args.interval), daemon=True).start()

    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(store, bus))
    httpd.daemon_threads = True
    print(f"履歴: {store.projects_dir}({len(store.sessions)} セッション)")
    print(f"http://{args.host}:{args.port} を開いてください(Ctrl+C で終了)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
