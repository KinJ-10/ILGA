#!/usr/bin/env python3
"""Loopback-only server and offline entry point for the ILGA 10m demo."""
from __future__ import annotations

import argparse
import json
import mimetypes
import os
import secrets
import subprocess
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PureWindowsPath, PurePosixPath
from urllib.parse import unquote, urlsplit

from capture import Session
from reporting import offline_run

HERE = Path(__file__).resolve().parent
DOWNLOADS = {"sensor.csv", "markers.csv", "metadata.json", "analysis.json", "report.html", "waveform.png"}


def windows_folder(path, platform=None, distro=None):
    """Represent a trusted absolute directory as an Explorer-compatible path."""
    if (platform or os.name) == "nt":
        return str(PureWindowsPath(str(path)))
    path = PurePosixPath(path)
    if not path.is_absolute():
        raise ValueError("保存先の絶対パスを確認できません。")
    parts = path.parts
    if len(parts) >= 3 and parts[1] == "mnt" and len(parts[2]) == 1 and parts[2].isalpha():
        return str(PureWindowsPath(parts[2].upper() + ":/", *parts[3:]))
    distro = distro or os.environ.get("WSL_DISTRO_NAME", "Ubuntu")
    if not distro or any(c in distro for c in "/\\"):
        raise ValueError("WSLの保存先を確認できません。")
    return str(PureWindowsPath("//wsl.localhost/" + distro, *parts[1:]))


def current_folder(session, run_id):
    """Never construct a path from caller input. Reject stale IDs and escaped roots."""
    session.require_run(run_id)
    root = (HERE / "runs").resolve()
    if session.root.resolve() != root:
        raise ValueError("この保存先はフォルダー表示の対象外です。")
    candidate = session.run.resolve()
    if candidate.parent != root or candidate.name != session.run.name or session.run.is_symlink() or not candidate.is_dir():
        raise ValueError("現在の測定の保存フォルダーが見つかりません。")
    return candidate


def launch_explorer(path):
    executable = str(Path(os.environ.get("SystemRoot", "C:/Windows")) / "explorer.exe") if os.name == "nt" else "/mnt/c/Windows/explorer.exe"
    subprocess.Popen([executable, windows_folder(path)], shell=False)


def status_payload(session):
    with session.lock:
        payload = session.snapshot()
        payload["save_path"] = None
        if session.run:
            try:
                payload["save_path"] = windows_folder(current_folder(session, session.run.name))
            except ValueError:
                pass
        return payload


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, port=8765, root=None, session=None):
        super().__init__(("127.0.0.1", port), Handler)
        self.session = session or Session(root or HERE / "runs")
        self.token = secrets.token_urlsafe(32)
        self.authority = f"127.0.0.1:{self.server_port}"
        self.origin = "http://" + self.authority


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def log_message(self, *_):
        pass

    def reply(self, code, body, mime="application/json; charset=utf-8"):
        if isinstance(body, dict):
            body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; object-src 'none'; frame-ancestors 'none'; base-uri 'none'")
        self.end_headers()
        self.wfile.write(body)

    def trusted_host(self):
        return self.headers.get("Host") == self.server.authority

    def do_GET(self):
        if not self.trusted_host():
            return self.reply(403, {"error": "このアドレスからは操作できません。起動時のURLを開いてください。"})
        path = unquote(urlsplit(self.path).path)
        if path == "/api/status":
            return self.reply(200, status_payload(self.server.session))
        if path == "/api/session":
            return self.reply(200, {"token": self.server.token})
        static = {"/": "index.html", "/app.js": "app.js", "/style.css": "style.css"}
        target = HERE / static[path] if path in static else None
        if path.startswith("/runs/"):
            parts = path.split("/")
            if len(parts) == 4 and parts[3] in DOWNLOADS:
                root = self.server.session.root.resolve()
                candidate = (root / parts[2] / parts[3]).resolve()
                if candidate.parent.parent == root:
                    target = candidate
        if target is None or not target.is_file():
            return self.reply(404, {"error": "指定された保存物が見つかりません。"})
        mime = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        if mime.startswith("text/") or mime == "application/json":
            mime += "; charset=utf-8"
        self.reply(200, target.read_bytes(), mime)

    def do_POST(self):
        if (not self.trusted_host()
                or self.headers.get("Origin") not in (None, self.server.origin)
                or not secrets.compare_digest(self.headers.get("X-ILGA-Token", ""), self.server.token)):
            return self.reply(403, {"error": "操作の確認ができません。ページを再読み込みしてください。"})
        try:
            if self.headers.get("Content-Type") != "application/json":
                raise ValueError("送信形式を確認してください。ページを再読み込みしてください。")
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 8192:
                raise ValueError("入力が大きすぎるか空です。")
            data = json.loads(self.rfile.read(size))
            if not isinstance(data, dict):
                raise ValueError("入力形式を確認してください。")
            session = self.server.session
            if self.path == "/api/start":
                session.start(data)
            elif self.path == "/api/marker":
                session.mark(data.get("run_id"), data.get("event"))
            elif self.path == "/api/stop":
                session.stop(data.get("run_id"))
            elif self.path == "/api/observations":
                session.update_observations(data.get("run_id"), data)
            elif self.path == "/api/open-folder":
                if set(data) != {"run_id"}:
                    raise ValueError("保存フォルダーは現在の測定から選択してください。")
                with session.lock:
                    folder = current_folder(session, data.get("run_id"))
                    try:
                        launch_explorer(folder)
                    except OSError as exc:
                        raise ValueError("保存フォルダーを開けませんでした。Windows Explorerの起動状態を確認してください。") from exc
                    return self.reply(200, {**status_payload(session), "message": "保存フォルダーを開く操作をWindowsへ送りました。"})
            else:
                return self.reply(404, {"error": "不明な操作です。"})
            self.reply(200, status_payload(session))
        except (ValueError, TypeError, KeyError):
            import sys
            error = sys.exception()
            message = str(error) if type(error) is ValueError else "入力内容を確認してください。"
            if isinstance(error, json.JSONDecodeError):
                message = "入力内容を読み取れません。ページを再読み込みしてください。"
            self.reply(400, {"error": message})
        except OSError:
            self.reply(500, {"error": "保存先に書き込めません。空き容量とアクセス権を確認してください。"})


def main(argv=None):
    parser = argparse.ArgumentParser(description="ILGA 10m歩行デモ")
    parser.add_argument("--offline-test", nargs=3, metavar=("SENSOR", "MARKERS", "METADATA"))
    parser.add_argument("--output-root", type=Path, default=HERE / "runs")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--serve", action="store_true", help="互換用。引数なしでもWeb UIを起動します")
    args = parser.parse_args(argv)
    if args.offline_test:
        run = offline_run(args.output_root, *(Path(v) for v in args.offline_test))
        print(run)
        summary = json.loads((run / "analysis.json").read_text(encoding="utf-8"))
        return 2 if summary["demo"]["quality"] == "計算不可" else 0
    try:
        server = Server(args.port, args.output_root)
    except OSError:
        print("起動できません。同じアプリが起動済みの場合はその画面を使用するか、--portで別の番号を指定してください。")
        return 1
    print(f"ILGA 歩行デモ: {server.origin}", flush=True)
    if not args.no_browser:
        webbrowser.open(server.origin)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.session.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
