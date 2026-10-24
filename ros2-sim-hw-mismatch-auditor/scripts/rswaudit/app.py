"""A local, zero-dependency web App around the auditor.

Why a built-in server instead of Electron/Tkinter: the project promises to run
anywhere Python runs, with no third-party dependency and no network access. A
ThreadingHTTPServer plus one static HTML file satisfies that, works on Windows,
Linux and macOS, and keeps the whole UI in one auditable file.

Security posture (it is a local dev tool, not a service):
  * binds 127.0.0.1 by default;
  * every request must carry the per-session token that the CLI prints and the
    page stores, which stops other local pages from driving the tool;
  * uploads are limited to an allow-list of extensions and 8 MB each;
  * file names are reduced to a basename and confined to the session directory;
  * the server never executes anything from the uploaded content.
"""

from __future__ import annotations

import base64
import json
import os
import threading
import uuid
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from . import __tool__, __version__

ALLOWED_SUFFIXES = {".urdf", ".xacro", ".xml", ".sdf", ".json", ".stl", ".obj",
                    ".csv", ".txt", ".log", ".yaml", ".yml"}
MAX_UPLOAD = 8 * 1024 * 1024

STATE: dict[str, Any] = {"root": None, "token": "", "jobs": {}}


def _safe_name(name: str) -> str:
    base = Path(str(name or "upload")).name
    cleaned = "".join(ch for ch in base if ch.isalnum() or ch in "._- ")
    cleaned = cleaned.strip().replace(" ", "_")
    return cleaned or "upload.bin"


def _job_dir(session: str) -> Path:
    root: Path = STATE["root"]
    target = root / session
    target.mkdir(parents=True, exist_ok=True)
    return target


class Handler(BaseHTTPRequestHandler):
    server_version = __tool__ + "-app/" + __version__
    protocol_version = "HTTP/1.1"

    # -- plumbing ---------------------------------------------------------- #
    def log_message(self, fmt: str, *args: Any) -> None:   # keep the console clean
        return

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload: dict, code: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send(code, body, "application/json; charset=utf-8")

    def _authorised(self, query: dict) -> bool:
        token = self.headers.get("X-RSW-Token") or (query.get("token") or [""])[0]
        return bool(STATE["token"]) and token == STATE["token"]

    def _session(self, query: dict) -> str:
        raw = (query.get("s") or [""])[0]
        allowed = set("0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_-")
        if raw and len(raw) <= 32 and set(raw) <= allowed:
            return raw
        return "default"

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0 or length > MAX_UPLOAD * 4:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return {}

    # -- routes ------------------------------------------------------------ #
    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        if parsed.path in ("/", "/index.html"):
            page = (Path(__file__).with_name("webui.html")).read_bytes()
            self._send(200, page, "text/html; charset=utf-8")
            return
        if not self._authorised(query):
            self._json({"error": "未授权的 token，请用命令行打印的地址打开"}, 403)
            return
        if parsed.path == "/api/state":
            session = self._session(query)
            folder = _job_dir(session)
            files = sorted(p.name for p in folder.iterdir() if p.is_file())
            self._json({"files": files, "version": __version__,
                        "session": session})
            return
        if parsed.path.startswith("/out/"):
            self._serve_output(parsed.path, query)
            return
        self._json({"error": "unknown path"}, 404)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        if not self._authorised(query):
            self._json({"error": "未授权"}, 403)
            return
        session = self._session(query)
        folder = _job_dir(session)
        body = self._body()
        if parsed.path == "/api/upload":
            self._json(self._upload(folder, body))
            return
        if parsed.path == "/api/run":
            self._json(self._run(folder, body))
            return
        if parsed.path == "/api/clear":
            for path in folder.iterdir():
                if path.is_file():
                    path.unlink()
            self._json({"ok": True})
            return
        self._json({"error": "unknown path"}, 404)

    # -- handlers ---------------------------------------------------------- #
    def _upload(self, folder: Path, body: dict) -> dict:
        name = _safe_name(body.get("name", ""))
        suffix = Path(name).suffix.lower()
        if suffix not in ALLOWED_SUFFIXES:
            return {"error": "不支持的文件类型 " + (suffix or "(无扩展名)")
                            + "，允许：urdf/xacro/xml/sdf/json/stl/obj/csv/txt/yaml"}
        try:
            data = base64.b64decode(body.get("data", ""), validate=False)
        except Exception:
            return {"error": "文件内容不是合法的 base64"}
        if len(data) > MAX_UPLOAD:
            return {"error": "文件超过 8 MB"}
        target = (folder / name).resolve()
        if folder.resolve() not in target.parents:
            return {"error": "非法文件名"}
        target.write_bytes(data)
        return {"ok": True, "name": name, "size": len(data)}

    def _serve_output(self, path: str, query: dict) -> None:
        parts = [p for p in path.split("/") if p]
        if len(parts) < 3:
            self._json({"error": "bad path"}, 404)
            return
        session = parts[1]
        name = _safe_name(parts[2])
        target = (_job_dir(session) / "out" / name).resolve()
        if not target.exists():
            self._json({"error": "文件不存在"}, 404)
            return
        suffix = target.suffix.lower()
        kind = {".md": "text/markdown; charset=utf-8",
                ".json": "application/json; charset=utf-8",
                ".jsonl": "text/plain; charset=utf-8",
                ".patch": "text/plain; charset=utf-8"}.get(
                    suffix, "application/octet-stream")
        self._send(200, target.read_bytes(), kind)

    def _run(self, folder: Path, body: dict) -> dict:
        from .cli import run_model_audit
        model = _safe_name(body.get("model", ""))
        model_path = folder / model
        if not model.strip() or not model_path.exists():
            return {"error": "请先上传模型文件（URDF / MJCF / SDF / JSON）"}
        log = _safe_name(body["log"]) if body.get("log") else None
        if log and not (folder / log).exists():
            return {"error": "日志文件不存在"}
        reality: dict[str, float] = {}
        if body.get("reality"):
            reality_path = folder / _safe_name(body["reality"])
            if reality_path.exists():
                try:
                    reality = {str(k): float(v) for k, v in
                               json.loads(reality_path.read_text(encoding="utf-8")).items()}
                except (ValueError, TypeError, OSError):
                    reality = {}
        target = None
        raw_target = str(body.get("target") or "").replace(";", ",").strip()
        if raw_target:
            parts = [p.strip() for p in raw_target.split(",") if p.strip()]
            if len(parts) != 3:
                return {"error": "目标点格式应为 x,y,z（例如 0.45,0,0.30）"}
            try:
                target = tuple(float(p) for p in parts)
            except ValueError:
                return {"error": "目标点三个分量都必须是数字，例如 0.45,0,0.30"}
        mesh = _safe_name(body["mesh"]) if body.get("mesh") else None
        if mesh and not (folder / mesh).exists():
            return {"error": "网格文件不存在"}
        try:
            out_dir = folder / "out"
            cmd = ("python scripts/audit.py model --model " + model
                   + (" --log " + log if log else "")
                   + " --speed " + str(body.get("speed", 0.3)))
            result, written = run_model_audit(
                model_path=str(model_path), log=str(folder / log) if log else None,
                out=str(out_dir), reality=reality,
                mesh_path=str(folder / mesh) if mesh else None,
                density=float(body["density"]) if body.get("density") else
                (1000.0 if mesh else None),
                speed=float(body.get("speed", 0.3)),
                yaw_rate=float(body.get("yaw_rate", 0.0)),
                lateral=float(body.get("lateral", 0.0)),
                target=target,
                payload=float(body.get("payload") or 0.0),
                used_radius=float(body["used_radius"]) if body.get("used_radius") else None,
                used_track=float(body["used_track"]) if body.get("used_track") else None,
                quiet=True, command=cmd)
        except (FileNotFoundError, ValueError) as exc:
            return {"error": str(exc)}
        except Exception as exc:                      # never leak a traceback silently
            return {"error": exc.__class__.__name__ + ": " + str(exc)}

        findings = [f.to_dict() for f in result.all_findings()]
        report_path = out_dir / "report.md"
        return {
            "ok": True,
            "summary": result.counts(),
            "blocking": len(result.blocking()),
            "findings": findings,
            "notes": result.notes,
            "report": report_path.read_text(encoding="utf-8", errors="replace"),
            "artifacts": sorted(p.name for p in out_dir.iterdir() if p.is_file()),
            "session": folder.name,
        }


def serve(host: str = "127.0.0.1", port: int = 8765, open_browser: bool = True,
          root: str | None = None) -> int:
    base = Path(root or (Path.cwd() / "audit-app")).resolve()
    base.mkdir(parents=True, exist_ok=True)
    STATE["root"] = base
    # RSW_APP_TOKEN exists so automated tests (and scripted launches) can use a
    # known token; by default a fresh random one is generated every start.
    STATE["token"] = os.environ.get("RSW_APP_TOKEN") or uuid.uuid4().hex
    try:
        httpd = ThreadingHTTPServer((host, port), Handler)
    except OSError as exc:
        print("无法在 " + host + ":" + str(port) + " 启动：" + str(exc))
        print("试试换一个端口：python scripts/audit.py app --port 8766")
        return 1
    url = "http://" + host + ":" + str(port) + "/?token=" + STATE["token"]
    print("=" * 70)
    print(__tool__ + " Web App  ·  v" + __version__)
    print("地址: " + url)
    print("上传目录: " + str(base))
    print("Ctrl+C 停止")
    print("=" * 70)
    if open_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        httpd.server_close()
    return 0
