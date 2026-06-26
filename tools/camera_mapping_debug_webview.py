#!/usr/bin/env python3
"""Camera mapping and live sensor debug webview for Indoory/XLeRobot.

The page shows the configured logical camera mapping next to the streams that
are actually arriving over the optimized camera ZMQ socket and the WebXR HTTP
camera endpoints. It is intentionally read-only: it never opens a camera device
and only subscribes to existing publishers.
"""

from __future__ import annotations

import argparse
import json
import os
import ssl
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

try:
    import msgpack
    import zmq
except Exception as exc:  # pragma: no cover - runtime dependency check
    print(f"[err] pyzmq and msgpack are required: {exc}", file=sys.stderr)
    raise


DEFAULT_ENV_FILE = "/home/pi/indoory_ros/robot/xlerobot_robot_io.env"
DEFAULT_CAMERA_TOPICS = (
    "rgb.front.0",
    "rgb.wrist_left.0",
    "rgb.wrist_right.0",
    "rgb.floor.0",
)
STATE_TOPIC_PREFIXES = (
    "tf.links.",
    "proprio.",
    "odom.",
    "joint_states.",
    "scan.",
)
CAMERA_HTTP_PATHS = {
    "head": {
        "jpg": "/api/head_rgb.jpg?robot=0",
        "mjpg": "/api/head_rgb.mjpg?robot=0",
        "h264": "/api/head_rgb.mp4?robot=0",
    },
    "wrist_left": {
        "jpg": "/api/wrist_rgb.jpg?side=left&robot=0",
        "mjpg": "/api/wrist_rgb.mjpg?side=left&robot=0",
        "h264": "/api/wrist_rgb.mp4?side=left&robot=0",
    },
    "wrist_right": {
        "jpg": "/api/wrist_rgb.jpg?side=right&robot=0",
        "mjpg": "/api/wrist_rgb.mjpg?side=right&robot=0",
        "h264": "/api/wrist_rgb.mp4?side=right&robot=0",
    },
    "floor": {
        "jpg": "/api/floor_rgb.jpg?robot=0",
        "mjpg": "/api/floor_rgb.mjpg?robot=0",
        "h264": "/api/floor_rgb.mp4?robot=0",
    },
}


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Indoory Camera Mapping Debug</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #080b0d;
      --panel: #11171a;
      --panel2: #172024;
      --line: #2b3940;
      --line2: #394b54;
      --text: #e8f1f3;
      --muted: #93a6ac;
      --ok: #4af28e;
      --warn: #ffd166;
      --bad: #ff6b72;
      --accent: #59d5ff;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font: 13px/1.38 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    header {
      position: sticky;
      top: 0;
      z-index: 5;
      display: flex;
      align-items: center;
      gap: 12px;
      min-height: 54px;
      padding: 10px 16px;
      border-bottom: 1px solid var(--line);
      background: #0b1012;
    }
    h1 { margin: 0; font-size: 18px; letter-spacing: 0; }
    h2 { margin: 0 0 10px; font-size: 14px; letter-spacing: 0; }
    h3 { margin: 0 0 7px; font-size: 13px; letter-spacing: 0; }
    .pill {
      display: inline-flex;
      align-items: center;
      min-height: 24px;
      padding: 0 9px;
      border: 1px solid var(--line2);
      border-radius: 999px;
      color: var(--muted);
      font-size: 12px;
      white-space: nowrap;
    }
    .pill.ok { color: var(--ok); border-color: rgba(74,242,142,.45); }
    .pill.warn { color: var(--warn); border-color: rgba(255,209,102,.5); }
    .pill.bad { color: var(--bad); border-color: rgba(255,107,114,.55); }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 430px;
      min-height: calc(100vh - 54px);
    }
    .content {
      min-width: 0;
      padding: 14px;
      display: grid;
      grid-template-rows: auto auto minmax(320px, 1fr);
      gap: 14px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(230px, 1fr));
      gap: 12px;
    }
    .card {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      min-width: 0;
      overflow: hidden;
    }
    .cardHead {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      min-height: 42px;
      padding: 10px;
      border-bottom: 1px solid var(--line);
      background: var(--panel2);
    }
    .preview {
      width: 100%;
      aspect-ratio: 4 / 3;
      display: grid;
      place-items: center;
      background: #050809;
      border-bottom: 1px solid var(--line);
      overflow: hidden;
    }
    .preview img {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
    }
    .preview .empty { color: var(--muted); padding: 14px; text-align: center; }
    .kv { display: grid; grid-template-columns: 92px minmax(0, 1fr); gap: 4px 8px; padding: 10px; }
    .kv div:nth-child(odd) { color: var(--muted); }
    .mono {
      font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      overflow-wrap: anywhere;
    }
    .section {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      min-width: 0;
      overflow: hidden;
    }
    .sectionHeader {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      min-height: 42px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      background: var(--panel2);
    }
    table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
    th, td { padding: 8px 7px; text-align: left; border-bottom: 1px solid #223039; vertical-align: top; }
    th { color: var(--muted); font-size: 12px; font-weight: 700; }
    td { font-size: 12px; }
    td.num { color: #d7f7ff; white-space: nowrap; }
    td.good { color: var(--ok); }
    td.warn { color: var(--warn); }
    td.bad { color: var(--bad); }
    .good { color: var(--ok); }
    .warn { color: var(--warn); }
    .bad { color: var(--bad); }
    .side {
      min-width: 0;
      border-left: 1px solid var(--line);
      background: #0d1315;
      display: grid;
      grid-template-rows: auto minmax(0, 1fr);
      max-height: calc(100vh - 54px);
      position: sticky;
      top: 54px;
    }
    .sideTop { padding: 12px; border-bottom: 1px solid var(--line); }
    .rawTabs { display: flex; gap: 8px; flex-wrap: wrap; }
    button {
      height: 30px;
      border: 1px solid var(--line2);
      border-radius: 6px;
      background: #142026;
      color: var(--text);
      font-weight: 650;
    }
    button.active { border-color: var(--accent); color: var(--accent); }
    pre {
      margin: 0;
      padding: 12px;
      white-space: pre-wrap;
      word-break: break-word;
      overflow: auto;
      min-height: 0;
      color: #c5ead2;
      font: 12px/1.42 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    .links { display: flex; gap: 7px; flex-wrap: wrap; padding: 0 10px 10px; }
    a { color: var(--accent); text-decoration: none; }
    a:hover { text-decoration: underline; }
    @media (max-width: 1280px) {
      main { grid-template-columns: 1fr; }
      .side { position: static; max-height: 48vh; border-left: 0; border-top: 1px solid var(--line); }
      .grid { grid-template-columns: repeat(2, minmax(230px, 1fr)); }
    }
    @media (max-width: 720px) {
      .grid { grid-template-columns: 1fr; }
      header { flex-wrap: wrap; }
      .content { padding: 10px; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Indoory Camera Mapping Debug</h1>
    <span id="overall" class="pill warn">waiting</span>
    <span id="zmq" class="pill">zmq -</span>
    <span id="webxr" class="pill">webxr -</span>
    <span id="updated" class="pill">updated -</span>
  </header>
  <main>
    <div class="content">
      <div class="grid" id="cards"></div>
      <div class="section">
        <div class="sectionHeader">
          <h2>Observed ZMQ Video Streams</h2>
          <span class="pill" id="topicCount">0 topics</span>
        </div>
        <div style="overflow:auto">
          <table>
            <thead>
              <tr>
                <th>topic</th><th>logical</th><th>encoding</th><th>size</th><th>seq</th>
                <th>age</th><th>fps</th><th>KiB/s</th><th>device/source</th>
              </tr>
            </thead>
            <tbody id="topicRows"></tbody>
          </table>
        </div>
      </div>
      <div class="section">
        <div class="sectionHeader">
          <h2>Robot Sensor Data</h2>
          <span class="pill" id="sensorAge">age -</span>
        </div>
        <div style="overflow:auto">
          <table>
            <thead><tr><th>sensor</th><th>topic</th><th>age</th><th>values</th></tr></thead>
            <tbody id="sensorRows"></tbody>
          </table>
        </div>
      </div>
    </div>
    <aside class="side">
      <div class="sideTop">
        <h2>Raw Data</h2>
        <div class="rawTabs">
          <button data-raw="mapping" class="active">mapping</button>
          <button data-raw="camera">camera</button>
          <button data-raw="sensors">sensors</button>
          <button data-raw="webxr">webxr</button>
        </div>
      </div>
      <pre id="raw">waiting</pre>
    </aside>
  </main>
  <script>
    const cardsEl = document.getElementById("cards");
    const topicRowsEl = document.getElementById("topicRows");
    const sensorRowsEl = document.getElementById("sensorRows");
    const rawEl = document.getElementById("raw");
    const overallEl = document.getElementById("overall");
    const zmqEl = document.getElementById("zmq");
    const webxrEl = document.getElementById("webxr");
    const updatedEl = document.getElementById("updated");
    const topicCountEl = document.getElementById("topicCount");
    const sensorAgeEl = document.getElementById("sensorAge");
    let latest = null;
    let rawTab = "mapping";

    document.querySelectorAll("button[data-raw]").forEach((btn) => {
      btn.addEventListener("click", () => {
        rawTab = btn.dataset.raw;
        document.querySelectorAll("button[data-raw]").forEach((b) => b.classList.toggle("active", b === btn));
        renderRaw();
      });
    });

    function esc(v) {
      return String(v == null ? "" : v).replace(/[&<>"']/g, (c) => ({
        "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
      }[c]));
    }
    function fmtAge(v) {
      return v == null ? "-" : `${Number(v).toFixed(3)}s`;
    }
    function fmtRate(v) {
      return v == null ? "-" : Number(v).toFixed(1);
    }
    function statusClass(ok, enabled=true) {
      if (!enabled) return "warn";
      if (ok === true) return "ok";
      if (ok === false) return "bad";
      return "warn";
    }
    function logicalForTopic(topic, mapping) {
      for (const cam of mapping.logical_cameras || []) {
        if (cam.zmq_topic === topic) return cam.name;
      }
      return "";
    }
    function previewUrl(cam) {
      const topic = encodeURIComponent(cam.zmq_topic || "");
      const name = encodeURIComponent(cam.name || "");
      return `/api/snapshot?camera=${name}&topic=${topic}&t=${Date.now()}`;
    }
    function card(cam, zmqTopic) {
      const enabled = cam.enabled !== false;
      const hasZmq = !!zmqTopic;
      const stat = statusClass(hasZmq, enabled);
      const actualDevice = zmqTopic?.meta?.device || "-";
      const expectedDevice = cam.resolved_device || cam.config_device || "-";
      const mismatch = enabled && actualDevice !== "-" && expectedDevice !== "-" && actualDevice !== expectedDevice;
      const links = Object.entries(cam.http || {}).map(([kind, href]) => {
        return `<a target="_blank" href="${esc(href)}">${esc(kind)}</a>`;
      }).join("");
      return `
        <div class="card">
          <div class="cardHead">
            <h3>${esc(cam.name)}</h3>
            <span class="pill ${stat}">${enabled ? (hasZmq ? "live" : "missing") : "disabled"}</span>
          </div>
          <div class="preview">
            ${enabled ? `<img data-preview="${esc(cam.name)}" src="${previewUrl(cam)}" alt="${esc(cam.name)} preview" onerror="this.replaceWith(Object.assign(document.createElement('div'),{className:'empty',textContent:'no snapshot'}))">` : `<div class="empty">disabled</div>`}
          </div>
          <div class="kv">
            <div>topic</div><div class="mono">${esc(cam.zmq_topic)}</div>
            <div>expected</div><div class="mono">${esc(expectedDevice)}</div>
            <div>actual</div><div class="mono ${mismatch ? "bad" : ""}">${esc(actualDevice)}</div>
            <div>port</div><div class="mono">${esc(cam.physical_port || zmqTopic?.physical_port || "-")}</div>
            <div>format</div><div class="mono">${esc(cam.input_format || "-")} / ${esc(zmqTopic?.encoding || "-")}</div>
            <div>flip</div><div class="mono">${esc(cam.flip || "-")}</div>
            <div>frame</div><div class="mono">${esc(zmqTopic?.meta?.frame_id || "-")}</div>
            <div>payload</div><div class="mono">${esc(zmqTopic ? `${zmqTopic.data_bytes} B, init ${zmqTopic.init_bytes} B` : "-")}</div>
          </div>
          <div class="links">${links}</div>
        </div>
      `;
    }
    function renderCards(data) {
      const topics = data.camera_topics || {};
      cardsEl.innerHTML = (data.mapping.logical_cameras || []).map((cam) => {
        return card(cam, topics[cam.zmq_topic]);
      }).join("");
    }
    function renderTopics(data) {
      const rows = Object.entries(data.camera_topics || {}).sort().map(([topic, item]) => {
        const logical = logicalForTopic(topic, data.mapping);
        const meta = item.meta || {};
        const source = [meta.device, meta.source, meta.camera, meta.flip ? `flip=${meta.flip}` : ""].filter(Boolean).join(" | ");
        return `<tr>
          <td class="mono">${esc(topic)}</td>
          <td>${esc(logical || "-")}</td>
          <td>${esc(item.encoding || "-")}</td>
          <td class="num">${esc(item.width || "-")}x${esc(item.height || "-")}</td>
          <td class="num">${esc(item.chunk_seq ?? item.seq ?? "-")}</td>
          <td class="num ${item.age_s > 1.5 ? "bad" : "good"}">${fmtAge(item.age_s)}</td>
          <td class="num">${fmtRate(item.fps)}</td>
          <td class="num">${fmtRate(item.kib_s)}</td>
          <td class="mono">${esc(source || "-")}</td>
        </tr>`;
      });
      topicRowsEl.innerHTML = rows.join("") || `<tr><td colspan="9" class="warn">No rgb.* camera ZMQ messages observed yet.</td></tr>`;
      topicCountEl.textContent = `${rows.length} topics`;
    }
    function renderSensors(data) {
      const s = data.sensors || {};
      const rows = (s.rows || []).map((item) => {
        const cls = item.age_s == null ? "warn" : item.age_s > 1.5 ? "bad" : "good";
        return `<tr>
          <td>${esc(item.name)}</td>
          <td class="mono">${esc(item.topic || "-")}</td>
          <td class="num ${cls}">${fmtAge(item.age_s)}</td>
          <td class="mono">${esc(item.summary || "-")}</td>
        </tr>`;
      });
      sensorRowsEl.innerHTML = rows.join("") || `<tr><td colspan="4" class="warn">No robot state messages observed yet.</td></tr>`;
      sensorAgeEl.textContent = s.max_age_s == null ? "age -" : `max age ${Number(s.max_age_s).toFixed(3)}s`;
      sensorAgeEl.className = `pill ${s.max_age_s == null ? "warn" : s.max_age_s > 1.5 ? "bad" : "ok"}`;
    }
    function renderHeader(data) {
      const live = Object.values(data.camera_topics || {}).filter((x) => x.age_s != null && x.age_s < 1.5).length;
      const expected = (data.mapping.logical_cameras || []).filter((x) => x.enabled !== false).length;
      overallEl.textContent = `${live}/${expected} camera topics live`;
      overallEl.className = `pill ${live === expected ? "ok" : live ? "warn" : "bad"}`;
      zmqEl.textContent = data.camera_endpoint || "zmq -";
      webxrEl.textContent = data.webxr?.ok ? "webxr ok" : "webxr missing";
      webxrEl.className = `pill ${data.webxr?.ok ? "ok" : "warn"}`;
      updatedEl.textContent = `updated ${new Date().toLocaleTimeString()}`;
    }
    function renderRaw() {
      if (!latest) return;
      const payload = {
        mapping: latest.mapping,
        camera: latest.camera_topics,
        sensors: latest.sensors,
        webxr: latest.webxr,
      }[rawTab];
      rawEl.textContent = JSON.stringify(payload, null, 2);
    }
    function render(data) {
      latest = data;
      renderHeader(data);
      renderCards(data);
      renderTopics(data);
      renderSensors(data);
      renderRaw();
    }
    async function poll() {
      try {
        const res = await fetch(`/api/state?t=${Date.now()}`, {cache: "no-store"});
        render(await res.json());
      } catch (err) {
        overallEl.textContent = `api error: ${err}`;
        overallEl.className = "pill bad";
      }
      setTimeout(poll, 500);
    }
    setInterval(() => {
      document.querySelectorAll("img[data-preview]").forEach((img) => {
        const cam = (latest?.mapping?.logical_cameras || []).find((x) => x.name === img.dataset.preview);
        if (cam) img.src = previewUrl(cam);
      });
    }, 900);
    poll();
  </script>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--state-endpoint", default="tcp://127.0.0.1:8855")
    parser.add_argument("--camera-endpoint", default="tcp://127.0.0.1:8866")
    parser.add_argument("--webxr-base", default="https://127.0.0.1:8443")
    parser.add_argument("--env-file", default=DEFAULT_ENV_FILE)
    parser.add_argument("--snapshot-timeout-s", type=float, default=0.8)
    return parser.parse_args()


def is_enabled_value(value: Any) -> bool:
    text = str(value or "").strip()
    return bool(text) and text.lower() not in {"none", "off", "false", "0", "disabled"}


def read_env_file(path: str | Path) -> dict[str, str]:
    values: dict[str, str] = {}
    env_path = Path(path).expanduser()
    if not env_path.exists():
        return values
    for raw_line in env_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def resolve_device(path: str) -> str:
    if not is_enabled_value(path):
        return str(path or "")
    try:
        return str(Path(path).expanduser().resolve(strict=True))
    except OSError:
        return str(path)


def by_path_name_for_device(device: str) -> str:
    resolved = resolve_device(device)
    if not resolved.startswith("/dev/video"):
        return ""
    by_path_dir = Path("/dev/v4l/by-path")
    if not by_path_dir.is_dir():
        return ""
    resolved_path = Path(resolved)
    candidates: list[str] = []
    for link in by_path_dir.iterdir():
        try:
            if link.resolve(strict=True) != resolved_path:
                continue
        except OSError:
            continue
        name = link.name
        if "video-index0" in name:
            candidates.insert(0, name)
        else:
            candidates.append(name)
    return candidates[0] if candidates else ""


def physical_port_from_by_path_name(name: str) -> str:
    value = str(name or "")
    if not value:
        return ""
    value = value.replace("-usbv2-", "-usb-")
    if "-video-index" in value:
        value = value.rsplit("-video-index", 1)[0]
    if value.endswith(":1.0") or value.endswith(":1.1") or value.endswith(":1.2") or value.endswith(":1.3"):
        value = value.rsplit(":", 1)[0]
    return value


def physical_port_from_device(device: str) -> str:
    text = str(device or "").strip()
    if not text or text.lower() in {"none", "off", "false", "0", "disabled"}:
        return ""
    if "/dev/v4l/by-path/" in text:
        return physical_port_from_by_path_name(Path(text).name)
    by_path = by_path_name_for_device(text)
    if by_path:
        return physical_port_from_by_path_name(by_path)
    return text


def realsense_physical_port() -> str:
    by_id_dir = Path("/dev/v4l/by-id")
    if not by_id_dir.is_dir():
        return "realsense-sdk"
    for link in sorted(by_id_dir.iterdir()):
        name = link.name
        if "RealSense" not in name or "video-index0" not in name:
            continue
        by_path = by_path_name_for_device(str(link))
        if by_path:
            return physical_port_from_by_path_name(by_path)
    return "realsense-sdk"


def port_uniqueness(cameras: list[dict[str, Any]]) -> dict[str, Any]:
    enabled = [cam for cam in cameras if cam.get("enabled") is not False]
    by_port: dict[str, list[str]] = {}
    for cam in enabled:
        port = str(cam.get("physical_port") or "").strip() or "unknown"
        by_port.setdefault(port, []).append(str(cam.get("name") or ""))
    duplicates = {port: names for port, names in by_port.items() if port != "unknown" and len(names) > 1}
    return {
        "ok": not duplicates and len(by_port) == len(enabled),
        "enabled_cameras": len(enabled),
        "unique_ports": len(by_port),
        "duplicates": duplicates,
        "by_port": by_port,
    }


def replace_robot_id(path: str, robot_id: int) -> str:
    return path.replace("robot=0", f"robot={int(robot_id)}")


def build_mapping(
    *,
    env: dict[str, str],
    robot_id: int,
    webxr_base: str,
) -> dict[str, Any]:
    base = webxr_base.rstrip("/")

    def http_paths(name: str) -> dict[str, str]:
        return {
            kind: base + replace_robot_id(path, robot_id)
            for kind, path in CAMERA_HTTP_PATHS[name].items()
        }

    cameras = [
        {
            "name": "head",
            "label": "head/front RealSense RGB",
            "zmq_topic": f"rgb.front.{robot_id}",
            "config_device": env.get("TELEOP_FRONT_CAMERA_DEVICE") or "realsense",
            "resolved_device": "realsense",
            "physical_port": realsense_physical_port(),
            "input_format": env.get("RGB_WIRE_FORMAT") or "h264_fmp4",
            "flip": "none",
            "enabled": True,
            "http": http_paths("head"),
        },
        {
            "name": "wrist_left",
            "label": "left wrist RGB",
            "zmq_topic": f"rgb.wrist_left.{robot_id}",
            "config_device": env.get("WRIST_LEFT_CAMERA_DEVICE", ""),
            "resolved_device": resolve_device(env.get("WRIST_LEFT_CAMERA_DEVICE", "")),
            "physical_port": physical_port_from_device(env.get("WRIST_LEFT_CAMERA_DEVICE", "")),
            "input_format": env.get("TELEOP_WRIST_LEFT_CAMERA_INPUT_FORMAT")
            or env.get("WRIST_LEFT_INPUT_FORMAT")
            or "MJPG",
            "flip": env.get("TELEOP_WRIST_LEFT_CAMERA_FLIP")
            or env.get("WRIST_LEFT_FLIP")
            or "horizontal",
            "enabled": is_enabled_value(env.get("WRIST_LEFT_CAMERA_DEVICE", "")),
            "http": http_paths("wrist_left"),
        },
        {
            "name": "wrist_right",
            "label": "right wrist RGB",
            "zmq_topic": f"rgb.wrist_right.{robot_id}",
            "config_device": env.get("WRIST_RIGHT_CAMERA_DEVICE", ""),
            "resolved_device": resolve_device(env.get("WRIST_RIGHT_CAMERA_DEVICE", "")),
            "physical_port": physical_port_from_device(env.get("WRIST_RIGHT_CAMERA_DEVICE", "")),
            "input_format": env.get("TELEOP_WRIST_RIGHT_CAMERA_INPUT_FORMAT")
            or env.get("WRIST_RIGHT_INPUT_FORMAT")
            or "MJPG",
            "flip": env.get("TELEOP_WRIST_RIGHT_CAMERA_FLIP")
            or env.get("WRIST_RIGHT_FLIP")
            or "both",
            "enabled": is_enabled_value(env.get("WRIST_RIGHT_CAMERA_DEVICE", "")),
            "http": http_paths("wrist_right"),
        },
        {
            "name": "floor",
            "label": "floor RGB",
            "zmq_topic": f"rgb.floor.{robot_id}",
            "config_device": env.get("FLOOR_CAMERA_DEVICE", ""),
            "resolved_device": resolve_device(env.get("FLOOR_CAMERA_DEVICE", "")),
            "physical_port": physical_port_from_device(env.get("FLOOR_CAMERA_DEVICE", "")),
            "input_format": env.get("TELEOP_FLOOR_CAMERA_INPUT_FORMAT")
            or env.get("FLOOR_INPUT_FORMAT")
            or "MJPG",
            "flip": env.get("TELEOP_FLOOR_CAMERA_FLIP")
            or env.get("FLOOR_FLIP")
            or "none",
            "enabled": is_enabled_value(env.get("FLOOR_CAMERA_DEVICE", "")),
            "http": http_paths("floor"),
        },
    ]
    return {
        "teleop_camera_feeds": env.get("TELEOP_CAMERA_FEEDS", ""),
        "logical_cameras": cameras,
        "port_uniqueness": port_uniqueness(cameras),
        "raw_env": {
            key: env.get(key, "")
            for key in (
                "TELEOP_CAMERA_FEEDS",
                "TELEOP_FRONT_CAMERA_DEVICE",
                "WRIST_LEFT_CAMERA_DEVICE",
                "WRIST_RIGHT_CAMERA_DEVICE",
                "FLOOR_CAMERA_DEVICE",
                "TELEOP_WRIST_LEFT_CAMERA_INPUT_FORMAT",
                "TELEOP_WRIST_RIGHT_CAMERA_INPUT_FORMAT",
                "TELEOP_FLOOR_CAMERA_INPUT_FORMAT",
                "TELEOP_WRIST_LEFT_CAMERA_FLIP",
                "TELEOP_WRIST_RIGHT_CAMERA_FLIP",
                "TELEOP_FLOOR_CAMERA_FLIP",
            )
        },
    }


def safe_json_value(value: Any, *, bytes_limit: int = 0) -> Any:
    if isinstance(value, (bytes, bytearray, memoryview)):
        data = bytes(value)
        if bytes_limit > 0:
            return {
                "type": "bytes",
                "len": len(data),
                "head_hex": data[:bytes_limit].hex(),
            }
        return {"type": "bytes", "len": len(data)}
    if isinstance(value, dict):
        return {str(k): safe_json_value(v, bytes_limit=bytes_limit) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json_value(v, bytes_limit=bytes_limit) for v in value[:200]]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


class LatestTopicStore:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._items: dict[str, dict[str, Any]] = {}

    def put(self, topic: str, msg: dict[str, Any]) -> None:
        now = time.monotonic()
        data = msg.get("data")
        init = msg.get("init")
        data_bytes = len(bytes(data or b"")) if isinstance(data, (bytes, bytearray)) else 0
        init_bytes = len(bytes(init or b"")) if isinstance(init, (bytes, bytearray)) else 0
        with self._lock:
            prev = self._items.get(topic)
            times = deque(prev.get("_times", ()), maxlen=60) if prev else deque(maxlen=60)
            byte_times = deque(prev.get("_byte_times", ()), maxlen=60) if prev else deque(maxlen=60)
            times.append(now)
            byte_times.append((now, data_bytes + init_bytes))
            meta = {k: v for k, v in msg.items() if k not in {"data", "init"}}
            self._items[topic] = {
                "topic": topic,
                "stamp_t": now,
                "encoding": msg.get("encoding"),
                "codec": msg.get("codec"),
                "container": msg.get("container"),
                "width": msg.get("width"),
                "height": msg.get("height"),
                "seq": msg.get("seq"),
                "chunk_seq": msg.get("chunk_seq"),
                "data_bytes": data_bytes,
                "init_bytes": init_bytes,
                "physical_port": physical_port_from_device(str(msg.get("device") or "")),
                "meta": safe_json_value(meta, bytes_limit=16),
                "_times": times,
                "_byte_times": byte_times,
                "_jpeg": bytes(data) if msg.get("encoding") == "jpeg" and data_bytes else None,
            }

    def summary(self) -> dict[str, dict[str, Any]]:
        now = time.monotonic()
        out: dict[str, dict[str, Any]] = {}
        with self._lock:
            for topic, item in self._items.items():
                times = list(item.get("_times") or [])
                byte_times = list(item.get("_byte_times") or [])
                fps = 0.0
                kib_s = 0.0
                if len(times) >= 2:
                    duration = max(0.001, times[-1] - times[0])
                    fps = (len(times) - 1) / duration
                if len(byte_times) >= 2:
                    duration = max(0.001, byte_times[-1][0] - byte_times[0][0])
                    kib_s = sum(size for _ts, size in byte_times[1:]) / duration / 1024.0
                clean = {
                    key: value
                    for key, value in item.items()
                    if not key.startswith("_") and key != "stamp_t"
                }
                clean["age_s"] = round(max(0.0, now - float(item["stamp_t"])), 4)
                clean["fps"] = round(fps, 2)
                clean["kib_s"] = round(kib_s, 1)
                out[topic] = clean
        return out

    def jpeg(self, topic: str) -> bytes | None:
        with self._lock:
            item = self._items.get(str(topic))
            if not item:
                return None
            jpeg = item.get("_jpeg")
            return bytes(jpeg) if jpeg else None


class MessageStore:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._items: dict[str, tuple[float, dict[str, Any]]] = {}

    def put(self, topic: str, msg: dict[str, Any]) -> None:
        with self._lock:
            self._items[str(topic)] = (time.monotonic(), safe_json_value(msg, bytes_limit=8))

    def get(self, topic: str) -> dict[str, Any] | None:
        with self._lock:
            item = self._items.get(str(topic))
            return None if item is None else dict(item[1])

    def all(self) -> dict[str, dict[str, Any]]:
        now = time.monotonic()
        with self._lock:
            return {
                topic: {"age_s": round(max(0.0, now - ts), 4), "msg": msg}
                for topic, (ts, msg) in self._items.items()
            }


class ZmqSubscriber(threading.Thread):
    def __init__(
        self,
        *,
        endpoint: str,
        subscriptions: tuple[str, ...],
        kind: str,
        camera_store: LatestTopicStore | None = None,
        message_store: MessageStore | None = None,
    ) -> None:
        super().__init__(name=f"camera-debug-{kind}-sub", daemon=True)
        self.endpoint = endpoint
        self.subscriptions = tuple(subscriptions)
        self.kind = kind
        self.camera_store = camera_store
        self.message_store = message_store
        self.last_error = ""
        self.last_topic = ""
        self.recv_count = 0
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVHWM, 8)
        sock.setsockopt(zmq.RCVTIMEO, 250)
        for sub in self.subscriptions:
            sock.setsockopt(zmq.SUBSCRIBE, sub.encode("utf-8"))
        sock.connect(self.endpoint)
        try:
            while not self._stop.is_set():
                try:
                    topic_b, payload_b = sock.recv_multipart()
                except zmq.Again:
                    continue
                except Exception as exc:
                    self.last_error = str(exc)
                    time.sleep(0.1)
                    continue
                topic = topic_b.decode("utf-8", "replace")
                try:
                    msg = msgpack.unpackb(payload_b, raw=False)
                except Exception as exc:
                    self.last_error = f"decode {topic}: {exc}"
                    continue
                if not isinstance(msg, dict):
                    continue
                self.recv_count += 1
                self.last_topic = topic
                if self.camera_store is not None:
                    self.camera_store.put(topic, msg)
                if self.message_store is not None:
                    self.message_store.put(topic, msg)
        finally:
            sock.close(linger=0)


def first_present(dct: dict[str, Any], *keys: str) -> Any:
    cur: Any = dct
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def summarize_sensors(store: MessageStore, robot_id: int) -> dict[str, Any]:
    topics = {
        "tf.links": f"tf.links.{robot_id}",
        "proprio": f"proprio.{robot_id}",
        "odom": f"odom.{robot_id}",
        "joint_states": f"joint_states.{robot_id}",
        "scan": f"scan.{robot_id}",
    }
    all_items = store.all()
    rows: list[dict[str, Any]] = []
    max_age: float | None = None
    for name, topic in topics.items():
        item = all_items.get(topic)
        if item is None:
            rows.append({"name": name, "topic": topic, "age_s": None, "summary": "missing"})
            continue
        age = item["age_s"]
        max_age = age if max_age is None else max(max_age, age)
        msg = item["msg"]
        summary = summarize_sensor_message(name, msg)
        rows.append({"name": name, "topic": topic, "age_s": age, "summary": summary})
    return {
        "rows": rows,
        "max_age_s": max_age,
        "raw": all_items,
    }


def summarize_sensor_message(name: str, msg: dict[str, Any]) -> str:
    if name == "scan":
        scan = msg.get("msg") if isinstance(msg.get("msg"), dict) else msg
        ranges = scan.get("ranges")
        count = len(ranges) if isinstance(ranges, list) else scan.get("num_ranges")
        return (
            f"frame={scan.get('frame_id') or first_present(scan, 'header', 'frame_id')} "
            f"ranges={count} min={scan.get('range_min')} max={scan.get('range_max')}"
        )
    if name == "joint_states":
        joint = msg.get("msg") if isinstance(msg.get("msg"), dict) else msg
        names = joint.get("name") or joint.get("names") or []
        pos = joint.get("position") or joint.get("positions") or []
        pairs = []
        if isinstance(names, list) and isinstance(pos, list):
            for key, value in list(zip(names, pos))[:8]:
                try:
                    pairs.append(f"{key}={float(value):.3f}")
                except Exception:
                    pairs.append(f"{key}={value}")
        return ", ".join(pairs) or f"keys={','.join(sorted(joint.keys())[:8])}"
    if name == "tf.links":
        links = msg.get("links") or msg.get("tf_links") or msg.get("targets") or msg
        if isinstance(links, dict):
            return f"links={len(links)} keys={','.join(list(links.keys())[:6])}"
        if isinstance(links, list):
            return f"links={len(links)}"
        return f"keys={','.join(sorted(msg.keys())[:8])}"
    if name == "odom":
        odom = msg.get("msg") if isinstance(msg.get("msg"), dict) else msg
        p = first_present(odom, "pose", "pose", "position") or {}
        q = first_present(odom, "pose", "pose", "orientation") or {}
        return (
            f"pos=({p.get('x', '-')},{p.get('y', '-')},{p.get('z', '-')}) "
            f"quat=({q.get('x', '-')},{q.get('y', '-')},{q.get('z', '-')},{q.get('w', '-')})"
        )
    if name == "proprio":
        keys = sorted(k for k in msg.keys() if k not in {"schema", "topic", "stamp_ns"})
        return f"keys={','.join(keys[:10])}"
    return f"keys={','.join(sorted(msg.keys())[:8])}"


def fetch_json(url: str, timeout_s: float) -> tuple[bool, Any]:
    ctx = ssl._create_unverified_context()
    req = urllib.request.Request(url, headers={"Cache-Control": "no-cache"})
    try:
        with urllib.request.urlopen(req, timeout=timeout_s, context=ctx) as resp:
            data = resp.read(2_000_000)
            return True, json.loads(data.decode("utf-8", "replace"))
    except Exception as exc:
        return False, {"error": str(exc), "url": url}


def fetch_bytes(url: str, timeout_s: float) -> tuple[int, str, bytes]:
    ctx = ssl._create_unverified_context()
    req = urllib.request.Request(url, headers={"Cache-Control": "no-cache", "Connection": "close"})
    try:
        with urllib.request.urlopen(req, timeout=timeout_s, context=ctx) as resp:
            return int(resp.status), str(resp.headers.get("Content-Type") or ""), resp.read(2_000_000)
    except urllib.error.HTTPError as exc:
        return int(exc.code), "text/plain", exc.read(4096)
    except Exception as exc:
        return 599, "text/plain", str(exc).encode("utf-8", "replace")


class CameraDebugServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], args: argparse.Namespace) -> None:
        super().__init__(server_address, Handler)
        self.args = args
        self.env = read_env_file(args.env_file)
        self.camera_store = LatestTopicStore()
        self.state_store = MessageStore()
        self.camera_sub = ZmqSubscriber(
            endpoint=str(args.camera_endpoint),
            subscriptions=("rgb.",),
            kind="camera",
            camera_store=self.camera_store,
        )
        self.state_sub = ZmqSubscriber(
            endpoint=str(args.state_endpoint),
            subscriptions=STATE_TOPIC_PREFIXES,
            kind="state",
            message_store=self.state_store,
        )
        self.camera_sub.start()
        self.state_sub.start()


class Handler(BaseHTTPRequestHandler):
    server: CameraDebugServer

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stderr.write(
            "[camera_debug_webview] "
            + self.log_date_time_string()
            + " "
            + (fmt % args)
            + "\n"
        )

    def do_GET(self) -> None:  # noqa: N802 - stdlib callback
        parsed = urllib.parse.urlsplit(self.path)
        if parsed.path == "/":
            self._send_text(INDEX_HTML, "text/html")
        elif parsed.path == "/api/state":
            self._send_json(self._state_payload())
        elif parsed.path == "/api/snapshot":
            self._snapshot(parsed)
        elif parsed.path == "/api/zmq_jpeg":
            self._zmq_jpeg(parsed)
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def _state_payload(self) -> dict[str, Any]:
        args = self.server.args
        self.server.env = read_env_file(args.env_file)
        mapping = build_mapping(
            env=self.server.env,
            robot_id=int(args.robot_id),
            webxr_base=str(args.webxr_base),
        )
        mapping["env_file"] = str(Path(args.env_file).expanduser())
        status_url = str(args.webxr_base).rstrip("/") + "/api/status"
        webxr_ok, webxr_status = fetch_json(status_url, timeout_s=0.35)
        return {
            "ok": True,
            "robot_id": int(args.robot_id),
            "state_endpoint": str(args.state_endpoint),
            "camera_endpoint": str(args.camera_endpoint),
            "webxr_base": str(args.webxr_base).rstrip("/"),
            "mapping": mapping,
            "camera_topics": self.server.camera_store.summary(),
            "camera_subscriber": {
                "recv_count": self.server.camera_sub.recv_count,
                "last_topic": self.server.camera_sub.last_topic,
                "last_error": self.server.camera_sub.last_error,
            },
            "state_subscriber": {
                "recv_count": self.server.state_sub.recv_count,
                "last_topic": self.server.state_sub.last_topic,
                "last_error": self.server.state_sub.last_error,
            },
            "sensors": summarize_sensors(self.server.state_store, int(args.robot_id)),
            "webxr": {
                "ok": webxr_ok,
                "status_url": status_url,
                "status": webxr_status,
            },
        }

    def _snapshot(self, parsed: urllib.parse.SplitResult) -> None:
        qs = urllib.parse.parse_qs(parsed.query)
        camera = (qs.get("camera") or [""])[0]
        topic = (qs.get("topic") or [""])[0]
        jpeg = self.server.camera_store.jpeg(topic)
        if jpeg:
            self._send_bytes(jpeg, "image/jpeg", cache=False)
            return
        topics = self.server.camera_store.summary()
        topic_item = topics.get(topic)
        if topic_item is None and camera != "head":
            self.send_error(HTTPStatus.NOT_FOUND, f"no ZMQ frame for {topic}")
            return
        path = CAMERA_HTTP_PATHS.get(camera, {}).get("jpg")
        if not path:
            self.send_error(HTTPStatus.NOT_FOUND, "unknown camera")
            return
        url = str(self.server.args.webxr_base).rstrip("/") + replace_robot_id(path, int(self.server.args.robot_id))
        code, content_type, data = fetch_bytes(url, timeout_s=float(self.server.args.snapshot_timeout_s))
        if code >= 400 or not data:
            self.send_error(code if code < 600 else HTTPStatus.BAD_GATEWAY, data.decode("utf-8", "replace")[:200])
            return
        self._send_bytes(data, content_type or "image/jpeg", cache=False)

    def _zmq_jpeg(self, parsed: urllib.parse.SplitResult) -> None:
        qs = urllib.parse.parse_qs(parsed.query)
        topic = (qs.get("topic") or [""])[0]
        jpeg = self.server.camera_store.jpeg(topic)
        if not jpeg:
            self.send_error(HTTPStatus.NOT_FOUND, "no jpeg frame for topic")
            return
        self._send_bytes(jpeg, "image/jpeg", cache=False)

    def _send_json(self, value: Any) -> None:
        data = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._send_bytes(data, "application/json", cache=False)

    def _send_text(self, text: str, content_type: str) -> None:
        self._send_bytes(text.encode("utf-8"), content_type, cache=True)

    def _send_bytes(self, data: bytes, content_type: str, *, cache: bool) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store, max-age=0" if not cache else "no-cache")
        self.end_headers()
        self.wfile.write(data)


def main() -> int:
    args = parse_args()
    server = CameraDebugServer((str(args.host), int(args.port)), args)
    host_label = "127.0.0.1" if str(args.host) in {"0.0.0.0", "::"} else str(args.host)
    print(
        f"[camera_debug_webview] listening http://{host_label}:{int(args.port)} "
        f"camera={args.camera_endpoint} state={args.state_endpoint} webxr={args.webxr_base}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.camera_sub.stop()
        server.state_sub.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
