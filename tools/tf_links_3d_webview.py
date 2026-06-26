#!/usr/bin/env python3
"""Live 3D tf.links viewer for the XLeRobot fast ZMQ stream."""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
import signal
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlsplit

try:
    import msgpack
    import zmq
except Exception as exc:  # pragma: no cover - runtime dependency check
    print(f"[err] pyzmq and msgpack are required: {exc}", file=sys.stderr)
    raise


TELEOP_SRC = Path(os.environ.get("TELEOPERATION_SRC", "/home/pi/teleoperation/src")).expanduser()
TELEOP_WEBXR_DIR = Path(os.environ.get("TELEOPERATION_WEBXR_DIR", "/home/pi/teleoperation/teleoperation/webxr")).expanduser()
if TELEOP_SRC.is_dir() and str(TELEOP_SRC) not in sys.path:
    sys.path.insert(0, str(TELEOP_SRC))

try:
    from indoory_isaac_sim.apps.teleop.vr_web_teleop_overlay import (
        ROBOT_ASSET_DIR,
        xlerobot_model_description,
    )
except Exception as exc:  # pragma: no cover - rendered as API error
    ROBOT_ASSET_DIR = Path("/home/pi/teleoperation/src/indoory_isaac_sim/assets/data/robots/xlerobot")
    XLEROBOT_MODEL_IMPORT_ERROR: Exception | None = exc
    xlerobot_model_description = None
else:
    XLEROBOT_MODEL_IMPORT_ERROR = None


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Indoory tf.links 3D Viewer</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #070b0d;
      --panel: #121a1e;
      --panel2: #172328;
      --line: #2b3b43;
      --text: #e6f1f4;
      --muted: #95a8ad;
      --ok: #3bf58e;
      --warn: #ffd66b;
      --bad: #ff6b72;
      --right: #28e8ff;
      --left: #ffd54a;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      height: 100vh;
      min-height: 100vh;
      overflow: hidden;
      font: 14px/1.35 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
      display: grid;
      grid-template-columns: minmax(0, 1fr) 380px;
    }
    main { min-width: 0; min-height: 0; display: grid; grid-template-rows: auto minmax(0, 1fr); }
    header {
      min-height: 54px;
      display: flex;
      align-items: center;
      gap: 14px;
      padding: 10px 16px;
      border-bottom: 1px solid var(--line);
      background: #0b1114;
    }
    h1 { margin: 0; font-size: 18px; font-weight: 700; }
    .pill {
      display: inline-flex;
      align-items: center;
      height: 26px;
      padding: 0 10px;
      border: 1px solid var(--line);
      border-radius: 999px;
      color: var(--muted);
      white-space: nowrap;
    }
    .pill.ok { color: var(--ok); border-color: rgba(59,245,142,.45); }
    .pill.warn { color: var(--warn); border-color: rgba(255,214,107,.45); }
    .pill.bad { color: var(--bad); border-color: rgba(255,107,114,.5); }
    #stageWrap { position: relative; min-height: 0; height: 100%; overflow: hidden; }
    #stage { width: 100%; height: 100%; display: block; background: #020607; cursor: grab; }
    #stage:active { cursor: grabbing; }
    #hint {
      position: absolute;
      left: 14px;
      bottom: 12px;
      color: var(--muted);
      background: rgba(7,11,13,.72);
      border: 1px solid rgba(80,105,113,.45);
      border-radius: 6px;
      padding: 8px 10px;
      pointer-events: none;
    }
    aside {
      min-width: 0;
      min-height: 0;
      border-left: 1px solid var(--line);
      background: var(--panel);
      display: grid;
      grid-template-rows: auto auto minmax(0, 1fr);
    }
    .section { padding: 14px; border-bottom: 1px solid var(--line); }
    .section h2 { margin: 0 0 10px; font-size: 13px; color: var(--muted); font-weight: 700; }
    .toolbar { display: flex; gap: 8px; flex-wrap: wrap; }
    button {
      height: 34px;
      padding: 0 11px;
      border-radius: 6px;
      border: 1px solid #35505b;
      background: #132025;
      color: var(--text);
      font-weight: 650;
    }
    button:hover { background: #1b2c33; }
    label {
      display: flex;
      align-items: center;
      gap: 8px;
      height: 30px;
      color: var(--muted);
    }
    input[type="checkbox"] { accent-color: #36d399; }
    table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
    th, td { padding: 7px 6px; text-align: left; border-bottom: 1px solid #223139; }
    th { color: var(--muted); font-size: 12px; }
    td { font-size: 12px; }
    td.num { color: #d5f6ff; }
    .scroll { overflow: auto; min-height: 0; }
    pre {
      margin: 0;
      padding: 12px 14px;
      white-space: pre-wrap;
      word-break: break-word;
      color: #bcebd1;
      font: 12px/1.4 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    @media (max-width: 980px) {
      body { grid-template-columns: 1fr; grid-template-rows: minmax(60vh, 1fr) auto; }
      aside { border-left: 0; border-top: 1px solid var(--line); max-height: 42vh; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>Indoory tf.links 3D Viewer</h1>
      <span id="status" class="pill warn">waiting</span>
      <span id="age" class="pill">age -</span>
      <span id="source" class="pill">source -</span>
    </header>
    <div id="stageWrap">
      <canvas id="stage"></canvas>
      <div id="hint">Drag rotate, wheel zoom. Cyan is right EE, yellow is left EE.</div>
    </div>
  </main>
  <aside>
    <div class="section">
      <h2>View</h2>
      <div class="toolbar">
        <button id="resetView">Reset</button>
        <button id="topView">Top</button>
        <button id="sideView">Side</button>
        <button id="frontView">Front</button>
      </div>
      <label><input id="showGrid" type="checkbox" checked> Grid</label>
      <label><input id="showLabels" type="checkbox" checked> Labels</label>
      <label><input id="showRaw" type="checkbox" checked> Raw tf target table</label>
    </div>
    <div class="section">
      <h2>EE Poses From tf.links</h2>
      <table>
        <thead><tr><th>Name</th><th>X</th><th>Y</th><th>Z</th><th>Age</th></tr></thead>
        <tbody id="poseRows"></tbody>
      </table>
    </div>
    <div class="scroll">
      <pre id="raw">waiting for /api/state</pre>
    </div>
  </aside>
  <script>
    const canvas = document.getElementById("stage");
    const ctx = canvas.getContext("2d");
    const statusEl = document.getElementById("status");
    const ageEl = document.getElementById("age");
    const sourceEl = document.getElementById("source");
    const poseRows = document.getElementById("poseRows");
    const rawEl = document.getElementById("raw");
    const showGridEl = document.getElementById("showGrid");
    const showLabelsEl = document.getElementById("showLabels");
    const showRawEl = document.getElementById("showRaw");
    let state = null;
    let view = { yaw: -0.72, pitch: -0.48, zoom: 1.0, panX: 0, panY: 0 };
    let renderFit = { centerX: 0, centerY: 0, scale: 520 };
    let dragging = false;
    let lastMouse = [0, 0];
    const ARM_MOUNT = {
      right: [-0.135, -0.133, 0.760],
      left: [-0.135, 0.133, 0.760],
    };

    function resize() {
      const rect = canvas.getBoundingClientRect();
      const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
      const w = Math.max(320, Math.floor(rect.width * dpr));
      const h = Math.max(260, Math.floor(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }
    }

    function qRotate(q, v) {
      if (!Array.isArray(q) || q.length < 4) return v;
      const x = Number(q[0] || 0), y = Number(q[1] || 0), z = Number(q[2] || 0), w = Number(q[3] || 1);
      const vx = v[0], vy = v[1], vz = v[2];
      const tx = 2 * (y * vz - z * vy);
      const ty = 2 * (z * vx - x * vz);
      const tz = 2 * (x * vy - y * vx);
      return [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
      ];
    }

    function rotateForView(p) {
      let x = Number(p[0] || 0);
      let y = Number(p[1] || 0);
      let z = Number(p[2] || 0);
      const cy = Math.cos(view.yaw), sy = Math.sin(view.yaw);
      const x1 = cy * x - sy * y;
      const y1 = sy * x + cy * y;
      const cp = Math.cos(view.pitch), sp = Math.sin(view.pitch);
      const y2 = cp * y1 - sp * z;
      const z2 = sp * y1 + cp * z;
      return [x1, z2, y2];
    }

    function addCuboidPoints(points, min, max) {
      points.push(
        [min[0], min[1], min[2]], [max[0], min[1], min[2]],
        [max[0], max[1], min[2]], [min[0], max[1], min[2]],
        [min[0], min[1], max[2]], [max[0], min[1], max[2]],
        [max[0], max[1], max[2]], [min[0], max[1], max[2]],
      );
    }

    function scenePoints() {
      const points = [
        ARM_MOUNT.right, ARM_MOUNT.left,
        [0, 0, 0.86], [0.12, 0, 0.86], [0, 0.12, 0.86], [0, 0, 0.98],
        [0, 0, 0.44], [0.05, 0, 0.46],
      ];
      addCuboidPoints(points, [-0.24, -0.31, 0.73], [0.33, 0.31, 0.79]);
      addCuboidPoints(points, [-0.18, -0.24, 0.79], [0.27, 0.24, 0.86]);
      addCuboidPoints(points, [-0.08, -0.08, 0.86], [0.08, 0.08, 1.17]);
      ["gripper_right", "jaw_right", "gripper_left", "jaw_left", "head_pan", "head_tilt"].forEach(name => {
        const pose = poseFromTargets(name);
        if (pose) points.push([pose[0], pose[1], pose[2]]);
      });
      return points;
    }

    function updateRenderFit() {
      const projected = scenePoints().map(rotateForView);
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      projected.forEach(p => {
        minX = Math.min(minX, p[0]);
        maxX = Math.max(maxX, p[0]);
        minY = Math.min(minY, p[1]);
        maxY = Math.max(maxY, p[1]);
      });
      if (!Number.isFinite(minX) || !Number.isFinite(maxX) || maxX <= minX || maxY <= minY) {
        renderFit = { centerX: 0, centerY: 0.95, scale: 520 * view.zoom };
        return;
      }
      const pad = 90 * Math.max(1, Math.min(2, window.devicePixelRatio || 1));
      const sx = (canvas.width - pad * 2) / Math.max(0.05, maxX - minX);
      const sy = (canvas.height - pad * 2) / Math.max(0.05, maxY - minY);
      renderFit = {
        centerX: (minX + maxX) * 0.5,
        centerY: (minY + maxY) * 0.5,
        scale: Math.max(120, Math.min(1800, Math.min(sx, sy) * view.zoom)),
      };
    }

    function project(p) {
      const r = rotateForView(p);
      return {
        x: canvas.width * 0.5 + view.panX + (r[0] - renderFit.centerX) * renderFit.scale,
        y: canvas.height * 0.52 + view.panY - (r[1] - renderFit.centerY) * renderFit.scale,
        depth: r[2],
        scale: renderFit.scale,
      };
    }

    function line(a, b, color, width = 2) {
      const pa = project(a), pb = project(b);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      ctx.moveTo(pa.x, pa.y);
      ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
    }

    function dot(p, color, r = 6) {
      const pp = project(p);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(pp.x, pp.y, r, 0, Math.PI * 2);
      ctx.fill();
    }

    function text(p, value, color = "#d8edf2") {
      if (!showLabelsEl.checked) return;
      const pp = project(p);
      ctx.fillStyle = "rgba(2,6,7,.72)";
      const width = Math.min(280, Math.max(70, value.length * 6.5 + 12));
      ctx.fillRect(pp.x + 7, pp.y - 21, width, 20);
      ctx.strokeStyle = "rgba(120,150,160,.35)";
      ctx.strokeRect(pp.x + 7, pp.y - 21, width, 20);
      ctx.fillStyle = color;
      ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
      ctx.fillText(value, pp.x + 12, pp.y - 7);
    }

    function cuboid(min, max, color) {
      const p = [
        [min[0], min[1], min[2]], [max[0], min[1], min[2]],
        [max[0], max[1], min[2]], [min[0], max[1], min[2]],
        [min[0], min[1], max[2]], [max[0], min[1], max[2]],
        [max[0], max[1], max[2]], [min[0], max[1], max[2]],
      ];
      [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]]
        .forEach(([a,b]) => line(p[a], p[b], color, 1.4));
    }

    function drawAxis(origin, quat, size) {
      const axes = [
        [[size, 0, 0], "#ff4b45", "x"],
        [[0, size, 0], "#39e56b", "y"],
        [[0, 0, size], "#4e85ff", "z"],
      ];
      axes.forEach(([axis, color, label]) => {
        const d = qRotate(quat, axis);
        const end = [origin[0] + d[0], origin[1] + d[1], origin[2] + d[2]];
        line(origin, end, color, 2.5);
        text(end, label, color);
      });
    }

    function drawGrid() {
      if (!showGridEl.checked) return;
      for (let i = -6; i <= 8; i++) {
        const x = i * 0.1;
        line([x, -0.5, 0.74], [x, 0.5, 0.74], "rgba(80,105,113,.30)", 1);
      }
      for (let i = -5; i <= 5; i++) {
        const y = i * 0.1;
        line([-0.6, y, 0.74], [0.8, y, 0.74], "rgba(80,105,113,.30)", 1);
      }
    }

    function poseFromTargets(name) {
      const targets = state && state.targets ? state.targets : {};
      return Array.isArray(targets[name]) ? targets[name] : null;
    }

    function drawScene() {
      resize();
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const grad = ctx.createLinearGradient(0, 0, 0, canvas.height);
      grad.addColorStop(0, "#061013");
      grad.addColorStop(1, "#020607");
      ctx.fillStyle = grad;
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      updateRenderFit();
      drawGrid();
      cuboid([-0.24, -0.31, 0.73], [0.33, 0.31, 0.79], "#254dff");
      cuboid([-0.18, -0.24, 0.79], [0.27, 0.24, 0.86], "#315df8");
      cuboid([-0.08, -0.08, 0.86], [0.08, 0.08, 1.17], "#6c7479");
      const rightMount = ARM_MOUNT.right;
      const leftMount = ARM_MOUNT.left;
      dot(rightMount, "#28e8ff", 4);
      dot(leftMount, "#ffd54a", 4);
      text(rightMount, "right arm_base_joint", "#28e8ff");
      text(leftMount, "left arm_base_joint_2", "#ffd54a");
      drawAxis([0, 0, 0.86], [0, 0, 0, 1], 0.11);
      const poses = [
        ["right", poseFromTargets("gripper_right") || poseFromTargets("jaw_right"), rightMount, "#28e8ff"],
        ["left", poseFromTargets("gripper_left") || poseFromTargets("jaw_left"), leftMount, "#ffd54a"],
      ];
      poses.forEach(([name, pose, mount, color]) => {
        if (!pose) return;
        const p = [pose[0], pose[1], pose[2]];
        const q = [pose[3], pose[4], pose[5], pose[6]];
        line(mount, p, color, 3);
        dot(p, color, 7);
        drawAxis(p, q, 0.055);
        text(p, `${name} ${fmt(pose[0])}, ${fmt(pose[1])}, ${fmt(pose[2])}`, color);
      });
      const headPan = poseFromTargets("head_pan");
      const headTilt = poseFromTargets("head_tilt");
      if (headPan) dot(headPan, "#9cc2ff", 4);
      if (headTilt) dot(headTilt, "#9cc2ff", 4);
      if (headPan && headTilt) line(headPan, headTilt, "#9cc2ff", 1.8);
    }

    function fmt(v) {
      const n = Number(v);
      return Number.isFinite(n) ? n.toFixed(3) : "-";
    }

    function updateUi(data) {
      state = data;
      const ok = Boolean(data.ok) && Number(data.age_s) < 1.0;
      statusEl.textContent = ok ? "tf.links live" : (data.ok ? "stale" : "waiting");
      statusEl.className = "pill " + (ok ? "ok" : data.ok ? "warn" : "bad");
      ageEl.textContent = "age " + (Number.isFinite(Number(data.age_s)) ? Number(data.age_s).toFixed(3) + "s" : "-");
      sourceEl.textContent = data.source_note || data.endpoint || "source -";
      const targets = data.targets || {};
      const names = ["gripper_right", "jaw_right", "gripper_left", "jaw_left", "head_pan", "head_tilt"];
      poseRows.innerHTML = names.map(name => {
        const p = targets[name];
        const color = name.includes("right") ? "var(--right)" : name.includes("left") ? "var(--left)" : "#9cc2ff";
        return `<tr><td style="color:${color}">${name}</td><td class="num">${p ? fmt(p[0]) : "-"}</td><td class="num">${p ? fmt(p[1]) : "-"}</td><td class="num">${p ? fmt(p[2]) : "-"}</td><td>${fmt(data.age_s)}</td></tr>`;
      }).join("");
      rawEl.style.display = showRawEl.checked ? "block" : "none";
      if (showRawEl.checked) {
        rawEl.textContent = JSON.stringify(data, null, 2);
      }
      drawScene();
    }

    async function poll() {
      try {
        const res = await fetch("/api/state", { cache: "no-store" });
        updateUi(await res.json());
      } catch (err) {
        updateUi({ ok: false, error: String(err), targets: {} });
      }
    }

    canvas.addEventListener("mousedown", ev => {
      dragging = true;
      lastMouse = [ev.clientX, ev.clientY];
    });
    window.addEventListener("mouseup", () => dragging = false);
    window.addEventListener("mousemove", ev => {
      if (!dragging) return;
      const dx = ev.clientX - lastMouse[0];
      const dy = ev.clientY - lastMouse[1];
      lastMouse = [ev.clientX, ev.clientY];
      view.yaw += dx * 0.008;
      view.pitch = Math.max(-1.25, Math.min(0.45, view.pitch + dy * 0.006));
      drawScene();
    });
    canvas.addEventListener("wheel", ev => {
      ev.preventDefault();
      view.zoom = Math.max(0.45, Math.min(3.5, view.zoom * Math.exp(-ev.deltaY * 0.001)));
      drawScene();
    }, { passive: false });
    document.getElementById("resetView").onclick = () => { view = { yaw: -0.72, pitch: -0.48, zoom: 1.0, panX: 0, panY: 0 }; drawScene(); };
    document.getElementById("topView").onclick = () => { view.yaw = -Math.PI / 2; view.pitch = -1.23; drawScene(); };
    document.getElementById("sideView").onclick = () => { view.yaw = -0.02; view.pitch = -0.2; drawScene(); };
    document.getElementById("frontView").onclick = () => { view.yaw = -Math.PI / 2; view.pitch = -0.35; drawScene(); };
    [showGridEl, showLabelsEl, showRawEl].forEach(el => el.addEventListener("change", drawScene));
    window.addEventListener("resize", drawScene);
    setInterval(poll, 100);
    poll();
  </script>
</body>
</html>
"""


MODEL_INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Indoory tf.links 3D Viewer</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #050909;
      --panel: #121a1d;
      --panel2: #172226;
      --line: #2a3b42;
      --text: #e6f4f6;
      --muted: #94a8ae;
      --ok: #35f08b;
      --warn: #ffd166;
      --bad: #ff6b72;
      --right: #25e8ff;
      --left: #ffd34d;
      --head: #9ab8ff;
      --base: #7df0b0;
    }
    * { box-sizing: border-box; }
    html, body { height: 100%; }
    body {
      margin: 0;
      overflow: hidden;
      background: var(--bg);
      color: var(--text);
      font: 14px/1.35 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 420px;
    }
    main {
      min-width: 0;
      min-height: 0;
      display: grid;
      grid-template-rows: auto minmax(0, 1fr);
      background: #020606;
    }
    header {
      min-height: 54px;
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 10px 16px;
      border-bottom: 1px solid var(--line);
      background: #0b1113;
    }
    h1 { margin: 0; font-size: 18px; font-weight: 750; white-space: nowrap; }
    .pill {
      display: inline-flex;
      align-items: center;
      height: 26px;
      padding: 0 10px;
      border: 1px solid var(--line);
      border-radius: 999px;
      color: var(--muted);
      white-space: nowrap;
      font-size: 13px;
    }
    .pill.ok { color: var(--ok); border-color: rgba(53,240,139,.45); }
    .pill.warn { color: var(--warn); border-color: rgba(255,209,102,.45); }
    .pill.bad { color: var(--bad); border-color: rgba(255,107,114,.55); }
    #stageWrap {
      position: relative;
      min-height: 0;
      overflow: hidden;
      background:
        radial-gradient(circle at 50% 46%, rgba(31,64,72,.22), transparent 48%),
        linear-gradient(180deg, #061013 0%, #020607 100%);
    }
    #stage {
      width: 100%;
      height: 100%;
      display: block;
      cursor: grab;
    }
    #stage:active { cursor: grabbing; }
    #labels3d {
      position: absolute;
      inset: 0;
      pointer-events: none;
      overflow: hidden;
    }
    .label3d {
      position: absolute;
      transform: translate(9px, -50%);
      max-width: 240px;
      padding: 3px 6px;
      border: 1px solid rgba(123,154,164,.34);
      border-radius: 5px;
      background: rgba(3,8,9,.74);
      color: var(--text);
      font: 11px/1.2 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      white-space: nowrap;
      text-shadow: 0 1px 2px #000;
    }
    #legend {
      position: absolute;
      left: 14px;
      bottom: 12px;
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      max-width: min(780px, calc(100% - 28px));
      color: var(--muted);
      pointer-events: none;
    }
    #legend span {
      display: inline-flex;
      align-items: center;
      min-height: 28px;
      padding: 0 9px;
      border: 1px solid rgba(80,105,113,.5);
      border-radius: 999px;
      background: rgba(7,11,13,.70);
      white-space: nowrap;
    }
    aside {
      min-width: 0;
      min-height: 0;
      border-left: 1px solid var(--line);
      background: var(--panel);
      display: grid;
      grid-template-rows: auto auto auto minmax(0, 1fr);
    }
    .section { padding: 14px; border-bottom: 1px solid var(--line); }
    .section h2 {
      margin: 0 0 10px;
      color: var(--muted);
      font-size: 13px;
      font-weight: 750;
    }
    .toolbar { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 8px; }
    button {
      height: 34px;
      padding: 0 11px;
      border: 1px solid #35505b;
      border-radius: 6px;
      background: #132025;
      color: var(--text);
      font-weight: 700;
    }
    button:hover { background: #1b2c33; }
    label {
      display: flex;
      align-items: center;
      gap: 8px;
      min-height: 28px;
      color: var(--muted);
    }
    input[type="checkbox"] { accent-color: #36d399; }
    table {
      width: 100%;
      border-collapse: collapse;
      font-variant-numeric: tabular-nums;
    }
    th, td {
      padding: 7px 6px;
      border-bottom: 1px solid #223139;
      text-align: left;
      vertical-align: top;
    }
    th { color: var(--muted); font-size: 12px; }
    td { font-size: 12px; }
    td.num { color: #d7f7ff; }
    .scroll { min-height: 0; overflow: auto; }
    pre {
      margin: 0;
      padding: 12px 14px;
      white-space: pre-wrap;
      word-break: break-word;
      color: #bcebd1;
      font: 12px/1.4 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    @media (max-width: 1060px) {
      body { grid-template-columns: 1fr; grid-template-rows: minmax(62vh, 1fr) auto; }
      aside { border-left: 0; border-top: 1px solid var(--line); max-height: 38vh; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>Indoory tf.links 3D Viewer</h1>
      <span id="tfStatus" class="pill warn">tf.links waiting</span>
      <span id="modelStatus" class="pill warn">model loading</span>
      <span id="proprioStatus" class="pill warn">proprio waiting</span>
      <span id="source" class="pill">source -</span>
    </header>
    <div id="stageWrap">
      <canvas id="stage"></canvas>
      <div id="labels3d"></div>
      <div id="legend">
        <span style="color:var(--right)">cyan: right EE/tf.links</span>
        <span style="color:var(--left)">yellow: left EE/tf.links</span>
        <span style="color:var(--head)">blue labels: head camera TF tree</span>
        <span style="color:var(--base)">green labels: mobile base/root frames</span>
      </div>
    </div>
  </main>
  <aside>
    <div class="section">
      <h2>View</h2>
      <div class="toolbar">
        <button id="resetView">Reset</button>
        <button id="topView">Top</button>
        <button id="sideView">Side</button>
        <button id="frontView">Front</button>
      </div>
      <label><input id="showGrid" type="checkbox" checked> Ground grid</label>
      <label><input id="showLabels" type="checkbox" checked> Frame labels</label>
      <label><input id="showAllAxes" type="checkbox" checked> All URDF link axes</label>
      <label><input id="showTfAxes" type="checkbox" checked> tf.links axes</label>
      <label><input id="showRaw" type="checkbox"> Raw telemetry JSON</label>
    </div>
    <div class="section">
      <h2>tf.links Targets</h2>
      <table>
        <thead><tr><th>Name</th><th>X</th><th>Y</th><th>Z</th><th>Src</th></tr></thead>
        <tbody id="tfRows"></tbody>
      </table>
    </div>
    <div class="section">
      <h2>Rendered URDF Frames</h2>
      <table>
        <thead><tr><th>Frame</th><th>X</th><th>Y</th><th>Z</th></tr></thead>
        <tbody id="frameRows"></tbody>
      </table>
    </div>
    <div class="scroll">
      <pre id="raw">waiting for /api/state</pre>
    </div>
  </aside>

  <script>
    let gl = null;
    let lineProgram = null;
    let lineBuffer = null;
    let lineAttribs = null;
    let lineUniforms = null;
    let meshProgram = null;
    let meshAttribs = null;
    let meshUniforms = null;
    let headRgbProgram = null;
    let headRgbBuffer = null;
    let headRgbAttribs = null;
    let headRgbUniforms = null;
    let meshCache = new Map();
    let robotModel = null;
    let robotModelPromise = null;
    let robotModelErrorLogged = false;
    let robotTelemetry = { ok: false, tf_links: {}, tf_targets: {}, proprio: null, age_s: null };
    let robotRenderedEeBasePoints = { right: null, left: null };
    let lastOverlayViewProjections = [];
    let cameraFeeds = {
      wristLeft: { ready: false },
      head: { ready: false },
      wristRight: { ready: false },
    };
    let state = { hmd: [0, 0, 0, 0, 0, 0, 1] };
    const HEAD_CAMERA_FEED_DISTANCE_M = 1.05;
    const HEAD_CAMERA_FEED_HALF_H_M = 0.18;
    const ROBOT_OVERLAY_SCALE = 1.0;
    const ROBOT_OVERLAY_REFERENCE_Z_M = 0.85;
    const ROBOT_OVERLAY_DISTANCE_M = 1.0;
    const ROBOT_OVERLAY_DOWN_M = 0.0;
    function log(message, level) {
      console.log("[tf.links viewer]", level || "info", message);
    }
  </script>
  <script src="/static/app_rendering.js"></script>
  <script>
    const canvas = document.getElementById("stage");
    const labelsLayer = document.getElementById("labels3d");
    const tfStatusEl = document.getElementById("tfStatus");
    const modelStatusEl = document.getElementById("modelStatus");
    const proprioStatusEl = document.getElementById("proprioStatus");
    const sourceEl = document.getElementById("source");
    const tfRowsEl = document.getElementById("tfRows");
    const frameRowsEl = document.getElementById("frameRows");
    const rawEl = document.getElementById("raw");
    const showGridEl = document.getElementById("showGrid");
    const showLabelsEl = document.getElementById("showLabels");
    const showAllAxesEl = document.getElementById("showAllAxes");
    const showTfAxesEl = document.getElementById("showTfAxes");
    const showRawEl = document.getElementById("showRaw");

    const RIGHT_COLOR = [0.08, 0.92, 1.0, 0.98];
    const LEFT_COLOR = [1.0, 0.82, 0.12, 0.98];
    const HEAD_COLOR = [0.58, 0.72, 1.0, 0.88];
    const BASE_COLOR = [0.45, 0.95, 0.70, 0.88];
    const ALL_AXIS_ALPHA = 0.26;
    const latest = { state: { ok: false, targets: {} } };
    const frameMatrices = new Map();
    let labelItems = [];
    let lastFrameRowUpdate = 0;
    let modelLoadStarted = false;
    let dragging = false;
    let lastMouse = [0, 0];
    const orbit = {
      target: [0.02, 0.0, 0.72],
      yaw: -0.82,
      pitch: 0.34,
      distance: 1.88,
      autoFrame: true,
    };

    const IMPORTANT_FRAMES = [
      "root", "root_arm_1_link_1", "root_arm_1_link_2", "base_link", "top_base_link",
      "head_pan_link", "head_tilt_link", "head_camera_link",
      "head_camera_rgb_frame", "head_camera_rgb_optical_frame",
      "head_camera_depth_frame", "head_camera_depth_optical_frame",
      "Base", "Base_2", "Fixed_Jaw_tip", "Fixed_Jaw_tip_2",
    ];
    const BASE_TREE = [
      ["root", "root_arm_1_link_1"],
      ["root_arm_1_link_1", "root_arm_1_link_2"],
      ["root_arm_1_link_2", "base_link"],
      ["base_link", "top_base_link"],
    ];
    const HEAD_TREE = [
      ["top_base_link", "head_pan_link"],
      ["head_pan_link", "head_tilt_link"],
      ["head_tilt_link", "head_camera_link"],
      ["head_camera_link", "head_camera_rgb_frame"],
      ["head_camera_rgb_frame", "head_camera_rgb_optical_frame"],
      ["head_camera_link", "head_camera_depth_frame"],
      ["head_camera_depth_frame", "head_camera_depth_optical_frame"],
    ];
    const ARM_MOUNT_TREE = [
      ["base_link", "Base"],
      ["base_link", "Base_2"],
      ["Base", "Fixed_Jaw_tip"],
      ["Base_2", "Fixed_Jaw_tip_2"],
    ];
    const TF_ALIAS_TARGETS = new Set([
      "gripper_right", "jaw_right", "gripper_left", "jaw_left",
      "head_pan", "head_tilt", "base_link_odom",
    ]);

    function setPill(el, text, level) {
      el.textContent = text;
      el.className = "pill " + (level || "");
    }

    function fmt(value) {
      const n = Number(value);
      return Number.isFinite(n) ? n.toFixed(3) : "-";
    }

    function colorCss(color) {
      const r = Math.round((color[0] || 0) * 255);
      const g = Math.round((color[1] || 0) * 255);
      const b = Math.round((color[2] || 0) * 255);
      return `rgb(${r}, ${g}, ${b})`;
    }

    function resize() {
      const rect = canvas.getBoundingClientRect();
      const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
      const w = Math.max(360, Math.floor(rect.width * dpr));
      const h = Math.max(300, Math.floor(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }
      if (gl) gl.viewport(0, 0, canvas.width, canvas.height);
    }

    function vSub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
    function vAdd(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
    function vScale(v, s) { return [v[0] * s, v[1] * s, v[2] * s]; }
    function vDot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    function vCross(a, b) {
      return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
      ];
    }
    function vNormalize(v) {
      const len = Math.hypot(v[0], v[1], v[2]);
      if (!Number.isFinite(len) || len < 1e-9) return [0, 0, 0];
      return [v[0] / len, v[1] / len, v[2] / len];
    }

    function perspective(fovyRad, aspect, near, far) {
      const f = 1.0 / Math.tan(fovyRad * 0.5);
      const nf = 1.0 / (near - far);
      return new Float32Array([
        f / aspect, 0, 0, 0,
        0, f, 0, 0,
        0, 0, (far + near) * nf, -1,
        0, 0, (2 * far * near) * nf, 0,
      ]);
    }

    function lookAt(eye, center, up) {
      const z = vNormalize(vSub(eye, center));
      const x = vNormalize(vCross(up, z));
      const y = vCross(z, x);
      return new Float32Array([
        x[0], y[0], z[0], 0,
        x[1], y[1], z[1], 0,
        x[2], y[2], z[2], 0,
        -vDot(x, eye), -vDot(y, eye), -vDot(z, eye), 1,
      ]);
    }

    function viewMatrices() {
      const cp = Math.cos(orbit.pitch);
      const eye = [
        orbit.target[0] + orbit.distance * cp * Math.cos(orbit.yaw),
        orbit.target[1] + orbit.distance * cp * Math.sin(orbit.yaw),
        orbit.target[2] + orbit.distance * Math.sin(orbit.pitch),
      ];
      const projectionMatrix = perspective(
        48 * Math.PI / 180,
        Math.max(0.2, canvas.width / Math.max(1, canvas.height)),
        0.02,
        20.0
      );
      const viewMatrix = lookAt(eye, orbit.target, [0, 0, 1]);
      return {
        eye,
        projectionMatrix,
        viewMatrix,
        viewProjection: mulMat4(projectionMatrix, viewMatrix),
        drawView: { projectionMatrix, transform: { inverse: { matrix: viewMatrix } } },
      };
    }

    function setDefaultOrbit() {
      orbit.target = [0.02, 0.0, 0.72];
      orbit.yaw = -0.82;
      orbit.pitch = 0.34;
      orbit.distance = 1.88;
      orbit.autoFrame = true;
    }

    function framePointCandidates() {
      const points = [];
      const addPoint = point => {
        if (!Array.isArray(point) || point.length < 3) return;
        const x = Number(point[0]);
        const y = Number(point[1]);
        const z = Number(point[2]);
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return;
        points.push([x, y, z]);
      };
      Object.values((latest.state && latest.state.link_frames) || {}).forEach(addPoint);
      Object.entries((latest.state && latest.state.targets) || {}).forEach(([name, point]) => {
        if (name === "base_link_odom") return;
        addPoint(point);
      });
      frameMatrices.forEach(matrix => addPoint(matrixOrigin(matrix)));
      return points;
    }

    function autoFrameOrbit() {
      if (!orbit.autoFrame) return;
      const points = framePointCandidates();
      if (points.length < 4) return;
      let minX = Infinity, minY = Infinity, minZ = Infinity;
      let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
      points.forEach(([x, y, z]) => {
        minX = Math.min(minX, x); minY = Math.min(minY, y); minZ = Math.min(minZ, z);
        maxX = Math.max(maxX, x); maxY = Math.max(maxY, y); maxZ = Math.max(maxZ, z);
      });
      const center = [
        (minX + maxX) * 0.5,
        (minY + maxY) * 0.5,
        (minZ + maxZ) * 0.5,
      ];
      const spanX = Math.max(0.08, maxX - minX);
      const spanY = Math.max(0.08, maxY - minY);
      const spanZ = Math.max(0.08, maxZ - minZ);
      const radius = Math.max(0.35, Math.hypot(spanX, spanY, spanZ) * 0.5);
      const aspect = Math.max(0.2, canvas.width / Math.max(1, canvas.height));
      const fov = 48 * Math.PI / 180;
      const fitDistance = radius / Math.sin(fov * 0.5) * (aspect < 1.0 ? 1.18 : 1.04);
      const nextDistance = Math.max(0.70, Math.min(3.4, fitDistance));
      const blend = 0.16;
      orbit.target = [
        orbit.target[0] + (center[0] - orbit.target[0]) * blend,
        orbit.target[1] + (center[1] - orbit.target[1]) * blend,
        orbit.target[2] + (center[2] - orbit.target[2]) * blend,
      ];
      orbit.distance += (nextDistance - orbit.distance) * blend;
    }

    function matrixPoint(m, p) {
      const x = p[0], y = p[1], z = p[2];
      return [
        m[0] * x + m[4] * y + m[8] * z + m[12],
        m[1] * x + m[5] * y + m[9] * z + m[13],
        m[2] * x + m[6] * y + m[10] * z + m[14],
      ];
    }

    function matrixOrigin(m) {
      return [Number(m[12] || 0), Number(m[13] || 0), Number(m[14] || 0)];
    }

    function addLabel(point, text, color) {
      if (!showLabelsEl.checked || !point) return;
      labelItems.push({ point, text, color });
    }

    function appendMatrixAxis(vertices, matrix, size, alpha) {
      if (!matrix) return;
      const origin = matrixPoint(matrix, [0, 0, 0]);
      pushLine(vertices, origin, matrixPoint(matrix, [size, 0, 0]), [1.0, 0.20, 0.16, alpha]);
      pushLine(vertices, origin, matrixPoint(matrix, [0, size, 0]), [0.18, 0.95, 0.35, alpha]);
      pushLine(vertices, origin, matrixPoint(matrix, [0, 0, size]), [0.25, 0.50, 1.0, alpha]);
    }

    function quatRotate(q, v) {
      const x = Number(q[0] || 0), y = Number(q[1] || 0), z = Number(q[2] || 0), w = Number(q[3] == null ? 1 : q[3]);
      const vx = v[0], vy = v[1], vz = v[2];
      const uv = [y * vz - z * vy, z * vx - x * vz, x * vy - y * vx];
      const uuv = [y * uv[2] - z * uv[1], z * uv[0] - x * uv[2], x * uv[1] - y * uv[0]];
      return [
        vx + 2 * (w * uv[0] + uuv[0]),
        vy + 2 * (w * uv[1] + uuv[1]),
        vz + 2 * (w * uv[2] + uuv[2]),
      ];
    }

    function appendPoseAxis(vertices, pose, size, alpha) {
      if (!Array.isArray(pose) || pose.length < 7) return;
      const origin = [Number(pose[0] || 0), Number(pose[1] || 0), Number(pose[2] || 0)];
      const q = [pose[3], pose[4], pose[5], pose[6]];
      const x = vAdd(origin, quatRotate(q, [size, 0, 0]));
      const y = vAdd(origin, quatRotate(q, [0, size, 0]));
      const z = vAdd(origin, quatRotate(q, [0, 0, size]));
      pushLine(vertices, origin, x, [1.0, 0.20, 0.16, alpha]);
      pushLine(vertices, origin, y, [0.18, 0.95, 0.35, alpha]);
      pushLine(vertices, origin, z, [0.25, 0.50, 1.0, alpha]);
      pushCross(vertices, origin, size * 0.34, [0.95, 0.95, 0.95, alpha * 0.78]);
    }

    function appendGrid(vertices) {
      if (!showGridEl.checked) return;
      const color = [0.34, 0.45, 0.48, 0.28];
      for (let i = -10; i <= 10; i += 1) {
        const v = i * 0.1;
        pushLine(vertices, [v, -0.65, 0], [v, 0.65, 0], color);
        pushLine(vertices, [-0.65, v, 0], [0.65, v, 0], color);
      }
      pushLine(vertices, [-0.72, 0, 0], [0.72, 0, 0], [1.0, 0.20, 0.16, 0.45]);
      pushLine(vertices, [0, -0.72, 0], [0, 0.72, 0], [0.18, 0.95, 0.35, 0.45]);
    }

    function frameMatrix(name) {
      return frameMatrices.get(name) || null;
    }

    function appendTree(vertices, edges, color) {
      edges.forEach(([a, b]) => {
        const ma = frameMatrix(a);
        const mb = frameMatrix(b);
        if (!ma || !mb) return;
        pushLine(vertices, matrixOrigin(ma), matrixOrigin(mb), color);
      });
    }

    function sideForFrame(name) {
      if (String(name).endsWith("_2") || String(name).includes("Left")) return "left";
      if (String(name).includes("right") || String(name).includes("Right")) return "right";
      return "";
    }

    function labelImportantFrames() {
      IMPORTANT_FRAMES.forEach(name => {
        const m = frameMatrix(name);
        if (!m) return;
        let color = "#dceff3";
        if (name.startsWith("head_")) color = "var(--head)";
        if (name === "top_base_link") color = "var(--head)";
        if (name === "root" || name.includes("base_link") || name.startsWith("root_arm")) color = "var(--base)";
        if (name === "Base" || name === "Fixed_Jaw_tip") color = "var(--right)";
        if (name === "Base_2" || name === "Fixed_Jaw_tip_2") color = "var(--left)";
        addLabel(matrixOrigin(m), name, color);
      });
    }

    function drawRobotNode(node, localMatrix, viewProjection, values) {
      if (!node || !node.link) return;
      frameMatrices.set(String(node.link.name || ""), localMatrix);
      for (const visual of node.visuals || []) {
        const localVisual = mulMat4(mulMat4(localMatrix, visual.originMatrix), visual.scaleMatrix);
        drawMesh(visual.mesh, localVisual, viewProjection, visual.color);
      }
      for (const child of node.children || []) {
        const jointValue = Number(values[child.joint.name] || 0);
        const childMatrix = mulMat4(
          mulMat4(localMatrix, child.originMatrix),
          mat4FromJointMotion(child.joint, jointValue)
        );
        drawRobotNode(child.child, childMatrix, viewProjection, values);
      }
    }

    async function ensureModelLoaded() {
      if (robotModel || modelLoadStarted) return robotModelPromise;
      modelLoadStarted = true;
      robotModelPromise = loadRobotModel()
        .then(model => {
          robotModel = model;
          setPill(
            modelStatusEl,
            model ? `model ${model.stats.loadedVisuals}/${model.stats.loadedVisuals + model.stats.failedVisuals} visuals` : "model missing",
            model ? "ok" : "bad"
          );
          return model;
        })
        .catch(err => {
          setPill(modelStatusEl, "model error", "bad");
          rawEl.textContent = String(err);
          return null;
        });
      return robotModelPromise;
    }

    function renderRobot(viewProjection) {
      if (!robotModel || !robotModel.root) return false;
      const values = jointValuesFromProprio(latest.state.proprio);
      frameMatrices.clear();
      ensureMeshRenderer();
      gl.useProgram(meshProgram);
      gl.enable(gl.DEPTH_TEST);
      gl.depthFunc(gl.LEQUAL);
      gl.depthMask(true);
      drawRobotNode(robotModel.root, mat4Identity(), viewProjection, values);
      return true;
    }

    function renderOverlay(drawView, viewProjection) {
      const vertices = [];
      appendGrid(vertices);
      if (showAllAxesEl.checked) {
        frameMatrices.forEach(matrix => appendMatrixAxis(vertices, matrix, 0.032, ALL_AXIS_ALPHA));
      }
      appendTree(vertices, BASE_TREE, BASE_COLOR);
      appendTree(vertices, HEAD_TREE, HEAD_COLOR);
      appendTree(vertices, ARM_MOUNT_TREE, [0.80, 0.65, 1.0, 0.55]);
      ["root", "base_link", "top_base_link"].forEach(name => {
        const m = frameMatrix(name);
        if (m) appendMatrixAxis(vertices, m, 0.105, 0.88);
      });
      ["head_pan_link", "head_tilt_link", "head_camera_link", "head_camera_rgb_optical_frame", "head_camera_depth_optical_frame"].forEach(name => {
        const m = frameMatrix(name);
        if (m) appendMatrixAxis(vertices, m, 0.080, 0.92);
      });
      [
        ["Fixed_Jaw_tip", RIGHT_COLOR],
        ["Fixed_Jaw_tip_2", LEFT_COLOR],
        ["Base", RIGHT_COLOR],
        ["Base_2", LEFT_COLOR],
      ].forEach(([name, color]) => {
        const m = frameMatrix(name);
        if (m) appendMatrixAxis(vertices, m, name.startsWith("Fixed") ? 0.090 : 0.070, 0.96);
      });

      const targets = latest.state.targets || {};
      const linkFrames = latest.state.link_frames || {};
      if (showTfAxesEl.checked) {
        IMPORTANT_FRAMES.forEach(name => {
          const pose = linkFrames[name];
          if (!Array.isArray(pose)) return;
          appendPoseAxis(vertices, pose, 0.048, 0.58);
          addLabel([pose[0], pose[1], pose[2]], `server ${name}`, "#f4d6ff");
        });
        Object.entries(targets).forEach(([name, pose]) => {
          if (!TF_ALIAS_TARGETS.has(name) && linkFrames[name]) return;
          const side = name.includes("left") ? "left" : name.includes("right") ? "right" : "";
          const labelColor = side === "left" ? "var(--left)" : side === "right" ? "var(--right)" : "var(--head)";
          const lineColor = side === "left" ? LEFT_COLOR : side === "right" ? RIGHT_COLOR : HEAD_COLOR;
          appendPoseAxis(vertices, pose, 0.075, 0.95);
          if (Array.isArray(pose)) {
            addLabel([pose[0], pose[1], pose[2]], `tf.links ${name}`, labelColor);
            const renderedFrame = name.includes("left") ? "Fixed_Jaw_tip_2" : name.includes("right") ? "Fixed_Jaw_tip" : null;
            const rendered = renderedFrame ? frameMatrix(renderedFrame) : null;
            if (rendered) pushLine(vertices, matrixOrigin(rendered), [pose[0], pose[1], pose[2]], lineColor);
          }
        });
      }
      labelImportantFrames();
      gl.disable(gl.DEPTH_TEST);
      gl.depthMask(false);
      drawLineVertices(drawView, vertices);
      gl.depthMask(true);
      positionLabels(viewProjection);
    }

    function positionLabels(viewProjection) {
      labelsLayer.replaceChildren();
      if (!showLabelsEl.checked) return;
      const rect = canvas.getBoundingClientRect();
      labelItems.forEach(item => {
        const projected = projectWorldPointToNdc(viewProjection, item.point);
        if (!projected || projected.clip_w <= 0) return;
        const x = (projected.ndc[0] * 0.5 + 0.5) * rect.width;
        const y = (-projected.ndc[1] * 0.5 + 0.5) * rect.height;
        if (x < -40 || x > rect.width + 40 || y < -30 || y > rect.height + 30) return;
        const el = document.createElement("span");
        el.className = "label3d";
        el.textContent = item.text;
        el.style.left = `${x}px`;
        el.style.top = `${y}px`;
        el.style.color = item.color;
        labelsLayer.appendChild(el);
      });
    }

    function render() {
      if (!gl) return;
      resize();
      labelItems = [];
      autoFrameOrbit();
      const { projectionMatrix, viewMatrix, viewProjection, drawView } = viewMatrices();
      gl.clearColor(0.010, 0.026, 0.028, 1.0);
      gl.clearDepth(1.0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      renderRobot(viewProjection);
      renderOverlay(drawView, viewProjection);
      maybeUpdateFrameTable();
      requestAnimationFrame(render);
    }

    function targetColor(name) {
      if (name.includes("right")) return "var(--right)";
      if (name.includes("left")) return "var(--left)";
      return "var(--head)";
    }

    function updateTfTable() {
      const targets = latest.state.targets || {};
      const sources = latest.state.target_sources || {};
      const names = Object.keys(targets).sort((a, b) => {
        const order = ["gripper_right", "jaw_right", "gripper_left", "jaw_left", "head_pan", "head_tilt"];
        const ai = order.indexOf(a);
        const bi = order.indexOf(b);
        return (ai < 0 ? 100 : ai) - (bi < 0 ? 100 : bi) || a.localeCompare(b);
      });
      tfRowsEl.innerHTML = names.map(name => {
        const p = targets[name] || [];
        return `<tr><td style="color:${targetColor(name)}">${name}</td><td class="num">${fmt(p[0])}</td><td class="num">${fmt(p[1])}</td><td class="num">${fmt(p[2])}</td><td>${sources[name] || "-"}</td></tr>`;
      }).join("") || `<tr><td colspan="5">waiting</td></tr>`;
    }

    function maybeUpdateFrameTable() {
      const now = performance.now();
      if (now - lastFrameRowUpdate < 250) return;
      lastFrameRowUpdate = now;
      const rows = IMPORTANT_FRAMES
        .map(name => {
          const m = frameMatrix(name);
          if (!m) return null;
          const p = matrixOrigin(m);
          let color = "#dceff3";
          if (name.startsWith("head_") || name === "top_base_link") color = "var(--head)";
          if (name === "root" || name.includes("base_link") || name.startsWith("root_arm")) color = "var(--base)";
          if (name === "Base" || name === "Fixed_Jaw_tip") color = "var(--right)";
          if (name === "Base_2" || name === "Fixed_Jaw_tip_2") color = "var(--left)";
          return `<tr><td style="color:${color}">${name}</td><td class="num">${fmt(p[0])}</td><td class="num">${fmt(p[1])}</td><td class="num">${fmt(p[2])}</td></tr>`;
        })
        .filter(Boolean);
      frameRowsEl.innerHTML = rows.join("") || `<tr><td colspan="4">model loading</td></tr>`;
    }

    function updateTelemetry(data) {
      latest.state = data || { ok: false, targets: {} };
      robotTelemetry.proprio = latest.state.proprio || null;
      const tfAge = Number(latest.state.age_s);
      const tfLive = Boolean(latest.state.ok) && Number.isFinite(tfAge) && tfAge < 1.0;
      setPill(tfStatusEl, tfLive ? `tf.links ${tfAge.toFixed(3)}s` : (latest.state.ok ? "tf.links stale" : "tf.links waiting"), tfLive ? "ok" : latest.state.ok ? "warn" : "bad");
      const proprioAge = Number(latest.state.proprio_age_s);
      const proprioLive = latest.state.proprio && Number.isFinite(proprioAge) && proprioAge < 1.0;
      setPill(proprioStatusEl, proprioLive ? `proprio ${proprioAge.toFixed(3)}s` : (latest.state.proprio ? "proprio stale" : "proprio waiting"), proprioLive ? "ok" : latest.state.proprio ? "warn" : "bad");
      sourceEl.textContent = latest.state.source_note || latest.state.endpoint || "source -";
      updateTfTable();
      rawEl.style.display = showRawEl.checked ? "block" : "none";
      if (showRawEl.checked) rawEl.textContent = JSON.stringify(latest.state, null, 2);
    }

    async function poll() {
      try {
        const res = await fetch("/api/state", { cache: "no-store" });
        updateTelemetry(await res.json());
      } catch (err) {
        updateTelemetry({ ok: false, error: String(err), targets: {} });
      }
    }

    function initGl() {
      gl = canvas.getContext("webgl", { antialias: true, alpha: false, powerPreference: "high-performance" });
      if (!gl) {
        setPill(modelStatusEl, "webgl unavailable", "bad");
        return false;
      }
      gl.disable(gl.CULL_FACE);
      return true;
    }

    canvas.addEventListener("mousedown", ev => {
      dragging = true;
      orbit.autoFrame = false;
      lastMouse = [ev.clientX, ev.clientY];
    });
    window.addEventListener("mouseup", () => { dragging = false; });
    window.addEventListener("mousemove", ev => {
      if (!dragging) return;
      const dx = ev.clientX - lastMouse[0];
      const dy = ev.clientY - lastMouse[1];
      lastMouse = [ev.clientX, ev.clientY];
      if (ev.shiftKey) {
        const side = [Math.sin(orbit.yaw), -Math.cos(orbit.yaw), 0];
        const up = [0, 0, 1];
        orbit.target = vAdd(orbit.target, vAdd(vScale(side, -dx * 0.0014), vScale(up, dy * 0.0014)));
      } else {
        orbit.yaw += dx * 0.007;
        orbit.pitch = Math.max(-0.25, Math.min(1.42, orbit.pitch + dy * 0.005));
      }
    });
    canvas.addEventListener("wheel", ev => {
      ev.preventDefault();
      orbit.autoFrame = false;
      orbit.distance = Math.max(0.45, Math.min(5.0, orbit.distance * Math.exp(ev.deltaY * 0.001)));
    }, { passive: false });
    document.getElementById("resetView").onclick = () => {
      setDefaultOrbit();
    };
    document.getElementById("topView").onclick = () => {
      orbit.autoFrame = false;
      orbit.target = [0.02, 0.0, 0.55];
      orbit.yaw = -Math.PI / 2;
      orbit.pitch = 1.38;
      orbit.distance = 1.72;
    };
    document.getElementById("sideView").onclick = () => {
      orbit.autoFrame = false;
      orbit.target = [0.02, 0.0, 0.72];
      orbit.yaw = 0.0;
      orbit.pitch = 0.20;
      orbit.distance = 1.80;
    };
    document.getElementById("frontView").onclick = () => {
      orbit.autoFrame = false;
      orbit.target = [0.02, 0.0, 0.72];
      orbit.yaw = -Math.PI / 2;
      orbit.pitch = 0.20;
      orbit.distance = 1.80;
    };
    showRawEl.addEventListener("change", () => updateTelemetry(latest.state));
    [showGridEl, showLabelsEl, showAllAxesEl, showTfAxesEl].forEach(el => {
      el.addEventListener("change", () => {});
    });
    window.addEventListener("resize", resize);

    if (initGl()) {
      ensureModelLoaded();
      poll();
      setInterval(poll, 80);
      requestAnimationFrame(render);
    }
  </script>
</body>
</html>
"""


def _finite_float(value: Any, default: float | None = None) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


class FastTfState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.last_tf: dict[str, Any] | None = None
        self.last_tf_at = 0.0
        self.last_proprio: dict[str, Any] | None = None
        self.last_proprio_at = 0.0
        self.last_joint_states: dict[str, Any] | None = None
        self.last_joint_states_at = 0.0
        self.last_topic = ""
        self.last_error = ""
        self.frames: dict[str, int] = {}

    def update_tf(self, topic: str, msg: dict[str, Any]) -> None:
        with self.lock:
            self.last_tf = msg
            self.last_tf_at = time.monotonic()
            self.last_topic = topic
            self.last_error = ""
            self.frames[topic] = self.frames.get(topic, 0) + 1

    def update_proprio(self, topic: str, msg: dict[str, Any]) -> None:
        with self.lock:
            self.last_proprio = msg
            self.last_proprio_at = time.monotonic()
            self.last_error = ""
            self.frames[topic] = self.frames.get(topic, 0) + 1

    def update_joint_states(self, topic: str, msg: dict[str, Any]) -> None:
        with self.lock:
            self.last_joint_states = msg
            self.last_joint_states_at = time.monotonic()
            self.last_error = ""
            self.frames[topic] = self.frames.get(topic, 0) + 1

    def set_error(self, error: str) -> None:
        with self.lock:
            self.last_error = error

    def snapshot(self, endpoint: str) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            tf_msg = dict(self.last_tf or {})
            age_s = now - self.last_tf_at if self.last_tf_at else None
            proprio_msg = dict(self.last_proprio or {})
            proprio_age_s = now - self.last_proprio_at if self.last_proprio_at else None
            joint_states_msg = dict(self.last_joint_states or {})
            joint_states_age_s = (
                now - self.last_joint_states_at if self.last_joint_states_at else None
            )
            topic = self.last_topic
            error = self.last_error
            frames = dict(self.frames)
        targets: dict[str, list[float]] = {}
        sources: dict[str, str] = {}
        for entry in tf_msg.get("targets") or []:
            if not isinstance(entry, dict):
                continue
            name = str(entry.get("name") or "")
            pose = entry.get("pose")
            if not name or not isinstance(pose, (list, tuple)) or len(pose) < 7:
                continue
            parsed = [_finite_float(value, 0.0) for value in pose[:7]]
            if any(value is None for value in parsed):
                continue
            targets[name] = [float(value) for value in parsed if value is not None]
            sources[name] = str(entry.get("source") or "")
        link_frames: dict[str, list[float]] = {}
        raw_link_frames = tf_msg.get("link_frames")
        if isinstance(raw_link_frames, dict):
            frame_items = raw_link_frames.items()
        elif isinstance(raw_link_frames, list):
            frame_items = (
                (entry.get("name"), entry.get("pose"))
                for entry in raw_link_frames
                if isinstance(entry, dict)
            )
        else:
            frame_items = []
        for name_raw, pose in frame_items:
            name = str(name_raw or "")
            if not name or not isinstance(pose, (list, tuple)) or len(pose) < 7:
                continue
            parsed = [_finite_float(value, 0.0) for value in pose[:7]]
            if any(value is None for value in parsed):
                continue
            link_frames[name] = [float(value) for value in parsed if value is not None]
        ee_links = {
            "right": targets.get("gripper_right") or targets.get("jaw_right"),
            "left": targets.get("gripper_left") or targets.get("jaw_left"),
        }
        return {
            "ok": bool(tf_msg),
            "endpoint": endpoint,
            "topic": topic,
            "age_s": age_s,
            "frames": frames,
            "error": error or None,
            "stamp_ns": tf_msg.get("stamp_ns"),
            "frame": tf_msg.get("frame"),
            "source": tf_msg.get("source"),
            "source_note": tf_msg.get("source_note"),
            "targets": targets,
            "target_sources": sources,
            "ee_links": ee_links,
            "link_frames": link_frames,
            "tree_edges": tf_msg.get("tree_edges") or [],
            "base_pose_odom": tf_msg.get("base_pose_odom"),
            "proprio": proprio_msg or None,
            "proprio_age_s": proprio_age_s,
            "joint_states": joint_states_msg or None,
            "joint_states_age_s": joint_states_age_s,
            "raw_tf": tf_msg,
        }


class FastTfSubscriber(threading.Thread):
    def __init__(self, state: FastTfState, host: str, pub_port: int, robot_id: int) -> None:
        super().__init__(name="fast-tf-links-sub", daemon=True)
        self.state = state
        self.host = host
        self.pub_port = int(pub_port)
        self.robot_id = int(robot_id)
        self.stop_event = threading.Event()
        self.endpoint = f"tcp://{self.host}:{self.pub_port}"
        self.tf_topic = f"tf.links.{self.robot_id}"
        self.proprio_topic = f"proprio.{self.robot_id}"
        self.joint_states_topic = f"joint_states.{self.robot_id}"
        self.topics = [self.tf_topic, self.proprio_topic, self.joint_states_topic]

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        while not self.stop_event.is_set():
            sock = None
            try:
                ctx = zmq.Context.instance()
                sock = ctx.socket(zmq.SUB)
                sock.setsockopt(zmq.LINGER, 0)
                sock.setsockopt(zmq.RCVHWM, 8)
                sock.setsockopt(zmq.RCVTIMEO, 250)
                sock.connect(self.endpoint)
                for topic in self.topics:
                    sock.setsockopt(zmq.SUBSCRIBE, topic.encode("ascii"))
                poller = zmq.Poller()
                poller.register(sock, zmq.POLLIN)
                while not self.stop_event.is_set():
                    events = dict(poller.poll(250))
                    if sock not in events:
                        continue
                    topic_raw, payload_raw = sock.recv_multipart(flags=zmq.NOBLOCK)
                    topic = topic_raw.decode("ascii", errors="replace")
                    msg = msgpack.unpackb(payload_raw, raw=False)
                    if isinstance(msg, dict):
                        if topic == self.tf_topic:
                            self.state.update_tf(topic, msg)
                        elif topic == self.proprio_topic:
                            self.state.update_proprio(topic, msg)
                        elif topic == self.joint_states_topic:
                            self.state.update_joint_states(topic, msg)
            except Exception as exc:
                self.state.set_error(str(exc))
                time.sleep(0.5)
            finally:
                if sock is not None:
                    sock.close(0)


class ViewerHandler(BaseHTTPRequestHandler):
    server_version = "TfLinks3DWebView/1.0"

    def do_GET(self) -> None:
        parsed = urlsplit(self.path)
        path = parsed.path
        if path == "/":
            self._send_bytes(MODEL_INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/api/state":
            state = self.server.tf_state.snapshot(self.server.fast_endpoint)  # type: ignore[attr-defined]
            self._send_json(state)
            return
        if path == "/api/model/xlerobot.json":
            if xlerobot_model_description is None:
                self._send_json(
                    {
                        "ok": False,
                        "error": str(XLEROBOT_MODEL_IMPORT_ERROR or "model helper unavailable"),
                    },
                    status=500,
                )
                return
            self._send_json(xlerobot_model_description())
            return
        if path == "/static/app_rendering.js":
            self._send_file(TELEOP_WEBXR_DIR / "app_rendering.js", "application/javascript")
            return
        if path.startswith("/assets/robots/xlerobot/"):
            self._send_robot_asset(path)
            return
        if path == "/favicon.ico":
            self._send_bytes(b"", "image/x-icon", status=204)
            return
        self.send_error(404, "not found")

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.path.startswith("/api/state"):
            return
        super().log_message(fmt, *args)

    def _send_json(self, data: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(data, separators=(",", ":"), allow_nan=False, default=str).encode("utf-8")
        self._send_bytes(body, "application/json; charset=utf-8", status=status)

    def _send_file(self, path: Path, content_type: str | None = None) -> None:
        try:
            body = path.read_bytes()
        except OSError:
            self.send_error(404, "not found")
            return
        detected = content_type or mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        self._send_bytes(body, detected)

    def _send_robot_asset(self, path: str) -> None:
        rel = unquote(path.removeprefix("/assets/robots/xlerobot/"))
        asset_root = Path(ROBOT_ASSET_DIR).resolve()
        target = (asset_root / rel).resolve()
        if target != asset_root and asset_root not in target.parents:
            self.send_error(403, "invalid asset path")
            return
        content_type = mimetypes.guess_type(str(target))[0]
        if target.suffix.lower() == ".stl":
            content_type = "model/stl"
        elif target.suffix.lower() == ".ply":
            content_type = "application/octet-stream"
        self._send_file(target, content_type)

    def _send_bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if body:
            self.wfile.write(body)


def _public_host() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("TF_LINKS_WEB_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("TF_LINKS_WEB_PORT", "8097")))
    parser.add_argument("--fast-zmq-host", default=os.environ.get("TF_LINKS_FAST_ZMQ_HOST", "127.0.0.1"))
    parser.add_argument("--fast-zmq-pub-port", type=int, default=int(os.environ.get("FAST_ZMQ_PUB_PORT", "8855")))
    parser.add_argument("--robot-id", type=int, default=int(os.environ.get("FAST_ZMQ_ROBOT_ID", "0")))
    args = parser.parse_args()

    state = FastTfState()
    sub = FastTfSubscriber(state, args.fast_zmq_host, args.fast_zmq_pub_port, args.robot_id)
    sub.start()

    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    server.tf_state = state  # type: ignore[attr-defined]
    server.fast_endpoint = sub.endpoint  # type: ignore[attr-defined]

    def shutdown(_signum: int, _frame: Any) -> None:
        sub.stop()
        threading.Thread(target=server.shutdown, name="http-shutdown", daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    host = _public_host() if args.host in ("0.0.0.0", "::") else args.host
    print(
        f"tf.links 3D webview listening on http://{host}:{args.port}/ "
        f"(fast_zmq={sub.endpoint}, topics={','.join(sub.topics)})",
        flush=True,
    )
    try:
        server.serve_forever()
    finally:
        sub.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
