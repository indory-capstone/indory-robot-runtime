#!/usr/bin/env python3
"""Arms/head raw motor command webview with live fast-ZMQ tf.links telemetry."""

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
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlsplit

try:
    import msgpack
    import zmq
except Exception as exc:  # pragma: no cover - startup path
    if os.environ.get("ARMS_HEAD_TF_WEBVIEW_REEXEC") != "1":
        candidates = [
            Path(os.path.expanduser(os.environ.get("XLE_ROBOT_VENV", "~/xlerobot-io-venv"))) / "bin" / "python3",
            Path(os.path.expanduser("~/.miniforge3/envs/lerobot/bin/python3")),
        ]
        for python in candidates:
            if python.exists() and python.resolve() != Path(sys.executable).resolve():
                env = os.environ.copy()
                env["ARMS_HEAD_TF_WEBVIEW_REEXEC"] = "1"
                os.execve(str(python), [str(python), *sys.argv], env)
    print(f"[err] pyzmq and msgpack are required: {exc}", file=sys.stderr)
    raise SystemExit(1)


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from tf_links_3d_webview import (  # noqa: E402
    MODEL_INDEX_HTML,
    ROBOT_ASSET_DIR,
    TELEOP_WEBXR_DIR,
    XLEROBOT_MODEL_IMPORT_ERROR,
    FastTfState,
    FastTfSubscriber,
    xlerobot_model_description,
)


SCHEMA_VERSION_V11 = "xlerobot_v1.1"
DEFAULT_CALIBRATION_DIR = "~/.cache/huggingface/lerobot/calibration/robots/xlerobot"


def _truthy(value: str | None, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class JointDef:
    external: str
    xlerobot: str
    label: str
    group: str
    index: int


def _head_xle_names() -> tuple[str, str]:
    if _truthy(os.environ.get("XLEROBOT_SWAP_HEAD_IDS"), False):
        return "head_motor_2", "head_motor_1"
    return "head_motor_1", "head_motor_2"


HEAD_PAN_XLE, HEAD_TILT_XLE = _head_xle_names()

JOINT_DEFS: tuple[JointDef, ...] = (
    JointDef("left_hand_1", "left_arm_shoulder_pan", "shoulder pan", "left", 0),
    JointDef("left_hand_2", "left_arm_shoulder_lift", "shoulder lift", "left", 1),
    JointDef("left_hand_3", "left_arm_elbow_flex", "elbow flex", "left", 2),
    JointDef("left_hand_4", "left_arm_wrist_flex", "wrist flex", "left", 3),
    JointDef("left_hand_5", "left_arm_wrist_roll", "wrist roll", "left", 4),
    JointDef("left_hand_6", "left_arm_gripper", "gripper", "left", 5),
    JointDef("right_hand_1", "right_arm_shoulder_pan", "shoulder pan", "right", 6),
    JointDef("right_hand_2", "right_arm_shoulder_lift", "shoulder lift", "right", 7),
    JointDef("right_hand_3", "right_arm_elbow_flex", "elbow flex", "right", 8),
    JointDef("right_hand_4", "right_arm_wrist_flex", "wrist flex", "right", 9),
    JointDef("right_hand_5", "right_arm_wrist_roll", "wrist roll", "right", 10),
    JointDef("right_hand_6", "right_arm_gripper", "gripper", "right", 11),
    JointDef("head_pan", HEAD_PAN_XLE, "head pan", "head", 12),
    JointDef("head_tilt", HEAD_TILT_XLE, "head tilt", "head", 13),
)
JOINT_BY_EXTERNAL = {joint.external: joint for joint in JOINT_DEFS}


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Arms Head TF Debug</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b0f11;
      --panel: #141b1f;
      --panel2: #1b252a;
      --line: #304148;
      --text: #e8f2f4;
      --muted: #93a6ad;
      --ok: #4bd18a;
      --warn: #f4c95d;
      --bad: #ff6b6b;
      --cyan: #28d9ef;
      --left: #ffd45c;
      --head: #9fb9ff;
    }
    * { box-sizing: border-box; }
    html, body { min-height: 100%; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font: 13px/1.35 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main {
      width: min(1600px, calc(100vw - 22px));
      margin: 0 auto;
      padding: 14px 0 18px;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 12px;
    }
    h1 { margin: 0; font-size: 20px; line-height: 26px; font-weight: 760; }
    .sub {
      margin-top: 3px;
      color: var(--muted);
      font: 12px/1.4 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      overflow-wrap: anywhere;
    }
    .pills { display: flex; flex-wrap: wrap; gap: 8px; justify-content: flex-end; }
    .pill {
      display: inline-flex;
      align-items: center;
      min-height: 28px;
      padding: 0 9px;
      border: 1px solid var(--line);
      border-radius: 999px;
      color: var(--muted);
      background: var(--panel);
      white-space: nowrap;
      font-weight: 650;
    }
    .pill.ok { color: var(--ok); border-color: rgba(75,209,138,.45); }
    .pill.warn { color: var(--warn); border-color: rgba(244,201,93,.45); }
    .pill.bad { color: var(--bad); border-color: rgba(255,107,107,.5); }
    .layout {
      display: grid;
      grid-template-columns: 440px minmax(420px, 1fr) 390px;
      gap: 12px;
      align-items: start;
    }
    section {
      min-width: 0;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      overflow: hidden;
    }
    h2 {
      margin: 0;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      color: #dbe7ea;
      font-size: 13px;
      line-height: 18px;
      font-weight: 740;
    }
    .body { padding: 10px; }
    .stack { display: grid; gap: 12px; }
    .toolbar { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 9px; }
    button {
      min-height: 32px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: var(--panel2);
      color: var(--text);
      font-weight: 720;
      cursor: pointer;
    }
    button:hover { border-color: #4b6871; }
    button:active, button.active { border-color: var(--cyan); background: #15323a; }
    button.danger { border-color: #9a4450; background: #6f2730; }
    button.small { min-height: 28px; padding: 0 7px; font-size: 12px; }
    table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
    th, td {
      border-bottom: 1px solid #223139;
      padding: 7px 5px;
      text-align: left;
      vertical-align: middle;
    }
    th { color: var(--muted); font-size: 11px; font-weight: 720; }
    td { font-size: 12px; }
    td.num { color: #d7f7ff; text-align: right; }
    .group-row td {
      padding-top: 12px;
      color: var(--muted);
      background: #10171a;
      font-weight: 760;
      text-transform: uppercase;
      letter-spacing: .02em;
    }
    .joint-label { display: grid; gap: 2px; min-width: 0; }
    .joint-label b { color: var(--text); font-size: 12px; }
    .joint-label code {
      color: var(--muted);
      font: 11px/1.25 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      overflow-wrap: anywhere;
    }
    input[type="range"] {
      width: 100%;
      accent-color: var(--cyan);
    }
    .target-cell {
      display: grid;
      grid-template-columns: minmax(90px, 1fr) 48px;
      align-items: center;
      gap: 7px;
    }
    .button-row { display: flex; gap: 5px; justify-content: flex-end; }
    canvas {
      display: block;
      width: 100%;
      height: 58vh;
      min-height: 480px;
      max-height: 760px;
      background: #030708;
      border-bottom: 1px solid var(--line);
      cursor: grab;
    }
    canvas:active { cursor: grabbing; }
    .mesh-frame {
      display: block;
      width: 100%;
      height: 58vh;
      min-height: 480px;
      max-height: 760px;
      border: 0;
      border-bottom: 1px solid var(--line);
      background: #030708;
    }
    .metric-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }
    .metric {
      min-width: 0;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: var(--panel2);
      padding: 8px;
    }
    .metric .label { color: var(--muted); font-size: 11px; }
    .metric .value {
      margin-top: 5px;
      font-size: 18px;
      font-weight: 760;
      overflow-wrap: anywhere;
    }
    .log {
      min-height: 20px;
      color: var(--muted);
      font: 12px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      overflow-wrap: anywhere;
    }
    .scroll { max-height: 38vh; overflow: auto; }
    pre {
      margin: 0;
      padding: 10px;
      white-space: pre-wrap;
      word-break: break-word;
      color: #bcebd1;
      font: 12px/1.4 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    @media (max-width: 1240px) {
      .layout { grid-template-columns: 420px minmax(420px, 1fr); }
      .right-col { grid-column: 1 / -1; }
    }
    @media (max-width: 860px) {
      main { width: min(100vw - 16px, 720px); }
      header { align-items: flex-start; flex-direction: column; }
      .pills { justify-content: flex-start; }
      .layout { grid-template-columns: 1fr; }
      canvas { min-height: 380px; height: 52vh; }
      .mesh-frame { min-height: 420px; height: 54vh; }
      .metric-grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>Arms Head TF Debug</h1>
      <div id="endpoint" class="sub">fast_zmq waiting</div>
    </div>
    <div class="pills">
      <span id="robotPill" class="pill warn">robot waiting</span>
      <span id="tfPill" class="pill warn">tf waiting</span>
      <span id="jointPill" class="pill warn">joints waiting</span>
    </div>
  </header>

  <div class="layout">
    <section>
      <h2>Motor Commands</h2>
      <div class="body">
        <div class="toolbar">
          <button id="syncAll">Sync Current</button>
          <button id="sendAll">Send All Targets</button>
          <button id="centerHead">Center Head</button>
          <button id="stopBase" class="danger">Stop Base</button>
        </div>
        <table>
          <thead>
            <tr><th>Joint</th><th class="num">Now</th><th>Target</th><th></th></tr>
          </thead>
          <tbody id="jointRows"></tbody>
        </table>
        <div id="commandLog" class="log">idle</div>
      </div>
    </section>

    <section>
      <h2>Robot Server tf.links Mesh</h2>
      <iframe id="meshFrame" class="mesh-frame" src="/mesh" title="XLeRobot mesh and tf.links viewer"></iframe>
      <canvas id="tfCanvas" style="display:none"></canvas>
      <div class="body">
        <div class="toolbar">
          <button id="resetView">Reset</button>
          <button id="topView">Top</button>
          <button id="frontView">Front</button>
          <button id="sideView">Side</button>
        </div>
        <div class="metric-grid">
          <div class="metric"><div class="label">right ee</div><div id="rightEe" class="value">-</div></div>
          <div class="metric"><div class="label">left ee</div><div id="leftEe" class="value">-</div></div>
          <div class="metric"><div class="label">source</div><div id="tfSource" class="value">-</div></div>
        </div>
      </div>
    </section>

    <div class="stack right-col">
      <section>
        <h2>tf.links Targets</h2>
        <div class="body scroll">
          <table>
            <thead><tr><th>Name</th><th>X</th><th>Y</th><th>Z</th><th>Src</th></tr></thead>
            <tbody id="tfRows"></tbody>
          </table>
        </div>
      </section>
      <section>
        <h2>Status</h2>
        <div class="body">
          <div id="statusLog" class="log">waiting</div>
        </div>
      </section>
      <section>
        <h2>Raw Snapshot</h2>
        <pre id="rawJson">waiting</pre>
      </section>
    </div>
  </div>
</main>

<script>
const JOINT_GROUPS = {left: "Left Arm", right: "Right Arm", head: "Head Camera"};
const q = id => document.getElementById(id);
let latest = null;
let targetValues = new Map();
let rowsReady = false;
let dragging = false;
let lastMouse = [0, 0];
let view = {yaw: -0.72, pitch: -0.50, zoom: 1.0, panX: 0, panY: 0};
let renderFit = {centerX: 0, centerY: 0, scale: 500};

function fmt(value, digits = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : "-";
}
function poseText(p) {
  return Array.isArray(p) ? `${fmt(p[0], 3)}, ${fmt(p[1], 3)}, ${fmt(p[2], 3)}` : "-";
}
function setPill(id, level, text) {
  const el = q(id);
  el.textContent = text;
  el.className = `pill ${level}`;
}
function setLog(id, value) {
  q(id).textContent = typeof value === "string" ? value : JSON.stringify(value);
}
async function api(path, body = null) {
  const opts = body ? {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)} : {cache: "no-store"};
  const res = await fetch(path, opts);
  const data = await res.json();
  if (!data.ok) throw new Error(data.error || "request failed");
  return data;
}
function clamp(value, lo, hi) {
  return Math.max(lo, Math.min(hi, value));
}
function rowValue(joint) {
  if (targetValues.has(joint.external)) return targetValues.get(joint.external);
  const current = Number(joint.current);
  const value = Number.isFinite(current) ? current : joint.center;
  targetValues.set(joint.external, value);
  return value;
}
function renderRows(data) {
  if (rowsReady) return;
  const rows = [];
  let group = "";
  for (const joint of data.joints || []) {
    if (joint.group !== group) {
      group = joint.group;
      rows.push(`<tr class="group-row"><td colspan="4">${JOINT_GROUPS[group] || group}</td></tr>`);
    }
    const value = rowValue(joint);
    rows.push(`<tr data-joint="${joint.external}">
      <td><div class="joint-label"><b>${joint.label}</b><code>${joint.external} -> ${joint.xlerobot}</code></div></td>
      <td class="num now">-</td>
      <td><div class="target-cell">
        <input class="target" type="range" min="${Math.round(joint.min)}" max="${Math.round(joint.max)}" step="1" value="${Math.round(value)}">
        <span class="targetText">${Math.round(value)}</span>
      </div></td>
      <td><div class="button-row">
        <button class="small" data-step="-50">-50</button>
        <button class="small" data-step="50">+50</button>
        <button class="small" data-move="1">Move</button>
      </div></td>
    </tr>`);
  }
  q("jointRows").innerHTML = rows.join("");
  rowsReady = true;
}
function updateRows(data) {
  renderRows(data);
  const byName = new Map((data.joints || []).map(j => [j.external, j]));
  document.querySelectorAll("tr[data-joint]").forEach(row => {
    const name = row.dataset.joint;
    const joint = byName.get(name);
    if (!joint) return;
    row.querySelector(".now").textContent = fmt(joint.current, 0);
    const input = row.querySelector("input.target");
    const text = row.querySelector(".targetText");
    input.min = String(Math.round(joint.min));
    input.max = String(Math.round(joint.max));
    if (!targetValues.has(name)) targetValues.set(name, rowValue(joint));
    if (document.activeElement !== input) input.value = String(Math.round(targetValues.get(name)));
    text.textContent = String(Math.round(input.value));
  });
}
function collectTargets(group = null) {
  const targets = {};
  for (const joint of latest.joints || []) {
    if (group && joint.group !== group) continue;
    targets[joint.external] = Math.round(rowValue(joint));
  }
  return targets;
}
function syncTargets(group = null) {
  for (const joint of (latest && latest.joints) || []) {
    if (group && joint.group !== group) continue;
    const current = Number(joint.current);
    targetValues.set(joint.external, Number.isFinite(current) ? current : joint.center);
  }
  updateRows(latest);
}
async function sendTargets(targets) {
  const result = await api("/api/move", {targets});
  setLog("commandLog", result);
}

q("jointRows").addEventListener("input", ev => {
  if (!ev.target.classList.contains("target")) return;
  const row = ev.target.closest("tr[data-joint]");
  targetValues.set(row.dataset.joint, Number(ev.target.value));
  row.querySelector(".targetText").textContent = ev.target.value;
});
q("jointRows").addEventListener("click", async ev => {
  const btn = ev.target.closest("button");
  if (!btn) return;
  const row = btn.closest("tr[data-joint]");
  const name = row.dataset.joint;
  const joint = (latest.joints || []).find(j => j.external === name);
  if (!joint) return;
  try {
    if (btn.dataset.move) {
      await sendTargets({[name]: Math.round(rowValue(joint))});
    } else if (btn.dataset.step) {
      const base = Number.isFinite(Number(joint.current)) ? Number(joint.current) : rowValue(joint);
      const next = clamp(base + Number(btn.dataset.step), joint.min, joint.max);
      targetValues.set(name, next);
      await sendTargets({[name]: Math.round(next)});
    }
  } catch (err) {
    setLog("commandLog", String(err));
  }
});
q("syncAll").onclick = () => syncTargets();
q("sendAll").onclick = async () => {
  try { await sendTargets(collectTargets()); }
  catch (err) { setLog("commandLog", String(err)); }
};
q("centerHead").onclick = async () => {
  if (!latest) return;
  const targets = {};
  for (const joint of latest.joints || []) {
    if (joint.group === "head") {
      targets[joint.external] = Math.round(joint.center);
      targetValues.set(joint.external, joint.center);
    }
  }
  updateRows(latest);
  try { await sendTargets(targets); }
  catch (err) { setLog("commandLog", String(err)); }
};
q("stopBase").onclick = async () => {
  try { setLog("commandLog", await api("/api/stop", {})); }
  catch (err) { setLog("commandLog", String(err)); }
};

function resizeCanvas() {
  const canvas = q("tfCanvas");
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
  const w = Math.max(320, Math.floor(rect.width * dpr));
  const h = Math.max(320, Math.floor(rect.height * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
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
function scenePoints(tf) {
  const points = [[0, 0, 0], [0, 0, 1.1], [-0.35, -0.35, 0.7], [0.45, 0.35, 1.2]];
  Object.values(tf.link_frames || {}).forEach(p => Array.isArray(p) && points.push(p));
  Object.values(tf.targets || {}).forEach(p => Array.isArray(p) && points.push(p));
  return points;
}
function updateFit(tf) {
  const canvas = q("tfCanvas");
  const projected = scenePoints(tf).map(rotateForView);
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  projected.forEach(p => {
    minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
    minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
  });
  const pad = 70 * Math.max(1, Math.min(2, window.devicePixelRatio || 1));
  const sx = (canvas.width - pad * 2) / Math.max(0.05, maxX - minX);
  const sy = (canvas.height - pad * 2) / Math.max(0.05, maxY - minY);
  renderFit = {
    centerX: (minX + maxX) * 0.5,
    centerY: (minY + maxY) * 0.5,
    scale: Math.max(120, Math.min(1700, Math.min(sx, sy) * view.zoom)),
  };
}
function project(p) {
  const canvas = q("tfCanvas");
  const r = rotateForView(p);
  return {
    x: canvas.width * 0.5 + view.panX + (r[0] - renderFit.centerX) * renderFit.scale,
    y: canvas.height * 0.53 + view.panY - (r[1] - renderFit.centerY) * renderFit.scale,
    d: r[2],
  };
}
function drawLine(ctx, a, b, color, width = 1.5) {
  const pa = project(a), pb = project(b);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(pa.x, pa.y);
  ctx.lineTo(pb.x, pb.y);
  ctx.stroke();
}
function drawDot(ctx, p, color, radius = 5) {
  const pp = project(p);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pp.x, pp.y, radius, 0, Math.PI * 2);
  ctx.fill();
}
function drawText(ctx, p, text, color) {
  const pp = project(p);
  ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
  ctx.fillStyle = "rgba(3,7,8,.72)";
  const width = Math.min(260, Math.max(60, text.length * 6.4 + 12));
  ctx.fillRect(pp.x + 8, pp.y - 21, width, 20);
  ctx.strokeStyle = "rgba(120,150,160,.35)";
  ctx.strokeRect(pp.x + 8, pp.y - 21, width, 20);
  ctx.fillStyle = color;
  ctx.fillText(text, pp.x + 13, pp.y - 7);
}
function drawGrid(ctx) {
  for (let i = -6; i <= 8; i++) {
    const x = i * 0.1;
    drawLine(ctx, [x, -0.5, 0.74], [x, 0.5, 0.74], "rgba(90,116,124,.22)", 1);
  }
  for (let i = -5; i <= 5; i++) {
    const y = i * 0.1;
    drawLine(ctx, [-0.6, y, 0.74], [0.8, y, 0.74], "rgba(90,116,124,.22)", 1);
  }
}
function targetColor(name) {
  if (name.includes("right")) return "#28d9ef";
  if (name.includes("left")) return "#ffd45c";
  if (name.includes("head")) return "#9fb9ff";
  return "#7ee2a8";
}
function drawTf(tf) {
  const canvas = q("tfCanvas");
  const ctx = canvas.getContext("2d");
  resizeCanvas();
  const grad = ctx.createLinearGradient(0, 0, 0, canvas.height);
  grad.addColorStop(0, "#061013");
  grad.addColorStop(1, "#020607");
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  updateFit(tf);
  drawGrid(ctx);
  const frames = tf.link_frames || {};
  for (const edge of tf.tree_edges || []) {
    const a = frames[edge[0]], b = frames[edge[1]];
    if (Array.isArray(a) && Array.isArray(b)) drawLine(ctx, a, b, "rgba(210,235,240,.28)", 1.2);
  }
  for (const [name, pose] of Object.entries(tf.targets || {})) {
    if (!Array.isArray(pose)) continue;
    const color = targetColor(name);
    drawDot(ctx, pose, color, name.includes("gripper") || name.includes("jaw") ? 7 : 5);
    if (name.includes("gripper") || name.includes("jaw") || name.includes("head_")) {
      drawText(ctx, pose, name, color);
    }
  }
  if (!tf.ok) {
    ctx.fillStyle = "rgba(232,242,244,.85)";
    ctx.font = "16px system-ui, sans-serif";
    ctx.fillText("waiting for tf.links", 18, 32);
  }
}
const canvas = q("tfCanvas");
canvas.addEventListener("mousedown", ev => { dragging = true; lastMouse = [ev.clientX, ev.clientY]; });
window.addEventListener("mouseup", () => dragging = false);
window.addEventListener("mousemove", ev => {
  if (!dragging) return;
  const dx = ev.clientX - lastMouse[0], dy = ev.clientY - lastMouse[1];
  lastMouse = [ev.clientX, ev.clientY];
  view.yaw += dx * 0.008;
  view.pitch = Math.max(-1.25, Math.min(0.45, view.pitch + dy * 0.006));
  drawTf((latest && latest.tf) || {});
});
canvas.addEventListener("wheel", ev => {
  ev.preventDefault();
  view.zoom = Math.max(0.45, Math.min(3.5, view.zoom * Math.exp(-ev.deltaY * 0.001)));
  drawTf((latest && latest.tf) || {});
}, {passive: false});
q("resetView").onclick = () => { view = {yaw: -0.72, pitch: -0.50, zoom: 1.0, panX: 0, panY: 0}; drawTf((latest && latest.tf) || {}); };
q("topView").onclick = () => { view.yaw = -Math.PI / 2; view.pitch = -1.23; drawTf((latest && latest.tf) || {}); };
q("frontView").onclick = () => { view.yaw = -Math.PI / 2; view.pitch = -0.35; drawTf((latest && latest.tf) || {}); };
q("sideView").onclick = () => { view.yaw = 0.0; view.pitch = -0.25; drawTf((latest && latest.tf) || {}); };
window.addEventListener("resize", () => drawTf((latest && latest.tf) || {}));

function renderTfTables(tf) {
  const targets = tf.targets || {};
  const sources = tf.target_sources || {};
  const names = Object.keys(targets).sort((a, b) => a.localeCompare(b));
  q("tfRows").innerHTML = names.map(name => {
    const p = targets[name] || [];
    const color = targetColor(name);
    return `<tr><td style="color:${color}">${name}</td><td class="num">${fmt(p[0], 3)}</td><td class="num">${fmt(p[1], 3)}</td><td class="num">${fmt(p[2], 3)}</td><td>${sources[name] || "-"}</td></tr>`;
  }).join("");
}
function updateUi(data) {
  latest = data;
  q("endpoint").textContent = data.endpoint || "fast_zmq waiting";
  setPill("robotPill", data.robot_ready ? "ok" : "bad", data.robot_ready ? "robot ready" : "robot waiting");
  const tfAge = Number(data.tf && data.tf.age_s);
  const jointAge = Number(data.tf && data.tf.joint_states_age_s);
  setPill("tfPill", data.tf_live ? "ok" : (data.tf && data.tf.ok ? "warn" : "bad"), data.tf_live ? `tf ${tfAge.toFixed(2)}s` : "tf waiting");
  setPill("jointPill", data.joints_live ? "ok" : "warn", data.joints_live ? `joints ${jointAge.toFixed(2)}s` : "joints waiting");
  updateRows(data);
  renderTfTables(data.tf || {});
  drawTf(data.tf || {});
  q("rightEe").textContent = poseText(data.tf && data.tf.ee_links && data.tf.ee_links.right);
  q("leftEe").textContent = poseText(data.tf && data.tf.ee_links && data.tf.ee_links.left);
  q("tfSource").textContent = (data.tf && (data.tf.source || data.tf.source_note)) || "-";
  setLog("statusLog", {
    calibration_path: data.calibration_path,
    command_status: data.command_status,
    last_command: data.last_command,
    error: data.error || null,
  });
  q("rawJson").textContent = JSON.stringify(data, null, 2);
}
async function poll() {
  try {
    updateUi(await api("/api/state"));
  } catch (err) {
    setPill("robotPill", "bad", "web error");
    setLog("statusLog", String(err));
    drawTf({});
  } finally {
    setTimeout(poll, 250);
  }
}
poll();
</script>
</body>
</html>
"""


def pack(payload: dict[str, Any]) -> bytes:
    return msgpack.packb(payload, use_bin_type=True)


def unpack(payload: bytes) -> dict[str, Any]:
    decoded = msgpack.unpackb(payload, raw=False)
    if not isinstance(decoded, dict):
        raise ValueError("payload is not a dict")
    return decoded


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_calibration(robot_id: int | str, calibration_id: str | None = None) -> tuple[dict[str, Any], str | None]:
    explicit_path = os.environ.get("XLEROBOT_CALIBRATION_PATH")
    base_dir = Path(
        os.path.expanduser(os.environ.get("XLEROBOT_CALIBRATION_DIR", DEFAULT_CALIBRATION_DIR))
    )
    candidates: list[Path] = []
    if explicit_path:
        candidates.append(Path(os.path.expanduser(explicit_path)))
    for candidate_id in (
        calibration_id,
        os.environ.get("XLEROBOT_CALIBRATION_ID"),
        os.environ.get("XLEROBOT_ID"),
        str(robot_id),
        "my_xlerobot_pc",
        "None",
    ):
        if candidate_id:
            candidates.append(base_dir / f"{candidate_id}.json")
    seen: set[Path] = set()
    for path in candidates:
        expanded = path.expanduser()
        if expanded in seen:
            continue
        seen.add(expanded)
        if not expanded.is_file():
            continue
        try:
            loaded = json.loads(expanded.read_text())
        except Exception:
            continue
        if isinstance(loaded, dict):
            return loaded, str(expanded)
    return {}, None


class ArmsHeadTfBridge:
    def __init__(
        self,
        fast_host: str,
        pub_port: int,
        pull_port: int,
        rep_port: int,
        robot_id: int,
        timeout_ms: int,
        max_state_age_s: float,
        calibration_id: str | None,
    ) -> None:
        self.fast_host = fast_host
        self.pub_port = int(pub_port)
        self.pull_port = int(pull_port)
        self.rep_port = int(rep_port)
        self.robot_id = int(robot_id)
        self.timeout_ms = int(timeout_ms)
        self.max_state_age_s = float(max_state_age_s)
        self.endpoint = (
            f"fast_zmq tcp://{fast_host}:{pub_port} pub, "
            f"tcp://{fast_host}:{pull_port} pull, tcp://{fast_host}:{rep_port} rep"
        )
        self.tf_state = FastTfState()
        self.subscriber = FastTfSubscriber(self.tf_state, self.fast_host, self.pub_port, self.robot_id)
        self.calibration, self.calibration_path = load_calibration(self.robot_id, calibration_id)
        self.lock = threading.Lock()
        self.seq = 0
        self.last_command: dict[str, Any] | None = None
        self.last_error = ""
        self._rpc_cache: dict[str, tuple[float, dict[str, Any]]] = {}
        self.ctx = zmq.Context.instance()
        self.push = self.ctx.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.setsockopt(zmq.SNDHWM, 8)
        self.push.setsockopt(zmq.SNDTIMEO, 0)
        self.push.connect(f"tcp://{self.fast_host}:{self.pull_port}")

    def start(self) -> None:
        self.subscriber.start()

    def close(self) -> None:
        self.subscriber.stop()
        try:
            self.push.close(0)
        except Exception:
            pass

    def snapshot(self) -> dict[str, Any]:
        tf = self.tf_state.snapshot(f"tcp://{self.fast_host}:{self.pub_port}")
        health = self.cached_rpc("health", ttl_s=0.5)
        command_status = self.cached_rpc("command_status", ttl_s=0.5)
        joint_positions = self._joint_positions(tf.get("joint_states"))
        joints = [self._joint_status(joint, joint_positions.get(joint.external)) for joint in JOINT_DEFS]
        tf_age = finite_number(tf.get("age_s"))
        joint_age = finite_number(tf.get("joint_states_age_s"))
        health_body = health.get("health") if health.get("ok") else None
        robot_ready = bool(isinstance(health_body, dict) and health_body.get("base_attached"))
        return {
            "ok": True,
            "endpoint": self.endpoint,
            "robot_ready": robot_ready,
            "tf_live": tf_age is not None and tf_age <= self.max_state_age_s,
            "joints_live": joint_age is not None and joint_age <= self.max_state_age_s,
            "tf": tf,
            "joints": joints,
            "health": health_body,
            "command_status": command_status if command_status.get("ok") else None,
            "calibration_path": self.calibration_path,
            "last_command": self.last_command,
            "error": self.last_error or tf.get("error"),
        }

    def move(self, targets: Any) -> dict[str, Any]:
        if not isinstance(targets, dict):
            return {"ok": False, "error": "targets must be an object"}
        sparse: list[float | None] = [None] * len(JOINT_DEFS)
        public: dict[str, float] = {}
        for raw_name, raw_value in targets.items():
            name = str(raw_name)
            joint = JOINT_BY_EXTERNAL.get(name)
            if joint is None:
                return {"ok": False, "error": f"unknown joint {name!r}"}
            value = finite_number(raw_value)
            if value is None:
                return {"ok": False, "error": f"{name} target must be finite"}
            low, high = self._limits(joint)
            bounded = round(clamp(value, low, high))
            sparse[joint.index] = float(bounded)
            public[name] = float(bounded)
        if not public:
            return {"ok": False, "error": "no targets provided"}
        payload = {
            "schema": SCHEMA_VERSION_V11,
            "source_id": "arms_head_tf_debug_webview.move",
            "seq": self._next_seq(),
            "stamp_ns": time.time_ns(),
            "frame": "body",
            "joint_targets_sparse": sparse,
        }
        return self._send(payload, public)

    def stop_base(self) -> dict[str, Any]:
        return self.rpc("stop")

    def rpc(self, op: str, **payload: Any) -> dict[str, Any]:
        sock = self.ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        try:
            sock.connect(f"tcp://{self.fast_host}:{self.rep_port}")
            sock.send(pack({"op": op, **payload}))
            return unpack(sock.recv())
        except Exception as exc:
            self.last_error = str(exc)
            return {"ok": False, "error": str(exc)}
        finally:
            sock.close(0)

    def cached_rpc(self, op: str, ttl_s: float) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            cached = self._rpc_cache.get(op)
            if cached is not None and now - cached[0] < ttl_s:
                return dict(cached[1])
        reply = self.rpc(op)
        with self.lock:
            self._rpc_cache[op] = (now, dict(reply))
        return reply

    def _send(self, payload: dict[str, Any], targets: dict[str, float]) -> dict[str, Any]:
        try:
            self.push.send(pack(payload), flags=zmq.NOBLOCK)
        except Exception as exc:
            self.last_error = str(exc)
            return {"ok": False, "error": str(exc), "targets": targets}
        command = {"stamp": time.time(), "targets": targets, "seq": payload["seq"]}
        self.last_command = command
        self.last_error = ""
        return {"ok": True, **command}

    def _next_seq(self) -> int:
        with self.lock:
            self.seq += 1
            return self.seq

    def _joint_status(self, joint: JointDef, current: float | None) -> dict[str, Any]:
        low, high = self._limits(joint)
        return {
            "external": joint.external,
            "xlerobot": joint.xlerobot,
            "label": joint.label,
            "group": joint.group,
            "index": joint.index,
            "current": current,
            "min": low,
            "max": high,
            "center": round((low + high) * 0.5),
            "calibrated": joint.xlerobot in self.calibration,
        }

    def _limits(self, joint: JointDef) -> tuple[float, float]:
        cal = self.calibration.get(joint.xlerobot)
        if isinstance(cal, dict):
            low = finite_number(cal.get("range_min"))
            high = finite_number(cal.get("range_max"))
            if low is not None and high is not None and high > low:
                return low, high
        return 0.0, 4095.0

    def _joint_positions(self, joint_state: Any) -> dict[str, float]:
        if not isinstance(joint_state, dict):
            return {}
        names = joint_state.get("name") or joint_state.get("names")
        positions = joint_state.get("position")
        if not isinstance(names, list) or not isinstance(positions, list):
            return {}
        out: dict[str, float] = {}
        for index, name in enumerate(names):
            if index >= len(positions):
                continue
            value = finite_number(positions[index])
            if value is not None:
                out[str(name)] = value
        return out


def embedded_mesh_html() -> bytes:
    html = MODEL_INDEX_HTML.replace('fetch("/api/state"', 'fetch("/api/tf_state"')
    embedded_css = """
    body { grid-template-columns: minmax(0, 1fr) !important; }
    aside { display: none !important; }
    header { min-height: 46px !important; padding: 8px 12px !important; }
    h1 { font-size: 15px !important; }
    #legend { font-size: 12px; }
    """
    html = html.replace("</style>", embedded_css + "\n  </style>", 1)
    return html.encode("utf-8")


class DebugHandler(BaseHTTPRequestHandler):
    server_version = "ArmsHeadTfDebugWebView/1.0"
    bridge: ArmsHeadTfBridge

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path in {"/", "/index.html"}:
            self._send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/mesh":
            self._send_bytes(embedded_mesh_html(), "text/html; charset=utf-8")
            return
        if path == "/api/state":
            self._send_json(self.bridge.snapshot())
            return
        if path == "/api/tf_state":
            self._send_json(self.bridge.tf_state.snapshot(f"tcp://{self.bridge.fast_host}:{self.bridge.pub_port}"))
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
        if path == "/healthz":
            self._send_bytes(b"ok\n", "text/plain; charset=utf-8")
            return
        if path == "/favicon.ico":
            self._send_bytes(b"", "image/x-icon", status=204)
            return
        self.send_error(404, "not found")

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        try:
            payload = self._read_json()
            if path == "/api/move":
                self._send_json(self.bridge.move(payload.get("targets")))
                return
            if path == "/api/stop":
                self._send_json(self.bridge.stop_base())
                return
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)})
            return
        self.send_error(404, "not found")

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.path.startswith("/api/state"):
            return
        super().log_message(fmt, *args)

    def _read_json(self) -> dict[str, Any]:
        length = min(int(self.headers.get("Content-Length", "0")), 65536)
        body = self.rfile.read(length) if length else b"{}"
        payload = json.loads(body.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(payload, separators=(",", ":"), allow_nan=False, default=str).encode("utf-8")
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("ARMS_HEAD_TF_WEB_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("ARMS_HEAD_TF_WEB_PORT", "8790")))
    parser.add_argument("--fast-zmq-host", default=os.environ.get("ARMS_HEAD_TF_FAST_ZMQ_HOST", "127.0.0.1"))
    parser.add_argument("--fast-zmq-pub-port", type=int, default=int(os.environ.get("FAST_ZMQ_PUB_PORT", "8855")))
    parser.add_argument("--fast-zmq-pull-port", type=int, default=int(os.environ.get("FAST_ZMQ_PULL_PORT", "8856")))
    parser.add_argument("--fast-zmq-rep-port", type=int, default=int(os.environ.get("FAST_ZMQ_REP_PORT", "8857")))
    parser.add_argument("--fast-zmq-robot-id", type=int, default=int(os.environ.get("FAST_ZMQ_ROBOT_ID", "0")))
    parser.add_argument("--calibration-id", default=os.environ.get("XLEROBOT_CALIBRATION_ID"))
    parser.add_argument("--timeout-ms", type=int, default=int(os.environ.get("ARMS_HEAD_TF_TIMEOUT_MS", "250")))
    parser.add_argument("--max-state-age-s", type=float, default=float(os.environ.get("ARMS_HEAD_TF_MAX_STATE_AGE_S", "2.0")))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bridge = ArmsHeadTfBridge(
        args.fast_zmq_host,
        args.fast_zmq_pub_port,
        args.fast_zmq_pull_port,
        args.fast_zmq_rep_port,
        args.fast_zmq_robot_id,
        args.timeout_ms,
        args.max_state_age_s,
        args.calibration_id,
    )
    bridge.start()
    DebugHandler.bridge = bridge
    server = ThreadingHTTPServer((args.host, args.port), DebugHandler)

    def shutdown(_signum: int, _frame: Any) -> None:
        bridge.close()
        threading.Thread(target=server.shutdown, name="http-shutdown", daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    host = _public_host() if args.host in ("0.0.0.0", "::") else args.host
    print(
        f"Arms/head tf debug webview listening on http://{host}:{args.port}/ "
        f"({bridge.endpoint}, calibration={bridge.calibration_path or 'not found'})",
        flush=True,
    )
    try:
        server.serve_forever()
    finally:
        bridge.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
