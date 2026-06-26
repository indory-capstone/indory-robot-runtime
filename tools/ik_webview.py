#!/usr/bin/env python3
"""Browser UI for the XLeRobot URDF-backed IK solver with a live 3D model.

The page renders the actual xlerobot URDF (same assets/rendering as
tf_links_3d_webview). It shows the live robot pose from the fast ZMQ
``proprio`` stream and, when you solve an end-effector target, animates the
solved arm joints so you can see how the model moves to reach the target.
"""

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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlsplit


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from tools import ik_solver  # noqa: E402


TELEOP_SRC = Path(os.environ.get("TELEOPERATION_SRC", "/home/pi/teleoperation/src")).expanduser()
TELEOP_WEBXR_DIR = Path(
    os.environ.get("TELEOPERATION_WEBXR_DIR", "/home/pi/teleoperation/teleoperation/webxr")
).expanduser()
if TELEOP_SRC.is_dir() and str(TELEOP_SRC) not in sys.path:
    sys.path.insert(0, str(TELEOP_SRC))

try:
    from indoory_isaac_sim.apps.teleop.vr_web_teleop_overlay import (
        ROBOT_ASSET_DIR,
        xlerobot_model_description,
    )
except Exception as exc:  # pragma: no cover - rendered as API error
    ROBOT_ASSET_DIR = Path(
        "/home/pi/teleoperation/src/indoory_isaac_sim/assets/data/robots/xlerobot"
    )
    XLEROBOT_MODEL_IMPORT_ERROR: Exception | None = exc
    xlerobot_model_description = None
else:
    XLEROBOT_MODEL_IMPORT_ERROR = None


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Indoory IK Solver 3D</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #070b0d;
      --panel: #121a1f;
      --panel2: #1a242a;
      --line: #2b3b43;
      --text: #edf4f6;
      --muted: #95a6ae;
      --good: #4bd18a;
      --warn: #f4c95d;
      --bad: #ff6b6b;
      --cyan: #5ed6e8;
      --right: #28e8ff;
      --left: #ffd54a;
      --target: #ff5bd0;
      --reached: #6cff9e;
    }
    * { box-sizing: border-box; }
    html, body { height: 100%; }
    body {
      margin: 0;
      overflow: hidden;
      background: var(--bg);
      color: var(--text);
      font: 14px/1.35 Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 420px;
    }
    main { min-width: 0; min-height: 0; display: grid; grid-template-rows: auto minmax(0, 1fr); background: #020606; }
    header {
      min-height: 54px;
      display: flex;
      align-items: center;
      gap: 10px;
      padding: 10px 16px;
      border-bottom: 1px solid var(--line);
      background: #0b1114;
      flex-wrap: wrap;
    }
    h1 { margin: 0; font-size: 18px; font-weight: 760; white-space: nowrap; }
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
    .pill.good { color: var(--good); border-color: rgba(75,209,138,.45); }
    .pill.warn { color: var(--warn); border-color: rgba(244,201,93,.45); }
    .pill.bad { color: var(--bad); border-color: rgba(255,107,107,.5); }
    #stageWrap {
      position: relative;
      min-height: 0;
      overflow: hidden;
      background:
        radial-gradient(circle at 50% 46%, rgba(31,64,72,.22), transparent 48%),
        linear-gradient(180deg, #061013 0%, #020607 100%);
    }
    #stage { width: 100%; height: 100%; display: block; cursor: grab; }
    #stage:active { cursor: grabbing; }
    #labels3d { position: absolute; inset: 0; pointer-events: none; overflow: hidden; }
    .label3d {
      position: absolute;
      transform: translate(9px, -50%);
      max-width: 260px;
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
      max-width: calc(100% - 28px);
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
      overflow: hidden;
    }
    .section { padding: 13px 14px; border-bottom: 1px solid var(--line); }
    .section h2 { margin: 0 0 10px; color: var(--muted); font-size: 13px; font-weight: 760; }
    .toolbar { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 8px; }
    .row {
      display: grid;
      grid-template-columns: 96px minmax(0, 1fr);
      gap: 10px;
      align-items: center;
      margin-bottom: 8px;
    }
    label { color: var(--muted); font-size: 13px; }
    .check { display: flex; align-items: center; gap: 8px; min-height: 28px; }
    input, select {
      width: 100%;
      min-height: 36px;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: var(--panel2);
      color: var(--text);
      padding: 6px 9px;
      font: inherit;
      font-variant-numeric: tabular-nums;
    }
    input[type="checkbox"] { width: 18px; min-height: 18px; accent-color: var(--cyan); }
    .triple { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
    .quad { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 8px; }
    .buttons { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; margin-top: 4px; }
    button {
      min-height: 38px;
      padding: 0 10px;
      border: 1px solid #35505b;
      border-radius: 7px;
      background: #132025;
      color: var(--text);
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
    }
    button:hover { background: #1b2c33; }
    button.primary { border-color: #3b8d9b; background: #15313a; }
    button.danger { border-color: #a84855; background: #742630; }
    button.active { border-color: var(--cyan); background: #18323a; color: var(--cyan); }
    .metrics { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
    .metric { min-width: 0; border: 1px solid var(--line); border-radius: 7px; background: var(--panel2); padding: 9px; }
    .metric .label { color: var(--muted); font-size: 12px; line-height: 15px; }
    .metric .value {
      margin-top: 5px;
      font-size: 16px;
      line-height: 21px;
      font-weight: 760;
      font-variant-numeric: tabular-nums;
      overflow-wrap: anywhere;
    }
    .scroll { min-height: 0; overflow: auto; }
    pre {
      margin: 0;
      padding: 12px 14px;
      white-space: pre-wrap;
      word-break: break-word;
      color: #bcebd1;
      font: 12px/1.4 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    .muted { color: var(--muted); font-size: 12px; overflow-wrap: anywhere; }
    @media (max-width: 1040px) {
      body { grid-template-columns: 1fr; grid-template-rows: minmax(56vh, 1fr) auto; }
      aside { border-left: 0; border-top: 1px solid var(--line); max-height: 44vh; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>Indoory IK Solver 3D</h1>
      <span id="badge" class="pill warn">connecting</span>
      <span id="modelStatus" class="pill warn">model loading</span>
      <span id="proprioStatus" class="pill warn">proprio waiting</span>
      <span id="poseMode" class="pill">live</span>
    </header>
    <div id="stageWrap">
      <canvas id="stage"></canvas>
      <div id="labels3d"></div>
      <div id="legend">
        <span style="color:var(--right)">cyan: live EE (right)</span>
        <span style="color:var(--left)">yellow: live EE (left)</span>
        <span style="color:var(--target)">magenta: solved target</span>
        <span style="color:var(--reached)">green: FK reached</span>
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
      <div class="toolbar">
        <button id="poseLive" class="active">Live pose</button>
        <button id="poseSolution">Solution</button>
      </div>
      <div class="check"><input id="showGrid" type="checkbox" checked> <label for="showGrid">Ground grid</label></div>
      <div class="check"><input id="showLabels" type="checkbox" checked> <label for="showLabels">Labels</label></div>
      <div class="check"><input id="showMarkers" type="checkbox" checked> <label for="showMarkers">Target / EE markers</label></div>
    </div>

    <div class="section">
      <h2>Target</h2>
      <div class="row">
        <label for="side">side</label>
        <select id="side">
          <option value="right">right</option>
          <option value="left">left</option>
        </select>
      </div>
      <div class="row">
        <label for="mode">mode</label>
        <select id="mode">
          <option value="delta">delta</option>
          <option value="xyz">absolute xyz</option>
          <option value="current">current</option>
        </select>
      </div>
      <div class="row">
        <label for="solver">solver</label>
        <select id="solver">
          <option value="accurate">accurate (sub-mm)</option>
          <option value="fast">fast (realtime budget)</option>
        </select>
      </div>
      <div class="row">
        <label>xyz / delta</label>
        <div class="triple">
          <input id="x" inputmode="decimal" value="0.02">
          <input id="y" inputmode="decimal" value="0">
          <input id="z" inputmode="decimal" value="0">
        </div>
      </div>
      <div class="row">
        <label>quat xyzw</label>
        <div class="quad">
          <input id="qx" inputmode="decimal" value="">
          <input id="qy" inputmode="decimal" value="">
          <input id="qz" inputmode="decimal" value="">
          <input id="qw" inputmode="decimal" value="">
        </div>
      </div>
      <div class="row">
        <label for="send">send</label>
        <div class="check"><input id="send" type="checkbox"> <span class="muted">push to robot</span></div>
      </div>
      <div class="row">
        <label for="repeat">repeat</label>
        <input id="repeat" inputmode="numeric" value="1">
      </div>
      <div class="buttons">
        <button data-delta="0.02,0,0">+X</button>
        <button data-delta="-0.02,0,0">-X</button>
        <button data-delta="0,0,0">current</button>
        <button data-delta="0,0.02,0">+Y</button>
        <button data-delta="0,-0.02,0">-Y</button>
        <button id="solve" class="primary">Solve</button>
        <button data-delta="0,0,0.02">+Z</button>
        <button data-delta="0,0,-0.02">-Z</button>
        <button id="sendOnce" class="danger">Send</button>
      </div>
      <div id="log" class="muted" style="margin-top:8px;">ready</div>
    </div>

    <div class="section">
      <h2>Status</h2>
      <div class="metrics">
        <div class="metric"><div class="label">current xyz</div><div id="current" class="value">-</div></div>
        <div class="metric"><div class="label">target error</div><div id="error" class="value">-</div></div>
        <div class="metric"><div class="label">mode</div><div id="modeOut" class="value">-</div></div>
      </div>
    </div>

    <div class="scroll">
      <pre id="json">{}</pre>
    </div>
  </aside>

  <script>
    // Globals referenced by /static/app_rendering.js (declared by the page).
    let gl = null;
    let lineProgram = null, lineBuffer = null, lineAttribs = null, lineUniforms = null;
    let meshProgram = null, meshAttribs = null, meshUniforms = null;
    let headRgbProgram = null, headRgbBuffer = null, headRgbAttribs = null, headRgbUniforms = null;
    let meshCache = new Map();
    let robotModel = null, robotModelPromise = null, robotModelErrorLogged = false;
    let robotRenderedEeBasePoints = { right: null, left: null };
    let robotTelemetry = { ok: false, tf_links: {}, tf_targets: {}, proprio: null };
    let state = { hmd: [0, 0, 0, 0, 0, 0, 1] };
    function log(message, level) { console.log("[ik viewer]", level || "info", message); }
  </script>
  <script src="/static/app_rendering.js"></script>
  <script>
    const $ = id => document.getElementById(id);
    const canvas = $("stage");
    const labelsLayer = $("labels3d");
    const badge = $("badge");
    const modelStatusEl = $("modelStatus");
    const proprioStatusEl = $("proprioStatus");
    const poseModeEl = $("poseMode");
    const out = $("json");
    const logEl = $("log");
    const sideSel = $("side");
    const showGridEl = $("showGrid");
    const showLabelsEl = $("showLabels");
    const showMarkersEl = $("showMarkers");

    const RIGHT_COLOR = [0.16, 0.91, 1.0, 0.98];
    const LEFT_COLOR = [1.0, 0.84, 0.29, 0.98];
    const TARGET_COLOR = [1.0, 0.36, 0.82, 0.98];
    const REACHED_COLOR = [0.42, 1.0, 0.62, 0.98];

    const ANIM_MS = 520;
    let liveState = null;       // last /api/state payload
    let solveResult = null;     // last /api/solve payload (with joint_solution_rad)
    let poseSource = "live";    // "live" | "solution"
    let animFrom = null, animTo = null, animStart = 0;
    let displayValues = {};
    let targetPoses = { right: null, left: null };
    const frameMatrices = new Map();
    let labelItems = [];
    let dragging = false, lastMouse = [0, 0];
    const orbit = { target: [0.02, 0.0, 0.72], yaw: -0.82, pitch: 0.34, distance: 1.9 };
    const KEY_TARGET_BINDINGS = {
      r: { side: "right", axis: 0, dir: 1 }, f: { side: "right", axis: 0, dir: -1 },
      t: { side: "right", axis: 1, dir: 1 }, g: { side: "right", axis: 1, dir: -1 },
      y: { side: "right", axis: 2, dir: 1 }, h: { side: "right", axis: 2, dir: -1 },
      u: { side: "left", axis: 0, dir: 1 }, j: { side: "left", axis: 0, dir: -1 },
      i: { side: "left", axis: 1, dir: 1 }, k: { side: "left", axis: 1, dir: -1 },
      o: { side: "left", axis: 2, dir: 1 }, l: { side: "left", axis: 2, dir: -1 },
    };

    // ---- math helpers (mulMat4 / mat4Identity / etc. come from app_rendering.js) ----
    function vAdd(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
    function vSub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
    function vScale(v, s) { return [v[0] * s, v[1] * s, v[2] * s]; }
    function vDot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    function vCross(a, b) {
      return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    }
    function vNormalize(v) {
      const len = Math.hypot(v[0], v[1], v[2]);
      if (!Number.isFinite(len) || len < 1e-9) return [0, 0, 0];
      return [v[0] / len, v[1] / len, v[2] / len];
    }
    function quatRotate(q, v) {
      const x = Number(q[0] || 0), y = Number(q[1] || 0), z = Number(q[2] || 0), w = Number(q[3] == null ? 1 : q[3]);
      const uv = [y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]];
      const uuv = [y * uv[2] - z * uv[1], z * uv[0] - x * uv[2], x * uv[1] - y * uv[0]];
      return [v[0] + 2 * (w * uv[0] + uuv[0]), v[1] + 2 * (w * uv[1] + uuv[1]), v[2] + 2 * (w * uv[2] + uuv[2])];
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
      const projectionMatrix = perspective(48 * Math.PI / 180, Math.max(0.2, canvas.width / Math.max(1, canvas.height)), 0.02, 20.0);
      const viewMatrix = lookAt(eye, orbit.target, [0, 0, 1]);
      return {
        viewProjection: mulMat4(projectionMatrix, viewMatrix),
        drawView: { projectionMatrix, transform: { inverse: { matrix: viewMatrix } } },
      };
    }

    function resize() {
      const rect = canvas.getBoundingClientRect();
      const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
      const w = Math.max(360, Math.floor(rect.width * dpr));
      const h = Math.max(300, Math.floor(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
      if (gl) gl.viewport(0, 0, canvas.width, canvas.height);
    }

    function setPill(el, text, level) { el.textContent = text; el.className = "pill " + (level || ""); }
    function fmt(v, d = 3) { const n = Number(v); return Number.isFinite(n) ? n.toFixed(d) : "-"; }
    function vec(values, d = 3) {
      return Array.isArray(values) ? values.slice(0, 3).map(v => fmt(v, d)).join(", ") : "-";
    }
    function clonePose(pose) {
      return Array.isArray(pose) && pose.length >= 7 ? pose.slice(0, 7).map(v => Number(v)) : null;
    }
    function fallbackPose(side) {
      return side === "left" ? [0.10, 0.42, 0.72, 0, 0, 0, 1] : [0.02, -0.28, 0.70, 0, 0, 0, 1];
    }
    function currentPoseForSide(side) {
      return clonePose(liveState && liveState.current ? liveState.current[side] : null);
    }
    function ensureTargetPose(side) {
      const existing = clonePose(targetPoses[side]);
      if (existing) return existing;
      const live = currentPoseForSide(side);
      return live || fallbackPose(side);
    }
    function syncTargetInputs(side, pose) {
      sideSel.value = side;
      $("mode").value = "xyz";
      $("x").value = fmt(pose[0], 4);
      $("y").value = fmt(pose[1], 4);
      $("z").value = fmt(pose[2], 4);
      $("qx").value = fmt(pose[3], 6);
      $("qy").value = fmt(pose[4], 6);
      $("qz").value = fmt(pose[5], 6);
      $("qw").value = fmt(pose[6], 6);
    }
    function updateTargetFromInputs() {
      if ($("mode").value !== "xyz") return;
      try {
        const xyz = ["x", "y", "z"].map(numberValue);
        if (xyz.some(v => v === null)) return;
        let quat = ["qx", "qy", "qz", "qw"].map(numberValue);
        if (quat.some(v => v === null)) {
          const existing = ensureTargetPose(sideSel.value);
          quat = existing.slice(3, 7);
        }
        targetPoses[sideSel.value] = [...xyz, ...quat];
      } catch (_err) {
        return;
      }
    }
    function targetStepFromEvent(ev) {
      if (ev.shiftKey) return 0.05;
      if (ev.altKey) return 0.002;
      return 0.01;
    }
    function moveKeyboardTarget(ev) {
      const key = String(ev.key || "").toLowerCase();
      const binding = KEY_TARGET_BINDINGS[key];
      if (!binding) return false;
      const active = document.activeElement;
      if (active && ["INPUT", "SELECT", "TEXTAREA"].includes(active.tagName)) return false;
      ev.preventDefault();
      const pose = ensureTargetPose(binding.side);
      pose[binding.axis] += binding.dir * targetStepFromEvent(ev);
      targetPoses[binding.side] = pose;
      syncTargetInputs(binding.side, pose);
      badge.textContent = "target";
      badge.className = "pill warn";
      logEl.textContent = `${binding.side} target ${vec(pose, 4)}`;
      return true;
    }

    // ---- joint value sources ----
    function liveValues() { return jointValuesFromProprio(liveState ? liveState.proprio : null); }
    function solutionValues() {
      const base = liveValues();
      if (solveResult && Array.isArray(solveResult.joint_names_urdf) && Array.isArray(solveResult.joint_solution_rad)) {
        solveResult.joint_names_urdf.forEach((name, i) => {
          const v = Number(solveResult.joint_solution_rad[i]);
          if (Number.isFinite(v)) base[String(name)] = v;
        });
      }
      return base;
    }
    function lerpValues(a, b, t) {
      const out = {};
      const keys = new Set(Object.keys(a || {}).concat(Object.keys(b || {})));
      keys.forEach(k => {
        const av = Number((a && a[k] != null) ? a[k] : (b ? b[k] : 0)) || 0;
        const bv = Number((b && b[k] != null) ? b[k] : av) || 0;
        out[k] = av + (bv - av) * t;
      });
      return out;
    }
    function showLive() {
      poseSource = "live"; animFrom = null; animTo = null;
      $("poseLive").classList.add("active"); $("poseSolution").classList.remove("active");
      poseModeEl.textContent = "live";
    }
    function showSolution() {
      if (!solveResult) return;
      poseSource = "solution";
      animFrom = Object.assign({}, displayValues);
      animTo = solutionValues();
      animStart = performance.now();
      $("poseSolution").classList.add("active"); $("poseLive").classList.remove("active");
      poseModeEl.textContent = "solution";
    }

    // ---- rendering ----
    function drawRobotNode(node, localMatrix, viewProjection, values) {
      if (!node || !node.link) return;
      frameMatrices.set(String(node.link.name || ""), localMatrix);
      for (const visual of node.visuals || []) {
        const localVisual = mulMat4(mulMat4(localMatrix, visual.originMatrix), visual.scaleMatrix);
        drawMesh(visual.mesh, localVisual, viewProjection, visual.color);
      }
      for (const child of node.children || []) {
        const jointValue = Number(values[child.joint.name] || 0);
        const childMatrix = mulMat4(mulMat4(localMatrix, child.originMatrix), mat4FromJointMotion(child.joint, jointValue));
        drawRobotNode(child.child, childMatrix, viewProjection, values);
      }
    }

    function frameOrigin(name) {
      const m = frameMatrices.get(name);
      return m ? [Number(m[12] || 0), Number(m[13] || 0), Number(m[14] || 0)] : null;
    }
    function addLabel(point, text, color) {
      if (!showLabelsEl.checked || !Array.isArray(point)) return;
      labelItems.push({ point, text, color });
    }
    function appendGrid(vertices) {
      if (!showGridEl.checked) return;
      const color = [0.34, 0.45, 0.48, 0.26];
      for (let i = -10; i <= 10; i += 1) {
        const v = i * 0.1;
        pushLine(vertices, [v, -0.65, 0], [v, 0.65, 0], color);
        pushLine(vertices, [-0.65, v, 0], [0.65, v, 0], color);
      }
      pushLine(vertices, [-0.72, 0, 0], [0.72, 0, 0], [1.0, 0.20, 0.16, 0.42]);
      pushLine(vertices, [0, -0.72, 0], [0, 0.72, 0], [0.18, 0.95, 0.35, 0.42]);
    }
    function appendPoseAxis(vertices, pose, size) {
      const o = [Number(pose[0] || 0), Number(pose[1] || 0), Number(pose[2] || 0)];
      const q = [pose[3], pose[4], pose[5], pose[6]];
      pushLine(vertices, o, vAdd(o, quatRotate(q, [size, 0, 0])), [1.0, 0.20, 0.16, 0.95]);
      pushLine(vertices, o, vAdd(o, quatRotate(q, [0, size, 0])), [0.18, 0.95, 0.35, 0.95]);
      pushLine(vertices, o, vAdd(o, quatRotate(q, [0, 0, size])), [0.25, 0.50, 1.0, 0.95]);
    }

    function appendMarkers(vertices) {
      if (!showMarkersEl.checked) return;
      const side = sideSel.value;
      const eeColor = side === "left" ? LEFT_COLOR : RIGHT_COLOR;
      const eeColorCss = side === "left" ? "var(--left)" : "var(--right)";

      // Live current EE from /api/state.
      const live = liveState && liveState.current ? liveState.current[side] : null;
      if (Array.isArray(live) && live.length >= 3) {
        pushCross(vertices, [live[0], live[1], live[2]], 0.028, eeColor);
        addLabel([live[0], live[1], live[2]], `live EE ${fmt(live[0])}, ${fmt(live[1])}, ${fmt(live[2])}`, eeColorCss);
      }

      for (const markerSide of ["right", "left"]) {
        const targetPose = targetPoses[markerSide];
        if (!Array.isArray(targetPose) || targetPose.length < 7) continue;
        const selected = markerSide === side;
        const size = selected ? 0.05 : 0.035;
        const tipName = markerSide === "left" ? "Fixed_Jaw_tip_2" : "Fixed_Jaw_tip";
        const tip = frameOrigin(tipName);
        pushCross(vertices, [targetPose[0], targetPose[1], targetPose[2]], size, TARGET_COLOR);
        if (selected) appendPoseAxis(vertices, targetPose, 0.07);
        addLabel(
          [targetPose[0], targetPose[1], targetPose[2]],
          `${markerSide} target ${fmt(targetPose[0])}, ${fmt(targetPose[1])}, ${fmt(targetPose[2])}`,
          "var(--target)"
        );
        if (selected && tip) pushLine(vertices, tip, [targetPose[0], targetPose[1], targetPose[2]], TARGET_COLOR);
      }

      if (solveResult) {
        const tp = solveResult.target_pose_base;
        if (!targetPoses[side] && Array.isArray(tp) && tp.length >= 7) {
          const tipName = side === "left" ? "Fixed_Jaw_tip_2" : "Fixed_Jaw_tip";
          const tip = frameOrigin(tipName);
          pushCross(vertices, [tp[0], tp[1], tp[2]], 0.05, TARGET_COLOR);
          appendPoseAxis(vertices, tp, 0.07);
          addLabel([tp[0], tp[1], tp[2]], `target ${fmt(tp[0])}, ${fmt(tp[1])}, ${fmt(tp[2])}`, "var(--target)");
          if (tip) pushLine(vertices, tip, [tp[0], tp[1], tp[2]], TARGET_COLOR);
        }
        const reached = solveResult.fk_reached_xyz;
        if (Array.isArray(reached) && reached.length >= 3) {
          pushCross(vertices, [reached[0], reached[1], reached[2]], 0.03, REACHED_COLOR);
          addLabel([reached[0], reached[1], reached[2]], "FK reached", "var(--reached)");
        }
      }
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
        if (x < -60 || x > rect.width + 60 || y < -40 || y > rect.height + 40) return;
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
      const { viewProjection, drawView } = viewMatrices();
      gl.clearColor(0.010, 0.026, 0.028, 1.0);
      gl.clearDepth(1.0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

      let values;
      if (poseSource === "solution" && animTo) {
        const t = Math.min(1, (performance.now() - animStart) / ANIM_MS);
        const eased = t * t * (3 - 2 * t);
        values = lerpValues(animFrom, animTo, eased);
      } else {
        values = liveValues();
      }
      displayValues = values;

      frameMatrices.clear();
      if (robotModel && robotModel.root) {
        ensureMeshRenderer();
        gl.useProgram(meshProgram);
        gl.enable(gl.DEPTH_TEST);
        gl.depthFunc(gl.LEQUAL);
        gl.depthMask(true);
        drawRobotNode(robotModel.root, mat4Identity(), viewProjection, values);
      }

      const vertices = [];
      appendGrid(vertices);
      appendMarkers(vertices);
      gl.disable(gl.DEPTH_TEST);
      gl.depthMask(false);
      drawLineVertices(drawView, vertices);
      gl.depthMask(true);
      positionLabels(viewProjection);
      requestAnimationFrame(render);
    }

    // ---- model load ----
    function ensureModelLoaded() {
      if (robotModel || robotModelPromise) return robotModelPromise;
      robotModelPromise = loadRobotModel()
        .then(model => {
          robotModel = model;
          setPill(modelStatusEl, model ? `model ${model.stats.loadedVisuals}/${model.stats.loadedVisuals + model.stats.failedVisuals} visuals` : "model missing", model ? "good" : "bad");
          return model;
        })
        .catch(err => { setPill(modelStatusEl, "model error", "bad"); out.textContent = String(err); return null; });
      return robotModelPromise;
    }

    // ---- target body + solve (mirrors original ik_webview API) ----
    function numberValue(id) {
      const raw = $(id).value.trim();
      if (raw === "") return null;
      const value = Number(raw);
      if (!Number.isFinite(value)) throw new Error(`${id} must be finite`);
      return value;
    }
    function targetBody(forceSend = false) {
      const mode = $("mode").value;
      const body = {
        side: sideSel.value,
        mode,
        solver: $("solver").value,
        send: forceSend || $("send").checked,
        repeat: Math.max(1, Math.floor(Number($("repeat").value || 1))),
      };
      if (mode === "xyz") {
        body.xyz = [numberValue("x"), numberValue("y"), numberValue("z")];
        const quat = ["qx", "qy", "qz", "qw"].map(numberValue);
        if (quat.every(v => v !== null)) body.quat = quat;
      } else if (mode === "delta") {
        body.delta = [numberValue("x"), numberValue("y"), numberValue("z")];
      }
      return body;
    }
    function showResult(data) {
      out.textContent = JSON.stringify(data, null, 2);
      $("current").textContent = vec(data.current_pose_base);
      const errMm = data.target_error_m == null ? null : data.target_error_m * 1000;
      $("error").textContent = errMm == null ? "-" : `${errMm.toFixed(errMm < 1 ? 4 : 2)} mm`;
      $("error").style.color = errMm == null ? "" : (data.projected ? "var(--warn)" : "var(--good)");
      $("modeOut").textContent = `${data.solver || "?"} · ${data.projection_mode || "-"}`;
      badge.textContent = data.sent ? "sent" : (data.projected ? "nearest" : "solved");
      badge.className = data.ok ? "pill good" : "pill bad";
      solveResult = data;
      showSolution();
    }
    async function solve(forceSend = false) {
      try {
        logEl.textContent = forceSend ? "sending..." : "solving...";
        const res = await fetch("/api/solve", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(targetBody(forceSend)),
        });
        const data = await res.json();
        if (!res.ok || !data.ok) throw new Error(data.error || `HTTP ${res.status}`);
        showResult(data);
        logEl.textContent = forceSend ? "sent — arm animating to solution" : "solved — arm animating to solution";
      } catch (err) {
        badge.textContent = "error";
        badge.className = "pill bad";
        logEl.textContent = String(err);
      }
    }

    async function refreshState() {
      try {
        const res = await fetch("/api/state", { cache: "no-store" });
        const data = await res.json();
        liveState = data;
        const side = sideSel.value;
        const pose = data.current && data.current[side];
        if (poseSource === "live") $("current").textContent = vec(pose);
        const proprioAge = Number(data.proprio_age_s);
        const proprioLive = data.proprio && Number.isFinite(proprioAge) && proprioAge < 1.0;
        setPill(proprioStatusEl, proprioLive ? `proprio ${proprioAge.toFixed(2)}s` : (data.proprio ? "proprio stale" : "proprio waiting"), proprioLive ? "good" : data.proprio ? "warn" : "bad");
        if (badge.className.indexOf("good") < 0 && badge.className.indexOf("bad") < 0) {
          badge.textContent = data.ok ? "online" : "waiting";
          badge.className = data.ok ? "pill good" : "pill warn";
        }
      } catch (err) {
        setPill(proprioStatusEl, "offline", "bad");
        logEl.textContent = String(err);
      }
    }

    // ---- interactions ----
    document.querySelectorAll("[data-delta]").forEach(button => {
      button.addEventListener("click", () => {
        const values = button.dataset.delta.split(",");
        $("mode").value = "delta";
        $("x").value = values[0]; $("y").value = values[1]; $("z").value = values[2];
        solve(false);
      });
    });
    $("solve").addEventListener("click", () => solve(false));
    $("sendOnce").addEventListener("click", () => solve(true));
    $("poseLive").addEventListener("click", showLive);
    $("poseSolution").addEventListener("click", showSolution);
    sideSel.addEventListener("change", () => {
      const pose = targetPoses[sideSel.value];
      if (Array.isArray(pose)) syncTargetInputs(sideSel.value, pose);
      refreshState();
    });
    ["x", "y", "z", "qx", "qy", "qz", "qw"].forEach(id => $(id).addEventListener("input", updateTargetFromInputs));
    $("mode").addEventListener("change", updateTargetFromInputs);
    window.addEventListener("keydown", moveKeyboardTarget);

    canvas.addEventListener("mousedown", ev => { dragging = true; lastMouse = [ev.clientX, ev.clientY]; });
    window.addEventListener("mouseup", () => { dragging = false; });
    window.addEventListener("mousemove", ev => {
      if (!dragging) return;
      const dx = ev.clientX - lastMouse[0];
      const dy = ev.clientY - lastMouse[1];
      lastMouse = [ev.clientX, ev.clientY];
      if (ev.shiftKey) {
        const side = [Math.sin(orbit.yaw), -Math.cos(orbit.yaw), 0];
        orbit.target = vAdd(orbit.target, vAdd(vScale(side, -dx * 0.0014), vScale([0, 0, 1], dy * 0.0014)));
      } else {
        orbit.yaw += dx * 0.007;
        orbit.pitch = Math.max(-0.25, Math.min(1.42, orbit.pitch + dy * 0.005));
      }
    });
    canvas.addEventListener("wheel", ev => {
      ev.preventDefault();
      orbit.distance = Math.max(0.45, Math.min(5.0, orbit.distance * Math.exp(ev.deltaY * 0.001)));
    }, { passive: false });
    $("resetView").onclick = () => { orbit.target = [0.02, 0.0, 0.72]; orbit.yaw = -0.82; orbit.pitch = 0.34; orbit.distance = 1.9; };
    $("topView").onclick = () => { orbit.target = [0.02, 0.0, 0.55]; orbit.yaw = -Math.PI / 2; orbit.pitch = 1.38; orbit.distance = 1.72; };
    $("sideView").onclick = () => { orbit.target = [0.02, 0.0, 0.72]; orbit.yaw = 0.0; orbit.pitch = 0.20; orbit.distance = 1.8; };
    $("frontView").onclick = () => { orbit.target = [0.02, 0.0, 0.72]; orbit.yaw = -Math.PI / 2; orbit.pitch = 0.20; orbit.distance = 1.8; };
    window.addEventListener("resize", resize);

    function initGl() {
      gl = canvas.getContext("webgl", { antialias: true, alpha: false, powerPreference: "high-performance" });
      if (!gl) { setPill(modelStatusEl, "webgl unavailable", "bad"); return false; }
      gl.disable(gl.CULL_FACE);
      return true;
    }

    if (initGl()) {
      ensureModelLoaded();
      refreshState();
      setInterval(refreshState, 250);
      requestAnimationFrame(render);
    }
  </script>
</body>
</html>
"""


def _public_host() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def _args_from_request(server: ThreadingHTTPServer, body: dict[str, Any]) -> argparse.Namespace:
    mode = str(body.get("mode") or "current")
    side = str(body.get("side") or "right")
    if side not in ("right", "left"):
        raise ValueError("side must be right or left")
    xyz = body.get("xyz") if mode == "xyz" else None
    delta = body.get("delta") if mode == "delta" else None
    quat = body.get("quat") if xyz is not None else None
    solver = str(body.get("solver") or "accurate")
    if solver not in ("accurate", "fast"):
        raise ValueError("solver must be accurate or fast")
    return argparse.Namespace(
        host=server.fast_zmq_host,  # type: ignore[attr-defined]
        pub_port=server.fast_zmq_pub_port,  # type: ignore[attr-defined]
        pull_port=server.fast_zmq_pull_port,  # type: ignore[attr-defined]
        robot_id=server.robot_id,  # type: ignore[attr-defined]
        timeout_s=server.fast_timeout_s,  # type: ignore[attr-defined]
        side=side,
        pose=None,
        xyz=xyz,
        delta=delta,
        quat=quat,
        seed=None,
        tolerance_m=ik_solver.ROBOT_ARM_IK_TOLERANCE_M,
        solver=solver,
        fine_tolerance_m=ik_solver.ACCURATE_FINE_TOLERANCE_M,
        nearest_seeds=ik_solver.ACCURATE_NEAREST_SEEDS,
        no_emergency_seed=False,
        send=bool(body.get("send", False)),
        repeat=max(1, int(body.get("repeat") or 1)),
        repeat_dt_s=0.02,
        connect_settle_s=float(os.environ.get("IK_ZMQ_CONNECT_SETTLE_S", "0.03")),
        flush_s=float(os.environ.get("IK_ZMQ_FLUSH_S", "0.02")),
        source_id="indoory_ros.tools.ik_webview",
        priority=80,
        lease_ms=250,
        compact=False,
    )


def _proprio_for_render(proprio: dict[str, Any] | None) -> dict[str, Any] | None:
    """Lean proprio payload for live URDF rendering in the browser."""
    if not isinstance(proprio, dict):
        return None
    names = proprio.get("joint_names_urdf")
    values = proprio.get("joint_pos_urdf_rad")
    if not isinstance(names, list) or not isinstance(values, list):
        return None
    return {
        "joint_names_urdf": list(names),
        "joint_pos_urdf_rad": [
            (float(v) if isinstance(v, (int, float)) and math.isfinite(float(v)) else 0.0)
            for v in values
        ],
        "stamp_ns": proprio.get("stamp_ns"),
    }


def _state(server: ThreadingHTTPServer) -> dict[str, Any]:
    args = argparse.Namespace(
        host=server.fast_zmq_host,  # type: ignore[attr-defined]
        pub_port=server.fast_zmq_pub_port,  # type: ignore[attr-defined]
        robot_id=server.robot_id,  # type: ignore[attr-defined]
        timeout_s=min(0.25, float(server.fast_timeout_s)),  # type: ignore[attr-defined]
    )
    proprio, tf_links = ik_solver._latest_fast_state(args)
    current = {
        "right": ik_solver._pose_from_tf_links(tf_links, "right"),
        "left": ik_solver._pose_from_tf_links(tf_links, "left"),
    }
    return {
        "ok": proprio is not None or tf_links is not None,
        "fast_endpoint": f"tcp://{server.fast_zmq_host}:{server.fast_zmq_pub_port}",  # type: ignore[attr-defined]
        "robot_id": server.robot_id,  # type: ignore[attr-defined]
        "current": current,
        "proprio": _proprio_for_render(proprio),
        "proprio_age_s": _age_s(proprio),
        "tf_links_age_s": _age_s(tf_links),
    }


def _age_s(msg: dict[str, Any] | None) -> float | None:
    if not isinstance(msg, dict):
        return None
    stamp = msg.get("stamp_ns")
    try:
        value = float(stamp)
    except (TypeError, ValueError):
        return None
    if value <= 0:
        return None
    import time

    return max(0.0, time.time_ns() / 1e9 - value / 1e9)


class IkWebHandler(BaseHTTPRequestHandler):
    server_version = "IndooryIkWebView/2.0"

    def do_GET(self) -> None:
        parsed = urlsplit(self.path)
        path = parsed.path
        if path == "/":
            self._send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/api/state":
            self._send_json(_state(self.server))  # type: ignore[arg-type]
            return
        if path == "/api/model/xlerobot.json":
            if xlerobot_model_description is None:
                self._send_json(
                    {"ok": False, "error": str(XLEROBOT_MODEL_IMPORT_ERROR or "model helper unavailable")},
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

    def do_POST(self) -> None:
        parsed = urlsplit(self.path)
        if parsed.path != "/api/solve":
            self.send_error(404, "not found")
            return
        try:
            body_len = int(self.headers.get("Content-Length", "0") or "0")
            body = json.loads(self.rfile.read(body_len).decode("utf-8") or "{}")
            if not isinstance(body, dict):
                raise ValueError("body must be a JSON object")
            result = ik_solver.solve(_args_from_request(self.server, body))  # type: ignore[arg-type]
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": f"{type(exc).__name__}: {exc}"},
                status=400,
            )
            return
        self._send_json(result)

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.path.startswith("/api/state"):
            return
        super().log_message(fmt, *args)

    def _send_json(self, data: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False, allow_nan=False, default=str).encode("utf-8")
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("IK_WEB_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("IK_WEB_PORT", "8098")))
    parser.add_argument("--fast-zmq-host", default=os.environ.get("FAST_ZMQ_HOST", "127.0.0.1"))
    parser.add_argument("--fast-zmq-pub-port", type=int, default=int(os.environ.get("FAST_ZMQ_PUB_PORT", "8855")))
    parser.add_argument("--fast-zmq-pull-port", type=int, default=int(os.environ.get("FAST_ZMQ_PULL_PORT", "8856")))
    parser.add_argument("--robot-id", type=int, default=int(os.environ.get("FAST_ZMQ_ROBOT_ID", "0")))
    parser.add_argument("--fast-timeout-s", type=float, default=0.7)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), IkWebHandler)
    server.fast_zmq_host = args.fast_zmq_host  # type: ignore[attr-defined]
    server.fast_zmq_pub_port = args.fast_zmq_pub_port  # type: ignore[attr-defined]
    server.fast_zmq_pull_port = args.fast_zmq_pull_port  # type: ignore[attr-defined]
    server.robot_id = args.robot_id  # type: ignore[attr-defined]
    server.fast_timeout_s = args.fast_timeout_s  # type: ignore[attr-defined]

    def shutdown(_signum: int, _frame: Any) -> None:
        threading.Thread(target=server.shutdown, name="http-shutdown", daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    host = _public_host() if args.host in ("0.0.0.0", "::") else args.host
    print(
        f"IK 3D webview listening on http://{host}:{args.port}/ "
        f"(fast_zmq=tcp://{args.fast_zmq_host}:{args.fast_zmq_pub_port}/{args.fast_zmq_pull_port}, robot={args.robot_id})",
        flush=True,
    )
    if xlerobot_model_description is None:
        print(f"[warn] xlerobot model helper unavailable: {XLEROBOT_MODEL_IMPORT_ERROR}", file=sys.stderr, flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
