#!/usr/bin/env python3
import argparse
import array
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

try:
    import msgpack
    import zmq
except Exception as exc:  # pragma: no cover - optional fast command path
    msgpack = None
    zmq = None
    FAST_ZMQ_IMPORT_ERROR = exc
else:
    FAST_ZMQ_IMPORT_ERROR = None


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from ros_bridge.webview_monitor import RosbridgeMonitor  # noqa: E402


def _truthy(value: str | None, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


if (
    FAST_ZMQ_IMPORT_ERROR is not None
    and _truthy(os.environ.get("WEBVIEW_FAST_ZMQ_COMMAND", os.environ.get("ENABLE_FAST_ZMQ")), True)
    and os.environ.get("ROBOT_WEBVIEW_REEXEC") != "1"
):
    candidates = [
        Path(os.path.expanduser(os.environ.get("XLE_ROBOT_VENV", "~/xlerobot-io-venv"))) / "bin" / "python3",
        Path(os.path.expanduser("~/.miniforge3/envs/lerobot/bin/python3")),
    ]
    for python in candidates:
        if python.exists() and python.resolve() != Path(sys.executable).resolve():
            env = os.environ.copy()
            env["ROBOT_WEBVIEW_REEXEC"] = "1"
            os.execve(str(python), [str(python), *sys.argv], env)


DEFAULT_CMD_TOPICS = ["/xlerobot/cmd_vel", "/cmd_vel"]
DEFAULT_ODOM_TOPICS = ["/xlerobot/odom", "/odom"]
DEFAULT_SCAN_TOPICS = ["/xlerobot/scan", "/scan"]

INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Indoory Robot View</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #111417;
      --panel: #191f24;
      --panel-2: #20272d;
      --text: #ecf2f4;
      --muted: #96a4ab;
      --line: #344049;
      --good: #4bd18a;
      --warn: #f4c95d;
      --bad: #ff6b6b;
      --cyan: #5ed6e8;
      --pink: #ff8ab3;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    main {
      width: min(1180px, calc(100vw - 28px));
      margin: 0 auto;
      padding: 18px 0 22px;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 14px;
    }
    h1 {
      font-size: 22px;
      margin: 0;
      font-weight: 760;
    }
    .sub {
      margin-top: 4px;
      color: var(--muted);
      font-size: 13px;
    }
    .badge {
      display: inline-flex;
      align-items: center;
      min-height: 30px;
      padding: 5px 10px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: var(--panel);
      color: var(--muted);
      font-size: 13px;
      white-space: nowrap;
    }
    .badge.good { color: var(--good); }
    .badge.warn { color: var(--warn); }
    .badge.bad { color: var(--bad); }
    .grid {
      display: grid;
      grid-template-columns: 1.35fr 0.9fr;
      gap: 14px;
      align-items: stretch;
    }
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      min-width: 0;
    }
    .panel h2 {
      font-size: 14px;
      line-height: 20px;
      margin: 0;
      padding: 12px 13px;
      border-bottom: 1px solid var(--line);
      color: #d9e3e7;
      font-weight: 720;
    }
    .body { padding: 12px; }
    .metrics {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }
    .metric {
      background: var(--panel-2);
      border: 1px solid var(--line);
      border-radius: 7px;
      padding: 10px;
      min-width: 0;
    }
    .label {
      color: var(--muted);
      font-size: 12px;
      line-height: 16px;
      overflow-wrap: anywhere;
    }
    .value {
      margin-top: 6px;
      font-variant-numeric: tabular-nums;
      font-size: 22px;
      line-height: 28px;
      font-weight: 760;
      overflow-wrap: anywhere;
    }
    .unit {
      color: var(--muted);
      font-size: 12px;
      margin-left: 4px;
      font-weight: 560;
    }
    canvas {
      display: block;
      width: 100%;
      height: auto;
      background: #0c0f11;
      border: 1px solid var(--line);
      border-radius: 8px;
    }
    .stack {
      display: grid;
      gap: 14px;
    }
    .topic-list {
      display: grid;
      gap: 8px;
      font-size: 13px;
      color: var(--muted);
    }
    .topic {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      padding: 8px 10px;
      background: var(--panel-2);
      border: 1px solid var(--line);
      border-radius: 7px;
      min-width: 0;
    }
    .topic code {
      color: #dbe6ea;
      overflow-wrap: anywhere;
      font-size: 12px;
    }
    .age {
      font-variant-numeric: tabular-nums;
      white-space: nowrap;
    }
    .mono {
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
    }
    .control-strip {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-bottom: 12px;
    }
    .slider-label {
      display: grid;
      grid-template-columns: auto minmax(80px, 1fr) 42px;
      align-items: center;
      gap: 8px;
      color: var(--muted);
      font-size: 12px;
      min-width: 0;
    }
    input[type="range"] {
      width: 100%;
      accent-color: var(--cyan);
    }
    .teleop-layout {
      display: grid;
      grid-template-columns: 152px 1fr;
      gap: 12px;
      align-items: center;
    }
    .joystick-pad {
      position: relative;
      width: 152px;
      height: 152px;
      border-radius: 8px;
      border: 1px solid var(--line);
      background:
        linear-gradient(90deg, transparent 49%, #344049 49%, #344049 51%, transparent 51%),
        linear-gradient(0deg, transparent 49%, #344049 49%, #344049 51%, transparent 51%),
        #0c0f11;
      touch-action: none;
      user-select: none;
    }
    .joystick-knob {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 36px;
      height: 36px;
      transform: translate(-50%, -50%);
      border-radius: 50%;
      background: var(--cyan);
      box-shadow: 0 0 0 5px rgba(94, 214, 232, 0.18);
      pointer-events: none;
    }
    .teleop-buttons {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }
    button {
      min-height: 40px;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: var(--panel-2);
      color: var(--text);
      font-weight: 760;
      font-size: 13px;
      cursor: pointer;
      user-select: none;
    }
    button:active,
    button.active {
      border-color: var(--cyan);
      background: #18323a;
      color: #fff;
    }
    .stop-button {
      color: #fff;
      background: #7a2630;
      border-color: #a84855;
    }
    .control-state {
      min-height: 18px;
      margin-top: 10px;
      color: var(--muted);
      overflow-wrap: anywhere;
    }
    @media (max-width: 860px) {
      main { width: min(100vw - 20px, 720px); }
      header { align-items: flex-start; flex-direction: column; }
      .grid { grid-template-columns: 1fr; }
      .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
    @media (max-width: 520px) {
      .metrics { grid-template-columns: 1fr; }
      .control-strip { grid-template-columns: 1fr; }
      .teleop-layout { grid-template-columns: 1fr; }
      .joystick-pad { width: 100%; max-width: 220px; height: 180px; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>Indoory Robot View</h1>
        <div id="rosbridge" class="sub mono">fast transport: waiting</div>
      </div>
      <div id="stateBadge" class="badge warn">Connecting</div>
    </header>

    <section class="grid">
      <div class="stack">
        <section class="panel">
          <h2>Odometry</h2>
          <div class="body">
            <canvas id="pathCanvas" width="760" height="440"></canvas>
          </div>
        </section>
        <section class="panel">
          <h2>Command</h2>
          <div class="body">
            <canvas id="cmdCanvas" width="760" height="210"></canvas>
          </div>
        </section>
      </div>

      <div class="stack">
        <section class="panel">
          <h2>Teleop</h2>
          <div class="body">
            <div class="control-strip">
              <label class="slider-label">Linear <input id="linearScale" type="range" min="0.03" max="0.30" step="0.01" value="0.12"><span id="linearScaleValue" class="mono">0.12</span></label>
              <label class="slider-label">Angular <input id="angularScale" type="range" min="0.10" max="1.57" step="0.01" value="0.60"><span id="angularScaleValue" class="mono">0.60</span></label>
            </div>
            <div class="teleop-layout">
              <div id="joystickPad" class="joystick-pad">
                <div id="joystickKnob" class="joystick-knob"></div>
              </div>
              <div class="teleop-buttons">
                <button class="drive-button" data-x="1" data-y="0" data-z="0">W</button>
                <button class="drive-button" data-x="0" data-y="1" data-z="0">A</button>
                <button id="stopButton" class="stop-button">STOP</button>
                <button class="drive-button" data-x="0" data-y="-1" data-z="0">D</button>
                <button class="drive-button" data-x="-1" data-y="0" data-z="0">S</button>
                <button class="drive-button" data-x="0" data-y="0" data-z="1">CCW</button>
                <button class="drive-button" data-x="0" data-y="0" data-z="-1">CW</button>
              </div>
            </div>
            <div id="controlState" class="mono control-state">idle</div>
          </div>
        </section>

        <section class="panel">
          <h2>Live Values</h2>
          <div class="body metrics">
            <div class="metric"><div class="label">linear x</div><div class="value"><span id="vx">0.000</span><span class="unit">m/s</span></div></div>
            <div class="metric"><div class="label">linear y</div><div class="value"><span id="vy">0.000</span><span class="unit">m/s</span></div></div>
            <div class="metric"><div class="label">angular z</div><div class="value"><span id="wz">0.000</span><span class="unit">rad/s</span></div></div>
            <div class="metric"><div class="label">pose x</div><div class="value"><span id="px">0.000</span><span class="unit">m</span></div></div>
            <div class="metric"><div class="label">pose y</div><div class="value"><span id="py">0.000</span><span class="unit">m</span></div></div>
            <div class="metric"><div class="label">yaw</div><div class="value"><span id="yaw">0.0</span><span class="unit">deg</span></div></div>
          </div>
        </section>

        <section class="panel">
          <h2>Topics</h2>
          <div class="body topic-list">
            <div class="topic"><code id="cmdTopic">cmd</code><span id="cmdAge" class="age">-</span></div>
            <div class="topic"><code id="odomTopic">odom</code><span id="odomAge" class="age">-</span></div>
            <div class="topic"><code id="scanTopic">scan</code><span id="scanAge" class="age">-</span></div>
          </div>
        </section>

        <section class="panel">
          <h2>Laser Scan</h2>
          <div class="body">
            <canvas id="scanCanvas" width="460" height="360"></canvas>
          </div>
        </section>
      </div>
    </section>
  </main>

  <script>
    const pathCanvas = document.getElementById('pathCanvas');
    const cmdCanvas = document.getElementById('cmdCanvas');
    const scanCanvas = document.getElementById('scanCanvas');
    const pathCtx = pathCanvas.getContext('2d');
    const cmdCtx = cmdCanvas.getContext('2d');
    const scanCtx = scanCanvas.getContext('2d');
    const linearScale = q('linearScale');
    const angularScale = q('angularScale');
    const joystickPad = q('joystickPad');
    const joystickKnob = q('joystickKnob');
    const path = [];
    let last = {};
    let driveTimer = null;
    let activeButton = null;
    let joystickPointerId = null;
    const pressedKeys = new Set();
    const keyCommands = new Map([
      ['w', {x: 1, y: 0, z: 0}],
      ['arrowup', {x: 1, y: 0, z: 0}],
      ['s', {x: -1, y: 0, z: 0}],
      ['arrowdown', {x: -1, y: 0, z: 0}],
      ['a', {x: 0, y: 1, z: 0}],
      ['arrowleft', {x: 0, y: 1, z: 0}],
      ['d', {x: 0, y: -1, z: 0}],
      ['arrowright', {x: 0, y: -1, z: 0}],
      ['q', {x: 0, y: 0, z: 1}],
      ['e', {x: 0, y: 0, z: -1}]
    ]);

    function q(id) { return document.getElementById(id); }
    function fmt(v, n = 3) {
      if (typeof v !== 'number' || !Number.isFinite(v)) return '-';
      return v.toFixed(n);
    }
    function ageText(age) {
      if (typeof age !== 'number' || !Number.isFinite(age)) return '-';
      if (age < 1) return Math.round(age * 1000) + ' ms';
      return age.toFixed(1) + ' s';
    }
    function setBadge(ok, text) {
      const badge = q('stateBadge');
      badge.className = 'badge ' + (ok ? 'good' : 'bad');
      badge.textContent = text;
    }
    function yawFromQuat(qv) {
      if (!qv) return 0;
      const x = qv.x || 0, y = qv.y || 0, z = qv.z || 0, w = qv.w || 1;
      return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }
    function drawGrid(ctx, w, h) {
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#0c0f11';
      ctx.fillRect(0, 0, w, h);
      ctx.strokeStyle = '#1f2930';
      ctx.lineWidth = 1;
      for (let x = 0; x <= w; x += 40) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
      }
      for (let y = 0; y <= h; y += 40) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
      }
    }
    function drawPath(odom) {
      const w = pathCanvas.width, h = pathCanvas.height;
      drawGrid(pathCtx, w, h);
      if (!odom) return;
      const pose = odom.pose && odom.pose.pose;
      if (!pose) return;
      const p = pose.position || {};
      const yaw = yawFromQuat(pose.orientation);
      if (typeof p.x === 'number' && typeof p.y === 'number') {
        const prev = path[path.length - 1];
        if (!prev || Math.hypot(prev.x - p.x, prev.y - p.y) > 0.002) {
          path.push({x: p.x, y: p.y, yaw});
          if (path.length > 900) path.shift();
        }
      }
      const xs = path.map(v => v.x);
      const ys = path.map(v => v.y);
      const minX = Math.min(-0.5, ...xs), maxX = Math.max(0.5, ...xs);
      const minY = Math.min(-0.5, ...ys), maxY = Math.max(0.5, ...ys);
      const span = Math.max(maxX - minX, maxY - minY, 1.0);
      const scale = Math.min(w, h) * 0.78 / span;
      const cx = w / 2 - ((minX + maxX) / 2) * scale;
      const cy = h / 2 + ((minY + maxY) / 2) * scale;
      function map(v) { return {x: cx + v.x * scale, y: cy - v.y * scale}; }
      pathCtx.strokeStyle = '#5ed6e8';
      pathCtx.lineWidth = 3;
      pathCtx.beginPath();
      path.forEach((v, i) => {
        const m = map(v);
        if (i === 0) pathCtx.moveTo(m.x, m.y);
        else pathCtx.lineTo(m.x, m.y);
      });
      pathCtx.stroke();
      const now = path[path.length - 1];
      if (now) {
        const m = map(now);
        pathCtx.fillStyle = '#ff8ab3';
        pathCtx.beginPath(); pathCtx.arc(m.x, m.y, 7, 0, Math.PI * 2); pathCtx.fill();
        pathCtx.strokeStyle = '#ff8ab3';
        pathCtx.lineWidth = 4;
        pathCtx.beginPath();
        pathCtx.moveTo(m.x, m.y);
        pathCtx.lineTo(m.x + Math.cos(now.yaw) * 28, m.y - Math.sin(now.yaw) * 28);
        pathCtx.stroke();
      }
    }
    function drawCommand(cmd) {
      const w = cmdCanvas.width, h = cmdCanvas.height;
      drawGrid(cmdCtx, w, h);
      const msg = cmd && cmd.msg ? cmd.msg : {};
      const lin = msg.linear || {};
      const ang = msg.angular || {};
      const x = Number(lin.x || 0);
      const y = Number(lin.y || 0);
      const z = Number(ang.z || 0);
      const cx = w * 0.25, cy = h * 0.5;
      const scale = 260;
      cmdCtx.strokeStyle = '#5ed6e8';
      cmdCtx.fillStyle = '#5ed6e8';
      cmdCtx.lineWidth = 5;
      cmdCtx.beginPath();
      cmdCtx.moveTo(cx, cy);
      cmdCtx.lineTo(cx + y * scale, cy - x * scale);
      cmdCtx.stroke();
      cmdCtx.beginPath();
      cmdCtx.arc(cx + y * scale, cy - x * scale, 7, 0, Math.PI * 2);
      cmdCtx.fill();
      cmdCtx.fillStyle = '#dbe6ea';
      cmdCtx.font = '18px ui-monospace, monospace';
      cmdCtx.fillText('x ' + fmt(x) + ' m/s', w * 0.50, 76);
      cmdCtx.fillText('y ' + fmt(y) + ' m/s', w * 0.50, 108);
      cmdCtx.fillText('z ' + fmt(z) + ' rad/s', w * 0.50, 140);
    }
    function drawScan(scan) {
      const w = scanCanvas.width, h = scanCanvas.height;
      drawGrid(scanCtx, w, h);
      const msg = scan && scan.msg ? scan.msg : null;
      const cx = w / 2, cy = h / 2;
      scanCtx.strokeStyle = '#344049';
      scanCtx.beginPath(); scanCtx.arc(cx, cy, Math.min(w, h) * 0.38, 0, Math.PI * 2); scanCtx.stroke();
      if (!msg || !Array.isArray(msg.ranges)) {
        scanCtx.fillStyle = '#96a4ab';
        scanCtx.font = '15px ui-sans-serif, system-ui';
        scanCtx.fillText('No scan yet', cx - 38, cy + 5);
        return;
      }
      const maxRange = msg.range_max || 12;
      const minRange = msg.range_min || 0.15;
      const scale = Math.min(w, h) * 0.40 / maxRange;
      scanCtx.fillStyle = '#4bd18a';
      msg.ranges.forEach((r, i) => {
        if (typeof r !== 'number' || r < minRange || r > maxRange) return;
        const a = (msg.angle_min || 0) + i * (msg.angle_increment || 0);
        const x = cx + Math.cos(a) * r * scale;
        const y = cy - Math.sin(a) * r * scale;
        scanCtx.fillRect(x, y, 2, 2);
      });
    }
    function update(data) {
      last = data;
      const fast = data.fast_zmq || {};
      const sensor = data.fast_sensor || {};
      const cmdTransport = fast.enabled ? ('cmd ' + (fast.ok ? 'ok' : 'warn')) : 'cmd off';
      const sensorTransport = sensor.enabled ? ('sensors ' + (sensor.ok ? 'ok' : 'warn')) : 'sensors off';
      q('rosbridge').textContent = 'fast_zmq: ' + cmdTransport + ' · ' + sensorTransport;
      setBadge(data.connected, data.connected ? 'Connected' : 'Disconnected');
      const cmd = data.latest_cmd || {};
      const odom = data.latest_odom || {};
      const msg = cmd.msg || {};
      const lin = msg.linear || {};
      const ang = msg.angular || {};
      const pose = odom.msg && odom.msg.pose && odom.msg.pose.pose;
      const twist = odom.msg && odom.msg.twist && odom.msg.twist.twist;
      const vel = twist || {linear: lin, angular: ang};
      q('vx').textContent = fmt((vel.linear || {}).x || 0);
      q('vy').textContent = fmt((vel.linear || {}).y || 0);
      q('wz').textContent = fmt((vel.angular || {}).z || 0);
      q('px').textContent = fmt(pose && pose.position ? pose.position.x || 0 : 0);
      q('py').textContent = fmt(pose && pose.position ? pose.position.y || 0 : 0);
      q('yaw').textContent = fmt(yawFromQuat(pose && pose.orientation) * 180 / Math.PI, 1);
      q('cmdTopic').textContent = cmd.topic || 'fast:cmd_vel';
      q('odomTopic').textContent = odom.topic || 'fast:odom.0';
      q('scanTopic').textContent = (data.latest_scan && data.latest_scan.topic) || 'fast:scan.0';
      q('cmdAge').textContent = ageText(cmd.age_s);
      q('odomAge').textContent = ageText(odom.age_s);
      q('scanAge').textContent = ageText(data.latest_scan && data.latest_scan.age_s);
      drawPath(odom.msg);
      drawCommand(cmd);
      drawScan(data.latest_scan);
    }
    function updateScaleLabels() {
      q('linearScaleValue').textContent = Number(linearScale.value).toFixed(2);
      q('angularScaleValue').textContent = Number(angularScale.value).toFixed(2);
    }
    function setControlState(text, ok = true) {
      const state = q('controlState');
      state.textContent = text;
      state.style.color = ok ? '#96a4ab' : '#ff6b6b';
    }
    async function sendCmd(x, y, z) {
      try {
        const res = await fetch('/api/cmd_vel', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({x, y, z}),
          cache: 'no-store'
        });
        const body = await res.json();
        if (!body.ok) {
          setControlState(body.error || 'command failed', false);
          return false;
        }
        const base = body.base_cmd_vel || [body.command.x, body.command.y, body.command.z];
        setControlState((body.transport || 'cmd') + ' [' + base.map(v => fmt(Number(v))).join(', ') + ']');
        return true;
      } catch (err) {
        setControlState(String(err), false);
        return false;
      }
    }
    function scaledCommand(base) {
      return {
        x: Number(base.x || 0) * Number(linearScale.value),
        y: Number(base.y || 0) * Number(linearScale.value),
        z: Number(base.z || 0) * Number(angularScale.value)
      };
    }
    function startDriving(base, button) {
      stopDriving(false);
      activeButton = button || null;
      if (activeButton) activeButton.classList.add('active');
      const tick = () => {
        const cmd = scaledCommand(base);
        sendCmd(cmd.x, cmd.y, cmd.z);
      };
      tick();
      driveTimer = setInterval(tick, 50);
    }
    function stopDriving(sendStop = true) {
      if (driveTimer) clearInterval(driveTimer);
      driveTimer = null;
      if (activeButton) activeButton.classList.remove('active');
      activeButton = null;
      resetJoystick();
      if (sendStop) sendCmd(0, 0, 0);
    }
    function resetJoystick() {
      joystickKnob.style.left = '50%';
      joystickKnob.style.top = '50%';
    }
    function joystickCommand(event) {
      const rect = joystickPad.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      const radius = Math.max(24, Math.min(rect.width, rect.height) * 0.42);
      let dx = (event.clientX - cx) / radius;
      let dy = (event.clientY - cy) / radius;
      const mag = Math.hypot(dx, dy);
      if (mag > 1) {
        dx /= mag;
        dy /= mag;
      }
      joystickKnob.style.left = (50 + dx * 42) + '%';
      joystickKnob.style.top = (50 + dy * 42) + '%';
      return {x: -dy, y: -dx, z: 0};
    }
    function startJoystick(event) {
      stopDriving(false);
      joystickPointerId = event.pointerId;
      joystickPad.setPointerCapture(joystickPointerId);
      const tickBase = {current: joystickCommand(event)};
      const tick = () => {
        const cmd = scaledCommand(tickBase.current);
        sendCmd(cmd.x, cmd.y, cmd.z);
      };
      driveTimer = setInterval(tick, 100);
      tick();
      joystickPad.onpointermove = (moveEvent) => {
        if (moveEvent.pointerId === joystickPointerId) {
          tickBase.current = joystickCommand(moveEvent);
        }
      };
    }
    function endJoystick() {
      joystickPointerId = null;
      joystickPad.onpointermove = null;
      stopDriving(true);
    }
    document.querySelectorAll('.drive-button').forEach((button) => {
      const base = {
        x: Number(button.dataset.x),
        y: Number(button.dataset.y),
        z: Number(button.dataset.z)
      };
      button.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        button.setPointerCapture(event.pointerId);
        startDriving(base, button);
      });
      button.addEventListener('pointerup', () => stopDriving(true));
      button.addEventListener('pointercancel', () => stopDriving(true));
      button.addEventListener('lostpointercapture', () => stopDriving(true));
    });
    q('stopButton').addEventListener('click', () => stopDriving(true));
    joystickPad.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      startJoystick(event);
    });
    joystickPad.addEventListener('pointerup', endJoystick);
    joystickPad.addEventListener('pointercancel', endJoystick);
    joystickPad.addEventListener('lostpointercapture', endJoystick);
    window.addEventListener('blur', () => stopDriving(true));
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) stopDriving(true);
    });
    window.addEventListener('pagehide', () => {
      navigator.sendBeacon('/api/stop', new Blob(['{}'], {type: 'application/json'}));
    });
    linearScale.addEventListener('input', updateScaleLabels);
    angularScale.addEventListener('input', updateScaleLabels);
    function keyboardBase() {
      const base = {x: 0, y: 0, z: 0};
      pressedKeys.forEach((key) => {
        const cmd = keyCommands.get(key);
        if (!cmd) return;
        base.x += cmd.x;
        base.y += cmd.y;
        base.z += cmd.z;
      });
      const xy = Math.hypot(base.x, base.y);
      if (xy > 1) {
        base.x /= xy;
        base.y /= xy;
      }
      base.z = Math.max(-1, Math.min(1, base.z));
      return base;
    }
    function startKeyboardDrive() {
      stopDriving(false);
      const tick = () => {
        if (pressedKeys.size === 0) {
          stopDriving(true);
          return;
        }
        const cmd = scaledCommand(keyboardBase());
        sendCmd(cmd.x, cmd.y, cmd.z);
      };
      tick();
      driveTimer = setInterval(tick, 100);
    }
    document.addEventListener('keydown', (event) => {
      const tag = (event.target && event.target.tagName || '').toLowerCase();
      if (tag === 'input' || tag === 'textarea' || tag === 'select') return;
      const key = event.key.toLowerCase();
      if (key === ' ') {
        event.preventDefault();
        pressedKeys.clear();
        stopDriving(true);
        return;
      }
      if (!keyCommands.has(key)) return;
      event.preventDefault();
      if (!pressedKeys.has(key)) {
        pressedKeys.add(key);
        startKeyboardDrive();
      }
    });
    document.addEventListener('keyup', (event) => {
      const key = event.key.toLowerCase();
      if (!keyCommands.has(key)) return;
      event.preventDefault();
      pressedKeys.delete(key);
      if (pressedKeys.size === 0) stopDriving(true);
    });
    updateScaleLabels();
    async function poll() {
      try {
        const res = await fetch('/api/status', {cache: 'no-store'});
        update(await res.json());
      } catch (err) {
        setBadge(false, 'Webview lost');
      } finally {
        setTimeout(poll, 180);
      }
    }
    poll();
  </script>
</body>
</html>
"""


def _csv(value: str | None, fallback: list[str]) -> list[str]:
    if not value:
        return fallback
    items = [item.strip() for item in value.split(",") if item.strip()]
    return items or fallback


def _now() -> float:
    return time.time()


def _with_age(entry: dict[str, Any] | None) -> dict[str, Any] | None:
    if not entry:
        return None
    copied = dict(entry)
    copied["age_s"] = max(0.0, _now() - float(copied.get("received_unix", _now())))
    return copied


def _stamp_from_ns(stamp_ns: Any) -> dict[str, int]:
    try:
        ns = int(stamp_ns)
    except (TypeError, ValueError):
        ns = time.time_ns()
    return {"sec": ns // 1_000_000_000, "nanosec": ns % 1_000_000_000}


class FastCommandPort:
    def __init__(
        self,
        enabled: bool,
        host: str,
        pull_port: int,
        rep_port: int,
        timeout_ms: int,
    ):
        self.enabled = enabled
        self.host = host
        self.pull_port = pull_port
        self.rep_port = rep_port
        self.timeout_ms = timeout_ms
        self.lock = threading.Lock()
        self.seq = 0
        self.last_error = ""
        self._ctx = zmq.Context.instance() if enabled and zmq is not None else None
        self._push = None
        self._cached_health: dict[str, Any] = {}
        self._last_health_at = 0.0
        self._last_command: dict[str, Any] | None = None

    @property
    def available(self) -> bool:
        return self.enabled and self._ctx is not None and msgpack is not None and zmq is not None

    def snapshot(self) -> dict[str, Any]:
        if not self.enabled:
            return {"enabled": False, "ok": False}
        if not self.available:
            return {"enabled": True, "ok": False, "error": str(FAST_ZMQ_IMPORT_ERROR)}
        now = time.monotonic()
        if now - self._last_health_at < 1.0 and self._cached_health:
            return dict(self._cached_health)
        health = self._rpc("health")
        self._cached_health = health
        self._last_health_at = now
        return dict(health)

    def publish_cmd_vel(self, command: dict[str, float]) -> dict[str, Any]:
        if not self.available:
            return {"ok": False, "error": "fast_zmq unavailable", "transport": "fast_zmq"}
        with self.lock:
            sock = self._ensure_push()
            payload = {
                "schema": "xlerobot_v1.1",
                "source_id": "robot_webview.fast_zmq",
                "seq": self.seq,
                "stamp_ns": time.time_ns(),
                "frame": "body",
                "base_cmd_vel": [command["x"], command["y"], command["z"]],
            }
            self.seq += 1
            try:
                sock.send(msgpack.packb(payload, use_bin_type=True), flags=zmq.NOBLOCK)
                self.last_error = ""
                self._last_command = self._command_entry(command)
                return {"ok": True, "transport": "fast_zmq"}
            except Exception as exc:
                self.last_error = str(exc)
                return {"ok": False, "error": self.last_error, "transport": "fast_zmq"}

    def latest_command(self) -> dict[str, Any] | None:
        with self.lock:
            return dict(self._last_command) if self._last_command is not None else None

    def close(self) -> None:
        if self._push is not None:
            try:
                self._push.close(0)
            except Exception:
                pass
            self._push = None

    def _ensure_push(self):
        if self._push is not None:
            return self._push
        assert self._ctx is not None and zmq is not None
        sock = self._ctx.socket(zmq.PUSH)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SNDHWM, 1)
        sock.setsockopt(zmq.SNDTIMEO, 0)
        try:
            sock.setsockopt(zmq.CONFLATE, 1)
        except zmq.ZMQError:
            pass
        sock.connect(f"tcp://{self.host}:{self.pull_port}")
        self._push = sock
        return sock

    @staticmethod
    def _command_entry(command: dict[str, float]) -> dict[str, Any]:
        return {
            "topic": "fast:cmd_vel",
            "msg": {
                "linear": {"x": command["x"], "y": command["y"], "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": command["z"]},
            },
            "received_unix": _now(),
        }

    def _rpc(self, op: str) -> dict[str, Any]:
        if not self.available:
            return {"enabled": self.enabled, "ok": False, "error": str(FAST_ZMQ_IMPORT_ERROR)}
        assert self._ctx is not None and zmq is not None and msgpack is not None
        sock = self._ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        try:
            sock.connect(f"tcp://{self.host}:{self.rep_port}")
            sock.send(msgpack.packb({"op": op}, use_bin_type=True))
            reply = msgpack.unpackb(sock.recv(), raw=False)
            if not isinstance(reply, dict):
                raise ValueError("RPC reply is not a dict")
            return {
                "enabled": True,
                "ok": bool(reply.get("ok")),
                "host": self.host,
                "pull_port": self.pull_port,
                "rep_port": self.rep_port,
                "health": reply.get("health", reply),
                "error": reply.get("error", ""),
            }
        except Exception as exc:
            self.last_error = str(exc)
            return {
                "enabled": True,
                "ok": False,
                "host": self.host,
                "pull_port": self.pull_port,
                "rep_port": self.rep_port,
                "error": self.last_error,
            }
        finally:
            sock.close(0)


class FastSensorSubscriber(threading.Thread):
    def __init__(self, enabled: bool, host: str, pub_port: int, robot_id: int):
        super().__init__(name="fast-zmq-sensor", daemon=True)
        self.enabled = enabled
        self.host = host
        self.pub_port = pub_port
        self.robot_id = robot_id
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_by_topic: dict[str, dict[str, Any]] = {}
        self.error = "" if self.available else str(FAST_ZMQ_IMPORT_ERROR or "")
        self.connected = False
        self._ctx = zmq.Context.instance() if self.available else None

    @property
    def available(self) -> bool:
        return self.enabled and msgpack is not None and zmq is not None

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            latest = dict(self.latest_by_topic)
            connected = self.connected
            error = self.error
        odom = _with_age(latest.get(f"odom.{self.robot_id}"))
        scan = _with_age(latest.get(f"scan.{self.robot_id}"))
        return {
            "enabled": self.enabled,
            "ok": connected,
            "error": error,
            "latest_odom": odom,
            "latest_scan": scan,
        }

    def run(self) -> None:
        if not self.available:
            return
        assert self._ctx is not None and zmq is not None and msgpack is not None
        while not self.stop_event.is_set():
            sock = self._ctx.socket(zmq.SUB)
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.RCVHWM, 64)
            for topic in (f"odom.{self.robot_id}", f"scan.{self.robot_id}"):
                sock.setsockopt(zmq.SUBSCRIBE, topic.encode("ascii"))
            try:
                sock.connect(f"tcp://{self.host}:{self.pub_port}")
                poller = zmq.Poller()
                poller.register(sock, zmq.POLLIN)
                with self.lock:
                    self.connected = True
                    self.error = ""
                while not self.stop_event.is_set():
                    events = dict(poller.poll(200))
                    if sock not in events:
                        continue
                    topic_raw, payload_raw = sock.recv_multipart(flags=zmq.NOBLOCK)
                    topic = topic_raw.decode("ascii", errors="replace")
                    payload = msgpack.unpackb(payload_raw, raw=False)
                    if not isinstance(payload, dict):
                        continue
                    entry = self._entry_from_payload(topic, payload)
                    if entry is not None:
                        with self.lock:
                            self.latest_by_topic[topic] = entry
            except Exception as exc:
                with self.lock:
                    self.connected = False
                    self.error = str(exc)
                time.sleep(0.5)
            finally:
                try:
                    sock.close(0)
                except Exception:
                    pass

    def _entry_from_payload(self, topic: str, payload: dict[str, Any]) -> dict[str, Any] | None:
        stamp_ns = payload.get("stamp_ns", time.time_ns())
        if topic.startswith("odom."):
            msg = payload.get("msg")
            if not isinstance(msg, dict):
                return None
            return {"topic": f"fast:{topic}", "msg": msg, "received_unix": _now()}
        if topic.startswith("scan."):
            raw_ranges = payload.get("ranges", b"")
            ranges: list[float] = []
            if isinstance(raw_ranges, bytes):
                values = array.array("f")
                try:
                    values.frombytes(raw_ranges)
                    ranges = list(values)
                except ValueError:
                    ranges = []
            elif isinstance(raw_ranges, list):
                ranges = [float(value) for value in raw_ranges if isinstance(value, (int, float))]
            msg = {
                "header": {"stamp": _stamp_from_ns(stamp_ns), "frame_id": payload.get("frame", "laser")},
                "angle_min": payload.get("angle_min", -math.pi),
                "angle_max": payload.get("angle_max", math.pi),
                "angle_increment": payload.get("angle_increment", 0.0),
                "range_min": payload.get("range_min", 0.0),
                "range_max": payload.get("range_max", 12.0),
                "ranges": ranges,
                "intensities": [],
            }
            return {"topic": f"fast:{topic}", "msg": msg, "received_unix": _now()}
        return None


class RobotViewHandler(BaseHTTPRequestHandler):
    monitor: RosbridgeMonitor

    def do_GET(self) -> None:
        if self.path in {"/", "/index.html"}:
            self._send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if self.path == "/api/status":
            self._send_json(self.monitor.snapshot())
            return
        if self.path == "/healthz":
            self._send_bytes(b"ok\n", "text/plain; charset=utf-8")
            return
        self.send_error(404)

    def do_POST(self) -> None:
        if self.path == "/api/stop":
            self._send_json(self.monitor.publish_cmd_vel(0.0, 0.0, 0.0))
            return
        if self.path == "/api/cmd_vel":
            try:
                length = min(int(self.headers.get("Content-Length", "0")), 4096)
                body = self.rfile.read(length) if length > 0 else b"{}"
                payload = json.loads(body.decode("utf-8"))
                result = self.monitor.publish_cmd_vel(
                    float(payload.get("x", 0.0)),
                    float(payload.get("y", 0.0)),
                    float(payload.get("z", 0.0)),
                )
                self._send_json(result)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)})
            return
        if self.path == "/api/move":
            try:
                length = min(int(self.headers.get("Content-Length", "0")), 4096)
                body = self.rfile.read(length) if length > 0 else b"{}"
                payload = json.loads(body.decode("utf-8"))
                result = self.monitor.publish_move(
                    str(payload.get("direction", "stop")),
                    float(payload.get("speed", 1.0)),
                )
                self._send_json(result)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)})
            return
        self.send_error(404)

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

    def _send_json(self, payload: dict[str, Any]) -> None:
        self._send_bytes(json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8"), "application/json")

    def _send_bytes(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    parser = argparse.ArgumentParser(description="Web dashboard and teleop control for the Indoory fast ZMQ robot I/O.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--rosbridge-url", default=os.environ.get("ROSBRIDGE_URL", "ws://127.0.0.1:9090"))
    parser.add_argument("--cmd-topics", default=os.environ.get("WEBVIEW_CMD_TOPICS"))
    parser.add_argument("--odom-topics", default=os.environ.get("WEBVIEW_ODOM_TOPICS"))
    parser.add_argument("--scan-topics", default=os.environ.get("WEBVIEW_SCAN_TOPICS"))
    parser.add_argument("--control-topic", default=os.environ.get("WEBVIEW_CONTROL_TOPIC", "/xlerobot/cmd_vel"))
    parser.add_argument(
        "--rosbridge-monitor",
        action=argparse.BooleanOptionalAction,
        default=_truthy(os.environ.get("WEBVIEW_ROSBRIDGE_MONITOR"), False),
    )
    parser.add_argument("--max-linear-x", type=float, default=float(os.environ.get("WEBVIEW_MAX_LINEAR_X", "0.30")))
    parser.add_argument("--max-linear-y", type=float, default=float(os.environ.get("WEBVIEW_MAX_LINEAR_Y", "0.30")))
    parser.add_argument("--max-angular-z", type=float, default=float(os.environ.get("WEBVIEW_MAX_ANGULAR_Z", "1.57")))
    parser.add_argument(
        "--fast-zmq-command",
        action=argparse.BooleanOptionalAction,
        default=_truthy(os.environ.get("WEBVIEW_FAST_ZMQ_COMMAND", os.environ.get("ENABLE_FAST_ZMQ")), True),
    )
    parser.add_argument("--fast-zmq-host", default=os.environ.get("WEBVIEW_FAST_ZMQ_HOST", "127.0.0.1"))
    parser.add_argument("--fast-zmq-pub-port", type=int, default=int(os.environ.get("WEBVIEW_FAST_ZMQ_PUB_PORT", os.environ.get("FAST_ZMQ_PUB_PORT", "8855"))))
    parser.add_argument("--fast-zmq-pull-port", type=int, default=int(os.environ.get("WEBVIEW_FAST_ZMQ_PULL_PORT", os.environ.get("FAST_ZMQ_PULL_PORT", "8856"))))
    parser.add_argument("--fast-zmq-rep-port", type=int, default=int(os.environ.get("WEBVIEW_FAST_ZMQ_REP_PORT", os.environ.get("FAST_ZMQ_REP_PORT", "8857"))))
    parser.add_argument("--fast-zmq-timeout-ms", type=int, default=int(os.environ.get("WEBVIEW_FAST_ZMQ_TIMEOUT_MS", "200")))
    parser.add_argument("--fast-zmq-robot-id", type=int, default=int(os.environ.get("WEBVIEW_FAST_ZMQ_ROBOT_ID", os.environ.get("FAST_ZMQ_ROBOT_ID", "0"))))
    parser.add_argument(
        "--fast-zmq-sensor",
        action=argparse.BooleanOptionalAction,
        default=_truthy(os.environ.get("WEBVIEW_FAST_ZMQ_SENSOR", os.environ.get("ENABLE_FAST_ZMQ")), True),
    )
    parser.add_argument("--reconnect-s", type=float, default=1.0)
    args = parser.parse_args()

    fast_command = FastCommandPort(
        args.fast_zmq_command,
        args.fast_zmq_host,
        args.fast_zmq_pull_port,
        args.fast_zmq_rep_port,
        args.fast_zmq_timeout_ms,
    )
    fast_sensor = FastSensorSubscriber(
        args.fast_zmq_sensor,
        args.fast_zmq_host,
        args.fast_zmq_pub_port,
        args.fast_zmq_robot_id,
    )
    fast_sensor.start()
    monitor = RosbridgeMonitor(
        args.rosbridge_url,
        _csv(args.cmd_topics, DEFAULT_CMD_TOPICS),
        _csv(args.odom_topics, DEFAULT_ODOM_TOPICS),
        _csv(args.scan_topics, DEFAULT_SCAN_TOPICS),
        args.control_topic,
        args.max_linear_x,
        args.max_linear_y,
        args.max_angular_z,
        args.reconnect_s,
        args.rosbridge_monitor,
        fast_command,
        fast_sensor,
    )
    if args.rosbridge_monitor:
        monitor.start()

    RobotViewHandler.monitor = monitor
    server = ThreadingHTTPServer((args.host, args.port), RobotViewHandler)

    def shutdown(_signum: int, _frame: Any) -> None:
        monitor.stop_event.set()
        fast_sensor.stop_event.set()
        fast_command.close()
        threading.Thread(target=server.shutdown, name="http-shutdown", daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    host = socket.gethostname()
    print(
        f"Robot webview listening on http://{args.host}:{args.port} ({host}); "
        f"fast_zmq_command={args.fast_zmq_command} "
        f"tcp://{args.fast_zmq_host}:{args.fast_zmq_pull_port}; "
        f"fast_zmq_sensor={args.fast_zmq_sensor} "
        f"tcp://{args.fast_zmq_host}:{args.fast_zmq_pub_port}; "
        f"rosbridge_monitor={args.rosbridge_monitor}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
