#!/usr/bin/env python3
"""Tiny web UI for validating XLeRobot head pan/tilt motors."""

from __future__ import annotations

import argparse
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
except Exception as exc:  # pragma: no cover - startup path
    if os.environ.get("HEAD_MOTOR_WEBTEST_REEXEC") != "1":
        candidates = [
            Path(os.path.expanduser(os.environ.get("XLE_ROBOT_VENV", "~/xlerobot-io-venv"))) / "bin" / "python3",
            Path(os.path.expanduser("~/.miniforge3/envs/lerobot/bin/python3")),
        ]
        for python in candidates:
            if python.exists() and python.resolve() != Path(sys.executable).resolve():
                env = os.environ.copy()
                env["HEAD_MOTOR_WEBTEST_REEXEC"] = "1"
                os.execve(str(python), [str(python), *sys.argv], env)
    print(f"[err] pyzmq and msgpack are required: {exc}", file=sys.stderr)
    raise SystemExit(1)


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Head Motor Test</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #101316;
      --panel: #181f24;
      --panel2: #20282f;
      --line: #33414b;
      --text: #ecf3f5;
      --muted: #94a5ae;
      --cyan: #5ed6e8;
      --green: #4bd18a;
      --red: #ff6b6b;
      --yellow: #f4c95d;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main {
      width: min(940px, calc(100vw - 28px));
      margin: 0 auto;
      padding: 20px 0;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 14px;
      margin-bottom: 14px;
    }
    h1 {
      margin: 0;
      font-size: 22px;
      line-height: 28px;
      font-weight: 760;
    }
    .sub {
      margin-top: 4px;
      color: var(--muted);
      font-size: 12px;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      overflow-wrap: anywhere;
    }
    .badge {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: var(--panel);
      color: var(--yellow);
      padding: 6px 10px;
      min-height: 32px;
      white-space: nowrap;
      font-size: 13px;
      font-weight: 700;
    }
    .badge.good { color: var(--green); }
    .badge.bad { color: var(--red); }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }
    section {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      min-width: 0;
    }
    h2 {
      margin: 0;
      padding: 12px 13px;
      border-bottom: 1px solid var(--line);
      font-size: 14px;
      line-height: 20px;
    }
    .body {
      padding: 13px;
      display: grid;
      gap: 12px;
    }
    .metrics {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .metric {
      min-width: 0;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: var(--panel2);
      padding: 10px;
    }
    .label {
      color: var(--muted);
      font-size: 12px;
      line-height: 16px;
    }
    .value {
      margin-top: 6px;
      font-variant-numeric: tabular-nums;
      font-size: 24px;
      line-height: 30px;
      font-weight: 760;
      overflow-wrap: anywhere;
    }
    .controls {
      display: grid;
      grid-template-columns: repeat(3, minmax(72px, 1fr));
      gap: 8px;
    }
    button {
      border: 1px solid var(--line);
      background: var(--panel2);
      color: var(--text);
      min-height: 42px;
      border-radius: 7px;
      font-size: 14px;
      font-weight: 760;
      cursor: pointer;
    }
	    button:active {
	      border-color: var(--cyan);
	      background: #18323a;
	    }
	    button.key-active {
	      border-color: var(--cyan);
	      background: #18323a;
	      box-shadow: inset 0 0 0 1px rgba(94, 214, 232, 0.35);
	    }
	    button.primary {
      border-color: #3b8d9b;
      background: #15313a;
    }
    button.danger {
      border-color: #a84855;
      background: #742630;
    }
    .span3 { grid-column: span 3; }
    .sliders {
      display: grid;
      gap: 10px;
    }
    .slider-row {
      display: grid;
      grid-template-columns: 52px minmax(120px, 1fr) 72px;
      align-items: center;
      gap: 10px;
      color: var(--muted);
      font-size: 13px;
    }
    input[type="range"] {
      width: 100%;
      accent-color: var(--cyan);
    }
    .log {
      min-height: 20px;
      color: var(--muted);
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    @media (max-width: 760px) {
      header { align-items: flex-start; flex-direction: column; }
      .grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>Head Motor Test</h1>
      <div id="endpoint" class="sub">fast_zmq: waiting</div>
    </div>
    <div id="badge" class="badge">Connecting</div>
  </header>
  <div class="grid">
    <section>
      <h2>Current</h2>
      <div class="body">
        <div class="metrics">
          <div class="metric"><div class="label">pan</div><div id="panNow" class="value">-</div></div>
          <div class="metric"><div class="label">tilt</div><div id="tiltNow" class="value">-</div></div>
          <div class="metric"><div class="label">joint age</div><div id="ageNow" class="value">-</div></div>
          <div class="metric"><div class="label">commands</div><div id="cmdNow" class="value">-</div></div>
        </div>
        <div id="stateLog" class="log">waiting</div>
      </div>
    </section>
    <section>
      <h2>Nudge</h2>
      <div class="body">
	        <div class="controls">
	          <button data-pan="-5" data-key="j" title="J">Pan -5</button>
	          <button data-tilt="5" data-key="i" title="I">Tilt +5</button>
	          <button data-pan="5" data-key="l" title="L">Pan +5</button>
	          <button data-pan="-1">Pan -1</button>
	          <button data-tilt="-5" data-key="k" title="K">Tilt -5</button>
	          <button data-pan="1">Pan +1</button>
          <button class="danger span3" id="centerBtn">Center Calibration</button>
        </div>
        <div id="nudgeLog" class="log">idle</div>
      </div>
    </section>
    <section>
      <h2>Absolute Raw</h2>
      <div class="body">
        <div class="sliders">
          <label class="slider-row">Pan <input id="panSlider" type="range" min="0" max="4095" step="1" value="2048"><span id="panValue">2048</span></label>
          <label class="slider-row">Tilt <input id="tiltSlider" type="range" min="0" max="4095" step="1" value="2048"><span id="tiltValue">2048</span></label>
        </div>
        <div class="controls">
          <button id="syncBtn">Sync</button>
          <button id="sendRawBtn" class="primary span3">Send Absolute</button>
        </div>
        <div id="rangeLog" class="log">range: waiting</div>
        <div id="rawLog" class="log">idle</div>
      </div>
    </section>
    <section>
      <h2>RPC</h2>
      <div class="body">
        <div class="controls">
          <button id="debugBtn">Head Debug</button>
          <button id="stopBtn">Stop Base</button>
          <button id="refreshBtn">Refresh</button>
        </div>
        <div id="rpcLog" class="log">idle</div>
      </div>
    </section>
  </div>
</main>
<script>
	const q = id => document.getElementById(id);
	let latest = null;
	const KEY_NUDGES = {
	  i: {pan: 0, tilt: 5},
	  k: {pan: 0, tilt: -5},
	  j: {pan: -5, tilt: 0},
	  l: {pan: 5, tilt: 0},
	};
	const KEY_REPEAT_MS = 90;
	let keyboardBusy = false;
	let lastKeyboardAt = 0;
function fmt(v, d=0) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return '-';
  return Number(v).toFixed(d);
}
function badge(ok, text) {
  q('badge').textContent = text;
  q('badge').className = 'badge ' + (ok ? 'good' : 'bad');
}
function setLog(id, obj) {
  q(id).textContent = typeof obj === 'string' ? obj : JSON.stringify(obj);
}
async function api(path, body = null) {
  const opts = body ? {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)} : {cache: 'no-store'};
  const res = await fetch(path, opts);
  const data = await res.json();
  if (!data.ok) throw new Error(data.error || 'request failed');
  return data;
}
function syncSliders() {
  if (!latest || !latest.head) return;
  const pan = latest.head.pan_tick;
  const tilt = latest.head.tilt_tick;
  if (Number.isFinite(pan)) q('panSlider').value = Math.round(pan);
  if (Number.isFinite(tilt)) q('tiltSlider').value = Math.round(tilt);
  updateSliderLabels();
}
	function applyLimits(data) {
	  const limits = data.limits || {};
	  const pan = limits.head_pan || {};
  const tilt = limits.head_tilt || {};
  [['panSlider', pan], ['tiltSlider', tilt]].forEach(([id, lim]) => {
    const el = q(id);
    if (Number.isFinite(lim.min)) el.min = String(Math.round(lim.min));
    if (Number.isFinite(lim.max)) el.max = String(Math.round(lim.max));
    const value = Number(el.value);
    if (Number.isFinite(lim.min) && value < lim.min) el.value = String(Math.round(lim.min));
    if (Number.isFinite(lim.max) && value > lim.max) el.value = String(Math.round(lim.max));
  });
  const panText = Number.isFinite(pan.min) ? `pan ${Math.round(pan.min)}..${Math.round(pan.max)}` : 'pan -';
  const tiltText = Number.isFinite(tilt.min) ? `tilt ${Math.round(tilt.min)}..${Math.round(tilt.max)}` : 'tilt -';
	  q('rangeLog').textContent = `calibration ${panText}, ${tiltText}`;
	  updateSliderLabels();
	}
	function setKeyActive(key, active) {
	  const btn = document.querySelector(`button[data-key="${key}"]`);
	  if (btn) btn.classList.toggle('key-active', active);
	}
	async function sendNudge(panDeg, tiltDeg, source = 'button') {
	  try {
	    const result = await api('/api/nudge', {pan_deg: panDeg, tilt_deg: tiltDeg});
	    setLog('nudgeLog', {source, ...result});
	  } catch (err) {
	    setLog('nudgeLog', `${source}: ${String(err)}`);
	  }
	}
	function updateSliderLabels() {
  const pan = q('panSlider');
  const tilt = q('tiltSlider');
  q('panValue').textContent = `${pan.value} (${pan.min}..${pan.max})`;
  q('tiltValue').textContent = `${tilt.value} (${tilt.min}..${tilt.max})`;
}
async function poll() {
  try {
    const data = await api('/api/status');
    latest = data;
    applyLimits(data);
    q('endpoint').textContent = data.endpoint;
    badge(data.ready, data.ready ? 'Ready' : 'Not Ready');
    q('panNow').textContent = fmt(data.head && data.head.pan_tick, 0);
    q('tiltNow').textContent = fmt(data.head && data.head.tilt_tick, 0);
    q('ageNow').textContent = data.head ? fmt(data.head.age_ms, 0) + ' ms' : '-';
    q('cmdNow').textContent = String((data.health && data.health.accepted_commands) || 0);
    setLog('stateLog', data.message || 'ok');
  } catch (err) {
    badge(false, 'Disconnected');
    setLog('stateLog', String(err));
  } finally {
    setTimeout(poll, 250);
  }
}
	document.querySelectorAll('button[data-pan],button[data-tilt]').forEach(btn => {
	  btn.addEventListener('click', async () => {
	    const panDeg = Number(btn.dataset.pan || 0);
	    const tiltDeg = Number(btn.dataset.tilt || 0);
	    await sendNudge(panDeg, tiltDeg);
	  });
	});
	document.addEventListener('keydown', event => {
	  if (event.defaultPrevented || event.altKey || event.ctrlKey || event.metaKey) return;
	  const key = String(event.key || '').toLowerCase();
	  const nudge = KEY_NUDGES[key];
	  if (!nudge) return;
	  const tag = event.target && event.target.tagName;
	  if (tag === 'TEXTAREA' || tag === 'SELECT') return;
	  if (tag === 'INPUT' && event.target.type !== 'range') return;
	  event.preventDefault();
	  const now = performance.now();
	  if (keyboardBusy || (event.repeat && now - lastKeyboardAt < KEY_REPEAT_MS)) return;
	  keyboardBusy = true;
	  lastKeyboardAt = now;
	  setKeyActive(key, true);
	  sendNudge(nudge.pan, nudge.tilt, `key ${key.toUpperCase()}`)
	    .finally(() => { keyboardBusy = false; });
	});
	document.addEventListener('keyup', event => {
	  const key = String(event.key || '').toLowerCase();
	  if (KEY_NUDGES[key]) setKeyActive(key, false);
	});
	q('centerBtn').addEventListener('click', async () => {
  const limits = (latest && latest.limits) || {};
  const pan = limits.head_pan || {};
  const tilt = limits.head_tilt || {};
  try {
    setLog('nudgeLog', await api('/api/raw', {
      pan_tick: Number.isFinite(pan.center) ? pan.center : 2048,
      tilt_tick: Number.isFinite(tilt.center) ? tilt.center : 2048
    }));
  }
  catch (err) { setLog('nudgeLog', String(err)); }
});
q('syncBtn').addEventListener('click', syncSliders);
q('sendRawBtn').addEventListener('click', async () => {
  try {
    setLog('rawLog', await api('/api/raw', {pan_tick: Number(q('panSlider').value), tilt_tick: Number(q('tiltSlider').value)}));
  } catch (err) {
    setLog('rawLog', String(err));
  }
});
q('debugBtn').addEventListener('click', async () => {
  try { setLog('rpcLog', await api('/api/debug')); }
  catch (err) { setLog('rpcLog', String(err)); }
});
q('stopBtn').addEventListener('click', async () => {
  try { setLog('rpcLog', await api('/api/stop')); }
  catch (err) { setLog('rpcLog', String(err)); }
});
q('refreshBtn').addEventListener('click', poll);
q('panSlider').addEventListener('input', updateSliderLabels);
q('tiltSlider').addEventListener('input', updateSliderLabels);
updateSliderLabels();
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


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


class HeadMotorBridge:
    def __init__(
        self,
        fast_host: str,
        pub_port: int,
        pull_port: int,
        rep_port: int,
        robot_id: int,
        timeout_ms: int,
        max_state_age_s: float,
    ):
        self.fast_host = fast_host
        self.pub_port = pub_port
        self.pull_port = pull_port
        self.rep_port = rep_port
        self.robot_id = robot_id
        self.timeout_ms = timeout_ms
        self.max_state_age_s = max_state_age_s
        self.endpoint = f"fast_zmq {fast_host}:{pub_port}/{pull_port}/{rep_port}"
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_joint_state: dict[str, Any] | None = None
        self.latest_received_at = 0.0
        self.last_error = ""
        self.last_command: dict[str, Any] | None = None
        self._calibration_cache: dict[str, Any] = {}
        self._calibration_cache_at = 0.0
        self.seq = 0
        self.ctx = zmq.Context.instance()
        self.push = self.ctx.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.setsockopt(zmq.SNDHWM, 1)
        self.push.setsockopt(zmq.SNDTIMEO, 0)
        try:
            self.push.setsockopt(zmq.CONFLATE, 1)
        except zmq.ZMQError:
            pass
        self.push.connect(f"tcp://{fast_host}:{pull_port}")
        self.thread = threading.Thread(target=self._joint_state_loop, name="head-joint-state", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        try:
            self.push.close(0)
        except Exception:
            pass

    def status(self) -> dict[str, Any]:
        health = self.rpc("health")
        head = self.head_state()
        calibration = self.calibration()
        health_body = health.get("health") if health.get("ok") else None
        ready = bool(
            health.get("ok")
            and isinstance(health_body, dict)
            and health_body.get("base_attached")
            and head is not None
            and head["age_s"] <= self.max_state_age_s
        )
        message = "ok" if ready else self._not_ready_message(health, head)
        return {
            "ok": True,
            "ready": ready,
            "message": message,
            "endpoint": self.endpoint,
            "health": health_body,
            "head": head,
            "calibration": calibration,
            "limits": calibration.get("limits", {}),
            "last_error": self.last_error,
            "last_command": self.last_command,
        }

    def head_state(self) -> dict[str, Any] | None:
        with self.lock:
            msg = dict(self.latest_joint_state or {})
            received_at = self.latest_received_at
        names = msg.get("names") or msg.get("name")
        positions = msg.get("position")
        if not isinstance(names, list) or not isinstance(positions, list):
            return None
        by_name = {
            str(name): positions[idx]
            for idx, name in enumerate(names)
            if idx < len(positions)
        }
        pan = by_name.get("head_pan")
        tilt = by_name.get("head_tilt")
        age_s = max(0.0, time.time() - received_at) if received_at else math.inf
        return {
            "pan_tick": pan,
            "tilt_tick": tilt,
            "age_s": age_s,
            "age_ms": age_s * 1000.0 if math.isfinite(age_s) else None,
            "topic": f"joint_states.{self.robot_id}",
        }

    def nudge(self, pan_deg: float, tilt_deg: float) -> dict[str, Any]:
        status = self.status()
        if not status["ready"]:
            return {"ok": False, "error": status["message"], "status": status}
        pan_rad = math.radians(float(pan_deg))
        tilt_rad = math.radians(float(tilt_deg))
        public: dict[str, Any] = {"pan_deg": pan_deg, "tilt_deg": tilt_deg}
        if abs(pan_rad) <= 1e-12 and abs(tilt_rad) <= 1e-12:
            return {"ok": True, "message": "zero nudge", **public}
        payload = {
            "schema": "xlerobot_v1.1",
            "source_id": "head_motor_webtest.nudge",
            "seq": self._next_seq(),
            "stamp_ns": time.time_ns(),
            "frame": "body",
            "head_joint_relative_target": {
                "head_pan": pan_rad,
                "head_tilt": tilt_rad,
            },
        }
        return self._send_command(payload, public)

    def raw(self, pan_tick: Any, tilt_tick: Any) -> dict[str, Any]:
        health = self.rpc("health")
        if not health.get("ok"):
            return {"ok": False, "error": health.get("error", "fast RPC unavailable")}
        health_body = health.get("health")
        if not isinstance(health_body, dict) or not health_body.get("base_attached"):
            return {"ok": False, "error": "robot base is not attached"}
        calibration = self.calibration()
        targets: list[float | None] = [None] * 14
        if pan_tick is not None:
            lo, hi = self._axis_limits(calibration, "head_pan")
            targets[12] = clamp(float(pan_tick), lo, hi)
        if tilt_tick is not None:
            lo, hi = self._axis_limits(calibration, "head_tilt")
            targets[13] = clamp(float(tilt_tick), lo, hi)
        if targets[12] is None and targets[13] is None:
            return {"ok": False, "error": "provide pan_tick and/or tilt_tick"}
        payload = {
            "schema": "xlerobot_v1.1",
            "source_id": "head_motor_webtest.raw",
            "seq": self._next_seq(),
            "stamp_ns": time.time_ns(),
            "frame": "body",
            "joint_targets_sparse": targets,
        }
        return self._send_command(payload, {"pan_tick": targets[12], "tilt_tick": targets[13]})

    def calibration(self, *, refresh: bool = False) -> dict[str, Any]:
        now = time.monotonic()
        if (
            not refresh
            and self._calibration_cache
            and now - self._calibration_cache_at < 2.0
        ):
            return dict(self._calibration_cache)
        reply = self.rpc("head_debug")
        parsed = self._parse_calibration(reply)
        self._calibration_cache = parsed
        self._calibration_cache_at = now
        return dict(parsed)

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

    def _send_command(self, payload: dict[str, Any], public: dict[str, Any]) -> dict[str, Any]:
        try:
            self.push.send(pack(payload), flags=zmq.NOBLOCK)
            self.last_error = ""
            self.last_command = {"stamp": time.time(), **public}
            return {"ok": True, **public}
        except Exception as exc:
            self.last_error = str(exc)
            return {"ok": False, "error": str(exc), **public}

    def _next_seq(self) -> int:
        with self.lock:
            self.seq += 1
            return self.seq

    def _axis_limits(self, calibration: dict[str, Any], axis: str) -> tuple[float, float]:
        limits = calibration.get("limits") if isinstance(calibration, dict) else None
        axis_limits = limits.get(axis) if isinstance(limits, dict) else None
        lo = finite_number(axis_limits.get("min")) if isinstance(axis_limits, dict) else None
        hi = finite_number(axis_limits.get("max")) if isinstance(axis_limits, dict) else None
        if lo is None or hi is None or hi <= lo:
            return 0.0, 4095.0
        return lo, hi

    def _parse_calibration(self, reply: dict[str, Any]) -> dict[str, Any]:
        if not reply.get("ok"):
            return {"ok": False, "error": reply.get("error", "head_debug unavailable"), "limits": {}}
        calibration = reply.get("calibration")
        head = calibration.get("head") if isinstance(calibration, dict) else None
        limits: dict[str, dict[str, Any]] = {}
        if isinstance(head, dict):
            for axis in ("head_pan", "head_tilt"):
                raw = head.get(axis)
                if not isinstance(raw, dict):
                    continue
                lo = finite_number(raw.get("range_min"))
                hi = finite_number(raw.get("range_max"))
                if lo is None or hi is None or hi <= lo:
                    continue
                limits[axis] = {
                    "min": lo,
                    "max": hi,
                    "center": round((lo + hi) * 0.5),
                    "width": hi - lo,
                    "xlerobot_motor": raw.get("xlerobot_motor"),
                    "id": raw.get("id"),
                    "calibration_loaded": bool(raw.get("calibration_loaded")),
                }
        return {
            "ok": True,
            "path": calibration.get("calibration_path") if isinstance(calibration, dict) else None,
            "calibration_limits": bool(calibration.get("calibration_limits")) if isinstance(calibration, dict) else False,
            "limits": limits,
        }

    def _not_ready_message(self, health: dict[str, Any], head: dict[str, Any] | None) -> str:
        if not health.get("ok"):
            return f"fast RPC unavailable: {health.get('error', 'no reply')}"
        health_body = health.get("health")
        if not isinstance(health_body, dict) or not health_body.get("base_attached"):
            return "robot base is not attached"
        if head is None:
            return "waiting for joint_states"
        if head["age_s"] > self.max_state_age_s:
            return "joint_states are stale"
        return "not ready"

    def _joint_state_loop(self) -> None:
        topic = f"joint_states.{self.robot_id}"
        while not self.stop_event.is_set():
            sock = self.ctx.socket(zmq.SUB)
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.RCVHWM, 16)
            sock.setsockopt(zmq.SUBSCRIBE, topic.encode("ascii"))
            try:
                sock.connect(f"tcp://{self.fast_host}:{self.pub_port}")
                poller = zmq.Poller()
                poller.register(sock, zmq.POLLIN)
                while not self.stop_event.is_set():
                    events = dict(poller.poll(200))
                    if sock not in events:
                        continue
                    _topic_raw, payload_raw = sock.recv_multipart(flags=zmq.NOBLOCK)
                    payload = unpack(payload_raw)
                    with self.lock:
                        self.latest_joint_state = payload
                        self.latest_received_at = time.time()
            except Exception as exc:
                self.last_error = str(exc)
                time.sleep(0.5)
            finally:
                try:
                    sock.close(0)
                except Exception:
                    pass


class HeadMotorHandler(BaseHTTPRequestHandler):
    bridge: HeadMotorBridge

    def do_GET(self) -> None:
        if self.path in {"/", "/index.html"}:
            self._send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if self.path == "/api/status":
            self._send_json(self.bridge.status())
            return
        if self.path == "/healthz":
            self._send_bytes(b"ok\n", "text/plain; charset=utf-8")
            return
        self.send_error(404)

    def do_POST(self) -> None:
        try:
            payload = self._read_json()
            if self.path == "/api/nudge":
                self._send_json(self.bridge.nudge(
                    float(payload.get("pan_deg", 0.0)),
                    float(payload.get("tilt_deg", 0.0)),
                ))
                return
            if self.path == "/api/raw":
                self._send_json(self.bridge.raw(
                    payload.get("pan_tick"),
                    payload.get("tilt_tick"),
                ))
                return
            if self.path == "/api/debug":
                self._send_json(self.bridge.rpc("head_debug"))
                return
            if self.path == "/api/stop":
                self._send_json(self.bridge.rpc("stop"))
                return
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)})
            return
        self.send_error(404)

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

    def _read_json(self) -> dict[str, Any]:
        length = min(int(self.headers.get("Content-Length", "0")), 4096)
        body = self.rfile.read(length) if length > 0 else b"{}"
        payload = json.loads(body.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def _send_json(self, payload: dict[str, Any]) -> None:
        self._send_bytes(
            json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8"),
            "application/json",
        )

    def _send_bytes(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Head pan/tilt motor validation webserver")
    parser.add_argument("--host", default=os.environ.get("HEAD_WEB_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("HEAD_WEB_PORT", "8780")))
    parser.add_argument("--fast-zmq-host", default=os.environ.get("HEAD_WEB_FAST_ZMQ_HOST", "127.0.0.1"))
    parser.add_argument("--fast-zmq-pub-port", type=int, default=int(os.environ.get("FAST_ZMQ_PUB_PORT", "8855")))
    parser.add_argument("--fast-zmq-pull-port", type=int, default=int(os.environ.get("FAST_ZMQ_PULL_PORT", "8856")))
    parser.add_argument("--fast-zmq-rep-port", type=int, default=int(os.environ.get("FAST_ZMQ_REP_PORT", "8857")))
    parser.add_argument("--fast-zmq-robot-id", type=int, default=int(os.environ.get("FAST_ZMQ_ROBOT_ID", "0")))
    parser.add_argument("--timeout-ms", type=int, default=int(os.environ.get("HEAD_WEB_TIMEOUT_MS", "250")))
    parser.add_argument("--max-state-age-s", type=float, default=float(os.environ.get("HEAD_WEB_MAX_STATE_AGE_S", "2.0")))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bridge = HeadMotorBridge(
        args.fast_zmq_host,
        args.fast_zmq_pub_port,
        args.fast_zmq_pull_port,
        args.fast_zmq_rep_port,
        args.fast_zmq_robot_id,
        args.timeout_ms,
        args.max_state_age_s,
    )
    bridge.start()
    HeadMotorHandler.bridge = bridge
    server = ThreadingHTTPServer((args.host, args.port), HeadMotorHandler)

    def shutdown(_signum: int, _frame: Any) -> None:
        bridge.close()
        threading.Thread(target=server.shutdown, name="http-shutdown", daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    host = socket.gethostname()
    print(
        f"Head motor webtest listening on http://{args.host}:{args.port} ({host}); "
        f"fast_zmq=tcp://{args.fast_zmq_host}:{args.fast_zmq_pub_port}/"
        f"{args.fast_zmq_pull_port}/{args.fast_zmq_rep_port}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
