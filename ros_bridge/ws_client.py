#!/usr/bin/env python3
"""Small stdlib WebSocket client for rosbridge JSON messages."""

from __future__ import annotations

import base64
import hashlib
import json
import os
import select
import socket
import ssl
import struct
from typing import Any
from urllib.parse import urlparse


WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class WebSocketClosed(RuntimeError):
    pass


class SimpleWebSocket:
    def __init__(self, url: str, connect_timeout: float = 5.0, read_timeout: float = 5.0):
        self.url = url
        self.connect_timeout = connect_timeout
        self.read_timeout = read_timeout
        self.sock: socket.socket | ssl.SSLSocket | None = None

    def connect(self) -> None:
        parsed = urlparse(self.url)
        if parsed.scheme not in {"ws", "wss"}:
            raise ValueError(f"Unsupported rosbridge URL scheme: {parsed.scheme}")

        host = parsed.hostname
        if not host:
            raise ValueError(f"Missing host in rosbridge URL: {self.url}")

        port = parsed.port or (443 if parsed.scheme == "wss" else 80)
        path = parsed.path or "/"
        if parsed.query:
            path += f"?{parsed.query}"

        raw_sock = socket.create_connection((host, port), timeout=self.connect_timeout)
        if parsed.scheme == "wss":
            context = ssl.create_default_context()
            self.sock = context.wrap_socket(raw_sock, server_hostname=host)
        else:
            self.sock = raw_sock
        self.sock.settimeout(self.read_timeout)

        key = base64.b64encode(os.urandom(16)).decode("ascii")
        host_header = host if parsed.port is None else f"{host}:{port}"
        request = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host_header}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        )
        self.sock.sendall(request.encode("ascii"))

        response = self._read_http_headers()
        status_line = response.split("\r\n", 1)[0]
        if " 101 " not in status_line:
            raise ConnectionError(f"WebSocket handshake failed: {status_line}")

        headers: dict[str, str] = {}
        for line in response.split("\r\n")[1:]:
            if ":" in line:
                name, value = line.split(":", 1)
                headers[name.strip().lower()] = value.strip()

        expected_accept = base64.b64encode(
            hashlib.sha1((key + WEBSOCKET_GUID).encode("ascii")).digest()
        ).decode("ascii")
        if headers.get("sec-websocket-accept") != expected_accept:
            raise ConnectionError("WebSocket handshake failed: invalid Sec-WebSocket-Accept")

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self._send_frame(0x8, b"")
        except OSError:
            pass
        try:
            self.sock.close()
        finally:
            self.sock = None

    def send_json(self, payload: dict[str, Any]) -> None:
        data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._send_frame(0x1, data)

    def recv_json(self, timeout: float) -> dict[str, Any] | None:
        text = self.recv_text(timeout)
        if text is None:
            return None
        return json.loads(text)

    def recv_text(self, timeout: float) -> str | None:
        if self.sock is None:
            raise WebSocketClosed("WebSocket is not connected")

        ready, _, _ = select.select([self.sock], [], [], timeout)
        if not ready:
            return None

        opcode, fin, payload = self._recv_frame()
        if opcode == 0x8:
            raise WebSocketClosed("Remote WebSocket closed")
        if opcode == 0x9:
            self._send_frame(0xA, payload)
            return None
        if opcode == 0xA:
            return None
        if opcode != 0x1:
            return None

        chunks = [payload]
        while not fin:
            opcode, fin, payload = self._recv_frame()
            if opcode == 0x8:
                raise WebSocketClosed("Remote WebSocket closed")
            if opcode != 0x0:
                raise WebSocketClosed("Unexpected fragmented WebSocket frame")
            chunks.append(payload)
        return b"".join(chunks).decode("utf-8")

    def _read_http_headers(self) -> str:
        chunks: list[bytes] = []
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = self._recv_exact(1)
            chunks.append(chunk)
            data = b"".join(chunks)
            if len(data) > 16384:
                raise ConnectionError("WebSocket handshake response is too large")
        return data.decode("iso-8859-1")

    def _recv_exact(self, size: int) -> bytes:
        if self.sock is None:
            raise WebSocketClosed("WebSocket is not connected")
        data = bytearray()
        while len(data) < size:
            chunk = self.sock.recv(size - len(data))
            if not chunk:
                raise WebSocketClosed("Remote WebSocket closed")
            data.extend(chunk)
        return bytes(data)

    def _send_frame(self, opcode: int, payload: bytes) -> None:
        if self.sock is None:
            raise WebSocketClosed("WebSocket is not connected")

        first = 0x80 | opcode
        length = len(payload)
        if length < 126:
            header = struct.pack("!BB", first, 0x80 | length)
        elif length <= 0xFFFF:
            header = struct.pack("!BBH", first, 0x80 | 126, length)
        else:
            header = struct.pack("!BBQ", first, 0x80 | 127, length)

        mask = os.urandom(4)
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        self.sock.sendall(header + mask + masked)

    def _recv_frame(self) -> tuple[int, bool, bytes]:
        first, second = self._recv_exact(2)
        fin = bool(first & 0x80)
        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        if length == 126:
            length = struct.unpack("!H", self._recv_exact(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self._recv_exact(8))[0]

        mask = self._recv_exact(4) if masked else b""
        payload = self._recv_exact(length) if length else b""
        if masked:
            payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        return opcode, fin, payload
