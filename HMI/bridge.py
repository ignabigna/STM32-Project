#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Puente de Telemetría y Control en Tiempo Real (Fase B2)
=======================================================
Arquitectura:
  STM32F103 (RAM) ──SWD── ST-Link ── OpenOCD (:6666 TCL) ──► bridge.py ──WebSocket (:8765)──► HMI Web

Características:
  - 100% biblioteca estándar de Python 3 (sin dependencias externas; no requiere pip install).
  - Lector ELF32 integrado (.symtab / .strtab) que extrae automáticamente las direcciones
    de memoria RAM de `BP_A_Master_F.elf` y detecta recompilaciones en caliente.
  - Cliente asíncrono OpenOCD TCL RPC (puerto 6666) compatible con OpenOCD 0.10, 0.11 y 0.12.
    Realiza lecturas agrupadas en bloque de SRAM sin pausar (halt) el microcontrolador.
  - Servidor WebSocket RFC 6455 + servidor HTTP estático integrado en el puerto 8765.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import json
import mimetypes
import os
import struct
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

# Terminador del protocolo TCL RPC de OpenOCD (ASCII 0x1A / Ctrl-Z)
TCL_EOF = b"\x1a"
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

# Conversión mecánica: 2400 pulsos de encoder = 1 vuelta del engranaje conducido = 360°
TICKS_PER_GEAR_REV = 2400

# Catálogo de variables del firmware BP_A_Master_F
VAR_SPECS: Dict[str, Dict[str, Any]] = {
    "live_position": {
        "type": "int32_t",
        "size": 4,
        "fmt": "<i",
        "writable": False,
        "desc": "Posición real del engranaje en grados (360° = 1 vuelta = 2400 pulsos)",
    },
    "live_position_ticks": {
        "type": "int32_t",
        "size": 4,
        "fmt": "<i",
        "writable": False,
        "optional": True,
        "desc": "Posición cruda del motor en pulsos de encoder recibidos por CAN",
    },
    "live_motor_state": {
        "type": "uint8_t",
        "size": 1,
        "fmt": "<B",
        "writable": False,
        "desc": "0=IDLE 1=SCANNING 2=HOMED 3=HOMING 4=ERROR",
    },
    "telemetry_counter": {
        "type": "uint32_t",
        "size": 4,
        "fmt": "<I",
        "writable": False,
        "desc": "Paquetes de telemetría CAN 0x030 recibidos",
    },
    "system_mode": {
        "type": "OperatingMode_t",
        "size": 1,
        "fmt": "<B",
        "writable": True,
        "min": 0,
        "max": 1,
        "desc": "0=Local, 1=Remoto",
    },
    "remote_cmd": {
        "type": "uint8_t",
        "size": 1,
        "fmt": "<B",
        "writable": True,
        "min": 0,
        "max": 12,
        "desc": "1..4=Botones, 5..8=Giro +30/+90/+180/+360°, 9..12=Giro -30/-90/-180/-360° (se autolimpia)",
    },
    "last_system_mode": {
        "type": "OperatingMode_t",
        "size": 1,
        "fmt": "<B",
        "writable": False,
        "desc": "Detector de cambio de modo",
    },
    "current_led_state": {
        "type": "uint8_t",
        "size": 1,
        "fmt": "<B",
        "writable": False,
        "desc": "Reflejo del LED PB0 de la placa B",
    },
}


# ============================================================================
# Lector de Símbolos ELF32 (ARM Cortex-M, Little-Endian)
# ============================================================================
class ElfSymbolInfo:
    def __init__(self, name: str, addr: int, size: int) -> None:
        self.name = name
        self.addr = addr
        self.size = size


def parse_elf32_symbols(elf_path: Path) -> Dict[str, ElfSymbolInfo]:
    """Extrae la tabla de símbolos (.symtab) de un archivo ELF32 little-endian."""
    data = elf_path.read_bytes()
    if len(data) < 52 or data[:4] != b"\x7fELF":
        raise ValueError(f"El archivo no es un ELF válido: {elf_path}")
    if data[4] != 1 or data[5] != 1:
        raise ValueError("Solo se admiten archivos ELF de 32 bits Little-Endian (ARM Cortex-M).")

    e_shoff, _, _, _, e_shentsize, e_shnum, _ = struct.unpack_from("<IIIHHHH", data, 32)
    if e_shoff == 0 or e_shnum == 0:
        raise ValueError("El archivo ELF no contiene tabla de secciones.")

    sections: List[Tuple[int, int, int, int, int, int]] = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        sh_name, sh_type, _, sh_addr, sh_offset, sh_size, sh_link, _, _, sh_entsize = (
            struct.unpack_from("<IIIIIIIIII", data, off)
        )
        sections.append((sh_type, sh_offset, sh_size, sh_link, sh_entsize, sh_addr))

    symbols: Dict[str, ElfSymbolInfo] = {}
    SHT_SYMTAB = 2
    for sh_type, sh_offset, sh_size, sh_link, sh_entsize, _ in sections:
        if sh_type != SHT_SYMTAB or sh_entsize == 0:
            continue
        if sh_link >= len(sections):
            continue
        str_off = sections[sh_link][1]
        str_size = sections[sh_link][2]
        strtab = data[str_off : str_off + str_size]

        count = sh_size // sh_entsize
        for j in range(count):
            sym_off = sh_offset + j * sh_entsize
            st_name, st_value, st_size, st_info, _, _ = struct.unpack_from(
                "<IIIBBH", data, sym_off
            )
            if st_name == 0 or st_name >= len(strtab) or st_value == 0:
                continue
            end = strtab.find(b"\x00", st_name)
            if end < 0:
                continue
            name = strtab[st_name:end].decode("utf-8", errors="replace")
            st_type = st_info & 0x0F
            # STT_OBJECT (1) o STT_NOTYPE (0) en región SRAM (0x20000000..)
            if st_type in (0, 1) and (0x20000000 <= st_value < 0x20020000):
                symbols[name] = ElfSymbolInfo(name=name, addr=st_value, size=st_size)

    return symbols


# ============================================================================
# Cliente Asíncrono OpenOCD TCL RPC (:6666)
# ============================================================================
class OpenOcdTclClient:
    """Cliente para el servidor TCL de OpenOCD (puerto 6666 por defecto)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 6666) -> None:
        self.host = host
        self.port = port
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.lock = asyncio.Lock()
        self.connected = False

    async def connect(self) -> bool:
        async with self.lock:
            await self._close_unlocked()
            try:
                self.reader, self.writer = await asyncio.wait_for(
                    asyncio.open_connection(self.host, self.port), timeout=1.5
                )
                self.connected = True
                await self._init_tcl_helpers_unlocked()
                return True
            except Exception:
                await self._close_unlocked()
                return False

    async def close(self) -> None:
        async with self.lock:
            await self._close_unlocked()

    async def _close_unlocked(self) -> None:
        self.connected = False
        if self.writer is not None:
            try:
                self.writer.close()
                await self.writer.wait_closed()
            except Exception:
                pass
        self.reader = None
        self.writer = None

    async def _execute_unlocked(self, cmd: str, timeout: float = 1.5) -> str:
        if not self.connected or self.writer is None or self.reader is None:
            raise ConnectionError("No conectado a OpenOCD TCL")
        payload = cmd.encode("utf-8") + TCL_EOF
        self.writer.write(payload)
        await self.writer.drain()
        raw = await asyncio.wait_for(self.reader.readuntil(TCL_EOF), timeout=timeout)
        if raw.endswith(TCL_EOF):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace").strip()

    async def _init_tcl_helpers_unlocked(self) -> None:
        """Registra funciones auxiliares TCL compatibles con OpenOCD 0.10, 0.11 y 0.12.
        Nota:
          - En JimTCL no se debe usar `return` dentro de `catch {...}`, ya que `catch`
            captura el código JIM_RETURN (2) como distinto de cero.
          - Al finalizar el comando de nivel superior con `return -code 6 <valor>`,
            OpenOCD (`command_run_line`) envía el resultado por el socket TCL (:6666)
            sin imprimirlo en la consola de STM32CubeIDE.
        """
        tcl_init = r"""
proc _hmi_state {} {
    set st "unknown"
    if {[catch {
        set t [target current]
        if {$t ne ""} {
            set st [$t curstate]
        }
    } err] == 0} {
        return $st
    }
    return "unknown"
}
proc _hmi_read_block {addr count} {
    set out {}
    if {[catch {
        if {[info commands read_memory] ne ""} {
            set out [read_memory $addr 8 $count]
        } else {
            array unset _hmi_a
            mem2array _hmi_a 8 $addr $count
            for {set i 0} {$i < $count} {incr i} {
                lappend out $_hmi_a($i)
            }
        }
    } err] == 0} {
        return $out
    }
    return "ERR:$err"
}
proc _hmi_poll {addr count} {
    set st [_hmi_state]
    set mem [_hmi_read_block $addr $count]
    return -code 6 "$st|$mem"
}
proc _hmi_write {addr width val} {
    if {[catch {
        if {[info commands write_memory] ne ""} {
            write_memory $addr $width [list $val]
        } else {
            if {$width == 8} {
                mwb $addr $val
            } elseif {$width == 16} {
                mwh $addr $val
            } else {
                mww $addr $val
            }
        }
    } err] == 0} {
        return -code 6 "OK"
    }
    return -code 6 "ERR:$err"
}
return -code 6 "INIT_OK"
"""
        await self._execute_unlocked(tcl_init)

    async def poll_block(self, base_addr: int, size: int) -> Tuple[str, Optional[bytes]]:
        """Lee el estado del micro y un bloque contiguo de SRAM en un único comando TCL."""
        async with self.lock:
            try:
                resp = await self._execute_unlocked(f"_hmi_poll 0x{base_addr:08x} {size}")
            except Exception:
                await self._close_unlocked()
                raise

        if "|" not in resp:
            return "unknown", None

        raw_state, raw_mem = resp.split("|", 1)
        state = self._normalize_state(raw_state.strip())
        if raw_mem.startswith("ERR:"):
            return state, None

        tokens = raw_mem.split()
        if len(tokens) != size:
            return state, None

        try:
            data = bytes(int(tok, 0) & 0xFF for tok in tokens)
            return state, data
        except ValueError:
            return state, None

    async def write_int(self, addr: int, size_bytes: int, value: int) -> Tuple[bool, Optional[str]]:
        """Escribe una variable entera de 1, 2 o 4 bytes en la RAM del STM32."""
        width_bits = size_bytes * 8
        # Convertir a sin signo para write_memory / mwb / mww
        mask = (1 << width_bits) - 1
        uval = value & mask
        async with self.lock:
            try:
                resp = await self._execute_unlocked(
                    f"_hmi_write 0x{addr:08x} {width_bits} 0x{uval:x}"
                )
            except Exception as exc:
                await self._close_unlocked()
                return False, f"Error de comunicación con OpenOCD: {exc}"

        if resp == "OK":
            return True, None
        return False, resp.removeprefix("ERR:")

    @staticmethod
    def _normalize_state(st: str) -> str:
        st = st.lower()
        if st in ("running", "debug-running"):
            return "running"
        if st == "halted":
            return "halted"
        if st == "reset":
            return "reset"
        return "unknown"


# ============================================================================
# Conexión WebSocket RFC 6455 (100% stdlib) + Servidor HTTP Estático
# ============================================================================
class WebSocketClient:
    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter, peer: str) -> None:
        self.reader = reader
        self.writer = writer
        self.peer = peer
        self.closed = False
        self._write_lock = asyncio.Lock()

    async def send_json(self, obj: Dict[str, Any]) -> bool:
        if self.closed:
            return False
        payload = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        return await self._send_frame(0x1, payload)

    async def _send_frame(self, opcode: int, payload: bytes) -> bool:
        if self.closed:
            return False
        header = bytearray()
        header.append(0x80 | (opcode & 0x0F))
        length = len(payload)
        if length < 126:
            header.append(length)
        elif length < 65536:
            header.append(126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(127)
            header.extend(struct.pack("!Q", length))

        async with self._write_lock:
            try:
                self.writer.write(header)
                self.writer.write(payload)
                await self.writer.drain()
                return True
            except Exception:
                self.closed = True
                return False

    async def read_message(self) -> Optional[str]:
        fragments: List[bytes] = []
        while not self.closed:
            hdr = await self.reader.readexactly(2)
            b1, b2 = hdr[0], hdr[1]
            fin = bool(b1 & 0x80)
            opcode = b1 & 0x0F
            masked = bool(b2 & 0x80)
            length = b2 & 0x7F

            if length == 126:
                ext = await self.reader.readexactly(2)
                (length,) = struct.unpack("!H", ext)
            elif length == 127:
                ext = await self.reader.readexactly(8)
                (length,) = struct.unpack("!Q", ext)

            if length > 1024 * 1024:
                raise ValueError("Trama WebSocket demasiado grande")

            mask_key = await self.reader.readexactly(4) if masked else b""
            payload = await self.reader.readexactly(length) if length > 0 else b""
            if masked and payload:
                payload = bytes(b ^ mask_key[i & 3] for i, b in enumerate(payload))

            if opcode == 0x8:  # Close
                await self._send_frame(0x8, b"")
                self.closed = True
                return None
            if opcode == 0x9:  # Ping -> Pong
                await self._send_frame(0xA, payload)
                continue
            if opcode == 0xA:  # Pong
                continue
            if opcode in (0x1, 0x0):  # Text / Continuation
                fragments.append(payload)
                if fin:
                    return b"".join(fragments).decode("utf-8", errors="replace")

        return None

    async def close(self) -> None:
        self.closed = True
        try:
            self.writer.close()
            await self.writer.wait_closed()
        except Exception:
            pass


# ============================================================================
# Puente Principal (ELF + OpenOCD TCL + WebSocket)
# ============================================================================
class TelemetryBridge:
    def __init__(
        self,
        elf_path: Path,
        hmi_dir: Path,
        tcl_host: str = "127.0.0.1",
        tcl_port: int = 6666,
        ws_host: str = "localhost",
        ws_port: int = 8765,
        rate_hz: int = 20,
    ) -> None:
        self.elf_path = elf_path
        self.hmi_dir = hmi_dir
        self.tcl = OpenOcdTclClient(tcl_host, tcl_port)
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.rate_hz = max(1, min(100, rate_hz))

        self.clients: Set[WebSocketClient] = set()
        self.t0 = time.monotonic()

        self.elf_mtime: float = 0.0
        self.var_addrs: Dict[str, int] = {}
        self.block_base: int = 0
        self.block_size: int = 0
        self.has_native_ticks_var: bool = False

        self.target_state: str = "unknown"
        self.last_openocd_ok: Optional[bool] = None

    def load_elf_if_needed(self, force: bool = False) -> bool:
        """Carga (o recarga si fue recompilado) las direcciones de símbolos desde el .elf."""
        if not self.elf_path.exists():
            if force:
                print(f"[WARN] No se encontró el archivo ELF en: {self.elf_path}")
            return False

        mtime = self.elf_path.stat().st_mtime
        if not force and mtime == self.elf_mtime and self.var_addrs:
            return False

        symbols = parse_elf32_symbols(self.elf_path)
        new_addrs: Dict[str, int] = {}
        missing: List[str] = []

        for name, spec in VAR_SPECS.items():
            sym = symbols.get(name)
            if sym is not None:
                new_addrs[name] = sym.addr
            elif not spec.get("optional", False):
                missing.append(name)

        if missing:
            raise RuntimeError(
                f"Faltan variables en {self.elf_path.name}: {', '.join(missing)}"
            )

        self.var_addrs = new_addrs
        self.has_native_ticks_var = "live_position_ticks" in new_addrs
        self.elf_mtime = mtime

        min_addr = min(addr for addr in new_addrs.values())
        max_end = max(addr + VAR_SPECS[name]["size"] for name, addr in new_addrs.items())
        self.block_base = min_addr
        self.block_size = max_end - min_addr

        print(f"\n[ELF] Símbolos cargados desde: {self.elf_path.name}")
        for name, addr in sorted(new_addrs.items(), key=lambda kv: kv[1]):
            spec = VAR_SPECS[name]
            rw = "R/W" if spec["writable"] else "R  "
            print(f"      0x{addr:08X} ({spec['size']}B, {rw}) -> {name} [{spec['type']}]")
        if not self.has_native_ticks_var:
            print(
                "      [INFO] ELF previo detectado (live_position en pulsos): "
                "el puente convertirá live_position a grados automáticamente."
            )
        print()
        return True

    def build_status_msg(self) -> Dict[str, Any]:
        vars_meta: Dict[str, Any] = {}
        for name, spec in VAR_SPECS.items():
            if name not in self.var_addrs and name != "live_position_ticks":
                continue
            vars_meta[name] = {
                "type": spec["type"],
                "addr": self.var_addrs.get(name),
                "writable": spec["writable"],
                "desc": spec["desc"],
            }
        return {
            "type": "status",
            "openocd": self.tcl.connected,
            "target": self.target_state if self.tcl.connected else "unknown",
            "elf": self.elf_path.name if self.elf_path.exists() else "No encontrado",
            "rate_hz": self.rate_hz,
            "vars_meta": vars_meta,
        }

    async def broadcast(self, msg: Dict[str, Any]) -> None:
        if not self.clients:
            return
        dead: List[WebSocketClient] = []
        for client in list(self.clients):
            ok = await client.send_json(msg)
            if not ok:
                dead.append(client)
        for client in dead:
            self.clients.discard(client)

    async def handle_client_msg(self, client: WebSocketClient, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except Exception:
            return

        mtype = msg.get("type")
        if mtype == "hello":
            await client.send_json(self.build_status_msg())
            return

        if mtype == "write":
            req_id = msg.get("id")
            var_name = str(msg.get("var", ""))
            val = msg.get("value")

            if not self.tcl.connected:
                await client.send_json(
                    {
                        "type": "ack",
                        "id": req_id,
                        "ok": False,
                        "error": "OpenOCD no está conectado (:6666). Iniciá Debug en STM32CubeIDE.",
                    }
                )
                return

            spec = VAR_SPECS.get(var_name)
            addr = self.var_addrs.get(var_name)
            if spec is None or addr is None:
                await client.send_json(
                    {
                        "type": "ack",
                        "id": req_id,
                        "ok": False,
                        "error": f"Variable desconocida en ELF: {var_name}",
                    }
                )
                return

            if not spec.get("writable", False):
                await client.send_json(
                    {
                        "type": "ack",
                        "id": req_id,
                        "ok": False,
                        "error": f"La variable '{var_name}' es de solo lectura",
                    }
                )
                return

            try:
                ival = int(val)
            except (TypeError, ValueError):
                await client.send_json(
                    {
                        "type": "ack",
                        "id": req_id,
                        "ok": False,
                        "error": f"Valor inválido: {val}",
                    }
                )
                return

            vmin = spec.get("min")
            vmax = spec.get("max")
            if (vmin is not None and ival < vmin) or (vmax is not None and ival > vmax):
                await client.send_json(
                    {
                        "type": "ack",
                        "id": req_id,
                        "ok": False,
                        "error": f"Valor fuera de rango para {var_name} ({vmin}..{vmax}): {ival}",
                    }
                )
                return

            ok, err = await self.tcl.write_int(addr, spec["size"], ival)
            ack: Dict[str, Any] = {"type": "ack", "id": req_id, "ok": ok}
            if not ok and err:
                ack["error"] = err
            await client.send_json(ack)

    async def handle_http_or_ws(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer_info = writer.get_extra_info("peername")
        peer = f"{peer_info[0]}:{peer_info[1]}" if peer_info else "cliente"
        try:
            raw_headers = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), timeout=5.0)
        except Exception:
            writer.close()
            return

        header_text = raw_headers.decode("latin-1", errors="replace")
        lines = header_text.split("\r\n")
        request_line = lines[0] if lines else ""
        parts = request_line.split(" ")
        path = parts[1] if len(parts) >= 2 else "/"

        headers: Dict[str, str] = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()

        # Si no es un Upgrade WebSocket, servir los archivos estáticos de la HMI por HTTP
        if headers.get("upgrade", "").lower() != "websocket":
            await self._serve_static_http(writer, path)
            return

        ws_key = headers.get("sec-websocket-key", "")
        if not ws_key:
            writer.close()
            return

        accept_raw = hashlib.sha1((ws_key + WS_GUID).encode("ascii")).digest()
        accept_key = base64.b64encode(accept_raw).decode("ascii")
        handshake = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
        )
        writer.write(handshake.encode("ascii"))
        await writer.drain()

        client = WebSocketClient(reader, writer, peer)
        self.clients.add(client)
        print(f"[WS] Cliente HMI conectado ({peer}) - activos: {len(self.clients)}")

        # Enviar estado inicial apenas se conecta
        await client.send_json(self.build_status_msg())

        try:
            while not client.closed:
                msg_str = await client.read_message()
                if msg_str is None:
                    break
                await self.handle_client_msg(client, msg_str)
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        except Exception as exc:
            print(f"[WS] Error con cliente {peer}: {exc}")
        finally:
            self.clients.discard(client)
            await client.close()
            print(f"[WS] Cliente HMI desconectado ({peer}) - activos: {len(self.clients)}")

    async def _serve_static_http(self, writer: asyncio.StreamWriter, req_path: str) -> None:
        """Permite abrir también http://localhost:8765 directamente en el navegador."""
        clean = req_path.split("?", 1)[0].split("#", 1)[0]
        if clean in ("", "/"):
            clean = "/index.html"
        target = (self.hmi_dir / clean.lstrip("/")).resolve()
        try:
            if not str(target).startswith(str(self.hmi_dir.resolve())) or not target.is_file():
                body = b"404 Not Found"
                resp = (
                    "HTTP/1.1 404 Not Found\r\n"
                    "Content-Type: text/plain; charset=utf-8\r\n"
                    f"Content-Length: {len(body)}\r\n"
                    "Connection: close\r\n\r\n"
                ).encode("ascii") + body
            else:
                body = target.read_bytes()
                ctype, _ = mimetypes.guess_type(str(target))
                if not ctype:
                    ctype = "application/octet-stream"
                if ctype.startswith("text/") or ctype == "application/javascript":
                    ctype += "; charset=utf-8"
                resp = (
                    "HTTP/1.1 200 OK\r\n"
                    f"Content-Type: {ctype}\r\n"
                    f"Content-Length: {len(body)}\r\n"
                    "Cache-Control: no-cache\r\n"
                    "Connection: close\r\n\r\n"
                ).encode("ascii") + body
            writer.write(resp)
            await writer.drain()
        except Exception:
            pass
        finally:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    def _unpack_vars(self, block: bytes) -> Dict[str, int]:
        values: Dict[str, int] = {}
        for name, addr in self.var_addrs.items():
            spec = VAR_SPECS[name]
            offset = addr - self.block_base
            (val,) = struct.unpack_from(spec["fmt"], block, offset)
            values[name] = val

        # Si el ELF en ejecución todavía no fue recompilado con live_position_ticks,
        # live_position en RAM está en pulsos: convertimos a grados para la HMI.
        if not self.has_native_ticks_var and "live_position" in values:
            raw_ticks = values["live_position"]
            values["live_position_ticks"] = raw_ticks
            values["live_position"] = int((raw_ticks * 360) // TICKS_PER_GEAR_REV)

        return values

    async def poll_loop(self) -> None:
        period = 1.0 / self.rate_hz
        last_elf_check = 0.0

        while True:
            t_start = time.monotonic()

            # Verificar cada 2 segundos si el archivo .elf fue recompilado por STM32CubeIDE
            if t_start - last_elf_check > 2.0:
                last_elf_check = t_start
                try:
                    if self.load_elf_if_needed(force=False):
                        await self.broadcast(self.build_status_msg())
                        await self.broadcast(
                            {
                                "type": "log",
                                "message": f"Archivo {self.elf_path.name} recargado automáticamente.",
                                "level": "info",
                            }
                        )
                except Exception as exc:
                    print(f"[ELF] Error al recargar ELF: {exc}")

            if not self.tcl.connected:
                ok = await self.tcl.connect()
                if ok:
                    print(f"[OpenOCD] Conectado a {self.tcl.host}:{self.tcl.port}")
                    self.last_openocd_ok = True
                    await self.broadcast(self.build_status_msg())
                else:
                    if self.last_openocd_ok is not False:
                        print(
                            f"[OpenOCD] Esperando servidor TCL en {self.tcl.host}:{self.tcl.port} "
                            "(iniciá la sesión Debug en STM32CubeIDE)..."
                        )
                        self.last_openocd_ok = False
                        self.target_state = "unknown"
                        await self.broadcast(self.build_status_msg())
                    await asyncio.sleep(1.5)
                    continue

            try:
                state, block = await self.tcl.poll_block(self.block_base, self.block_size)
                if state != self.target_state:
                    self.target_state = state
                    await self.broadcast({"type": "target", "state": state})

                if block is not None and len(block) == self.block_size:
                    vars_dict = self._unpack_vars(block)
                    t_ms = int(round((time.monotonic() - self.t0) * 1000))
                    await self.broadcast(
                        {
                            "type": "telemetry",
                            "t": t_ms,
                            "target": state,
                            "vars": vars_dict,
                        }
                    )
            except Exception as exc:
                print(f"[OpenOCD] Conexión perdida con OpenOCD: {exc}")
                self.last_openocd_ok = False
                self.target_state = "unknown"
                await self.broadcast(self.build_status_msg())
                await asyncio.sleep(1.0)
                continue

            elapsed = time.monotonic() - t_start
            sleep_s = max(0.005, period - elapsed)
            await asyncio.sleep(sleep_s)

    async def run(self) -> None:
        self.load_elf_if_needed(force=True)
        try:
            server = await asyncio.start_server(
                self.handle_http_or_ws, self.ws_host, self.ws_port
            )
        except OSError as exc:
            print("=" * 72)
            print(f"[ERROR] No se pudo abrir el puerto {self.ws_port}: {exc}")
            print("        Probablemente haya otra ventana de bridge.py / iniciar_puente.bat")
            print("        abierta en segundo plano. Cerrala y volvé a intentar.")
            print("=" * 72)
            raise SystemExit(1)
        addrs = ", ".join(str(s.getsockname()) for s in (server.sockets or []))
        print("=" * 72)
        print(" PUENTE PYTHON HMI · BP_A_Master_F (OpenOCD TCL :6666 -> WebSocket :8765)")
        print("=" * 72)
        print(f"  WebSocket HMI : ws://{self.ws_host}:{self.ws_port}")
        print(f"  Web HMI (HTTP): http://{self.ws_host}:{self.ws_port}")
        print(f"  OpenOCD TCL   : {self.tcl.host}:{self.tcl.port}")
        print(f"  Archivo ELF   : {self.elf_path}")
        print(f"  Frecuencia    : {self.rate_hz} Hz")
        print(f"  Sockets       : {addrs}")
        print("=" * 72)

        async with server:
            await self.poll_loop()


def main() -> int:
    hmi_dir = Path(__file__).resolve().parent
    default_elf = hmi_dir.parent / "BP_A_Master_F" / "Debug" / "BP_A_Master_F.elf"

    parser = argparse.ArgumentParser(
        description="Puente OpenOCD TCL (:6666) a WebSocket (:8765) para la HMI de BP_A_Master_F."
    )
    parser.add_argument(
        "--elf",
        type=Path,
        default=default_elf,
        help=f"Ruta al archivo .elf (por defecto: {default_elf})",
    )
    parser.add_argument(
        "--tcl-host",
        default="127.0.0.1",
        help="Host del servidor TCL de OpenOCD (por defecto: 127.0.0.1)",
    )
    parser.add_argument(
        "--tcl-port",
        type=int,
        default=6666,
        help="Puerto TCL de OpenOCD (por defecto: 6666)",
    )
    parser.add_argument(
        "--ws-host",
        default="localhost",
        help="Host donde escuchar WebSocket/HTTP (por defecto: localhost)",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=8765,
        help="Puerto WebSocket/HTTP para la HMI (por defecto: 8765)",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=20,
        help="Frecuencia de muestreo en Hz (por defecto: 20)",
    )
    parser.add_argument(
        "--check-elf",
        action="store_true",
        help="Verifica la lectura de símbolos del archivo .elf y termina.",
    )
    parser.add_argument(
        "--test-once",
        action="store_true",
        help="Realiza una lectura única a OpenOCD (:6666), muestra las variables y termina.",
    )
    args = parser.parse_args()

    bridge = TelemetryBridge(
        elf_path=args.elf.resolve(),
        hmi_dir=hmi_dir,
        tcl_host=args.tcl_host,
        tcl_port=args.tcl_port,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
        rate_hz=args.rate,
    )

    if args.check_elf:
        bridge.load_elf_if_needed(force=True)
        return 0

    if args.test_once:
        import socket
        bridge.load_elf_if_needed(force=True)
        try:
            with socket.create_connection((bridge.tcl.host, bridge.tcl.port), timeout=2.0) as s:
                async def _dummy() -> None: pass
                # Enviar inicialización y lectura única por socket sincrónico
                init_cmd = r"""
proc _hmi_state {} {
    set st "unknown"
    if {[catch {
        set t [target current]
        if {$t ne ""} { set st [$t curstate] }
    } err] == 0} { return $st }
    return "unknown"
}
proc _hmi_read_block {addr count} {
    set out {}
    if {[catch {
        if {[info commands read_memory] ne ""} {
            set out [read_memory $addr 8 $count]
        } else {
            array unset _hmi_a
            mem2array _hmi_a 8 $addr $count
            for {set i 0} {$i < $count} {incr i} { lappend out $_hmi_a($i) }
        }
    } err] == 0} { return $out }
    return "ERR:$err"
}
proc _hmi_poll {addr count} {
    set st [_hmi_state]
    set mem [_hmi_read_block $addr $count]
    return -code 6 "$st|$mem"
}
return -code 6 "INIT_OK"
"""
                def tcl_req(cmd: str) -> str:
                    s.sendall(cmd.encode("utf-8") + TCL_EOF)
                    buf = bytearray()
                    while not buf.endswith(TCL_EOF):
                        chunk = s.recv(4096)
                        if not chunk:
                            break
                        buf.extend(chunk)
                    return bytes(buf[:-1]).decode("utf-8", errors="replace").strip()

                tcl_req(init_cmd)
                resp = tcl_req(f"_hmi_poll 0x{bridge.block_base:08x} {bridge.block_size}")
                print("Respuesta cruda TCL:", resp)
                raw_st, raw_mem = resp.split("|", 1)
                blk = bytes(int(tok, 0) & 0xFF for tok in raw_mem.split())
                vals = bridge._unpack_vars(blk)
                print(f"[OK] Target={raw_st} | Variables={json.dumps(vals, ensure_ascii=False)}")
                return 0
        except Exception as exc:
            print(f"[ERROR] {exc}")
            return 1

    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        print("\n[Puente] Detenido por el usuario.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
