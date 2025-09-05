"""
RS‑232 driver using pyserial & pyserial‑asyncio.

Features
--------
* Async client with persistent connection and locking
* Query helper: sends '?NAME\r\n' and parses 'NAME = VALUE'
* Optional EOT (0x04) handling
* Automatic serial‑port detection when no port is provided
* Simple synchronous wrapper and context manager
"""

import asyncio
from typing import Dict, Optional

import serial
import serial.tools.list_ports
import serial_asyncio


class AsyncSerialClient:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 9600,
        *,
        bytesize: int = serial.EIGHTBITS,
        parity: str = serial.PARITY_NONE,
        stopbits: int = serial.STOPBITS_ONE,
        timeout: float = 1.0,
        reconnect_delay: float = 5.0,
        eot: Optional[bytes] = b"\x04",
        auto_detect: bool = True,
        **kwargs,
    ):
        if port is None and auto_detect:
            port = self.detect_port()
            if port is None:
                raise ValueError("No serial device detected")

        self.port_name = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.reconnect_delay = reconnect_delay
        self.eot = eot
        self.extra = kwargs

        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._lock = asyncio.Lock()
        self._closing = False

    # ------------------------------------------------------------------ Connection
    async def connect(self) -> None:
        """Open the serial port (retry loop)."""
        while not self._closing:
            try:
                self._reader, self._writer = await serial_asyncio.open_serial_connection(
                    url=self.port_name,
                    baudrate=self.baudrate,
                    bytesize=self.bytesize,
                    parity=self.parity,
                    stopbits=self.stopbits,
                    **self.extra,
                )
                break
            except Exception:
                await asyncio.sleep(self.reconnect_delay)

    async def close(self) -> None:
        self._closing = True
        if self._writer:
            self._writer.close()
            await self._writer.wait_closed()

    async def _ensure_connection(self) -> None:
        if (
            not self._writer
            or self._writer.transport.is_closing()
            or not self._reader
        ):
            await self.connect()

    # ------------------------------------------------------------------ Helpers
    async def _readline(self, timeout: float) -> bytes:
        buf = bytearray()
        while True:
            b = await asyncio.wait_for(self._reader.read(1), timeout)
            if not b:
                raise ConnectionError("Serial connection closed")
            buf += b
            if buf.endswith(b"\r\n") or (self.eot and buf.endswith(self.eot)):
                return bytes(buf)

    # ------------------------------------------------------------------ Public API
    async def query(self, variable: str, timeout: Optional[float] = None) -> Dict[str, str]:
        """Send '?VAR\r\n' and parse 'VAR = VALUE'."""
        timeout = timeout or self.timeout
        for attempt in (0, 1):
            async with self._lock:
                await self._ensure_connection()
                self._writer.write(f"?{variable}\r\n".encode())
                try:
                    line = await self._readline(timeout)
                    break
                except Exception:
                    if self._writer:
                        self._writer.transport.close()
                    if attempt:
                        raise
                    await asyncio.sleep(self.reconnect_delay)
                    continue

        text = line.decode(errors="ignore").strip("\r\n")
        if self.eot and text.endswith(chr(self.eot[0])):
            text = text[:-1]

        name, _, value = text.partition("=")
        return {"variable": name.strip(), "value": value.strip(), "raw": text}

    # ------------------------------------------------------------------ Port detection
    @staticmethod
    def detect_port(
        test_variable: str = "PING",
        expected: str = "PONG",
        baudrate: int = 9600,
        timeout: float = 1.0,
    ) -> Optional[str]:
        """
        Scan available serial ports and return the one replying with
        the expected value.
        """
        for info in serial.tools.list_ports.comports():
            try:
                with serial.Serial(info.device, baudrate=baudrate, timeout=timeout) as ser:
                    ser.write(f"?{test_variable}\r\n".encode())
                    line = ser.readline().decode(errors="ignore").strip("\r\n")
                    if line.endswith("\x04"):
                        line = line[:-1]
                    name, _, value = line.partition("=")
                    if name.strip() == test_variable and value.strip() == expected:
                        return info.device
            except Exception:
                continue
        return None


# ---------------------------------------------------------------------- Synchronous wrapper
class SerialClient:
    """
    Thin synchronous facade over AsyncSerialClient.
    Suitable for classic blocking applications.
    """

    def __init__(self, port: Optional[str] = None, **kwargs):
        self._loop = asyncio.new_event_loop()
        self._client = AsyncSerialClient(port, **kwargs)
        self._loop.run_until_complete(self._client.connect())

    def query(self, variable: str, timeout: Optional[float] = None) -> Dict[str, str]:
        return self._loop.run_until_complete(self._client.query(variable, timeout))

    def close(self) -> None:
        self._loop.run_until_complete(self._client.close())
        self._loop.close()

    def __enter__(self) -> "SerialClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # expose port detection for convenience
    detect_port = staticmethod(AsyncSerialClient.detect_port)


# ---------------------------------------------------------------------- Example usage
if __name__ == "__main__":
    # Synchronous example with auto-detected port
    with SerialClient(reconnect_delay=2.0) as client:
        print(client.query("TEMP1"))

    # Asynchronous example
    async def main():
        aclient = AsyncSerialClient()
        await aclient.connect()
        print(await aclient.query("TEMP1"))
        await aclient.close()

    asyncio.run(main())
