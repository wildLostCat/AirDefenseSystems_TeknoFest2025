# async_serial_writer.py
import asyncio
import serial
import serial_asyncio


class AsyncSerialWriter:
    def __init__(self, port, baudrate=9600, reconnect_delay=2.0):
        self.port = port
        self.baudrate = baudrate
        self.reconnect_delay = reconnect_delay

        self._transport = None
        self._protocol = None
        self._task = None
        self._queue = asyncio.Queue()
        self._stopping = asyncio.Event()

    async def start(self):
        """Start connection and writer loop"""
        self._stopping.clear()
        self._task = asyncio.create_task(self._writer_loop())

    async def stop(self):
        """Stop gracefully"""
        self._stopping.set()
        if self._task:
            await self._task
        if self._transport:
            self._transport.close()
            self._transport = None

    async def write(self, data: str, wait=False, wait_flush=False, timeout=None):
        """
        Queue data for writing.
        - wait=True: wait until written
        - wait_flush=True: wait until flushed (serial buffers drained)
        """
        fut = asyncio.get_event_loop().create_future()
        await self._queue.put((data.encode(), fut, wait, wait_flush))
        if wait or wait_flush:
            return await asyncio.wait_for(fut, timeout=timeout)
        return None

    async def _writer_loop(self):
        loop = asyncio.get_event_loop()
        while not self._stopping.is_set():
            try:
                transport, protocol = await serial_asyncio.create_serial_connection(
                    loop, lambda: _WriterProtocol(self._queue), self.port, baudrate=self.baudrate
                )
                self._transport = transport
                self._protocol = protocol
                await self._stopping.wait()  # wait until stop
            except Exception as e:
                print(f"[AsyncSerialWriter] Connection failed: {e}, retrying in {self.reconnect_delay}s")
                await asyncio.sleep(self.reconnect_delay)
        print("[AsyncSerialWriter] writer loop stopped")


class _WriterProtocol(asyncio.Protocol):
    def __init__(self, queue: asyncio.Queue):
        self.queue = queue
        self.transport = None
        self._task = None

    def connection_made(self, transport):
        self.transport = transport
        print("[AsyncSerialWriter] Port opened")
        self._task = asyncio.create_task(self._drain_loop())

    def connection_lost(self, exc):
        print("[AsyncSerialWriter] Port closed")
        if self._task:
            self._task.cancel()
            self._task = None
        self.transport = None

    async def _drain_loop(self):
        while True:
            data, fut, wait, wait_flush = await self.queue.get()
            try:
                self.transport.write(data)
                if wait_flush:
                    await asyncio.sleep(0)  # yield back, pyserial doesn't support explicit flush here
                if wait or wait_flush:
                    if not fut.done():
                        fut.set_result(True)
            except Exception as e:
                if not fut.done():
                    fut.set_exception(e)
