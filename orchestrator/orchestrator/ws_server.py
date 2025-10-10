import asyncio
import logging
import threading
from queue import Queue, Empty
from typing import Callable, Optional, Set, Union

from websockets import ConnectionClosed, WebSocketServerProtocol
from websockets.asyncio.server import serve


class WebSocketServer:
    """
    A simple WebSocket server that receives messages via callback and broadcasts to all clients.

    Parameters
    ----------
    logger : logging.Logger
        Logger instance for logging messages
    host : str, optional
        The hostname to bind the server to, by default "localhost"
    port : int, optional
        The port number to listen on, by default 6123
    """

    def __init__(self, logger: logging.Logger, host: str = "localhost", port: int = 6123):
        self.host = host
        self.port = port
        self.connections: Set[WebSocketServerProtocol] = set()
        self.message_callback: Optional[Callable] = None
        self._running = False
        self._loop = None
        self._broadcast_queue: Queue = Queue()
        self._lock = threading.Lock()
        self.logger = logger

    def on_message(self, callback: Callable):
        """
        Register a callback function to handle incoming messages.

        Parameters
        ----------
        callback
            Function called with (connection_id, message) when a message is received
        """
        self.message_callback = callback
        self.logger.info("Message callback registered")

    async def _process_broadcast_queue(self):
        """
        Process messages from the broadcast queue and send to all clients.
        """
        while self._running:
            try:
                message = self._broadcast_queue.get(timeout=0.1)
                await self._send_to_all(message)
                self._broadcast_queue.task_done()
            except Empty:
                await asyncio.sleep(0.01)
            except Exception as e:
                self.logger.error(f"Error processing broadcast queue: {e}")

    async def _send_to_all(self, message: Union[str, bytes]):
        """
        Send a message to all connected clients.

        Parameters
        ----------
        message : Union[str, bytes]
            The message to send to all clients
        """
        if not self.connections:
            return

        disconnected = set()
        with self._lock:
            connections = self.connections.copy()

        for websocket in connections:
            try:
                self.logger.info(f"Sending message to {websocket.remote_address}")
                await websocket.send(message)
            except ConnectionClosed:
                disconnected.add(websocket)
            except Exception as e:
                self.logger.error(f"Error broadcasting to client: {e}")
                disconnected.add(websocket)

        if disconnected:
            with self._lock:
                self.connections -= disconnected

    async def _handle_client(self, websocket: WebSocketServerProtocol):
        """
        Handle a single client connection.

        Parameters
        ----------
        websocket : WebSocketServerProtocol
            The WebSocket connection instance
        """
        connection_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"

        with self._lock:
            self.connections.add(websocket)

        self.logger.info(f"Client connected: {connection_id} (total: {len(self.connections)})")

        try:
            async for message in websocket:
                if self.message_callback:
                    threading.Thread(
                        target=self.message_callback,
                        args=(message,),
                        daemon=True
                    ).start()
        except ConnectionClosed:
            self.logger.info(f"Client disconnected: {connection_id}")
        except Exception as e:
            self.logger.error(f"Error handling client {connection_id}: {e}")
        finally:
            with self._lock:
                self.connections.discard(websocket)
            self.logger.info(f"Client removed: {connection_id} (total: {len(self.connections)})")

    async def _run(self):
        """
        Run the WebSocket server.
        """
        self._loop = asyncio.get_event_loop()
        self._running = True

        broadcast_task = asyncio.create_task(self._process_broadcast_queue())

        try:
            async with serve(self._handle_client, self.host, self.port):
                self.logger.info(f"WebSocket server started on {self.host}:{self.port}")

                while self._running:
                    await asyncio.sleep(0.1)

        finally:
            self._running = False
            broadcast_task.cancel()
            try:
                await broadcast_task
            except asyncio.CancelledError:
                pass

    def start(self):
        """
        Start the WebSocket server in a separate thread.
        """
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._run())
            except KeyboardInterrupt:
                pass
            finally:
                loop.close()

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
        self.logger.info("WebSocket server thread started")

    def broadcast(self, message: Union[str, bytes]):
        """
        Broadcast a message to all connected clients.

        This method is thread-safe and can be called from any thread.

        Parameters
        ----------
        message : Union[str, bytes]
            The message to broadcast
        """
        if not self._running:
            self.logger.warning("Server not running, cannot broadcast")
            return

        self._broadcast_queue.put(message)
        self.logger.info(f"Message queued for broadcast: {len(message)} bytes")

    def is_running(self) -> bool:
        """
        Check if the server is running.
        """
        return self._running

    def has_connections(self) -> bool:
        """
        Check if there are any active connections.
        """
        with self._lock:
            return bool(self.connections)

    def stop(self):
        """
        Stop the WebSocket server.
        """
        self._running = False
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=5)
