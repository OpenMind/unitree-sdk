import logging
import threading
import time
from queue import Queue, Empty
from typing import Optional, Callable

import websockets
from websockets.sync.client import connect

class WebSocketClient:
    """
    A simple WebSocket client for connecting to a WebSocket server and sending messages.
    """

    def __init__(self, url: str, logger: logging.Logger):
        if not url:
            raise ValueError("WebSocket URL must be provided")

        self.logger = logger
        self.url = url
        self.running: bool = True
        self.connected: bool = False
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self.message_callback: Optional[Callable] = None
        self.message_queue: Queue = Queue()
        self.sender_thread: Optional[threading.Thread] = None
        self.receiver_thread: Optional[threading.Thread] = None

    def _receive_messages(self):
        """
        Internal method to handle receiving messages from the WebSocket connection.

        Continuously receives messages and processes them through the registered callback
        if one exists. Runs in a separate thread.
        """
        while self.running and self.connected:
            try:
                message = self.websocket.recv()
                if self.message_callback:
                    self.message_callback(message)
            except Exception as e:
                self.logger.error(f"Error in message processing: {e}")
                self.connected = False
                break

    def _send_messages(self):
        """
        Internal method to handle sending messages through the WebSocket connection.

        Continuously processes messages from the message queue and sends them through
        the WebSocket connection. Runs in a separate thread.
        """
        while self.running:
            try:
                if self.connected and self.websocket:
                    message = self.message_queue.get(timeout=0.1)
                    try:
                        self.websocket.send(message)
                    except Exception as e:
                        self.logger.error(f"Failed to send message: {e}")
                        self.message_queue.put(message)
                else:
                    time.sleep(0.1)
            except Empty:
                continue
            except Exception as e:
                self.logger.error(f"Error in send queue processing: {e}")
                self.connected = False

    def connect(self) -> bool:
        """
        Establish a connection to the WebSocket server.

        Attempts to connect to the WebSocket server and starts the receiver and sender
        threads if the connection is successful.

        Returns
        -------
        bool
            True if connection was successful, False otherwise
        """
        try:
            self.websocket = connect(self.url)
            self.connected = True

            if not self.receiver_thread or not self.receiver_thread.is_alive():
                self.receiver_thread = threading.Thread(
                    target=self._receive_messages, daemon=True
                )
                self.receiver_thread.start()

            if not self.sender_thread or not self.sender_thread.is_alive():
                self.sender_thread = threading.Thread(
                    target=self._send_messages, daemon=True
                )
                self.sender_thread.start()

            self.logger.info(f"Connected to {self.url}")
            return True
        except Exception as e:
            self.logger.error(f"Connection error: {e}")
            self.connected = False
            return False

    def send_message(self, message: str | bytes):
        """
        Queue a message to be sent through the WebSocket connection.

        Parameters
        ----------
        message : Union[str, bytes]
            The message to send, either as a string or bytes
        """
        if self.connected:
            self.message_queue.put(message)

    def _run_client(self):
        """
        Internal method to manage the WebSocket client lifecycle.

        Continuously attempts to maintain a connection to the WebSocket server,
        implementing automatic reconnection with a delay between attempts.
        """
        while self.running:
            if not self.connected:
                if self.connect():
                    self.logger.info("Connection established")
                else:
                    self.logger.info("Connection failed, retrying in 5 seconds")
                    threading.Event().wait(5)
            else:
                threading.Event().wait(1)

    def start(self):
        """
        Start the WebSocket client.

        Initializes and starts the main client thread that manages the WebSocket
        connection.
        """
        self.client_thread = threading.Thread(target=self._run_client, daemon=True)
        self.client_thread.start()
        self.logger.info("WebSocket client thread started")

    def stop(self):
        """
        Stop the WebSocket client.

        Closes the WebSocket connection, stops all threads, and cleans up resources.
        """
        self.running = False
        if self.websocket:
            try:
                self.websocket.close()
                self.logger.info("WebSocket connection closed")
            except Exception as _:
                pass

        try:
            while True:
                self.message_queue.get_nowait()
                self.message_queue.task_done()
        except Empty:
            pass

        self.connected = False
        self.logger.info("WebSocket client stopped")
