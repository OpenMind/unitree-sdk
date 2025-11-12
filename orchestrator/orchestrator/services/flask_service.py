import threading
from typing import Optional

from flask import Flask
from flask_cors import CORS


class FlaskService:
    """
    Manages the Flask application and HTTP server.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5000, logger=None):
        """
        Initialize the Flask service.

        Parameters:
        -----------
        host : str
            Host address to bind the server to.
        port : int
            Port number to bind the server to.
        logger
            Logger instance for logging operations.
        """
        self.host = host
        self.port = port
        self.logger = logger

        self.app = Flask(__name__)
        CORS(self.app)

        self.api_thread: Optional[threading.Thread] = None

    def register_routes(self, route_handler):
        """
        Register API routes with the Flask app.

        Parameters:
        -----------
        route_handler
            Object containing route handler methods.
        """
        # Route registration will be handled by the route_handler
        route_handler.register_routes(self.app)

    def start(self):
        """
        Start the Flask application in a separate thread.
        """
        self.api_thread = threading.Thread(target=self._run_flask_app, daemon=True)
        self.api_thread.start()

        if self.logger:
            self.logger.info(f"Flask API server starting on {self.host}:{self.port}")

    def _run_flask_app(self):
        """
        Run the Flask application.
        """
        try:
            self.app.run(
                host=self.host, port=self.port, use_reloader=False, debug=False
            )
        except Exception as e:
            if self.logger:
                self.logger.error(f"Flask server error: {e}")

    def stop(self):
        """
        Stop the Flask application.
        """
        if self.logger:
            self.logger.info("Flask server stop requested")

    def get_app(self) -> Flask:
        """
        Get the Flask application instance.

        Returns:
        --------
        Flask
            The Flask application instance.
        """
        return self.app
