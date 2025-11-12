import os
from typing import Any, Dict, Optional

import requests

env = os.getenv("ENV", "production")

if env == "development":
    base_url = "https://api-dev.openmind.org/api/core"
else:
    base_url = "https://api.openmind.org/api/core"


class CloudAPIService:
    """
    Handles HTTP API requests to cloud services.
    """

    def __init__(self, logger):
        """
        Initialize the cloud API service.

        Parameters:
        -----------
        logger : Logger
            Logger instance for logging messages.
        """
        self.logger = logger
        self.api_key = os.getenv("OM_API_KEY")

        if not self.api_key:
            self.logger.error("OM_API_KEY environment variable not set!")

        self.base_url = base_url

        self.headers = {
            "Authorization": f"Bearer {self.api_key}" if self.api_key else "",
            "Content-Type": "application/json",
        }

    def get_headers(
        self, custom_headers: Optional[Dict[str, str]] = None
    ) -> Dict[str, str]:
        """
        Get headers for API requests.

        Parameters:
        -----------
        custom_headers : Dict[str, str], optional
            Additional headers to include.

        Returns:
        --------
        Dict[str, str]
            Headers dictionary.
        """
        headers = self.headers.copy()
        if custom_headers:
            headers.update(custom_headers)
        return headers

    def make_request(
        self,
        method: str,
        endpoint: str,
        data: Optional[Dict[str, Any]] = None,
        params: Optional[Dict[str, Any]] = None,
        custom_headers: Optional[Dict[str, str]] = None,
        timeout: int = 30,
    ) -> Dict[str, Any]:
        """
        Make an HTTP request to the cloud API.

        Parameters:
        -----------
        method : str
            HTTP method (GET, POST, PUT, DELETE).
        endpoint : str
            API endpoint (e.g., '/teleops/status').
        data : Dict[str, Any], optional
            Request data for POST/PUT requests.
        params : Dict[str, Any], optional
            Query parameters.
        custom_headers : Dict[str, str], optional
            Additional headers.
        timeout : int
            Request timeout in seconds.

        Returns:
        --------
        Dict[str, Any]
            Response data including status, message, and response content.
        """
        if not self.api_key:
            return {"status": "error", "message": "API key not set"}

        url = f"{self.base_url}{endpoint}"
        headers = self.get_headers(custom_headers)

        try:
            self.logger.info(f"Making {method} request to {url}")

            response = requests.request(
                method=method.upper(),
                url=url,
                json=data,
                params=params,
                headers=headers,
                timeout=timeout,
            )

            result = {
                "status": "success" if response.ok else "error",
                "status_code": response.status_code,
                "message": f"{method} request completed",
                "url": url,
            }

            try:
                result["data"] = response.json()
            except ValueError:
                result["data"] = response.text

            if not response.ok:
                result["message"] = f"HTTP {response.status_code}: {response.reason}"
                self.logger.error(f"API request failed: {result['message']}")
            else:
                self.logger.info(f"API request successful: {response.status_code}")

            return result

        except requests.exceptions.Timeout:
            error_msg = f"Request timeout for {method} {url}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        except requests.exceptions.ConnectionError:
            error_msg = f"Connection error for {method} {url}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        except requests.exceptions.RequestException as e:
            error_msg = f"Request exception for {method} {url}: {str(e)}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        except Exception as e:
            error_msg = f"Unexpected error for {method} {url}: {str(e)}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

    def get(
        self, endpoint: str, params: Optional[Dict[str, Any]] = None, **kwargs
    ) -> Dict[str, Any]:
        """
        Make a GET request.

        Parameters:
        -----------
        endpoint : str
            API endpoint.
        params : Dict[str, Any], optional
            Query parameters.
        **kwargs
            Additional arguments for make_request.

        Returns:
        --------
        Dict[str, Any]
            Response data.
        """
        return self.make_request("GET", endpoint, params=params, **kwargs)

    def post(
        self, endpoint: str, data: Optional[Dict[str, Any]] = None, **kwargs
    ) -> Dict[str, Any]:
        """
        Make a POST request.

        Parameters:
        -----------
        endpoint : str
            API endpoint.
        data : Dict[str, Any], optional
            Request data.
        **kwargs
            Additional arguments for make_request.

        Returns:
        --------
        Dict[str, Any]
            Response data.
        """
        return self.make_request("POST", endpoint, data=data, **kwargs)

    def put(
        self, endpoint: str, data: Optional[Dict[str, Any]] = None, **kwargs
    ) -> Dict[str, Any]:
        """
        Make a PUT request.

        Parameters:
        -----------
        endpoint : str
            API endpoint.
        data : Dict[str, Any], optional
            Request data.
        **kwargs
            Additional arguments for make_request.

        Returns:
        --------
        Dict[str, Any]
            Response data.
        """
        return self.make_request("PUT", endpoint, data=data, **kwargs)

    def delete(self, endpoint: str, **kwargs) -> Dict[str, Any]:
        """
        Make a DELETE request.

        Parameters:
        -----------
        endpoint : str
            API endpoint.
        **kwargs
            Additional arguments for make_request.

        Returns:
        --------
        Dict[str, Any]
            Response data.
        """
        return self.make_request("DELETE", endpoint, **kwargs)
