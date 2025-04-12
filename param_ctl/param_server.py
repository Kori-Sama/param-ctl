# -*- coding: utf-8 -*-

import threading
import json
import os
import socket
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse

class ParamRequestHandler(SimpleHTTPRequestHandler):
    """Custom request handler for parameter server"""
    
    def __init__(self, *args, param_manager=None, **kwargs):
        self.param_manager = param_manager
        super().__init__(*args, **kwargs)
    
    def log_message(self, format, *args):
        # Suppress log messages
        pass
    
    def _send_response(self, status_code, content, content_type="application/json"):
        """Send a response with the specified content"""
        self.send_response(status_code)
        self.send_header("Content-type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        if isinstance(content, str):
            self.wfile.write(content.encode())
        else:
            self.wfile.write(content)
    
    def _handle_static_file(self, path):
        """Handle static file requests"""
        # Remove leading '/'
        if path.startswith('/'):
            path = path[1:]
        
        # Get path to static directory
        static_dir = os.path.join(os.path.dirname(__file__), 'static')
        file_path = os.path.join(static_dir, path)
        
        # Check if file exists
        if not os.path.exists(file_path) or not os.path.isfile(file_path):
            self.send_error(404, "File not found")
            return
        
        # Determine content type
        content_type = "text/plain"
        if file_path.endswith(".html"):
            content_type = "text/html"
        elif file_path.endswith(".css"):
            content_type = "text/css"
        elif file_path.endswith(".js"):
            content_type = "application/javascript"
        elif file_path.endswith(".json"):
            content_type = "application/json"
        elif file_path.endswith(".png"):
            content_type = "image/png"
        elif file_path.endswith(".jpg") or file_path.endswith(".jpeg"):
            content_type = "image/jpeg"
        
        # Read and send file
        with open(file_path, 'rb') as f:
            content = f.read()
            self._send_response(200, content, content_type)
    
    def _handle_index(self):
        """Handle index page request"""
        template_path = os.path.join(os.path.dirname(__file__), 'templates', 'index.html')
        with open(template_path, 'rb') as f:
            content = f.read()
            self._send_response(200, content, "text/html")
    
    def do_GET(self):
        """Handle GET requests"""
        parsed_url = urlparse(self.path)
        path = parsed_url.path
        
        # API endpoints
        if path == '/api/params':
            # Get all parameters
            response = json.dumps(self.param_manager.to_dict()).encode()
            self._send_response(200, response)
            
        elif path.startswith('/api/params/'):
            # Get specific parameter
            param_name = path.split('/')[3]
            try:
                param = self.param_manager.get_param(param_name)
                response = json.dumps(param.to_dict()).encode()
                self._send_response(200, response)
            except KeyError:
                error_msg = json.dumps({"error": f"Parameter {param_name} does not exist"}).encode()
                self._send_response(404, error_msg)
                
        # Static files
        elif path.startswith('/static/'):
            self._handle_static_file(path[7:])  # Remove '/static/' prefix
            
        # Index page
        elif path == '/' or path == '/index.html':
            self._handle_index()
            
        else:
            self.send_error(404, "Not found")
    
    def do_POST(self):
        """Handle POST requests"""
        parsed_url = urlparse(self.path)
        path = parsed_url.path
        
        # Update parameter
        if path.startswith('/api/params/'):
            param_name = path.split('/')[3]
            
            # Read request body
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            
            try:
                data = json.loads(post_data)
                if 'value' not in data:
                    error_msg = json.dumps({"error": "Missing parameter value"}).encode()
                    self._send_response(400, error_msg)
                    return
                
                self.param_manager.set(param_name, data['value'])
                param = self.param_manager.get_param(param_name)
                response = json.dumps(param.to_dict()).encode()
                self._send_response(200, response)
                
            except KeyError:
                error_msg = json.dumps({"error": f"Parameter {param_name} does not exist"}).encode()
                self._send_response(404, error_msg)
            except ValueError as e:
                error_msg = json.dumps({"error": str(e)}).encode()
                self._send_response(400, error_msg)
            except json.JSONDecodeError:
                error_msg = json.dumps({"error": "Invalid JSON data"}).encode()
                self._send_response(400, error_msg)
        else:
            self.send_error(404, "Not found")
    
    def do_OPTIONS(self):
        """Handle OPTIONS requests for CORS"""
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

class ParamServer:
    """Parameter server providing Web interface and API endpoints using Python's built-in HTTP server"""
    def __init__(self, param_manager, host="127.0.0.1", port=8080):
        """
        Initialize parameter server
        
        Args:
            param_manager (ParamManager): Parameter manager
            host (str): Server host address
            port (int): Server port
        """
        self.param_manager = param_manager
        self.host = host
        self.port = port
        self.server = None
        self.server_thread = None
    
    def _create_handler_class(self):
        """Create a handler class with access to param_manager"""
        param_manager = self.param_manager
        
        class CustomHandler(ParamRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, param_manager=param_manager, **kwargs)
        
        return CustomHandler
    
    def start(self, debug=False):
        """
        Start the server
        
        Args:
            debug (bool): Whether to enable debug mode
        """
        if self.server_thread and self.server_thread.is_alive():
            print("Server is already running")
            return
        
        def run_server():
            try:
                handler_class = self._create_handler_class()
                self.server = HTTPServer((self.host, self.port), handler_class)
                print(f"Parameter server started, visit http://{self.host}:{self.port} to view interface")
                self.server.serve_forever()
            except socket.error as e:
                print(f"Server error: {e}")
            except Exception as e:
                print(f"Unexpected error: {e}")
        
        self.server_thread = threading.Thread(target=run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
    
    def stop(self):
        """Stop the server"""
        if self.server:
            print("Stopping parameter server...")
            self.server.shutdown()
            self.server.server_close()
            self.server = None
            print("Parameter server stopped")
        else:
            print("Server is not running")