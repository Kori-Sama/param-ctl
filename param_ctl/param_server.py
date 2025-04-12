# -*- coding: utf-8 -*-

import threading
from flask import Flask, request, jsonify, render_template, send_from_directory
import os
import logging


class ParamServer:
    """Parameter server providing Web interface and API endpoints"""

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
        self.app = Flask(__name__)
        self.server_thread = None
        self._setup_routes()

        # Set log level
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

    def _setup_routes(self):
        """Set up routes"""
        # Static file route
        @self.app.route('/static/<path:path>')
        def send_static(path):
            static_dir = os.path.join(os.path.dirname(__file__), 'static')
            return send_from_directory(static_dir, path)

        # Homepage
        @self.app.route('/')
        def index():
            return render_template('index.html')

        # Get all parameters
        @self.app.route('/api/params', methods=['GET'])
        def get_params():
            return jsonify(self.param_manager.to_dict())

        # Get single parameter
        @self.app.route('/api/params/<name>', methods=['GET'])
        def get_param(name):
            try:
                param = self.param_manager.get_param(name)
                return jsonify(param.to_dict())
            except KeyError:
                return jsonify({"error": f"Parameter {name} does not exist"}), 404

        # Update parameter
        @self.app.route('/api/params/<name>', methods=['POST'])
        def update_param(name):
            try:
                data = request.get_json()
                if 'value' not in data:
                    return jsonify({"error": "Missing parameter value"}), 400

                self.param_manager.set(name, data['value'])
                param = self.param_manager.get_param(name)
                return jsonify(param.to_dict())
            except KeyError:
                return jsonify({"error": f"Parameter {name} does not exist"}), 404
            except ValueError as e:
                return jsonify({"error": str(e)}), 400

    def start(self, debug=False, use_reloader=False):
        """
        Start the server

        Args:
            debug (bool): Whether to enable debug mode
            use_reloader (bool): Whether to enable auto-reload
        """
        if self.server_thread and self.server_thread.is_alive():
            print("Server is already running")
            return

        def run_server():
            self.app.run(host=self.host, port=self.port,
                         debug=debug, use_reloader=use_reloader)

        if debug and use_reloader:
            # Run directly in debug mode without threading
            run_server()
        else:
            # Run in a thread in non-debug mode
            self.server_thread = threading.Thread(target=run_server)
            self.server_thread.daemon = True
            self.server_thread.start()
            print(
                f"Parameter server started, visit http://{self.host}:{self.port} to view interface")

    def stop(self):
        """Stop the server (Note: This method may not be reliable in multi-threaded environments)"""
        # Flask doesn't provide a graceful stop method, this is just an interface
        if self.server_thread and self.server_thread.is_alive():
            print("Attempting to stop server...")
            # Actually cannot reliably stop Flask server, need to handle termination signal in main program
            # or use other methods like werkzeug.serving.make_server
