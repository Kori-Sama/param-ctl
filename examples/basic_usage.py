# -*- coding: utf-8 -*-
"""
Basic usage example of parameter remote controller

This example demonstrates the basic usage of the parameter remote controller
"""

import sys
import os
import time

# Add parent directory to path to import param_ctl package
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from param_ctl import ParamManager, ParamServer

def main():
    # Create parameter manager
    pm = ParamManager()
    
    # Register various types of parameters
    pm.register("int_param", 10, int, "Integer parameter example", (0, 100))
    pm.register("float_param", 3.14, float, "Float parameter example", (0, 10))
    pm.register("bool_param", True, bool, "Boolean parameter example")
    pm.register("str_param", "hello", str, "String parameter example")
    
    # Start parameter server
    server = ParamServer(pm, host="127.0.0.1", port=8080)
    server.start()
    print("Parameter server started, visit http://127.0.0.1:8080 to adjust parameters")
    
    try:
        # Main loop
        while True:
            # Get parameter values
            int_val = pm.get("int_param")
            float_val = pm.get("float_param")
            bool_val = pm.get("bool_param")
            str_val = pm.get("str_param")
            
            # Print current parameter values
            print("\nCurrent parameter values:")
            print(f"int_param = {int_val}")
            print(f"float_param = {float_val}")
            print(f"bool_param = {bool_val}")
            print(f"str_param = {str_val}")
            
            # Wait for a while
            print("\nWaiting for 5 seconds... You can modify parameters in the browser")
            time.sleep(5)
            
    except KeyboardInterrupt:
        print("\nProgram exited")

if __name__ == "__main__":
    main()