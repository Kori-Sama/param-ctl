# -*- coding: utf-8 -*-
"""
PID Controller Example with param_ctl

This example demonstrates a simple PID controller implementation with
real-time parameter adjustment using param_ctl
"""

import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Add parent directory to path to import param_ctl package
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from param_ctl import ParamManager, ParamServer

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
    
    def compute(self, process_variable):
        """Compute PID control output"""
        # Calculate time delta
        now = time.time()
        dt = now - self.last_time
        
        # Calculate error
        error = self.setpoint - process_variable
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.error_sum += error * dt
        i_term = self.ki * self.error_sum
        
        # Derivative term
        if dt > 0:
            d_term = self.kd * (error - self.last_error) / dt
        else:
            d_term = 0.0
        
        # Save current error and time for next iteration
        self.last_error = error
        self.last_time = now
        
        # Calculate output
        output = p_term + i_term + d_term
        return output

class SimpleSystem:
    """A simple dynamic system to control"""
    def __init__(self, initial_value=0.0, inertia=0.95, noise_level=0.1):
        self.value = initial_value
        self.inertia = inertia
        self.noise_level = noise_level
    
    def update(self, control_input):
        # System dynamics: current value has inertia, input affects the value with some noise
        self.value = self.inertia * self.value + (1 - self.inertia) * control_input
        self.value += np.random.normal(0, self.noise_level)
        return self.value

def main():
    # Create parameter manager
    pm = ParamManager()
    
    # Register PID parameters
    pm.register("kp", 1.0, float, "Proportional gain", (0.0, 10.0))
    pm.register("ki", 0.1, float, "Integral gain", (0.0, 5.0))
    pm.register("kd", 0.05, float, "Derivative gain", (0.0, 2.0))
    pm.register("setpoint", 5.0, float, "Desired target value", (0.0, 10.0))
    
    # Register system parameters
    pm.register("system_inertia", 0.95, float, "System inertia (0-1)", (0.5, 0.99))
    pm.register("noise_level", 0.1, float, "System noise level", (0.0, 1.0))
    
    # Start parameter server
    server = ParamServer(pm, host="127.0.0.1", port=8080)
    server.start()
    print("Parameter server started, visit http://127.0.0.1:8080 to adjust parameters")
    
    # Initialize PID controller and system
    pid = PIDController(
        kp=pm.get("kp"),
        ki=pm.get("ki"),
        kd=pm.get("kd"),
        setpoint=pm.get("setpoint")
    )
    system = SimpleSystem(
        initial_value=0.0,
        inertia=pm.get("system_inertia"),
        noise_level=pm.get("noise_level")
    )
    
    # Data for plotting
    max_data_points = 100
    times = []
    setpoints = []
    system_values = []
    control_signals = []
    
    # Set up the plot
    plt.figure(figsize=(12, 6))
    plt.ion()  # Interactive mode on
    
    try:
        # Control loop
        start_time = time.time()
        
        while True:
            # Update parameters from param_ctl
            pid.kp = pm.get("kp")
            pid.ki = pm.get("ki")
            pid.kd = pm.get("kd")
            pid.setpoint = pm.get("setpoint")
            system.inertia = pm.get("system_inertia")
            system.noise_level = pm.get("noise_level")
            
            # Compute control signal
            control_signal = pid.compute(system.value)
            
            # Update system
            system_value = system.update(control_signal)
            
            # Store data for plotting
            current_time = time.time() - start_time
            times.append(current_time)
            setpoints.append(pid.setpoint)
            system_values.append(system_value)
            control_signals.append(control_signal)
            
            # Keep only the most recent data points
            if len(times) > max_data_points:
                times.pop(0)
                setpoints.pop(0)
                system_values.pop(0)
                control_signals.pop(0)
            
            # Update plot
            plt.clf()
            plt.subplot(2, 1, 1)
            plt.plot(times, setpoints, 'r--', label='Setpoint')
            plt.plot(times, system_values, 'b-', label='System Value')
            plt.ylabel('Value')
            plt.legend()
            plt.title(f'PID Control (kp={pid.kp:.2f}, ki={pid.ki:.2f}, kd={pid.kd:.2f})')
            
            plt.subplot(2, 1, 2)
            plt.plot(times, control_signals, 'g-', label='Control Signal')
            plt.xlabel('Time (s)')
            plt.ylabel('Control Signal')
            plt.legend()
            
            plt.pause(0.1)
            
            # Print current status
            print(f"\rTime: {current_time:.1f}s | Setpoint: {pid.setpoint:.2f} | Value: {system_value:.2f} | Error: {pid.setpoint - system_value:.2f} | Control: {control_signal:.2f}", end="")
            
            # Small delay for simulation purposes
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        plt.ioff()
        print("\nProgram exited")

if __name__ == "__main__":
    main()