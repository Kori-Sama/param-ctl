# -*- coding: utf-8 -*-
"""
PID Controller Example

This example demonstrates how to use the parameter remote controller in a PID control system.
You can adjust PID parameters in real-time through the Web interface.
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
    """Simple PID controller implementation"""
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional coefficient
        self.ki = ki  # Integral coefficient
        self.kd = kd  # Derivative coefficient
        self.prev_error = 0  # Previous error
        self.integral = 0    # Integral term
        
    def update(self, setpoint, measured_value, dt):
        """Update PID controller
        
        Args:
            setpoint: Target value
            measured_value: Current measured value
            dt: Time interval
            
        Returns:
            Control output value
        """
        # Calculate error
        error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        # Calculate output
        output = p_term + i_term + d_term
        
        return output

class SimpleSystem:
    """Simple system model for simulating control process"""
    def __init__(self, initial_value=0, inertia=0.8, noise_level=0.1):
        self.value = initial_value
        self.inertia = inertia  # System inertia
        self.noise_level = noise_level  # Noise level
        
    def update(self, control_input, dt):
        """Update system state
        
        Args:
            control_input: Control input
            dt: Time interval
            
        Returns:
            New system state
        """
        # Add some inertia and random noise
        self.value = self.inertia * self.value + (1 - self.inertia) * control_input
        self.value += np.random.normal(0, self.noise_level)
        return self.value

def main():
    # Create parameter manager
    pm = ParamManager()
    
    # Register PID parameters
    pm.register("kp", 1.0, float, "Proportional coefficient", (0, 5))
    pm.register("ki", 0.1, float, "Integral coefficient", (0, 1))
    pm.register("kd", 0.5, float, "Derivative coefficient", (0, 2))
    pm.register("setpoint", 10.0, float, "Target value", (0, 20))
    pm.register("system_inertia", 0.8, float, "System inertia", (0, 0.99))
    pm.register("noise_level", 0.2, float, "Noise level", (0, 1))
    
    # Start parameter server
    server = ParamServer(pm, host="127.0.0.1", port=8080)
    server.start()
    print("Parameter server started, visit http://127.0.0.1:8080 to adjust parameters")
    
    # Create system and controller
    system = SimpleSystem(initial_value=0, 
                         inertia=pm.get("system_inertia"),
                         noise_level=pm.get("noise_level"))
    
    pid = PIDController(pm.get("kp"), pm.get("ki"), pm.get("kd"))
    
    # Setup plot
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_ylim(0, 20)
    ax.set_xlim(0, 100)
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.set_title('PID Controller Simulation')
    ax.grid(True)
    
    # Data storage
    time_data = []
    setpoint_data = []
    value_data = []
    control_data = []
    
    # Create lines
    setpoint_line, = ax.plot([], [], 'r-', label='Setpoint')
    value_line, = ax.plot([], [], 'b-', label='System value')
    control_line, = ax.plot([], [], 'g-', label='Control output', alpha=0.5)
    
    ax.legend()
    
    # Update function
    def update_plot(frame):
        # Get latest parameters
        pid.kp = pm.get("kp")
        pid.ki = pm.get("ki")
        pid.kd = pm.get("kd")
        setpoint = pm.get("setpoint")
        system.inertia = pm.get("system_inertia")
        system.noise_level = pm.get("noise_level")
        
        # Time interval
        dt = 0.1
        
        # Calculate control output
        control_output = pid.update(setpoint, system.value, dt)
        
        # Update system
        system_value = system.update(control_output, dt)
        
        # Update data
        time_data.append(frame)
        setpoint_data.append(setpoint)
        value_data.append(system_value)
        control_data.append(control_output)
        
        # Keep only the last 100 data points
        if len(time_data) > 100:
            time_data.pop(0)
            setpoint_data.pop(0)
            value_data.pop(0)
            control_data.pop(0)
        
        # Update plot
        setpoint_line.set_data(time_data, setpoint_data)
        value_line.set_data(time_data, value_data)
        control_line.set_data(time_data, control_data)
        
        ax.set_xlim(max(0, frame - 100), max(100, frame))
        
        return setpoint_line, value_line, control_line
    
    # Create animation
    ani = FuncAnimation(fig, update_plot, frames=range(10000), 
                        interval=100, blit=True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()