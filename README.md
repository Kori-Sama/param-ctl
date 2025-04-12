# Parameter Remote Controller

A lightweight parameter remote controller for modifying parameters remotely during embedded device debugging.

## Features

- Lightweight HTTP server suitable for running on embedded devices
- Clean web interface for visual parameter modification
- Flexible parameter definition mechanism supporting multiple parameter types
- Simple API interface for easy integration into actual programs

## Installation

```bash
pip install param-controller
```

## Usage

### 1. Define Parameters

```python
from param_ctl import ParamManager

# Create parameter manager
pm = ParamManager()

# Register parameters
pm.register("threshold", 128, int, "Image processing threshold", (0, 255))
pm.register("kp", 1.0, float, "PID proportional coefficient", (0, 10))
pm.register("ki", 0.1, float, "PID integral coefficient", (0, 1))
pm.register("kd", 0.5, float, "PID derivative coefficient", (0, 5))
```

### 2. Start Server

```python
from param_ctl import ParamServer

# Create and start parameter server
server = ParamServer(pm, host="0.0.0.0", port=8080)
server.start()
```

### 3. Use Parameters in Program

```python
# Get parameter value
threshold = pm.get("threshold")

# Use parameter
ret, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

# You can also use parameter objects directly
pid_controller = PID(pm.get("kp"), pm.get("ki"), pm.get("kd"))
```

### 4. Modify Parameters via Web Interface

Open your browser and visit `http://<device-ip>:8080` to access the parameter control interface.

## Examples

See the example code in the `examples` directory.
