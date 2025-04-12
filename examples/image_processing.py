# -*- coding: utf-8 -*-
"""
Image Processing Example

This example demonstrates how to use the parameter remote controller in an image processing application.
You can adjust image processing parameters in real-time through the Web interface.
"""

import cv2
import numpy as np
import sys
import os

# Add parent directory to path to import param_ctl package
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from param_ctl import ParamManager, ParamServer

def main():
    # Create parameter manager
    pm = ParamManager()
    
    # Register image processing parameters
    pm.register("threshold", 128, int, "Binarization threshold", (0, 255))
    pm.register("blur_size", 5, int, "Gaussian blur kernel size", (1, 31))
    pm.register("canny_low", 50, int, "Canny edge detection low threshold", (0, 255))
    pm.register("canny_high", 150, int, "Canny edge detection high threshold", (0, 255))
    pm.register("show_original", True, bool, "Show original image")
    pm.register("show_binary", True, bool, "Show binary image")
    pm.register("show_edges", True, bool, "Show edge detection result")
    
    # Start parameter server
    server = ParamServer(pm, host="127.0.0.1", port=8080)
    server.start()
    print("Parameter server started, visit http://127.0.0.1:8080 to adjust parameters")
    
    # Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return
    
    try:
        while True:
            # Read a frame
            ret, frame = cap.read()
            if not ret:
                print("Cannot get frame")
                break
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Get parameter values
            threshold = pm.get("threshold")
            blur_size = pm.get("blur_size")
            canny_low = pm.get("canny_low")
            canny_high = pm.get("canny_high")
            show_original = pm.get("show_original")
            show_binary = pm.get("show_binary")
            show_edges = pm.get("show_edges")
            
            # Ensure blur_size is odd
            if blur_size % 2 == 0:
                blur_size += 1
            
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
            
            # Binarization
            _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)
            
            # Edge detection
            edges = cv2.Canny(blurred, canny_low, canny_high)
            
            # Show images
            if show_original:
                cv2.imshow("Original Image", frame)
            else:
                cv2.destroyWindow("Original Image")
                
            if show_binary:
                cv2.imshow("Binary Image", binary)
            else:
                cv2.destroyWindow("Binary Image")
                
            if show_edges:
                cv2.imshow("Edge Detection", edges)
            else:
                cv2.destroyWindow("Edge Detection")
            
            # Press ESC to exit
            if cv2.waitKey(1) == 27:
                break
    
    finally:
        # Release resources
        cap.release()
        cv2.destroyAllWindows()
        print("Program exited")

if __name__ == "__main__":
    main()