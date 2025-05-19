# Drone PID Control with Image-Based Correction

This project implements a **PID-controlled drone alignment system** using **image feedback** to correct for drift and maintain position alignment. Developed in **Python** (alternative to MATLAB Simulink), it uses **ArUco marker tracking** to simulate the drone's position in 2D space and applies PID control to minimize error from the image center.

## Project Objectives

- Simulate a 2D drone with drift.
- Detect and track an ArUco marker as a proxy for the drone.
- Compute position error relative to image center.
- Apply a PID controller to correct drift using image feedback.

##  Features

- 2D drone dynamics with drift model
- ArUco marker detection using OpenCV
- Real-time offset computation from image center
- Clean, modular PID controller in Python

## Project Structure
.
├── dynamics.py            # Simulates drone motion and drift
├── Controller.py          # Contains the PID controller logic
├── Image.py               # ArUco marker detection & offset computation
├── main.py                # Integrates all components
├── generate.py            # Generates ArUco markers with offsets
├── offcenter_aruco.png    # Generated ArUco marker image with offset



## How It Works

1. **ArUco Marker Detection**  
   Uses OpenCV to detect the marker position in the image.

2. **Offset Calculation**  
   Computes the position offset between marker center and image center.

3. **PID Control**  
   Converts offset into velocity corrections to re-align the drone.

4. **Simulation**  
   Drone movement is simulated over time and plotted with matplotlib.

## Requirements

Install all dependencies:

pip install numpy opencv-contrib-python matplotlib simple-pid

## Running the Simulation

python Main.py

Make sure your working directory includes:

A valid image with an ArUco marker

