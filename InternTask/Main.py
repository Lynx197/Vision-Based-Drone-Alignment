import numpy as np
import matplotlib.pyplot as plt
from dynamics import Drone2D  
import matplotlib.pyplot as plt
from Image import compute_marker_offset
from Controller import DronePIDController
from generate import generate_offcenter_aruco
import cv2

if __name__ == "__main__":

    img = generate_offcenter_aruco(offset=(40, 30))  # Right and Up
    cv2.imwrite("offcenter_aruco.png", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    # Load ArUco image and compute offset
    image = cv2.imread("offcenter_aruco.png")
    result = compute_marker_offset(image)
    offset = result["offset"]  # This is the initial (dx, dy)
    print(offset)

    # Initialize drone model and controller
    drone = Drone2D(initial_pos=offset)
    controller = DronePIDController(kp=0.08, ki=0.00005, kd=0.01, dt=0.01)

    # Simulate
    dt = 0.01
    positions = []
    while(1):
        if(np.linalg.norm(offset)<0.5):
            break
        control = controller.compute_control(offset)
        drone.apply_control(control, dt)
        positions.append(drone.get_position())

        # Simulate updated offset (as if updated via image detection)
        pos = drone.get_position()
        print(pos)
        offset = pos - np.array([0.0, 0.0])  # image center at (0,0)

    # Plot
    positions = np.array(positions)
    plt.plot(positions[:, 0], positions[:, 1], label="Drone Path")
    plt.scatter([0], [0], color='red', label='Image Center')
    plt.title("PID-Controlled Drone Alignment")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid()
    plt.legend()
    plt.axis('equal')
    plt.show()
