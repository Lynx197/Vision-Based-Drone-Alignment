import cv2
import numpy as np

def generate_offcenter_aruco(marker_id=0,
                              aruco_dict_type=cv2.aruco.DICT_4X4_50,
                              marker_size=100,
                              image_size=(480, 640),
                              offset=(100, -50)):
    # Load predefined dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)

    # Allocate space for the marker
    marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
    cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_img, 1)

    # Create white background image
    image = np.ones(image_size, dtype=np.uint8) * 255

    # Compute placement position
    center_x = image_size[1] // 2 + offset[0]
    center_y = image_size[0] // 2 + offset[1]
    start_x = center_x - marker_size // 2
    start_y = center_y - marker_size // 2

    # Paste the marker into the image
    image[start_y:start_y + marker_size, start_x:start_x + marker_size] = marker_img

    return image

if __name__ == "__main__":
    img = generate_offcenter_aruco(offset=(80, -40))  # Right and up
    cv2.imshow("Off-Center ArUco Marker", img)
    cv2.imwrite("offcenter_aruco.png", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
