import cv2
import numpy as np

def compute_marker_offset(image, aruco_dict_type=cv2.aruco.DICT_4X4_50):
    # Convert to grayscale if needed
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()

    # Load ArUco dictionary and detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        print("No markers detected.")
        return None

    # Assume single marker
    corners = corners[0][0]  # Shape: (4, 2)
    center_x = np.mean(corners[:, 0])
    center_y = np.mean(corners[:, 1])
    marker_center = np.array([center_x, center_y])

    # Image center
    h, w = gray.shape
    image_center = np.array([w / 2, h / 2])

    # Offset: positive if marker is right/down of center
    offset = marker_center - image_center

    return {
        "marker_center": marker_center,
        "image_center": image_center,
        "offset": offset
    }

if __name__ == "__main__":
    image = cv2.imread("offcenter_aruco.png")
    result = compute_marker_offset(image)

    if result:
        print("Marker Center (px):", result["marker_center"])
        print("Image Center (px):", result["image_center"])
        print("Offset (dx, dy):", result["offset"])

        # Visualization (optional)
        output = image.copy()
        cv2.circle(output, tuple(result["marker_center"].astype(int)), 5, (0, 0, 255), -1)
        cv2.circle(output, tuple(result["image_center"].astype(int)), 5, (255, 0, 0), -1)
        cv2.line(output,
                 tuple(result["image_center"].astype(int)),
                 tuple(result["marker_center"].astype(int)),
                 (0, 255, 0), 2)
        cv2.imshow("Marker Offset", output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
