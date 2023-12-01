import cv2
import numpy as np

def find_blue_center(image_path):
    # Read the image
    image = cv2.imread(image_path)

    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the blue color in HSV
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Threshold the image to get only blue pixels
    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    # Find contours in the binary image
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        print("No blue region found in the image.")
        return None

    # Find the largest contour (assuming only one continuous region exists)
    largest_contour = max(contours, key=cv2.contourArea)

    # Calculate the center of the contour
    moments = cv2.moments(largest_contour)
    center_x = int(moments["m10"] / moments["m00"])
    center_y = int(moments["m01"] / moments["m00"])

    return center_x, center_y

# Example usage:
image_path = "../test/blue_spot.png"
center = find_blue_center(image_path)

if center is not None:
    print(f"Center of the blue region: ({center[0]}, {center[1]})")
