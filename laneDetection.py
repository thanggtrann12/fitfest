import cv2
import numpy as np
from pid import PIDController

# Open the video stream
cap = cv2.VideoCapture('write_video_2.avi')

# Initialize the PID controller
pid = PIDController(Kp=0.1, Ki=0.01, Kd=0.01, setpoint=0)

while True:
    # Read a frame from the video stream
    ret, frame = cap.read()

    # Get the height and width of the frame
    height, width = frame.shape[:2]

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to the grayscale image
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection to the blurred image
    edges = cv2.Canny(blur, 50, 150)

    # Define the ROI
    roi = edges[int(height/2):height, :]

    # Display the ROI
    cv2.rectangle(frame, (0, int(height/2)),
                  (width-1, height-1), (0, 0, 255), 2)

    # Detect the lane lines in the ROI using the Hough transform
    lines = cv2.HoughLinesP(roi, 1, np.pi/180, 100,
                            minLineLength=100, maxLineGap=50)

    if lines is not None:
        # Calculate the slope and intercept of each line
        slopes = []
        intercepts = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)
            slopes.append(slope)
            intercepts.append(intercept)

        # Calculate the average slope and intercept of the lane lines
        avg_slope = np.mean(slopes)
        avg_intercept = np.mean(intercepts)

        # Calculate the x-coordinate of the lane lines at the bottom of the frame
        y1 = height - 1
        x1 = int((y1 - avg_intercept) / avg_slope)
        y2 = int(height / 2) + 50
        x2 = int((y2 - avg_intercept) / avg_slope)

        # Calculate the center of the lane within the ROI
        midpoint = width / 2
        lane_center = int((x1 + x2) / 2)
        deviation = lane_center - midpoint

        # Calculate the distance from the center of the lane
        pixels_per_cm = 20  # Example value, replace with your own value

# Calculate the distance from the center of the lane in cm
        if deviation < 0:
            distance = abs(deviation) / pixels_per_cm
            message = "Distance from center to left is {:.2f} cm".format(
                distance)
            if distance < 30:
                print("turn right")
            else:
                print("straight")
        else:
            distance = deviation / pixels_per_cm
            message = "Distance from center to right is {:.2f} cm".format(
                distance)
            if distance < 30:
                print("turn left")
            else:
                print("straight")
    # Calculate the angle for steering based on the deviation
        steering_angle = deviation * 0.1
        pidval = pid.update(steering_angle)

        # Draw the lane lines and the center line on the frame
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.line(frame, (int(midpoint), height-1),
                 (lane_center, y2), (0, 0, 255), 3)
        cv2.putText(frame, message, (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        cv2.imshow('Lane Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
