import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Set up Pi Camera
cap = cv2.VideoCapture(0)

# Set up GPIO for servo motor control
servo_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
servo = GPIO.PWM(servo_pin, 50)  # 50Hz frequency
servo.start(0)

# Initialize previous frame
ret, prev_frame = cap.read()
prev_frame = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
prev_frame = cv2.GaussianBlur(prev_frame, (21, 21), 0)

def move_servo(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin, True)
    servo.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin, False)
    servo.ChangeDutyCycle(0)

try:
    while True:
        # Capture current frame
        ret, current_frame = cap.read()
        if not ret:
            break

        # Convert to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # Compute absolute difference between current frame and previous frame
        frame_diff = cv2.absdiff(prev_frame, gray)
        _, thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < 500:
                continue
            (x, y, w, h) = cv2.boundingRect(contour)
            cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Move servo motor based on the detected motion (Example: turning to 90 degrees)
            move_servo(90)
            time.sleep(1)  # Pause to observe the movement
            move_servo(0)  # Reset position

        # Display the resulting frame
        cv2.imshow('Security Feed', current_frame)

        # Update the previous frame
        prev_frame = gray.copy()

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    servo.stop()
