import cv2
import dlib
from scipy.spatial import distance
import serial
import numpy as np

# Constants
EYE_AR_THRESHOLD = 0.3  # Eye aspect ratio to indicate drowsiness
MOUTH_AR_THRESHOLD = 0.5  # Mouth aspect ratio to indicate fatigue
EYE_AR_CONSEC_FRAMES = 48  # Number of consecutive frames the eye must be below the threshold to trigger an alert
MOUTH_AR_CONSEC_FRAMES = 48  # Number of consecutive frames the mouth must be above the threshold to trigger an alert

# Initialize dlib's face detector and facial landmark predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("c:\\Users\\Tino\\Desktop\\FINAL YEAR PROJECT\\shape_predictor_68_face_landmarks.dat")  # Path to the facial landmark predictor

# Calculate the eye aspect ratio (EAR) given the eye landmarks
def eye_aspect_ratio(eye):
    # Compute the euclidean distances between the two sets of vertical eye landmarks (x, y)-coordinates
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])

    # Compute the euclidean distance between the horizontal eye landmark (x, y)-coordinates
    C = distance.euclidean(eye[0], eye[3])

    # Compute the eye aspect ratio
    ear = (A + B) / (2.0 * C)

    return ear

# Calculate the mouth aspect ratio (MAR) given the mouth landmarks
def mouth_aspect_ratio(mouth):
    # Compute the euclidean distances between the vertical mouth landmarks (x, y)-coordinates
    A = distance.euclidean(mouth[2], mouth[10])
    B = distance.euclidean(mouth[4], mouth[8])

    # Compute the euclidean distance between the horizontal mouth landmarks (x, y)-coordinates
    C = distance.euclidean(mouth[0], mouth[6])

    # Compute the mouth aspect ratio
    mar = (A + B) / (2.0 * C)

    return mar

# Load Haar cascade classifier for face detection
face_cascade = cv2.CascadeClassifier("C:\\Users\\Tino\\Desktop\\FINAL YEAR PROJECT\\haarcascade_frontalface_default.xml")

# Open the video stream
video_capture = cv2.VideoCapture(0)

# Initialize variables
frames_counter = 0
eye_drowsy_counter = 0
mouth_drowsy_counter = 0
is_eye_drowsy = False
is_mouth_drowsy = False

# Initialize serial communication with Arduino
ser = serial.Serial('COM3', 9600)  # Update the COM port according to your system

while True:
    # Read the current frame from the video stream
    ret, frame = video_capture.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale frame using Haar cascade
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Loop over the faces detected
    for (x, y, w, h) in faces:
        # Convert the face region to dlib rectangle
        rect = dlib.rectangle(int(x), int(y), int(x + w), int(y + h))

        # Detect facial landmarks in the face region
        shape = predictor(gray, rect)

        # Extract the eye landmarks for calculating eye aspect ratio (EAR)
        left_eye = []
        right_eye = []

        for i in range(36, 42):  # Left eye landmarks (from 36 to 41)
            left_eye.append((shape.part(i).x, shape.part(i).y))
        for i in range(42, 48):  # Right eye landmarks (from 42 to 47)
            right_eye.append((shape.part(i).x, shape.part(i).y))

        # Extract the mouth landmarks for calculating mouth aspect ratio (MAR)
        mouth = []

        for i in range(48, 68):  # Mouth landmarks (from 48 to 67)
            mouth.append((shape.part(i).x, shape.part(i).y))
        # Calculate the eye aspect ratio (EAR) for each eye
        left_ear = eye_aspect_ratio(left_eye)
        right_ear = eye_aspect_ratio(right_eye)

        # Average the eye aspect ratio for both eyes
        ear = (left_ear + right_ear) / 2.0

        # Check if the driver is drowsy based on eye aspect ratio (EAR)
        if ear < EYE_AR_THRESHOLD:
            eye_drowsy_counter += 1
            if eye_drowsy_counter >= EYE_AR_CONSEC_FRAMES:
                is_eye_drowsy = True
                ser.write(b'D')  # Send a signal to Arduino to trigger an alert
        else:
            eye_drowsy_counter = 0
            is_eye_drowsy = False

        # Calculate the mouth aspect ratio (MAR)
        mar = mouth_aspect_ratio(mouth)

        # Check if the driver is fatigued based on mouth aspect ratio (MAR)
        if mar > MOUTH_AR_THRESHOLD:
            mouth_drowsy_counter += 1
            if mouth_drowsy_counter >= MOUTH_AR_CONSEC_FRAMES:
                is_mouth_drowsy = True
                ser.write(b'D')  # Send a signal to Arduino to trigger an alert
        else:
            mouth_drowsy_counter = 0
            is_mouth_drowsy = False

        # Draw the eye landmarks on the frame
        for (x, y) in left_eye:
            cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
        for (x, y) in right_eye:
            cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)

        # Draw the mouth landmarks on the frame
        for (x, y) in mouth:
            cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)

        # Display the eye aspect ratio (EAR) and mouth aspect ratio (MAR) on the frame
        cv2.putText(frame, f"EAR: {ear:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, f"MAR: {mar:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display the driver's state on the frame
        if is_eye_drowsy:
            cv2.putText(frame, "Drowsy Eyes", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if is_mouth_drowsy:
            cv2.putText(frame, "Yawning", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow("Driver Drowsiness Detection", frame)

    # Check for the key press
    key = cv2.waitKey(1) & 0xFF

    # Break the loop if 'q' is pressed
    if key == ord("q"):
        break

# Release the video stream and close all windows
video_capture.release()
cv2.destroyAllWindows()