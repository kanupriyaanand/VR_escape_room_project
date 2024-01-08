# import cv2
# import mediapipe as mp
# import socket

# # Create a socket object
# client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# # Connect to the Unity server's address and port
# host, port = "127.0.0.1", 25001
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect((host, port))

# # Initialize Mediapipe components
# mp_drawing = mp.solutions.drawing_utils
# mp_pose = mp.solutions.pose
# pose = mp_pose.Pose()

# # Previous nose position for jump detection
# prev_nose_y = 0.0

# # Function to detect jump


# def detect_jump(nose_landmark):
#     global prev_nose_y

#     # Set a threshold for jump detection
#     jump_threshold = 0.2

#     # Check if the vertical position of the nose has increased significantly
#     if nose_landmark.y - prev_nose_y > jump_threshold:
#         prev_nose_y = nose_landmark.y
#         return True
#     else:
#         prev_nose_y = nose_landmark.y
#         return False


# # Open a video capture object (0 is the default camera)
# cap = cv2.VideoCapture(0)

# # Main loop for pose detection
# with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
#     while cap.isOpened():
#         # Read a frame from the camera
#         ret, frame = cap.read()
#         if not ret:
#             break

#         # Convert the frame to RGB
#         rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#         # Process the frame with mediapipe
#         results = pose.process(rgb_frame)

#         # Extract nose landmark
#         if results.pose_landmarks:
#             nose_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]

#             # Detect jump
#             if detect_jump(nose_landmark):
#                 print("Jump detected!")

#                 # Send a message to Unity
#                 message_to_send = "Jump detected!"
#                 sock.sendall(message_to_send.encode("UTF-8"))
#         # receiveing data in Byte fron C#, and converting it to String
#                 receivedData = sock.recv(1024).decode("UTF-8")
#                 # client_socket.send(message_to_send.encode())

#         # Draw landmarks on the frame
#         mp_drawing.draw_landmarks(
#             frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

#         # Display the frame
#         cv2.imshow('MediaPipe Pose', frame)

#         # Exit the loop when 'q' is pressed
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# # Release the video capture object and close the socket
# cap.release()
# client_socket.close()
# cv2.destroyAllWindows()
import socket

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server's address and port
host = '127.0.0.1'
port = 12345
client_socket.connect((host, port))

# Receive and print messages from the server
while True:
    data_received = client_socket.recv(1024).decode()
    if not data_received:
        break

    print("Received from server:", data_received)

# Close the connection
client_socket.close()