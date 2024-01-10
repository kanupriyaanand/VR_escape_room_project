

import numpy as np
import cv2
import mediapipe as mp
import math

import time
import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific address and port
host = '127.0.0.1'
port = 12345
server_socket.connect((host, port))
# server_socket.bind((host, port))
flag_start = 1

# # Listen for incoming connections
# server_socket.listen()


# Accept a connection from a client
# client_socket, client_address = server_socket.accept()
# print(f"Connection from {client_address}")

dataString = str("0")
# proper
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Set the initial side for the first ball to be drawn on
full_body = False
cc = 0
flag2 = 0
flag_j = 0
flag_k = 0
total_distance1 = 0
total_distance2 = 0
t1 = 317
t2 = 300
flag_k = 0
flag1 = 0
initial_x = 0
initial_y = 0
full_body_frame_count = 0
full_body_height_dis = []
full_body_height = None
start_game = False
ball_spawned = False
ball_wrist_collision = False
counter = 0
stage = None
turn = 0
c = 0
k = 0
pixel_distance = 0
prev_x_pixel = 0
prev_y_pixel = 0
state = "standing"
wrist_pos = None
prev_right_ankle = None
prev_left_ankle = None
# state = "sleeping"
# Initialize the video capture device
cap = cv2.VideoCapture(0)

# Set the ball size and color

ball_color_list = [(0, 255, 0), (255, 255, 0)]
ball_color = ball_color_list[1]

# Set the initial side for the first ball to be drawn on
current_side = 'right'

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
ball_size = int(width*0.05)
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

height_range = int(height*0.2)
widht_range = int(0.2*width)

left_xmin = int(width - widht_range)
ball_x = None
ball_y = None
right_tibia_dis = []
right_tibia = None
start_running = False
last_pos = None
last_pos1 = None
back_last_pos = None


# Set up progress bar parameters
progress_bar_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)*0.8)
progress_bar_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)*0.1)
progress_bar_pos = (50, 50)
progress_bar_color = (0, 255, 0)
frame_count = 0
progress_bar_frame_reset = 2
progress_bar_fill = 0


def get_nose_position():
    results = pose.process(frame_rgb)
    if results.pose_landmarks:
        nose_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ANKLE.value]
        x_pixel = int(nose_landmark.x * frame.shape[1])
        y_pixel = int(nose_landmark.y * frame.shape[0])
        return x_pixel, y_pixel
    return None, None


def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    return int(math.sqrt(dx*dx + dy*dy))


def calculate_angle(a, b, c):
    a = np.array(a)  # first
    b = np.array(b)  # mid
    c = np.array(c)  # last
    # y from endpoint to y to midpoint, x variables of 2nd and 3rd
    # a[1] = y value of shoulder , b[1] = val from midooint to elbow
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - \
        np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians*180.0/np.pi)

    if angle > 180.0:
        angle = 360-angle
    return angle


def start_lift_test():
    global start_time, state, pixel_distance, prev_x_pixel, prev_y_pixel
    start_time = time.time()
    landmarks = results.pose_landmarks.landmark
    lefthip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,
               landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
    leftshoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                    landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
    leftknee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x,
                landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y]
   


cap = cv2.VideoCapture(0)
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while True:
        # Capture the frame from the video capture device
        ret, frame = cap.read()

        frame = cv2.flip(frame, 1)

        # Get the height and width of the frame
        height, width, _ = frame.shape

        # Convert the BGR image to RGB.
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = frame_rgb
        image.flags.writeable = False
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Make detections for both poses on their respective frames.
        results = pose.process(frame_rgb)

        pose_landmarks = []
        pose_landmark_confs = []

        # Draw the pose on the frames if there's at least one pose detected.
        if results.pose_landmarks is not None:
            # mp_drawing.draw_landmarks(frame1, results1.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            for landmark in results.pose_landmarks.landmark:
                pose_landmarks.append(
                    [landmark.x * frame.shape[1], landmark.y * frame.shape[0]])
                pose_landmark_confs.append(landmark.visibility)

        # for i, landmark in enumerate(pose_landmarks):
        #     conf = pose_landmark_confs[i]
        #     if conf > 0.5:
        #         if current_side == 'right' and i == 15:
        #             wrist_pos = (int(landmark[0]), int(landmark[1]))
        #             cv2.circle(frame, (int(landmark[0]), int(landmark[1])), ball_size, ball_color, -1)
        #         if current_side == 'left' and i == 16:
        #             wrist_pos = (int(landmark[0]), int(landmark[1]))
        #             cv2.circle(frame, (int(landmark[0]), int(landmark[1])), ball_size, ball_color, -1)

        point_count = 0
        for conf in pose_landmark_confs:
            if conf > 0.5:
                point_count = point_count + 1

        if point_count >= 32:
            full_body_frame_count = full_body_frame_count + 1
            if full_body_frame_count < 30:
                cv2.putText(frame, "Full Body Visible, Stand straight", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            if full_body_frame_count >= 30 and full_body_frame_count < 100:
                cv2.putText(frame, "Starting game...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                # calculate distances here
                right_tibia_dis.append(
                    distance(pose_landmarks[26], pose_landmarks[28]))

            if full_body_frame_count == 100:
                right_tibia = int(sum(right_tibia_dis) / len(right_tibia_dis))
                right_tibia = int(0.7*right_tibia)
                # print(right_tibia)
                start_running = True
                start_game = True

        if start_running:
            cv2.putText(frame, "Start Running ", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cur_pos = pose_landmarks[28]
            cur_pos1 = pose_landmarks[27]
            back_cur_pos= pose_landmarks[16]
            if last_pos is not None and last_pos1 is not None and back_last_pos is not None:
                ankle_dist = distance(cur_pos, last_pos)
                ankle_dist1 = distance(cur_pos1, last_pos1)
                wrist_dist = distance(back_cur_pos, back_last_pos)
                print("backward wrist movement", wrist_dist)
                print(ankle_dist)
                print("other ankle dist", ankle_dist1)
                if wrist_dist > 30:
                    print("backward motion")
                    dataString = str(500)
                    print("sent", dataString)
                    server_socket.sendall(dataString.encode("UTF-8"))
                print(ankle_dist)
                print("other ankle dist", ankle_dist1)
                if ankle_dist == 0:
                    print("still")
                    dataString = str(0)
                    print("sent", dataString)
                    server_socket.sendall(dataString.encode("UTF-8"))
                if ankle_dist > 30 and ankle_dist < 50:
                    print("running detected")
                    dataString = str(ankle_dist)
                    print("sent", dataString)
                    server_socket.sendall(dataString.encode("UTF-8"))
                    # receiveing data in Byte fron C#, and converting it to String
                    # receivedData = server_socket.recv(
                    #     1024).decode("UTF-8")
                    # print(receivedData)
                if ankle_dist > 60 and ankle_dist1 > 60:
                    print("jump")
                    dataString = str(150)
                    print("sent jump", dataString)
                    server_socket.sendall(dataString.encode("UTF-8"))

                if int(frame_count % progress_bar_frame_reset) == 0:
                    progress_bar_fill = int(
                        min(ankle_dist / right_tibia, 1) * progress_bar_width)
                progress_bar = cv2.rectangle(
                    img=frame,
                    pt1=(progress_bar_pos[0], progress_bar_pos[1]),
                    pt2=(progress_bar_pos[0] + progress_bar_fill,
                         progress_bar_pos[1] + progress_bar_height),
                    color=progress_bar_color,
                    thickness=cv2.FILLED
                )

            last_pos = cur_pos
            last_pos1 = cur_pos1
            back_last_pos=back_cur_pos
            

            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                      mp_drawing.DrawingSpec(
                                          color=(245, 117, 66), thickness=2, circle_radius=2),
                                      mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))
        # Show the frame
        # cv2.imshow('Webcam Feed', frame)
        cv2.imshow('Webcam feed', image)

        key = cv2.waitKey(1) & 0xFF

        # Check for a key press to quit the program
        if key == ord('q'):
            break

# Release the video capture device and close all windows

server_socket.close()
cap.release()
cv2.destroyAllWindows()
# %%


