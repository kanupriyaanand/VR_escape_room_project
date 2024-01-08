
import numpy as np
import cv2
import mediapipe as mp
import math


import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific address and port
host = '127.0.0.1'
port = 12345
server_socket.connect((host, port))
# server_socket.bind((host, port))


# # Listen for incoming connections
# server_socket.listen()


# Accept a connection from a client
# client_socket, client_address = server_socket.accept()
# print(f"Connection from {client_address}")

dataString = str("0")
# proper
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
# engine = pyttsx3.init("sapi5")
# voice = engine.getProperty("voices")
# engine.setProperty('voice', voice[1].id)
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


def get_nose_position():
    results = pose.process(frame_rgb)
    if results.pose_landmarks:
        nose_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ANKLE.value]
        x_pixel = int(nose_landmark.x * frame.shape[1])
        y_pixel = int(nose_landmark.y * frame.shape[0])
        return x_pixel, y_pixel
    return None, None


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
    angle1 = calculate_angle(rightknee, righthip, rightshoulder)
    if angle1 >= 170:
        # speak(" u are standing up")
        pixel_distance = 0
        # speak("test started")
        prev_x_pixel, prev_y_pixel = get_nose_position()


# In[ ]:


# In[2]:

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

            if full_body_frame_count == 100:
                start_game = True

        if start_game:
            # situps
            try:
                if flag1 == 0:
                    # speak(" game started")
                    # speak(" stand")
                    flag1 = 1
                landmarks = results.pose_landmarks.landmark
                lefthip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,
                           landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
                leftshoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                                landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
                leftknee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x,
                            landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y]
                leftankle = [landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].x,
                             landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].y]
                righthip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x,
                            landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
                rightshoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,
                                 landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
                rightknee = [landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x,
                             landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]
                rightankle = [landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x,
                              landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y]
                leftelbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,
                             landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
                rightelbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,
                              landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
    #             righthip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
                leftwrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,
                             landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
                rightwrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,
                              landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
                nose = [landmarks[mp_pose.PoseLandmark.NOSE.value].x,
                        landmarks[mp_pose.PoseLandmark.NOSE.value].y]
                nose_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ANKLE.value]
                # leftheel = [landmarks[mp_pose.PoseLandmark.LEFT_HEEL.value].x, landmarks[mp_pose.PoseLandmark.LEFT_HEEL.value].y]
                # rightheel = [landmarks[mp_pose.PoseLandmark.RIGHT_HEEL.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HEEL.value].y]
                angle1 = calculate_angle(lefthip, leftknee, leftankle)
                angle2 = calculate_angle(leftknee, lefthip, leftshoulder)
                angle3 = calculate_angle(righthip, rightknee, rightankle)
                angle4 = calculate_angle(rightknee, righthip, rightshoulder)
                angle5 = calculate_angle(lefthip, leftshoulder, leftelbow)
                angle6 = calculate_angle(righthip, rightshoulder, rightelbow)
                angle7 = calculate_angle(rightshoulder, rightelbow, rightwrist)
                angle8 = calculate_angle(leftshoulder, leftelbow, leftwrist)
                angle9 = calculate_angle(rightelbow, rightwrist, nose)
                angle10 = calculate_angle(leftelbow, leftwrist, lefthip)

    #             print(landmarks)
                cv2.putText(image, str(angle1) + "knee",
                            tuple(np.multiply(
                                leftknee, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, str(angle2),
                            tuple(np.multiply(
                                lefthip, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, str(angle6),
                            tuple(np.multiply(
                                lefthip, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, str(angle3),
                            tuple(np.multiply(leftshoulder,
                                  [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, str(angle4),
                            tuple(np.multiply(leftelbow, [
                                  640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                if turn == 0:
                    # speak(" lift left leg leg 2cm ")
                    total_distance1 = 0
                    total_distance2 = 0
                    turn = 1
                    flag_j = 0
                # if  angle1>=160 and ( angle3>=165) and angle2<=160 and angle4>=120 and angle4>=165 and flag_i == 0 and flag_j==1:
                #     if prev_left_ankle is not None:

                #         # ankle_y = ankle.y * frame.shape[0]
                #         # if ankle_y:
                #         x1 = left_ankle.x * frame.shape[1]
                #         y1 = left_ankle.y * frame.shape[0]
                #         x2 = prev_left_ankle.x * frame.shape[1]
                #         y2 = prev_left_ankle.y * frame.shape[0]
                #         distance = math.sqrt((x2 - x1) * 2 + (y2 - y1) * 2)
                #         total_distance += distance
                #         print(total_distance)
                #         if total_distance >= 60:
                #             speak(" good, lift other leg ")
                #             turn = turn + 1
                #             flag_i = 1
                #             flag_j = 0
                #         else:
                #             # speak(" lift higher ")
                #             k+=1
                #             if k>=5:
                #                 speak("lift higher  ")
                #                 k = 0

                #     prev_left_ankle = left_ankle

                right_ankle = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ANKLE.value]
                left_ankle = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ANKLE.value]
                if left_ankle and right_ankle and flag_j == 0 and turn == 1 :
                    if prev_right_ankle is not None and prev_left_ankle is not None:

                        # ankle_y = ankle.y * frame.shape[0]
                        # if ankle_y:
                        x1 = right_ankle.x * frame.shape[1]
                        y1 = right_ankle.y * frame.shape[0]
                        x2 = prev_right_ankle.x * frame.shape[1]
                        y2 = prev_right_ankle.y * frame.shape[0]
                        r1 = left_ankle.x * frame.shape[1]
                        c1 = left_ankle.y * frame.shape[0]
                        r2 = prev_left_ankle.x * frame.shape[1]
                        c2 = prev_left_ankle.y * frame.shape[0]
                        distance = math.sqrt((x2 - x1) * 2 + (y2 - y1) * 2)
                        distance2 = math.sqrt((r2 - r1) * 2 + (c2 - c1) * 2)
                        total_distance1 += distance
                        total_distance2 += distance2
                        print(" total dist ", total_distance1)
                        if total_distance1 >= 10 and total_distance2 >= 10:
                            # speak(" good,  ")
                            print("jump detected")
                            dataString = str("6")
                            print("sent", dataString)
                            server_socket.sendall(dataString.encode("UTF-8"))
                            # receiveing data in Byte fron C#, and converting it to String
                            receivedData = server_socket.recv(
                                1024).decode("UTF-8")
                            #print(receivedData)
                            turn = 0
                            flag_i = 0
                            flag_j = 1
                        # else:
                            # speak(" lift higher ")

                            # c+=1
                            # if c>=5:
                            #     speak("lift higher  ")
                            #     c = 0

                    prev_right_ankle = right_ankle
                    prev_left_ankle = left_ankle

                # if flag_k == 0 and flag1 == 1:
                #     # speak("hi")
                #     results = pose.process(frame_rgb)
                #     if results.pose_landmarks:
                #         nose_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_KNEE.value]
                #         initial_x  = int(nose_landmark.x * frame.shape[1])
                #         initial_y =  int(nose_landmark.y * frame.shape[0])
                #         print(" inittttt " , initial_x)
                #         print("inittt y ", initial_y)
                #         # speak(" initial x is " , initial_x)
                #         flag_k = 1
                # # if (angle1>=170 or angle3>=20 or angle3<=90 or angle2>=165 or angle4<=90 or angle4>=20 )and flag2 == 0:
                # if flag2==0:
                # # if angle1>=170 and angle3>=20 and angle3<=90 and angle2>=165 and angle4<=90 and angle4>=45 and angle7<=90 and angle8<=90 and flag2 == 0:
                #     speak(" heloo")
                #     results = pose.process(frame_rgb)
                #     if results.pose_landmarks:
                #         nose_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_KNEE.value]
                #         x_pixel = int(nose_landmark.x * frame.shape[1])
                #         y_pixel = int(nose_landmark.y * frame.shape[0])
                #         print(" x pixel " , x_pixel)
                #         print(" y pixel ", y_pixel)
                #         if y_pixel<=t2:
                #             speak(" get down")
                #             flag_z = 1
                #             flag2 =1
                #         else:
                #             speak(" lift higher")
                # if flag_z==1:
                #     results = pose.process(frame_rgb)

                #     if results.pose_landmarks:
                #         nose_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_KNEE.value]
                #         x_pixel = int(nose_landmark.x * frame.shape[1])
                #         y_pixel = int(nose_landmark.y * frame.shape[0])
                #         print(" x_pixel after gwttign doqn is ", x_pixel)
                #         print(" y_pixel after gettin down is ", y_pixel)

                #         if x_pixel>=initial_x + 70  and y_pixel>=initial_y+70:
                #             speak(" hurdle crossed yay")

                # elif angle1<=150 and angle3<=150 and angle2>160 and angle4 >160 and angle7<150 and angle7>90 and angle8<150 and angle8>90:
                #     k+=1
                #     if k>=10:
                #         speak("wrong posture ")
                #         speak(" lift straight")
                #         k = 0

            except:
                pass
            # rectangle att the top left hand corner
            cv2.rectangle(image, (0, 0), (225, 73), (245, 117, 16), -1)
            # rep ka data
            cv2.putText(image, 'REPS', (15, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(image, str(counter), (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
            # stage ka data
            cv2.putText(image, 'STAGE', (65, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(image, stage, (60, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)

            # render the detections
            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                      mp_drawing.DrawingSpec(
                                          color=(245, 117, 66), thickness=2, circle_radius=2),
                                      mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))

            #print(results)

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

# %%
# sendData = 1

# dataString = str("0")


# def get_angle(a, b, c):
#     """
#     Returns the angle between three points in degrees.
#     """
#     radians = math.atan2(c[1]-b[1], c[0]-b[0]) - \
#         math.atan2(a[1]-b[1], a[0]-b[0])
#     return math.degrees(radians)


# mp_drawing = mp.solutions.drawing_utils
# mp_pose = mp.solutions.pose

# cap1 = cv2.VideoCapture(0)

# initial_landmarks = []
# ready = 0

# jump_detected = 0
# jump_reset = 0
# right_turn_detected = False
# right_turn_reset = True
# left_turn_detected = False
# left_turn_reset = True
# right_knee_detected = False
# right_knee_reset = True
# left_knee_detected = False
# left_knee_reset = True

# with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose1:
#     while True:
#         # Capture frame-by-frame from both webcams
#         ret1, frame1 = cap1.read()

#         if not (ret1):
#             break
#         # print("hi")

#         # dataString = str("0")

#         # Flip the image horizontally for a more intuitive mirror-view
#         frame1 = cv2.flip(frame1, 1)

#         # Convert the BGR image to RGB.
#         frame1_rgb = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)

#         # Set flag to draw the poses on the images.
#         draw_pose = True

#         # Make detections for both poses on their respective frames.
#         results1 = pose1.process(frame1_rgb)

#         pose_landmarks1 = []
#         pose_landmark_confs1 = []

#         # Draw the pose on the frames if there's at least one pose detected.
#         if results1.pose_landmarks is not None:
#             # mp_drawing.draw_landmarks(frame1, results1.pose_landmarks, mp_pose.POSE_CONNECTIONS)
#             for landmark in results1.pose_landmarks.landmark:
#                 pose_landmarks1.append(
#                     [landmark.x * frame1.shape[1], landmark.y * frame1.shape[0]])
#                 pose_landmark_confs1.append(landmark.visibility)

#         # if (all(conf > 0.5 for conf in pose_landmark_confs1) and ready == 0):
#         if (pose_landmark_confs1[0] > 0.5 and ready == 0):
#             initial_landmarks = pose_landmarks1
#             print("gi")
#             # print(pose_landmarks1[29], "   ", pose_landmarks1[30])
#             ready = 1
#         elif (ready == 1):
#             continue
#         print("init", initial_landmarks)

#         for i, landmark in enumerate(pose_landmarks1):
#             conf = pose_landmark_confs1[i]
#             if conf > 0.5:
#                 cv2.circle(frame1, (int(landmark[0]), int(
#                     landmark[1])), 5, (255, 255, 0), -1)
#             # if i in [23, 11, 13]:
#             #     cv2.circle(frame1, (int(landmark[0]), int(
#             #         landmark[1])), 5, (0, 0, 255), -1)
#             if i in [0]:
#                 cv2.circle(frame1, (int(landmark[0]), int(
#                     landmark[1])), 5, (0, 0, 255), -1)

#         if len(pose_landmarks1) >= 2 and len(pose_landmark_confs1) >= 2:
#             print(pose_landmarks1)

#             # if (pose_landmark_confs1[29] and pose_landmark_confs1[30]):
#             #     if (abs(pose_landmarks1[30][1] - initial_landmarks[30][1]) > 2 and abs(pose_landmarks1[29][1]-initial_landmarks[29][1]) > 2 and not jump_detected and jump_reset):
#             #         print("Jump detected")
#             #         jump_detected = True
#             #         dataString = str("6")
#             #         jump_reset = False
#             #     if (abs(pose_landmarks1[30][1] - initial_landmarks[30][1]) < 2 and abs(pose_landmarks1[29][1]-initial_landmarks[29][1]) < 2 and not jump_reset):
#             #         print("Jump reset")
#             #         jump_reset = True
#             #     if jump_reset and jump_detected:
#             #         jump_detected = False
#             if (pose_landmark_confs1[0]):
#                 print("abs diff", abs(
#                     pose_landmarks1[0][1] - initial_landmarks[0][1]))
#                 if (abs(pose_landmarks1[0][1] - initial_landmarks[0][1]) > 2 and jump_detected == 0 and jump_reset == 1):
#                     # if (abs(pose_landmarks1[30][1] - initial_landmarks[30][1]) > 2 and abs(pose_landmarks1[29][1]-initial_landmarks[29][1]) > 2 and not jump_detected and jump_reset):
#                     print("Jump detected")
#                     jump_detected = 1
#                     dataString = str("6")
#                     print(dataString)
#                     server_socket.sendall(dataString.encode("UTF-8"))
#                     jump_reset = 0

#                 if (abs(pose_landmarks1[0][1] - initial_landmarks[0][1]) < 2 and jump_reset == 0):
#                     print("Jump reset")
#                     jump_reset = 1
#                 if jump_reset and jump_detected:
#                     jump_detected = 0

#             #     nose_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]

#             # # Append x and y coordinates to the list
#             #     nose_values.append([nose_landmark.x * frame.shape[1], nose_landmark.y * frame.shape[0]])

#             # if (pose_landmark_confs1[23] and pose_landmark_confs1[25] and pose_landmark_confs1[29] and pose_landmark_confs1[24] and pose_landmark_confs1[26] and pose_landmark_confs1[30]):
#             #     right_knee_angle = abs(
#             #         get_angle((pose_landmarks1[23]), pose_landmarks1[25], pose_landmarks1[29]))
#             #     left_knee_angle = abs(
#             #         get_angle((pose_landmarks1[24]), pose_landmarks1[26], pose_landmarks1[30]))
#             #     if not right_knee_detected and right_knee_angle < 150 and right_knee_reset:
#             #         right_knee_detected = True
#             #         right_knee_reset = False
#             #         print("Right knee detected!")
#             #         # Converting Vector3 to a string, example "0,0,0"
#             #         dataString = str("3")
#             #         # print(dataString)
#             #     # Check if the right leg up has been reset
#             #     if right_knee_angle > 150 and not right_knee_reset:
#             #         right_knee_reset = True
#             #         print("Right knee reset.")
#             #     # If a right leg up has been detected, wait for the right_leg_up to be reset before detecting another right_leg_up
#             #     if right_knee_detected and right_knee_reset:
#             #         right_knee_detected = False

#             #     if not left_knee_detected and left_knee_angle < 150 and left_knee_reset:
#             #         left_knee_detected = True
#             #         left_knee_reset = False
#             #         print("Left knee detected!")
#             #         # Converting Vector3 to a string, example "0,0,0"
#             #         dataString = str("-3")
#             #         # print(dataString)
#             #     # Check if the left leg up has been reset
#             #     if left_knee_angle > 150 and not left_knee_reset:
#             #         left_knee_reset = True
#             #         print("Left knee reset.")
#             #     # If a right leg up has been detected, wait for the right_leg_up to be reset before detecting another right_leg_up
#             #     if left_knee_detected and left_knee_reset:
#             #         left_knee_detected = False

#             # if pose_landmark_confs1[13] > 0.5 and pose_landmark_confs1[11] > 0.5 and pose_landmark_confs1[23] > 0.5 and pose_landmark_confs1[12] > 0.5 and pose_landmark_confs1[24] > 0.5 and pose_landmark_confs1[14] > 0.5:
#             #     right_shoulder_position = (pose_landmarks1[11])
#             #     left_shoulder_position = (pose_landmarks1[12])

#             #     right_shoulder_angle = abs(get_angle(
#             #         (pose_landmarks1[23]), right_shoulder_position, (pose_landmarks1[13])))

#             #     left_shoulder_angle = abs(get_angle(
#             #         (pose_landmarks1[24]), left_shoulder_position, (pose_landmarks1[14])))

#             #     # Check if a right arm is currently up
#             #     if not right_turn_detected and right_shoulder_angle > 70 and right_turn_reset:
#             #         right_turn_detected = True
#             #         right_turn_reset = False

#             #         print("Right turn detected!")
#             #         # Converting Vector3 to a string, example "0,0,0"
#             #         dataString = str("1")
#             #         print(dataString)

#             #     # Check if the right_turn has been reset
#             #     if right_shoulder_angle < 70 and not right_turn_reset:
#             #         right_turn_reset = True
#             #         print("Right turn reset.")

#             #     # If a right_turn has been detected, wait for the right_turn to be reset before detecting another right_turn
#             #     if right_turn_detected and right_turn_reset:
#             #         right_turn_detected = False

#             #     # Check if the left arm is currently up
#             #     if not left_turn_detected and left_shoulder_angle > 70 and left_turn_reset:
#             #         left_turn_detected = True
#             #         left_turn_reset = False

#             #         print("left turn detected!")
#             #         # Converting Vector3 to a string, example "0,0,0"
#             #         dataString = str("-1")
#             #         # print(dataString)

#             #     # Check if the left_turn has been reset
#             #     if left_shoulder_angle < 70 and not left_turn_reset:
#             #         left_turn_reset = True
#             #         print("left turn reset.")

#             #     # If a left turn has been detected, wait for the left turn to be reset before detecting another left turn
#             #     if left_turn_detected and left_turn_reset:
#             #         left_turn_detected = False

#         # Converting string to Byte, and sending it to C#
#         # sock.sendall(dataString.encode("UTF-8"))
#         # receiveing data in Byte fron C#, and converting it to String
#         # receivedData = sock.recv(1024).decode("UTF-8")
#         # print(receivedData)
#         cv2.imshow('MediaPipe Pose Detection - Webcam 1', frame1)

#         # Exit the loop if the 'q' key is pressed.
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# # Release the capture object and destroy all windows.

# import socket
# import time

# host, port = "127.0.0.1", 25001
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect((host, port))

# startPos = [0, 0, 0] #Vector3   x = 0, y = 0, z = 0
# while True:
#     time.sleep(0.5) #sleep 0.5 sec
#     startPos[0] +=1 #increase x by one
#     posString = ','.join(map(str, startPos)) #Converting Vector3 to a string, example "0,0,0"
#     print(posString)

#     sock.sendall(posString.encode("UTF-8")) #Converting string to Byte, and sending it to C#
#     receivedData = sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
#     print(receivedData)


