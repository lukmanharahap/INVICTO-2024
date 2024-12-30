import os
from ultralytics import YOLO
import cv2
import serial
import time
import math
import torch

# Select device for model inference
device = "cuda" if torch.cuda.is_available() else "cpu"


# Initialize serial communication
def initialize_serial(port="/dev/ttyUSB0", baudrate=115200):
    try:
        com = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            xonxoff=0,
            rtscts=0,
            dsrdtr=0,
        )
        print("Serial port opened successfully.")
        return com
    except serial.SerialException as e:
        print("Failed to open serial port:", e)
        exit()


# Initialize cameras
def initialize_camera(camera_id, width=640, height=360, fps=30):
    cap = cv2.VideoCapture(camera_id)
    assert cap.isOpened(), f"Cannot open camera {camera_id}"
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


# Calculate distance based on known parameters
def calc_distance(focal_length, known_width, pixel_width):
    return (focal_length * known_width) / pixel_width


# Calculate angle based on object position
def calc_angle(x_center, frame_center_x, focal_length):
    return math.degrees(math.atan2(x_center - frame_center_x, focal_length))


# Main processing loop
def process_frame(frame, threshold, model, focal_length, frame_center):
    results = model(frame)[0]
    data = {
        "ball_distance": 0,
        "ball_angle": 0,
        "ball_exists": 0,
        "silo_distances_angles": [(9999, 999) for _ in range(5)],
        "silo_ball_counts": [0 for _ in range(5)],
    }

    for result in results.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result

        if score > threshold:
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            width = x2 - x1
            height = y2 - y1
            pixel_width = width

            if class_id == 0:
                cv2.rectangle(
                    frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1
                )
                cv2.putText(
                    frame,
                    results.names[int(class_id)].upper(),
                    (int(x1), int(y1 - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (90, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            if class_id == 1:
                cv2.rectangle(
                    frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1
                )
                cv2.putText(
                    frame,
                    results.names[int(class_id)].upper(),
                    (int(x1), int(y1 - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (90, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            if class_id == 2:  # Ball
                data["ball_distance"] = int(calc_distance(5.5, 1800, pixel_width))
                data["ball_angle"] = int(
                    calc_angle(x_center, frame_center[0], focal_length)
                )
                data["ball_exists"] = 1

            if class_id == 3:  # Silo
                distance = int(calc_distance(5.1, 2500, pixel_width))
                angle = int(calc_angle(x_center, frame_center[0], focal_length))
                for i in range(len(data["silo_distances_angles"])):
                    if data["silo_distances_angles"][i] == (9999, 999):
                        data["silo_distances_angles"][i] = (distance, angle)
                        break

    return data


# Main function
def main():
    # Initialize hardware and model
    com = initialize_serial()
    cap = initialize_camera(0)
    cap2 = initialize_camera(2)
    model = YOLO("models\ball_silo.pt").to(device)

    frame_center = (320, 180)  # 640x360 resolution
    focal_length = 240  # adjust as needed

    while True:
        ret_main, frame_main = cap_main.read()
        ret_internal, frame_internal = cap_internal.read()

        if not ret_main or not ret_internal:
            print("Failed to grab frame")
            break

        # Process main frame and get data
        data = process_frame(frame_main, 0.7, model, focal_length, frame_center)

        # Check for ball presence using the internal camera
        ball_inside_robot = (
            1
            if process_frame(frame_internal, 0.75, model, focal_length, frame_center)[
                "ball_exists"
            ]
            else 0
        )

        if ball_inside_robot:
            data["ball_exists"] = 1

        # Prepare data string for transmission
        silo_data = ",".join(
            f"{dist},{angle}" for dist, angle in data["silo_distances_angles"]
        )
        data_string = (
            f"{data['ball_distance']},{data['ball_angle']},{data['ball_exists']},"
            + silo_data
            + ","
            + ",".join(map(str, data["silo_ball_counts"]))
        )

        # Send data via serial
        com.write(data_string.encode())

        # Display main and internal frames for debugging
        cv2.imshow("Main Camera", frame_main)
        cv2.imshow("Internal Camera", frame_internal)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap_main.release()
    cap_internal.release()
    com.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
