import cv2
import time
import serial
from ultralytics import YOLO
import time

# Set up the serial connection
print("Setting up serial connection...")
ser = serial.Serial('/dev/ttyACM0', 9600)  # Change this to the appropriate port for your Arduino Nano
time.sleep(2)  # Wait for the connection to be established

# Check if the serial connection is open
if ser.isOpen():
    print("Serial connection established!")
else:
    print("Failed to establish serial connection.")
    sys.exit()


def detect_and_point():
    model = YOLO('yolov8n.pt')
    cap = cv2.VideoCapture(0)  # Open the webcam

    centers = []
    start_time = time.time()

    while True:
        ret, frame = cap.read()  # Read a frame from the webcam
        if not ret:
            break

        # Inference
        results = model(frame)

        # Find the largest human detection and its center
        largest_area = 0
        largest_center = None

        for box in results[0].boxes:
            if box.cls == 0:  # 0 is typically the class for 'person' in COCO
                # Convert the tensor to a numpy array
                box_coords = box.xyxy[0].cpu().numpy()

                # Compute the area of the box
                area = (box_coords[2] - box_coords[0]) * (box_coords[3] - box_coords[1])
                center = ((box_coords[0] + box_coords[2]) / 2, (box_coords[1] + box_coords[3]) / 2)

                if area > largest_area:
                    largest_area = area
                    largest_center = center

                # Write text above bounding box and draw bounding box
                if center == largest_center:
                    cv2.putText(frame, "Target Human", (int(box_coords[0]), int(box_coords[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
                    cv2.rectangle(frame, 
                                  (int(box_coords[0]), int(box_coords[1])), 
                                  (int(box_coords[2]), int(box_coords[3])), 
                                  (36,255,12), 2)
                else:
                    cv2.putText(frame, "Human", (int(box_coords[0]), int(box_coords[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
                    cv2.rectangle(frame, 
                                  (int(box_coords[0]), int(box_coords[1])), 
                                  (int(box_coords[2]), int(box_coords[3])), 
                                  (0,0,255), 2)

        # Only append to the list when a human is detected in the frame
        if largest_center is not None:
            centers.append(largest_center)

        elapsed_time = time.time() - start_time

        if elapsed_time > 1:  # 1 second has passed
            if centers:  # only compute average if there are detected centers
                avg_center = (sum(x[0] for x in centers) / len(centers), sum(x[1] for x in centers) / len(centers))

                # Compute servo angle based on largest_center and your frame dimensions
                servo_angle = int(avg_center[0] / frame.shape[1] * 180)

                # Write the angle to both servos, the second servo opens to 180
                ser.write(f"{servo_angle},{180}\n".encode())
            else:
                # If no humans are detected, close the second servo to 0
                ser.write("0,0\n".encode())

            centers = []  # Reset the list
            start_time = time.time()  # Reset the timer


        # Display the frame
        cv2.imshow("Frame", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Run the function
detect_and_point()

# Close the serial connection
print("Closing serial connection...")
ser.close()
