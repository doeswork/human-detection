import cv2
from ultralytics import YOLO

def detect():
    model = YOLO('yolov8n.pt')

    cap = cv2.VideoCapture(0)  # Open the webcam

    while True:
        ret, frame = cap.read()  # Read a frame from the webcam
        if not ret:
            break

        # Inference
        results = model(frame)

        # Loop over detections and print message if 'person' is detected, also draw bounding boxes
        counter = 0
        for box in results[0].boxes:
            if box.cls == 0:  # 0 is typically the class for 'person' in COCO
                counter += 1
                print(f"Human {counter} detected at position:", box.xyxy)

                # Convert the tensor to a numpy array
                box_coords = box.xyxy[0].cpu().numpy()

                # Draw bounding boxes on the frame
                cv2.rectangle(frame, (int(box_coords[0]), int(box_coords[1])), (int(box_coords[2]), int(box_coords[3])), (0, 255, 0), 2)
                
                # Add text to the top of bounding box
                cv2.putText(frame, f'Human {counter}', (int(box_coords[0]), int(box_coords[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)

        # Display the frame
        cv2.imshow("Frame", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Run the function
detect()
