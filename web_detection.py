import cv2
import torch

def detect():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True).to(device)  # or yolov5m, yolov5l, yolov5x, custom

    # Use URL of video stream instead of local webcam
    video_url = 'http://10.0.0.27:8080/video_feed'
    cap = cv2.VideoCapture(video_url)

    while True:
        ret, frame = cap.read()  # Read a frame from the video stream
        if not ret:
            break

        # Inference
        results = model(frame)

        # Loop over detections and print message if 'person' is detected, also draw bounding boxes
        counter = 0
        for *box, cls, conf in results.xyxy[0]:
            if int(cls) == 0:  # 0 is typically the class for 'person' in COCO
                counter += 1
                print(f"Human {counter} detected at position:", box)

                # Draw bounding boxes on the frame
                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                
                # Add text to the top of bounding box
                cv2.putText(frame, f'Human {counter}', (int(box[0]), int(box[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)

        # Display the frame
        cv2.imshow("Frame", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Run the function
detect()
