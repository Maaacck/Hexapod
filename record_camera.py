import cv2
import datetime

# Set the camera index (usually 0 for the first camera)
# If you have multiple cameras, you might need to try 1, 2, etc.
camera_index = 0

# Set the desired resolution (width, height)
# Check your camera's supported resolutions using 'v4l2-ctl --list-formats-ext'
frame_width = 640
frame_height = 480

# Set the desired frames per second (FPS)
fps = 30

# Create a VideoCapture object
cap = cv2.VideoCapture(camera_index)

# Check if camera opened successfully
if not cap.isOpened():
    print(f"Error: Could not open video device {camera_index}.")
    exit()

# Set resolution and FPS
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
cap.set(cv2.CAP_PROP_FPS, fps)

# Define the codec and create VideoWriter object
# You can choose other codecs like 'XVID', 'MJPG', 'DIVX'
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Codec for .mp4 files
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
output_filename = f"output_{timestamp}.mp4"
out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

if not out.isOpened():
    print(f"Error: Could not create video writer for {output_filename}.")
    cap.release()
    exit()

print(f"Recording started. Press 'q' to stop.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame.")
        break

    # Write the frame to the output file
    out.write(frame)

    # Optional: Display the live feed (requires a display connected to the Pi)
    # cv2.imshow('Live Feed', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything when job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
print(f"Recording stopped. Video saved as {output_filename}")
