import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import face_recognition

# Explicit paths mapped via the Docker workspace volume
BASE_DIR = "/ros2_ws/src/vision"
DATA_DIR = os.path.join(BASE_DIR, "data")
TARGET_IMAGE_PATH = os.path.join(DATA_DIR, "target.jpg")

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker')
        
        # ROS2 Setup
        self.publisher_ = self.create_publisher(Image, '/vision/tracking_feed', 10)
        self.srv = self.create_service(Trigger, '/vision/capture_target', self.capture_target_callback)
        self.bridge = CvBridge()
        
        # Hardware Setup: Connects to the Pi loopback camera device
        self.cap = cv2.VideoCapture(10)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera at /dev/video10. Verify host loopback service.")
            
        # State Variables
        self.latest_frame = None
        self.target_encoding = None
        
        # Check for pre-existing tracking image at startup
        self.bootstrap_existing_target()
        
        # Execute processing loop at 10Hz to manage Pi 5 CPU overhead
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Face Tracker Node successfully initialized.")

    def bootstrap_existing_target(self):
        """Checks if a target image already exists in the data directory and loads it."""
        if os.path.exists(TARGET_IMAGE_PATH):
            self.get_logger().info(f"Found existing target image at {TARGET_IMAGE_PATH}. Loading profile...")
            try:
                # Load the raw image file
                existing_img = face_recognition.load_image_file(TARGET_IMAGE_PATH)
                # Compute its facial encoding
                encodings = face_recognition.face_encodings(existing_img)
                
                if encodings:
                    self.target_encoding = encodings[0]
                    self.get_logger().info("Successfully loaded target profile. Verification mode active.")
                else:
                    self.get_logger().warning("Target image found, but no clear face could be resolved from it.")
            except Exception as e:
                self.get_logger().error(f"Failed to parse existing target image: {str(e)}")

    def capture_target_callback(self, request, response):
        """Synchronous service routine to capture a target face from the live feed."""
        if self.latest_frame is None:
            response.success = False
            response.message = "Capture failed: No valid camera frames received yet."
            return response

        # Convert working frame to RGB for deep learning analysis
        rgb_frame = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)

        if not face_locations:
            response.success = False
            response.message = "Capture failed: No face detected in the current frame."
            self.get_logger().warn("Service called, but no faces are within the frame boundaries.")
            return response

        # Extract the vector representation of the primary face found
        encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        self.target_encoding = encodings[0]

        # Enforce persistence onto the host computer hard drive via the mount boundary
        try:
            os.makedirs(DATA_DIR, exist_ok=True)
            cv2.imwrite(TARGET_IMAGE_PATH, self.latest_frame)
            
            response.success = True
            response.message = f"Success: Target saved to data/target.jpg. Profile locked."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to write image data to disk: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def timer_callback(self):
        """Core vision frame processing and publisher loop."""
        ret, frame = self.cap.read()
        if not ret:
            return
            
        # Retain a copy of the pristine frame for potential service capture
        self.latest_frame = frame.copy()

        # Scale down input array by 50% to optimize calculation speed
        small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

        # Detect spatial bounding boxes
        face_locations = face_recognition.face_locations(rgb_small_frame)
        
        # Operational Mode 1: Active Verification against a loaded profile
        if face_locations and self.target_encoding is not None:
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
            
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                # Run metric distance calculation between vectors (tolerance 0.6 is optimal)
                matches = face_recognition.compare_faces([self.target_encoding], face_encoding, tolerance=0.6)
                
                # Rescale dimensions back up to original coordinate space
                top *= 2; right *= 2; bottom *= 2; left *= 2
                
                if matches[0]:
                    color = (0, 255, 0)  # Green box for correct target match
                    label = "TARGET MATCH"
                else:
                    color = (0, 0, 255)  # Red box for unrecognized face
                    label = "Unknown"
                    
                # Render visual overlays onto output image matrix
                cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
                cv2.putText(frame, label, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)
        
        # Operational Mode 2: Awaiting Initial Target Capture
        elif face_locations:
            for (top, right, bottom, left) in face_locations:
                top *= 2; right *= 2; bottom *= 2; left *= 2
                # Draw neutral Blue box signaling the node is functional but unprovisioned
                cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 2)
                cv2.putText(frame, "Awaiting Target Capture", (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 0, 0), 1)

        # Repackage the OpenCV frame back into a standard ROS network packet and push
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(annotated_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()