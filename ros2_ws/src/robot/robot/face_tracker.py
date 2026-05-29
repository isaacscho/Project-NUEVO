import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import face_recognition

# Explicit paths mapped via the Docker workspace volume
BASE_DIR = "/ros2_ws/src/vision"
DATA_DIR = os.path.join(BASE_DIR, "data")

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker')
        
        # ROS2 Setup
        self.publisher_ = self.create_publisher(Image, '/vision/tracking_feed', 10)
        self.match_pub = self.create_publisher(Bool, '/vision/match_status', 10)
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
        """Pre-loads the database profiles for the guy and the girl."""
        self.guy_encoding = None
        self.girl_encoding = None
        self.target_encoding = None # Remains empty until scan station is reached

        # Update these paths to match your exact directory structure
        guy_path = "/ros2_ws/src/robot/test/guy.jpg"
        girl_path = "/ros2_ws/src/robot/test/girl.jpg"

        # Load Guy Profile
        if os.path.exists(guy_path):
            img = face_recognition.load_image_file(guy_path)
            encodings = face_recognition.face_encodings(img)
            if encodings: 
                self.guy_encoding = encodings[0]
                self.get_logger().info(f"Loaded guy.jpg from {guy_path}")
        else:
            self.get_logger().error(f"Could not find guy.jpg at {guy_path}")

        # Load Girl Profile
        if os.path.exists(girl_path):
            img = face_recognition.load_image_file(girl_path)
            encodings = face_recognition.face_encodings(img)
            if encodings: 
                self.girl_encoding = encodings[0]
                self.get_logger().info(f"Loaded girl.jpg from {girl_path}")
        else:
            self.get_logger().error(f"Could not find girl.jpg at {girl_path}")

    def capture_target_callback(self, request, response):
        """Captures live feed and classifies it as either the guy or the girl."""
        if self.latest_frame is None:
            response.success = False
            response.message = "No valid camera frames received yet."
            return response

        rgb_frame = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)

        if not face_locations:
            response.success = False
            response.message = "No face detected in frame."
            return response

        live_encoding = face_recognition.face_encodings(rgb_frame, face_locations)[0]

        # Compare the live face against our two database profiles
        is_guy = False
        is_girl = False
        
        if self.guy_encoding is not None:
            is_guy = face_recognition.compare_faces([self.guy_encoding], live_encoding, tolerance=0.6)[0]
        if self.girl_encoding is not None:
            is_girl = face_recognition.compare_faces([self.girl_encoding], live_encoding, tolerance=0.6)[0]

        os.makedirs(DATA_DIR, exist_ok=True)

        if is_guy:
            self.target_encoding = self.guy_encoding
            response.message = "guy.jpg"
            # Save the proof to disk
            cv2.imwrite(os.path.join(DATA_DIR, "scanned_order_guy.jpg"), self.latest_frame)
            response.success = True
            self.get_logger().info("Target identified and locked: GUY")
            
        elif is_girl:
            self.target_encoding = self.girl_encoding
            response.message = "girl.jpg"
            # Save the proof to disk
            cv2.imwrite(os.path.join(DATA_DIR, "scanned_order_girl.jpg"), self.latest_frame)
            response.success = True
            self.get_logger().info("Target identified and locked: GIRL")
            
        else:
            response.success = False
            response.message = "Stranger detected. Does not match guy.jpg or girl.jpg."
            self.get_logger().warn("Unrecognized customer at scan station.")

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
                    self.match_pub.publish(Bool(data=True))
                else:
                    color = (0, 0, 255)  # Red box for unrecognized face
                    label = "Unknown"
                    self.match_pub.publish(Bool(data=False))
                    
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