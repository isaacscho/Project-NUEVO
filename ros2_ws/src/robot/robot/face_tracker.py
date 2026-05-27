import face_recognition
import cv2
import serial
import struct
import os
import numpy as np

# ============================================
# UART CONFIGURATION (NUEVO v4 Bridge)
# ============================================
PORT = "/dev/serial0" 
BAUD_RATE = 200000
MAGIC_HEADER = b"NUEV"

SYS_CMD_ID = 3         
DC_SET_VELOCITY_ID = 18

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def build_nuevo_frame(tlv_type: int, payload: bytes, frame_num: int = 1) -> bytes:
    device_id = 0x00
    flags = 0x00
    num_tlvs = 1
    
    tlv_len = len(payload)
    tlv_data = struct.pack("<BB", tlv_type, tlv_len) + payload
    
    num_total_bytes = 12 + len(tlv_data)
    header_fmt = "<4s H H B B B B"
    temp_header = struct.pack(header_fmt, MAGIC_HEADER, num_total_bytes, 0, device_id, frame_num, num_tlvs, flags)
    
    crc = crc16_ccitt(temp_header + tlv_data)
    final_header = struct.pack(header_fmt, MAGIC_HEADER, num_total_bytes, crc, device_id, frame_num, num_tlvs, flags)
    
    return final_header + tlv_data

def load_known_faces(known_faces_dir="known_faces"):
    """Scans the directory for images and memorizes their facial encodings."""
    known_encodings = []
    known_names = []
    
    print(f"Scanning '{known_faces_dir}' for known identities...")
    if not os.path.exists(known_faces_dir):
        print(f"Warning: Directory '{known_faces_dir}' not found. Creating it now.")
        os.makedirs(known_faces_dir)
        return known_encodings, known_names

    for filename in os.listdir(known_faces_dir):
        if filename.lower().endswith((".jpg", ".jpeg", ".png")):
            # The name is the filename without the extension
            name = os.path.splitext(filename)[0]
            filepath = os.path.join(known_faces_dir, filename)
            
            # Load the image and get the encoding
            image = face_recognition.load_image_file(filepath)
            encodings = face_recognition.face_encodings(image)
            
            if len(encodings) > 0:
                known_encodings.append(encodings[0])
                known_names.append(name)
                print(f" -> Memorized: {name}")
            else:
                print(f" -> Could not find a clear face in {filename}")
                
    return known_encodings, known_names

def main():
    print("Initializing NUEVO UART Bridge...")
    try:
        arduino = serial.Serial(PORT, BAUD_RATE, timeout=1)
    except Exception as e:
        print(f"Warning: UART not connected ({e}). Running in vision-only mode.")
        arduino = None

    # 1. Load the Memory
    known_face_encodings, known_face_names = load_known_faces()

    # 2. Initialize the camera
    video_capture = cv2.VideoCapture(0)
    print("\nCamera active. Looking for faces...")

    try:
        while True:
            ret, frame = video_capture.read()
            if not ret:
                break
                
            # Resize frame for faster processing (optional, but recommended for Pi)
            small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            rgb_small_frame = small_frame[:, :, ::-1]

            # Find all face locations and their 128-d encodings in the current frame
            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
            
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                # Scale back up face locations since the frame we detected in was scaled to 1/2 size
                top *= 2
                right *= 2
                bottom *= 2
                left *= 2

                # Default to Unknown
                name = "Unknown"

                # Check if the face matches any known memory
                if len(known_face_encodings) > 0:
                    # Get the mathematical distance to all known faces (lower distance = closer match)
                    face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                    best_match_index = np.argmin(face_distances)
                    
                    # If the closest match is within the strict tolerance (0.6 is default)
                    if face_distances[best_match_index] < 0.6:
                        name = known_face_names[best_match_index]

                # Console Output
                print(f"Spotted: {name} at X: {(left+right)//2}")

                # ====================================================
                # HARDWARE ACTUATION
                # Example: Only move if it is someone you know!
                # ====================================================
                if arduino and name != "Unknown":
                    pass
                    # payload = struct.pack("<...", ...)
                    # frame = build_nuevo_frame(DC_SET_VELOCITY_ID, payload)
                    # arduino.write(frame)
            
                # Draw a box and label around the face for debugging
                color = (0, 255, 0) if name != "Unknown" else (0, 0, 255) # Green for known, Red for unknown
                cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
                cv2.putText(frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)
                
            cv2.imshow('Video', frame)
            
            # Hit 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        video_capture.release()
        cv2.destroyAllWindows()
        if arduino:
            arduino.close()

if __name__ == "__main__":
    main()