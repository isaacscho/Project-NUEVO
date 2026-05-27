import face_recognition
import cv2
import serial
import struct
import time

# ============================================
# UART CONFIGURATION (NUEVO v4 Bridge)
# ============================================
PORT = "/dev/serial0" 
BAUD_RATE = 200000
MAGIC_HEADER = b"NUEV"

SYS_CMD_ID = 3         
DC_SET_VELOCITY_ID = 18

def crc16_ccitt(data: bytes) -> int:
    """Calculates standard CRC16-CCITT for NUEVO v4 frames."""
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
    """Packs standard payloads into the custom TLV wire format."""
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

def main():
    print("Initializing NUEVO UART Bridge...")
    try:
        arduino = serial.Serial(PORT, BAUD_RATE, timeout=1)
    except Exception as e:
        print(f"Warning: UART not connected ({e}). Running in vision-only mode.")
        arduino = None

    # Initialize the camera
    video_capture = cv2.VideoCapture(0)
    print("Camera active. Using 'face_recognition' library to find faces...")

    try:
        while True:
            # Grab a single frame of video
            ret, frame = video_capture.read()
            if not ret:
                break
                
            # Convert the image from BGR color (OpenCV default) to RGB color (face_recognition default)
            rgb_frame = frame[:, :, ::-1]

            # Find all the faces in the current frame of video
            face_locations = face_recognition.face_locations(rgb_frame)
            
            if face_locations:
                print(f"Found {len(face_locations)} face(s)!")
                
                # face_locations returns tuples in (top, right, bottom, left) order
                top, right, bottom, left = face_locations[0]
                face_center_x = (left + right) // 2
                print(f"Primary face centered at X: {face_center_x}")
                
                # ====================================================
                # HARDWARE ACTUATION
                # Translate face_center_x into a motor command here
                # ====================================================
                if arduino:
                    pass
                    # Example payload construction:
                    # payload = struct.pack("<...", ...)
                    # frame = build_nuevo_frame(DC_SET_VELOCITY_ID, payload)
                    # arduino.write(frame)
            
            # Draw a box around the faces for debugging
            for (top, right, bottom, left) in face_locations:
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                
            # Display the resulting image
            cv2.imshow('Video', frame)
            
            # Hit 'q' on the keyboard to quit!
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