import serial
import struct
import time

# ============================================
# UART CONFIGURATION
# ============================================
PORT = "/dev/ttyUSB0"
BAUD_RATE = 200000

# ============================================
# NUEVO PROTOCOL DEFINITIONS (UPDATE THESE)
# ============================================
# You MUST check your local 'TLV_TypeDefs.json' and 'config.h' for these exact values.
MAGIC_HEADER = b"NUEV"      # Replace with the actual magic[4] bytes
STEP_MOVE_TLV_ID = 0x42     # Replace with the actual numeric ID for STEP_MOVE
STEPPER_ID = 0              # From your C++ code (StepperManager::getStepper(0))
ELEVATION_STEPS = 533       # Target steps for the lift

def crc16_ccitt(data: bytes) -> int:
    """Calculates standard CRC16-CCITT (Poly: 0x1021, Init: 0xFFFF)."""
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
    """Builds a compact TLV frame matching the NUEVO v4 specifications."""
    device_id = 0x00
    flags = 0x00
    num_tlvs = 1
    
    # Pack the TLV Header: [tlvType: uint8][tlvLen: uint8] + [payload]
    tlv_len = len(payload)
    tlv_data = struct.pack("<BB", tlv_type, tlv_len) + payload
    
    # Calculate Total Bytes: Frame Header (12 bytes) + TLV Data
    num_total_bytes = 12 + len(tlv_data)
    
    # Frame Header Layout (Little-Endian):
    # magic[4], numTotalBytes(uint16), crc16(uint16), deviceId(uint8), frameNum(uint8), numTlvs(uint8), flags(uint8)
    header_fmt = "<4s H H B B B B"
    
    # Pack temporary header with CRC = 0 to calculate the checksum
    temp_header = struct.pack(header_fmt, MAGIC_HEADER, num_total_bytes, 0, device_id, frame_num, num_tlvs, flags)
    
    # Compute CRC over the entire frame (Header + TLVs)
    frame_without_crc = temp_header + tlv_data
    crc = crc16_ccitt(frame_without_crc)
    
    # Repack the header with the actual computed CRC
    final_header = struct.pack(header_fmt, MAGIC_HEADER, num_total_bytes, crc, device_id, frame_num, num_tlvs, flags)
    
    return final_header + tlv_data

def main():
    print(f"Connecting to {PORT} at {BAUD_RATE} baud...")
    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=1) as arduino:
            # Give the serial connection time to initialize
            time.sleep(2) 
            
            # -----------------------------------------------------
            # CONSTRUCT PAYLOAD
            # Assuming payload is: [stepperId: uint8][steps: int32]
            # -----------------------------------------------------
            payload = struct.pack("<B i", STEPPER_ID, ELEVATION_STEPS)
            
            frame = build_nuevo_frame(STEP_MOVE_TLV_ID, payload, frame_num=1)
            
            print(f"Sending STEP_MOVE (TLV: 0x{STEP_MOVE_TLV_ID:02X}, Steps: {ELEVATION_STEPS})...")
            print(f"Raw Binary Stream: {frame.hex(' ')}")
            
            arduino.write(frame)
            
            # Wait for physical movement
            time.sleep(5)
            print("Action complete.")
            
    except serial.SerialException as e:
        print(f"Serial Error: {e}")

if __name__ == "__main__":
    main()