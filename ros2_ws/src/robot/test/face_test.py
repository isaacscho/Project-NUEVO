#!/usr/bin/env python3
import os
import face_recognition

# Reference the active test folder directory
TEST_DIR = os.path.dirname(os.path.abspath(__file__))
GUY_PATH = os.path.join(TEST_DIR, "guy.jpg")
GIRL_PATH = os.path.join(TEST_DIR, "girl.jpg")
PROBE_PATH = os.path.join(TEST_DIR, "probe.jpg")

def run_3way_face_test():
    print("==================================================")
    print("      PROJECT NUEVO: 3-IMAGE DATABASE TEST        ")
    print("==================================================")
    
    # 1. Verification of file system requirements
    missing_files = []
    for name, path in [("guy.jpg", GUY_PATH), ("girl.jpg", GIRL_PATH), ("probe.jpg", PROBE_PATH)]:
        if not os.path.exists(path):
            missing_files.append(name)
            
    if missing_files:
        print(f"[ERROR] Missing required files in {TEST_DIR}:")
        for f in missing_files:
            print(f"  - {f}")
        print("\nPlease ensure all three images are placed in the test directory before running.")
        return

    print("[STEP 1] Loading known database and probe image...")
    guy_img = face_recognition.load_image_file(GUY_PATH)
    girl_img = face_recognition.load_image_file(GIRL_PATH)
    probe_img = face_recognition.load_image_file(PROBE_PATH)

    print("[STEP 2] Computing 128-dimensional spatial embeddings...")
    guy_enc = face_recognition.face_encodings(guy_img)
    girl_enc = face_recognition.face_encodings(girl_img)
    probe_enc = face_recognition.face_encodings(probe_img)

    if not guy_enc:
        print("[FAILURE] Extraction error: No face resolved in guy.jpg")
        return
    if not girl_enc:
        print("[FAILURE] Extraction error: No face resolved in girl.jpg")
        return
    if not probe_enc:
        print("[FAILURE] Extraction error: No face resolved in probe.jpg (Test image)")
        return

    # Construct the database arrays
    known_encodings = [guy_enc[0], girl_enc[0]]
    known_names = ["Guy", "Girl"]

    print("\n[STEP 3] Evaluating probe against database matrices...")
    
    # Calculate the precise Euclidean distances to all known faces
    distances = face_recognition.face_distance(known_encodings, probe_enc[0])
    
    # Check boolean matches against the optimal 0.60 threshold constraint
    matches = face_recognition.compare_faces(known_encodings, probe_enc[0], tolerance=0.6)

    print("\n--------------------------------------------------")
    print("DATABASE SEARCH RESULTS:")
    print("--------------------------------------------------")
    
    matched_any = False
    for i in range(len(known_names)):
        name = known_names[i]
        dist = distances[i]
        is_match = matches[i]
        
        status = "MATCH CONFIRMED" if is_match else "NO MATCH"
        print(f"  vs. {name:4} -> Distance: {dist:.4f} [{status}]")
        
        if is_match:
            matched_any = True

    print("--------------------------------------------------")
    print("FINAL ALGORITHMIC VERDICT:")
    if matched_any:
        # Find the index of the smallest distance to identify the closest match
        best_match_idx = int(distances.argmin())
        if matches[best_match_idx]:
            print(f"  Identity Identified: {known_names[best_match_idx].upper()}")
    else:
        print("  Identity Identified: UNKNOWN INDIVIDUAL (No database entry cleared threshold)")
    print("--------------------------------------------------\n")

if __name__ == "__main__":
    run_3way_face_test()