from __future__ import annotations

import os
import cv2
import face_recognition


class FaceTracker:
    def __init__(
        self,
        guy_path: str = "/ros2_ws/src/vision/data/faces/guy.jpg",
        girl_path: str = "/ros2_ws/src/vision/data/faces/girl.jpg",
        tolerance: float = 0.6,
        logger=None,
    ) -> None:
        self.guy_path = guy_path
        self.girl_path = girl_path
        self.tolerance = tolerance
        self.logger = logger

        self.guy_encoding = None
        self.girl_encoding = None
        self.target_encoding = None
        self.target_label = None

        self._load_profiles()

    def _log_info(self, msg: str) -> None:
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def _log_warn(self, msg: str) -> None:
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f"[WARN] {msg}")

    def _log_error(self, msg: str) -> None:
        if self.logger:
            self.logger.error(msg)
        else:
            print(f"[ERROR] {msg}")

    def _load_face_encoding(self, path: str):
        if not os.path.exists(path):
            self._log_error(f"Missing face profile: {path}")
            return None

        image = face_recognition.load_image_file(path)
        encodings = face_recognition.face_encodings(image)

        if not encodings:
            self._log_warn(f"No face found in profile image: {path}")
            return None

        self._log_info(f"Loaded face profile: {path}")
        return encodings[0]

    def _load_profiles(self) -> None:
        self.guy_encoding = self._load_face_encoding(self.guy_path)
        self.girl_encoding = self._load_face_encoding(self.girl_path)

    def capture_target(self, frame_bgr) -> tuple[bool, str]:
        """
        Classify the current live frame as guy.jpg or girl.jpg.
        Returns: (success, message)
        """
        if frame_bgr is None:
            return False, "No valid camera frame available."

        rgb_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)

        if not face_locations:
            return False, "No face detected."

        live_encoding = face_recognition.face_encodings(rgb_frame, face_locations)[0]

        if self.guy_encoding is not None:
            is_guy = face_recognition.compare_faces(
                [self.guy_encoding],
                live_encoding,
                tolerance=self.tolerance,
            )[0]
            if is_guy:
                self.target_encoding = self.guy_encoding
                self.target_label = "guy.jpg"
                return True, "guy.jpg"

        if self.girl_encoding is not None:
            is_girl = face_recognition.compare_faces(
                [self.girl_encoding],
                live_encoding,
                tolerance=self.tolerance,
            )[0]
            if is_girl:
                self.target_encoding = self.girl_encoding
                self.target_label = "girl.jpg"
                return True, "girl.jpg"

        return False, "Face did not match guy.jpg or girl.jpg."

    def is_target_visible(self, frame_bgr) -> bool:
        """
        Return True if the locked target face is visible in the current frame.
        """
        if frame_bgr is None or self.target_encoding is None:
            return False

        rgb_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Downscale for speed on Raspberry Pi.
        small_frame = cv2.resize(rgb_frame, (0, 0), fx=0.5, fy=0.5)

        face_locations = face_recognition.face_locations(small_frame)
        if not face_locations:
            return False

        face_encodings = face_recognition.face_encodings(small_frame, face_locations)

        for encoding in face_encodings:
            matches = face_recognition.compare_faces(
                [self.target_encoding],
                encoding,
                tolerance=self.tolerance,
            )
            if matches and matches[0]:
                return True

        return False