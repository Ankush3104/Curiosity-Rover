import cv2
import mediapipe as mp
import socket
import math
import time
from collections import deque
from enum import Enum
import numpy as np

# ==== Configuration ====
ESP32_IP = "192.168.4.1"
ESP32_PORT = 1234
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Command throttling settings
COMMAND_COOLDOWN = 0.1
GESTURE_STABILITY_FRAMES = 3
SPEED_UPDATE_INTERVAL = 0.2
CONNECTION_CHECK_INTERVAL = 2.0 # Check connection every 2 seconds

# ==== Enums for Clarity ====
# ==== Enums for Clarity ====
class Gesture(Enum):
    FORWARD = "F"
    LEFT = "L"
    RIGHT = "R"
    BACKWARD = "B"
    STOP = "S"
    SPEED_CONTROL = "SPEED"  # This is for internal logic, not sent
    UNKNOWN = "UNKNOWN"    # This is for internal logic, not sent# ==== Enums for Clarity ====
class Gesture(Enum):
    FORWARD = "F"
    LEFT = "L"
    RIGHT = "R"
    BACKWARD = "B"
    STOP = "S"
    SPEED_CONTROL = "SPEED"  # This is for internal logic, not sent
    UNKNOWN = "UNKNOWN"    # This is for internal logic, not sent

# ==== UDP Connection Manager ====
class UdpManager:
    """Manages the UDP connection state."""
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0) # Shorter timeout for non-blocking checks
        self.is_connected = False
        self.last_check_time = 0

    def check_connection(self):
        """Attempts to verify the connection and updates the status."""
        try:
            # Send a lightweight ping to the ESP32
            self.sock.sendto(b"PING", (self.ip, self.port))
            # A successful send is enough to consider it connected for UDP
            if not self.is_connected:
                print(f"✅ Curiosity Rover CONNECTED at {self.ip}:{self.port}")
            self.is_connected = True
        except Exception:
            if self.is_connected:
                print(f"⚠️ Curiosity Rover DISCONNECTED.")
            self.is_connected = False
        self.last_check_time = time.time()

    def send(self, message: str):
        """Sends a message if connected."""
        if self.is_connected:
            try:
                self.sock.sendto(message.encode(), (self.ip, self.port))
            except Exception as e:
                print(f"Error sending command: {e}")
                self.is_connected = False
        else:
            # Try to reconnect if we attempt to send while disconnected
            if time.time() - self.last_check_time > 1.0:
                 self.check_connection()


    def close(self):
        self.sock.close()


# ==== Mediapipe Setup ====
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# ==== Core Logic: HandProcessor Class (Corrected Version) ====
class HandProcessor:
    """Encapsulates all logic for processing a single detected hand."""
    FINGER_TIPS = [8, 12, 16, 20]
    FINGER_MCPS = [5, 9, 13, 17] # Knuckles at the base of the fingers
    THUMB_TIP = 4
    THUMB_IP = 3

    def __init__(self, landmarks, handedness):
        self.landmarks = landmarks
        self.handedness = handedness
        self.finger_states = self._get_finger_states()

    def _get_finger_states(self) -> list[int]:
        """Determines which fingers are extended using a robust method."""
        fingers = []

        # Robust Thumb Detection
        p0 = self.landmarks[0]
        p5 = self.landmarks[5]
        hand_vec = np.array([p5.x - p0.x, p5.y - p0.y])
        hand_vec_normalized = hand_vec / np.linalg.norm(hand_vec)
        hand_perp_vec = np.array([-hand_vec_normalized[1], hand_vec_normalized[0]])
        thumb_tip_vec = np.array([self.landmarks[self.THUMB_TIP].x - p0.x, self.landmarks[self.THUMB_TIP].y - p0.y])
        thumb_ip_vec = np.array([self.landmarks[self.THUMB_IP].x - p0.x, self.landmarks[self.THUMB_IP].y - p0.y])
        proj_tip = np.dot(thumb_tip_vec, hand_perp_vec)
        proj_ip = np.dot(thumb_ip_vec, hand_perp_vec)
        fingers.append(1 if (proj_tip < proj_ip if self.handedness == "Right" else proj_tip > proj_ip) else 0)

        # More Robust Finger Detection
        for tip_id, mcp_id in zip(self.FINGER_TIPS, self.FINGER_MCPS):
            dist_tip_to_wrist = self._calculate_distance(self.landmarks[tip_id], self.landmarks[0])
            dist_mcp_to_wrist = self._calculate_distance(self.landmarks[mcp_id], self.landmarks[0])
            fingers.append(1 if dist_tip_to_wrist > dist_mcp_to_wrist else 0)

        return fingers

    def recognize_gesture(self, calibration_data: dict = {}) -> dict:
        """Recognizes the gesture based on the state of the fingers."""
        is_thumb_up = self.finger_states[0] == 1
        is_index_up = self.finger_states[1] == 1
        other_fingers_down = sum(self.finger_states[2:]) == 0

        if is_thumb_up and is_index_up and other_fingers_down:
            return self._get_speed_control_data(calibration_data)

        finger_pattern = tuple(self.finger_states)
        gestures = {
            (0, 1, 0, 0, 0): Gesture.FORWARD, (0, 1, 1, 0, 0): Gesture.LEFT,
            (0, 1, 1, 1, 0): Gesture.RIGHT, (0, 1, 1, 1, 1): Gesture.BACKWARD,
            (0, 0, 0, 0, 0): Gesture.STOP, (1, 1, 1, 1, 1): Gesture.STOP,
        }
        gesture = gestures.get(finger_pattern, Gesture.UNKNOWN)

        return {'gesture': gesture, 'fingers': self.finger_states, 'speed_command': None,
                'speed_percentage': None, 'pinch_distance': None}

    def _get_speed_control_data(self, calibration_data: dict) -> dict:
        """Calculates speed based on the pinch distance."""
        thumb_tip = self.landmarks[self.THUMB_TIP]
        index_tip = self.landmarks[self.FINGER_TIPS[0]]
        pinch_distance = self._calculate_distance(thumb_tip, index_tip, use_z=False)
        min_dist, max_dist = calibration_data.get('min', 0.02), calibration_data.get('max', 0.15)
        if max_dist <= min_dist: max_dist = min_dist + 0.1

        clamped_dist = max(min_dist, min(max_dist, pinch_distance))
        speed_pct = ((clamped_dist - min_dist) / (max_dist - min_dist)) * 100
        speed_command = str(int(speed_pct // 10)) if speed_pct < 95 else "q"

        return {'gesture': Gesture.SPEED_CONTROL, 'fingers': self.finger_states, 'speed_command': speed_command,
                'speed_percentage': speed_pct, 'pinch_distance': pinch_distance}

    @staticmethod
    def _calculate_distance(p1, p2, use_z=True) -> float:
        dx, dy = p1.x - p2.x, p1.y - p2.y
        return math.sqrt(dx*dx + dy*dy + (p1.z - p2.z)**2) if use_z else math.hypot(dx, dy)

# ==== Gesture Tracking ====
class GestureTracker:
    def __init__(self, stability_frames=3):
        self.history = deque(maxlen=stability_frames)
        self.last_confirmed_gesture = Gesture.STOP
        self.last_sent_command = Gesture.STOP.value
        self.last_command_time = 0
        self.last_speed_time = 0
        self.last_speed_command = "5"

    def update(self, gesture: Gesture):
        self.history.append(gesture)
        if len(self.history) == self.history.maxlen and all(g == gesture for g in self.history):
            if gesture != self.last_confirmed_gesture:
                self.last_confirmed_gesture = gesture
                return True, gesture
        return False, self.last_confirmed_gesture

    def can_send_command(self):
        if time.time() - self.last_command_time >= COMMAND_COOLDOWN:
            self.last_command_time = time.time()
            return True
        return False

    def can_send_speed(self):
        if time.time() - self.last_speed_time >= SPEED_UPDATE_INTERVAL:
            self.last_speed_time = time.time()
            return True
        return False

# ==== UI & Main Loop ====
def draw_ui(frame: np.ndarray, gesture_data: dict, tracker: GestureTracker, fps: float, handedness: str, conn: UdpManager):
    h, w, _ = frame.shape
    cv2.rectangle(frame, (0, 0), (w, 110), (20, 20, 20), -1)

    # NEW: Curiosity Connection Status
    conn_status = "CONNECTED" if conn.is_connected else "DISCONNECTED"
    conn_color = (0, 255, 0) if conn.is_connected else (0, 0, 255)
    cv2.putText(frame, f"CURIOSITY: {conn_status}", (w - 220, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, conn_color, 2)
    cv2.putText(frame, f"FPS: {fps:.1f}", (w - 220, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    gesture = gesture_data.get('gesture', Gesture.UNKNOWN)

    if gesture == Gesture.SPEED_CONTROL:
        speed_pct = gesture_data['speed_percentage']
        speed_cmd = gesture_data['speed_command']
        cv2.putText(frame, "MODE: SPEED CONTROL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(frame, f"Speed: {speed_pct:.1f}% (Cmd: {speed_cmd})", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        bar_x, bar_y, bar_w, bar_h = 10, 85, 300, 20
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (80, 80, 80), -1)
        fill_w = int((speed_pct / 100) * bar_w)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), (0, 255, 255), -1)
    else:
        color = (0, 255, 0) if gesture != Gesture.STOP else (0, 100, 255)
        cv2.putText(frame, f"GESTURE: {gesture.value}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(frame, f"Fingers: {gesture_data.get('fingers', 'N/A')}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, f"Last Sent: {tracker.last_sent_command}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

def main():
    conn = UdpManager(ESP32_IP, ESP32_PORT)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("❌ Error: Could not open camera.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    tracker = GestureTracker(GESTURE_STABILITY_FRAMES)
    fps_start_time, fps_frame_count, fps = time.time(), 0, 0
    calibration_data = {}

    print("\n🚀 Enhanced Hand Gesture Control is LIVE.")
    print("   Press 'ESC' to quit.")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: break

            if time.time() - conn.last_check_time > CONNECTION_CHECK_INTERVAL:
                conn.check_connection()

            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb_frame)

            fps_frame_count += 1
            if (time.time() - fps_start_time) > 1.0:
                fps = fps_frame_count / (time.time() - fps_start_time)
                fps_start_time, fps_frame_count = time.time(), 0

            handedness, gesture_data = "N/A", {}
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                handedness = results.multi_handedness[0].classification[0].label
                hand_proc = HandProcessor(hand_landmarks.landmark, handedness)
                gesture_data = hand_proc.recognize_gesture(calibration_data)
                current_gesture = gesture_data['gesture']
                is_stable, confirmed_gesture = tracker.update(current_gesture)

                if current_gesture == Gesture.SPEED_CONTROL:
                    speed_cmd = gesture_data['speed_command']
                    if speed_cmd != tracker.last_speed_command and tracker.can_send_speed():
                        conn.send(speed_cmd)
                        tracker.last_speed_command = speed_cmd
                elif is_stable and tracker.can_send_command():
                    command = confirmed_gesture.value
                    if command != tracker.last_sent_command:
                        conn.send(command)
                        tracker.last_sent_command = command

                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                                          mp_drawing_styles.get_default_hand_landmarks_style(),
                                          mp_drawing_styles.get_default_hand_connections_style())
            else:
                if tracker.last_sent_command != Gesture.STOP.value and tracker.can_send_command():
                    conn.send(Gesture.STOP.value)
                    tracker.last_sent_command = Gesture.STOP.value
                    tracker.last_confirmed_gesture = Gesture.STOP
                    print("⚠️  No hand detected. Sending STOP command.")

            draw_ui(frame, gesture_data, tracker, fps, handedness, conn)
            cv2.imshow("Curiosity Rover Control", frame)

            if cv2.waitKey(5) & 0xFF == 27: break

    finally:
        print("\n👋 Shutting down...")
        cap.release()
        cv2.destroyAllWindows()
        hands.close()
        conn.close()
        print("✅ Cleanup complete.")

if __name__ == "__main__":
    main() 