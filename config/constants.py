"""
Centralna konfiguracja stałych dla systemu handover

Ten moduł definiuje wszystkie stałe konfiguracyjne używane w projekcie,
eliminując "magic numbers" i ułatwiając konserwację kodu.
"""

# === PERCEPTION MODULE ===

# Object Detector
YOLO_MODEL = "yolov5s"
YOLO_CONFIDENCE_THRESHOLD = 0.6
OBJECT_DETECTION_TOPIC = "/object_detections"
CAMERA_RGB_TOPIC = "/camera/color/image_raw"

# Pose Estimator 6D
MIN_DEPTH_M = 0.1  # Minimum valid depth in meters
MAX_DEPTH_M = 10.0  # Maximum valid depth in meters
DEPTH_SCALE = 1000.0  # Depth conversion factor: mm to m
DEPTH_TOPIC = "/camera/depth/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
OBJECT_POSE_TOPIC = "/object_pose"

# Human Hand Detector
HAND_DETECTION_RATE_HZ = 30
DEFAULT_HAND_POSITION = [0.6, 0.0, 1.0]  # [x, y, z] in meters
HUMAN_HAND_POSE_TOPIC = "/human_hand_pose"
HUMAN_REACHING_TOPIC = "/human_reaching"

# === MANIPULATION MODULE ===

# Grasp Planning
PREGRASP_OFFSET_Z = 0.10  # meters above object
LIFT_HEIGHT = 0.15  # meters to lift after grasp

# Handover Planning
HANDOVER_OFFSET_Z = -0.05  # meters below human hand (ergonomic)

# Gripper
GRIPPER_OPEN_WIDTH = 0.04  # meters
GRIPPER_CLOSED_WIDTH = 0.0  # meters
GRIPPER_MAX_FORCE = 30.0  # Newtons
GRIPPER_STATE_TOPIC = "/gripper_state"

# MoveIt Groups
ARM_MOVE_GROUP = "arm"
GRIPPER_MOVE_GROUP = "gripper"

# Planning Scene
TABLE_HEIGHT = 0.75  # meters
TABLE_SIZE = [1.0, 1.0, 0.05]  # [length, width, height] in meters

# === DECISION MODULE ===

# WMA Configuration
WMA_CHECKPOINT_PATH = "/path/to/unifolm_wma_checkpoint"
WMA_PLANNING_HORIZON = 8  # timesteps
WMA_TASK_NAME = "handover_decision"

# FSM States
STATE_IDLE = "idle"
STATE_APPROACH = "approach"
STATE_GRASP = "grasp"
STATE_LIFT = "lift"
STATE_HANDOVER = "handover"

# Actions
ACTION_TAKE_FROM_HUMAN = "TAKE_FROM_HUMAN"
ACTION_GIVE_TO_HUMAN = "GIVE_TO_HUMAN"
ACTION_IDLE = "IDLE"

# === COORDINATE FRAMES ===
FRAME_BASE_LINK = "base_link"
FRAME_CAMERA_LINK = "camera_link"
FRAME_END_EFFECTOR = "end_effector_link"

# === SAFETY ===
EMERGENCY_STOP_DISTANCE = 0.05  # meters - minimum distance to human
MAX_VELOCITY_SCALE = 0.5  # Maximum velocity scaling factor (0-1)
MAX_ACCELERATION_SCALE = 0.5  # Maximum acceleration scaling factor (0-1)
MOVEMENT_TIMEOUT_SEC = 10.0  # seconds - timeout for robot movements
