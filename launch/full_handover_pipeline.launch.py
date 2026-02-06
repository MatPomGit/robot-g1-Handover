"""
Plik uruchomieniowy kompletnego pipeline handover (Full Handover Pipeline Launch)

Ten launch file uruchamia KOMPLETNY system przekazywania obiektów,
włączając percepcję, detekcję, estymację pozy i wykonanie handover.

KROK PO KROKU:
1. Uruchamia statyczną transformację kamery (TF)
2. Uruchamia detektor obiektów (YOLOv5)
3. Uruchamia estymator pozy 6D
4. Uruchamia detektor dłoni człowieka
5. Uruchamia executor handover z WMA

UŻYCIE:
    ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py

CO ROBI TEN LAUNCH FILE:
- Uruchamia wszystkie komponenty potrzebne do handover
- Percepcja: wykrywanie obiektów i dłoni
- Decyzje: WMA podejmuje inteligentne decyzje
- Manipulacja: MoveIt 2 planuje i wykonuje ruchy

WYMAGANIA:
- Wszystkie node'y muszą być zbudowane i zainstalowane
- Kamera RGB-D (np. RealSense) uruchomiona osobno
- MoveIt 2 uruchomiony osobno
- Checkpoint WMA załadowany

TOPIKI UTWORZONE:
- /tf_static (transformacja kamery)
- /object_detections (Detection2DArray)
- /object_pose (PoseStamped)
- /human_hand_pose (PoseStamped)
- /human_reaching (Bool)

ARCHITEKTURA SYSTEMU:
    Kamera RGB-D
         |
         v
    [TF Camera] -----> /tf_static
         |
         v
    [Object Detector] -----> /object_detections
         |
         v
    [Pose Estimator 6D] -----> /object_pose
         |
    [Human Hand Detector] -----> /human_hand_pose, /human_reaching
         |
         v
    [Execute Handover WMA] -----> Sterowanie robotem
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generuje opis uruchomienia kompletnego pipeline
    
    KROK PO KROKU:
    1. Tworzy listę wszystkich node'ów potrzebnych do handover
    2. Każdy node uruchamia się w odpowiedniej kolejności
    3. ROS 2 zarządza lifecyclem wszystkich node'ów
    
    Returns:
        LaunchDescription: Lista node'ów do uruchomienia
    
    KOLEJNOŚĆ URUCHAMIANIA:
    1. TF Camera - musi być pierwsza (inne node'y potrzebują transformacji)
    2. Object Detector - wykrywa obiekty na obrazie
    3. Pose Estimator 6D - konwertuje detekcje 2D na pozycje 3D
    4. Human Hand Detector - wykrywa dłoń człowieka
    5. Execute Handover WMA - główny kontroler systemu
    """
    return LaunchDescription([
        # === NODE 1: Static TF Camera ===
        # Publikuje statyczną transformację base_link -> camera_link
        # DLACZEGO PIERWSZA: Inne node'y potrzebują tej transformacji
        # do konwersji pozycji z układu kamery do układu robota
        Node(package="g1_pick_and_handover",
             executable="static_tf_camera"),
        
        # === NODE 2: Object Detector ===
        # Wykrywa obiekty na obrazie RGB używając YOLOv5
        # INPUT: /camera/color/image_raw (Image)
        # OUTPUT: /object_detections (Detection2DArray)
        # FUNKCJA: Znajduje obiekty i rysuje bounding boxes
        Node(package="g1_pick_and_handover",
             executable="object_detector"),
        
        # === NODE 3: Pose Estimator 6D ===
        # Oblicza pozycję 3D obiektów z detekcji 2D + depth
        # INPUT: /object_detections, /camera/depth/image_raw, /camera/color/camera_info
        # OUTPUT: /object_pose (PoseStamped)
        # FUNKCJA: Konwertuje piksele na metry używając pinhole camera model
        Node(package="g1_pick_and_handover",
             executable="pose_estimator_6d"),
        
        # === NODE 4: Human Hand Detector ===
        # Wykrywa pozycję dłoni człowieka w przestrzeni 3D
        # INPUT: /camera/color/image_raw (w przyszłości)
        # OUTPUT: /human_hand_pose, /human_reaching
        # FUNKCJA: Używa MediaPipe/OpenPose do wykrywania kluczowych punktów dłoni
        Node(package="g1_pick_and_handover",
             executable="human_hand_detector"),
        
        # === NODE 5: Execute Handover WMA ===
        # Główny kontroler systemu handover z World Model AI
        # INPUT: /human_hand_pose, /human_reaching, /gripper_state, /camera/color/image_raw
        # OUTPUT: Sterowanie ramieniem robota i chwytakiem (via MoveIt 2)
        # FUNKCJA: WMA analizuje sytuację i podejmuje decyzje (TAKE/GIVE/IDLE)
        Node(package="g1_pick_and_handover",
             executable="execute_handover_wma")
    ])

# === UWAGI DLA STUDENTÓW ===
#
# DEBUGOWANIE:
# Jeśli coś nie działa, uruchamiaj node'y jeden po drugim:
# 1. ros2 run g1_pick_and_handover static_tf_camera
# 2. ros2 run g1_pick_and_handover object_detector
# 3. ros2 run g1_pick_and_handover pose_estimator_6d
# 4. ros2 run g1_pick_and_handover human_hand_detector
# 5. ros2 run g1_pick_and_handover execute_handover_wma
#
# SPRAWDZANIE TOPIKÓW:
# ros2 topic list                      # Lista wszystkich topików
# ros2 topic echo /object_detections   # Zobacz detekcje
# ros2 topic echo /object_pose         # Zobacz pozycje 3D
# ros2 topic hz /human_hand_pose       # Sprawdź częstotliwość
#
# WIZUALIZACJA:
# rviz2                                # Uruchom RViz2
# Dodaj displays: Image, TF, PoseStamped, Detection2DArray
#
# PARAMETRY (przykład dla production):
# Node(
#     package="g1_pick_and_handover",
#     executable="object_detector",
#     parameters=[{
#         'confidence_threshold': 0.7,  # Wyższa pewność
#         'model_size': 'yolov5m'       # Większy model
#     }],
#     output='screen'  # Zobacz logi
# )
