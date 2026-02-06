# Szybka Ściąga (Quick Reference)

## 🚀 Najczęściej Używane Komendy

### Uruchamianie Systemu

```bash
# Kompletny pipeline handover
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py

# Tylko percepcja
ros2 run g1_pick_and_handover object_detector
ros2 run g1_pick_and_handover pose_estimator_6d
ros2 run g1_pick_and_handover human_hand_detector

# Tylko manipulacja
ros2 run g1_pick_and_handover execute_grasp
ros2 run g1_pick_and_handover execute_handover_wma

# MoveIt 2
ros2 launch g1_moveit_config demo.launch.py
```

### Debugowanie i Monitorowanie

```bash
# Lista node'ów
ros2 node list

# Lista topików
ros2 topic list

# Monitorowanie topiku
ros2 topic echo /object_pose

# Częstotliwość publikacji
ros2 topic hz /camera/color/image_raw

# Info o node
ros2 node info /object_detector

# Parametry node'a
ros2 param list /object_detector
ros2 param get /object_detector confidence_threshold

# Wykres komunikacji
rqt_graph
```

### Wizualizacja

```bash
# RViz2
rviz2

# W RViz dodaj:
# - Image: /camera/color/image_raw
# - TF: transformacje
# - PoseStamped: /object_pose, /human_hand_pose
```

### Nagrywanie i Odtwarzanie

```bash
# Nagraj wszystko
ros2 bag record -a

# Nagraj wybrane topiki
ros2 bag record /camera/color/image_raw /object_pose

# Odtwórz
ros2 bag play my_recording.bag

# Odtwórz w pętli
ros2 bag play my_recording.bag --loop
```

## 📁 Struktura Projektu

```
robot-g1-Handover/
├── perception/              # Wykrywanie obiektów i dłoni
│   ├── object_detector.py
│   ├── pose_estimator_6d.py
│   └── human_hand_detector.py
├── manipulation/            # Planowanie i wykonywanie ruchów
│   ├── moveit_interface.py
│   ├── grasp_planner.py
│   └── execute_grasp.py
├── decision/                # Decyzje AI (WMA)
│   ├── wma_handover_manager.py
│   └── wma_task_manager.py
├── launch/                  # Pliki uruchomieniowe
│   └── full_handover_pipeline.launch.py
└── config/                  # Konfiguracja
    ├── constants.py
    ├── grasp_params.yaml
    └── moveit.yaml
```

## 🔧 Kluczowe Topiki ROS 2

| Topic | Typ | Opis |
|-------|-----|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Obraz RGB |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Mapa głębokości |
| `/object_detections` | `vision_msgs/Detection2DArray` | Wykryte obiekty 2D |
| `/object_pose` | `geometry_msgs/PoseStamped` | Pozycja 3D obiektu |
| `/human_hand_pose` | `geometry_msgs/PoseStamped` | Pozycja dłoni |
| `/human_reaching` | `std_msgs/Bool` | Intencja człowieka |
| `/gripper_state` | `std_msgs/Bool` | Stan chwytaka |

## 🎯 Kluczowe Parametry

### config/grasp_params.yaml
```yaml
grasp:
  approach_distance: 0.10    # Offset pre-grasp [m]
  lift_distance: 0.15        # Wysokość podniesienia [m]
  gripper_open: 0.04         # Otwarcie [m]
  gripper_closed: 0.0        # Zamknięcie [m]
  max_force: 30.0            # Maks siła [N]
```

### config/constants.py
```python
# Percepcja
YOLO_CONFIDENCE_THRESHOLD = 0.6
MIN_DEPTH_M = 0.1
MAX_DEPTH_M = 10.0

# Bezpieczeństwo
EMERGENCY_STOP_DISTANCE = 0.05  # m
MAX_VELOCITY_SCALE = 0.5
```

## 🔄 Automat Stanów (FSM)

### Chwytanie (Grasp)
```
idle → approach → grasp → lift
```

### Handover
```
IDLE → TAKE_FROM_HUMAN → HOLD → GIVE_TO_HUMAN → IDLE
```

## 🧠 Decyzje WMA

| Obserwacje | Decyzja |
|------------|---------|
| `human_reaching=True`, `gripper_occupied=False` | `TAKE_FROM_HUMAN` |
| `human_reaching=True`, `gripper_occupied=True` | `GIVE_TO_HUMAN` |
| `human_reaching=False` | `IDLE` |

## 🛠️ Typowe Problemy i Rozwiązania

### Problem: Kamera nie wykrywana
```bash
# Sprawdź urządzenie
rs-enumerate-devices

# Uruchom driver
ros2 run realsense2_camera realsense2_camera_node
```

### Problem: MoveIt nie planuje
```python
# Zwiększ timeout
self.arm.set_planning_time(15.0)

# Zwiększ tolerancję
self.arm.set_goal_position_tolerance(0.01)

# Zmień planner
self.arm.set_planner_id("RRTstar")
```

### Problem: YOLOv5 nie wykrywa
```bash
# Obniż próg
ros2 run g1_pick_and_handover object_detector \
    --ros-args -p confidence_threshold:=0.3

# Większy model
ros2 run g1_pick_and_handover object_detector \
    --ros-args -p model_name:=yolov5m
```

### Problem: WMA niedostępny
**To normalne!** System używa prostych reguł jako fallback.

## 📊 Metryki Wydajności

| Metryka | Oczekiwana wartość |
|---------|-------------------|
| FPS percepcji | > 10 FPS |
| Czas planowania | < 5s |
| Dokładność pozy | ± 2cm |
| Success rate grasp | > 80% |
| Latencja reakcji | < 1s |

## 🔍 Przydatne Skróty RViz

| Skrót | Akcja |
|-------|-------|
| `G` | Pokaż/ukryj Grid |
| `T` | Pokaż/ukryj TF |
| `Ctrl+S` | Zapisz konfigurację |
| `Ctrl+O` | Otwórz konfigurację |

## 📝 Wzorce Kodu

### Subskrypcja topiku
```python
self.create_subscription(
    PoseStamped, 
    '/object_pose', 
    self.callback, 
    10  # queue size
)
```

### Publikacja
```python
self.pub = self.create_publisher(PoseStamped, '/my_topic', 10)
msg = PoseStamped()
# ... wypełnij msg
self.pub.publish(msg)
```

### Timer
```python
self.timer = self.create_timer(0.1, self.timer_callback)  # co 100ms
```

### MoveIt Interface
```python
from manipulation.moveit_interface import MoveItInterface

self.moveit = MoveItInterface()
self.moveit.move_to_pose(target_pose)
self.moveit.close_gripper()
```

### Dodanie przeszkody
```python
from manipulation.planning_scene import add_table
add_table(self.moveit.scene)
```

## 🎓 Linki do Dokumentacji

- **ROS 2**: https://docs.ros.org/en/humble/
- **MoveIt 2**: https://moveit.picknik.ai/
- **YOLOv5**: https://github.com/ultralytics/yolov5
- **MediaPipe**: https://google.github.io/mediapipe/
- **PyTorch**: https://pytorch.org/docs/

## 📞 Pomoc

- **FAQ**: Zobacz `FAQ.md`
- **Tutoriale**: Zobacz `TUTORIALS.md`
- **Słownik**: Zobacz `GLOSSARY.md`
- **Issues**: https://github.com/MatPomGit/robot-g1-Handover/issues

## ⌨️ Zmienne Środowiskowe

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace
source ~/ros2_ws/install/setup.bash

# Dodaj do ~/.bashrc dla automatycznego source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 🧪 Testowanie Komponentów

### Test percepcji
```bash
ros2 run g1_pick_and_handover object_detector
ros2 topic echo /object_detections
```

### Test manipulacji
```bash
ros2 launch g1_moveit_config demo.launch.py
# W RViz: użyj interactive marker do planowania
```

### Test integracji
```bash
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

---

**Wydrukuj tę stronę i trzymaj przy biurku podczas pracy!** 📄🖨️
