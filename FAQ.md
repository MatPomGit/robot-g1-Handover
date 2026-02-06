# FAQ i Rozwiązywanie Problemów (Troubleshooting)

## 📖 Wprowadzenie

Ten dokument zawiera odpowiedzi na najczęściej zadawane pytania oraz rozwiązania typowych problemów spotykanych podczas pracy z systemem Robot G1 Handover.

## 🔧 Instalacja i Konfiguracja

### Q: Jak sprawdzić, czy ROS 2 jest poprawnie zainstalowany?

```bash
# Sprawdź wersję ROS 2
ros2 --version

# Sprawdź czy można uruchomić przykładowy node
ros2 run demo_nodes_cpp talker

# W drugim terminalu
ros2 run demo_nodes_cpp listener
```

**Oczekiwany rezultat**: Listener odbiera wiadomości od Talker.

### Q: Pakiet nie buduje się - błąd "package not found"

**Problem**: `colcon build` zwraca błąd o brakujących zależnościach.

**Rozwiązanie**:
```bash
# Zainstaluj zależności ROS 2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Sprawdź czy wszystkie zależności są zainstalowane
rosdep check g1_pick_and_handover
```

### Q: Jak zaktualizować zależności Python?

```bash
# Zaktualizuj pip
pip3 install --upgrade pip

# Zainstaluj wymagane pakiety
pip3 install -r requirements.txt

# Sprawdź zainstalowane wersje
pip3 list | grep torch
pip3 list | grep opencv
```

## 🎥 Kamera i Percepcja

### Q: Kamera nie jest wykrywana przez system

**Problem**: ROS 2 nie widzi topików z kamery (`/camera/color/image_raw`).

**Rozwiązanie**:
```bash
# Sprawdź czy kamera jest podłączona (dla RealSense)
rs-enumerate-devices

# Uruchom driver kamery RealSense
ros2 run realsense2_camera realsense2_camera_node

# Sprawdź dostępne topiki
ros2 topic list | grep camera
```

**Alternatywa**: Użyj danych testowych:
```bash
# Odtwórz bag z nagranym zapisem
ros2 bag play test_data.bag
```

### Q: YOLOv5 nie wykrywa obiektów

**Problem**: `/object_detections` jest pusty mimo obiektów w kadrze.

**Sprawdzenia**:
1. Czy obiekt jest w zbiorze COCO (80 klas)?
2. Czy próg pewności nie jest zbyt wysoki?
3. Czy oświetlenie jest odpowiednie?

**Rozwiązanie**:
```bash
# Obniż próg pewności
ros2 run g1_pick_and_handover object_detector \
    --ros-args -p confidence_threshold:=0.3

# Użyj większego modelu (dokładniejszy)
ros2 run g1_pick_and_handover object_detector \
    --ros-args -p model_name:=yolov5m
```

**Testowanie offline**:
```python
import torch
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
results = model('test_image.jpg')
results.show()  # Zobacz czy wykrywa obiekty
```

### Q: Nieprawidłowa pozycja 3D obiektu

**Problem**: `/object_pose` zwraca błędne współrzędne (x, y, z).

**Sprawdzenia**:
1. Czy kamera jest skalibrowana?
2. Czy transformacja TF jest poprawna?
3. Czy mapa głębokości jest poprawna?

**Rozwiązanie**:
```bash
# Sprawdź transformację kamery
ros2 run tf2_ros tf2_echo base_link camera_link

# Sprawdź parametry kalibracji
ros2 topic echo /camera/color/camera_info --once

# Wizualizuj w RViz
rviz2
# Dodaj: TF, Image, PoseStamped
```

**Kalibracja kamery** (jeśli potrzebna):
```bash
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 \
    image:=/camera/color/image_raw
```

### Q: MediaPipe nie wykrywa dłoni

**Problem**: `/human_hand_pose` nie publikuje pozycji dłoni.

**Rozwiązania**:
1. Upewnij się, że dłoń jest dobrze oświetlona
2. Dłoń powinna być wyraźnie widoczna (bez zasłonięć)
3. Sprawdź czy MediaPipe jest zainstalowany:
   ```bash
   pip3 show mediapipe
   ```

**Test MediaPipe**:
```python
import mediapipe as mp
import cv2

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

if results.multi_hand_landmarks:
    print("Dłoń wykryta!")
else:
    print("Brak detekcji dłoni")
```

## 🦾 MoveIt 2 i Manipulacja

### Q: MoveIt 2 nie planuje trajektorii

**Problem**: `Planning failed` w logach.

**Możliwe przyczyny i rozwiązania**:

1. **Pozycja poza workspace robota**
   ```python
   # Sprawdź czy pozycja jest w zasięgu
   distance = sqrt(x**2 + y**2 + z**2)
   print(f"Distance to target: {distance}m")
   # Dla G1: workspace zwykle 0.3-0.8m
   ```

2. **Kolizja z przeszkodami**
   ```bash
   # Wizualizuj scenę planowania w RViz
   # Dodaj display: PlanningScene
   # Sprawdź czy przeszkody są poprawnie zdefiniowane
   ```

3. **IK nie ma rozwiązania**
   ```python
   # Zwiększ tolerancję
   self.arm.set_goal_position_tolerance(0.01)
   self.arm.set_goal_orientation_tolerance(0.05)
   ```

4. **Timeout planowania**
   ```python
   # Zwiększ czas planowania
   self.arm.set_planning_time(15.0)
   ```

### Q: Robot wykonuje dziwne ruchy

**Problem**: Trajektoria jest nienaturalna lub robot uderza w przeszkody.

**Rozwiązania**:
```python
# Zmień planner
self.arm.set_planner_id("RRTstar")  # Zamiast RRTConnect

# Zmniejsz prędkość
self.arm.set_max_velocity_scaling_factor(0.3)

# Dodaj więcej przeszkód do sceny
from manipulation.planning_scene import add_table, add_human
add_table(self.scene)
add_human(self.scene)
```

### Q: Gripper nie chwyta obiektu

**Problem**: Chwytak zamyka się, ale obiekt wypada.

**Sprawdzenia**:
1. Czy robot jest dokładnie nad obiektem?
2. Czy chwytak jest wystarczająco duży?
3. Czy obiekt nie jest zbyt śliski?

**Rozwiązania**:
```yaml
# W config/grasp_params.yaml
grasp:
  approach_distance: 0.08  # Zmniejsz (bliżej obiektu)
  gripper_closed: 0.005    # Zwiększ siłę zamknięcia
  max_force: 40.0          # Zwiększ maksymalną siłę
```

**Dodanie czujnika siły** (zaawansowane):
```python
def check_grasp_success(self):
    """Sprawdź czy obiekt został chwycony"""
    force = self.get_gripper_force()
    if force < MIN_FORCE:
        self.get_logger().warn("Grasp may have failed")
        return False
    return True
```

## 🧠 WMA i Podejmowanie Decyzji

### Q: WMA nie jest dostępny

**Problem**: Błąd `WMA not available, using mock decision making`.

**To jest normalne!** WMA jest zaawansowanym modelem AI, który wymaga:
- Pretrenowanego checkpointu
- GPU (opcjonalnie, ale zalecane)
- Biblioteki `unifolm_wma` (nie jest publicznie dostępna)

**Rozwiązanie - tryb mock**:
System automatycznie używa prostych reguł if-else:
```python
if human_reaching and not gripper_occupied:
    action = "TAKE_FROM_HUMAN"
elif human_reaching and gripper_occupied:
    action = "GIVE_TO_HUMAN"
else:
    action = "IDLE"
```

**To wystarczy do nauki i testowania systemu!**

### Q: Jak wytrenować własny model WMA?

**Uwaga**: To zaawansowane zadanie wymagające znajomości ML.

**Krok po kroku**:
1. Zbierz dataset (100+ epizodów handover)
2. Format danych:
   ```
   dataset/
   ├── episode_0001/
   │   ├── observations/
   │   │   ├── rgb_000.png
   │   │   ├── depth_000.png
   │   │   └── state_000.json
   │   └── actions/
   │       └── action_000.txt
   ```
3. Wytrenuj model (wymaga PyTorch + GPU):
   ```bash
   python train_wma.py --dataset ./dataset --epochs 100
   ```

**Alternatywa**: Użyj Reinforcement Learning:
- Biblioteka: Stable Baselines 3
- Środowisko: Gymnasium
- Algorytm: SAC lub PPO

## 🚀 Launch Files i Node'y

### Q: Launch file nie uruchamia wszystkich node'ów

**Problem**: Tylko część node'ów się uruchamia.

**Debugowanie**:
```bash
# Uruchom z logami na ekranie
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py --screen

# Sprawdź aktywne node'y
ros2 node list

# Sprawdź czy node się crashuje
ros2 node info /node_name
```

**Typowe błędy**:
- Brak zależności Python
- Nieprawidłowa ścieżka do pliku konfiguracyjnego
- Konflikt nazw node'ów

### Q: Jak uruchomić pojedynczy node do testowania?

```bash
# Node z domyślnymi parametrami
ros2 run g1_pick_and_handover object_detector

# Node z custom parametrami
ros2 run g1_pick_and_handover object_detector \
    --ros-args \
    -p confidence_threshold:=0.7 \
    -p model_name:=yolov5m

# Node z plikiem konfiguracyjnym
ros2 run g1_pick_and_handover execute_grasp \
    --ros-args --params-file config/grasp_params.yaml
```

## 🔍 Debugowanie

### Q: Jak sprawdzić co się dzieje w systemie?

**Narzędzia diagnostyczne**:

1. **Lista topików**:
   ```bash
   ros2 topic list
   ```

2. **Monitorowanie topiku**:
   ```bash
   ros2 topic echo /object_pose
   ```

3. **Częstotliwość publikacji**:
   ```bash
   ros2 topic hz /camera/color/image_raw
   ```

4. **Wykres node'ów**:
   ```bash
   rqt_graph
   ```

5. **Logi node'a**:
   ```bash
   ros2 node info /object_detector
   ```

### Q: Jak włączyć verbose logging?

```bash
# Dla pojedynczego node'a
ros2 run g1_pick_and_handover object_detector \
    --ros-args --log-level debug

# Dla launch file
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py \
    log_level:=debug
```

**W kodzie Python**:
```python
# Dodaj więcej logów
self.get_logger().debug("Detailed debug info")
self.get_logger().info("General information")
self.get_logger().warn("Warning message")
self.get_logger().error("Error occurred")
```

### Q: Jak nagrać i odtworzyć dane?

**Nagrywanie (bag file)**:
```bash
# Nagraj wszystkie topiki
ros2 bag record -a

# Nagraj wybrane topiki
ros2 bag record /camera/color/image_raw /object_pose

# Nagraj z nazwą
ros2 bag record -o my_test /camera/color/image_raw
```

**Odtwarzanie**:
```bash
# Odtwórz bag
ros2 bag play my_test.bag

# Odtwórz w pętli
ros2 bag play my_test.bag --loop

# Odtwórz wolniej (50% prędkości)
ros2 bag play my_test.bag --rate 0.5
```

## ⚠️ Bezpieczeństwo

### Q: Jak zatrzymać robota w nagłych wypadkach?

**Emergency Stop**:
1. **Hardware E-STOP**: Czerwony przycisk (jeśli dostępny)
2. **Software E-STOP**: 
   ```bash
   ros2 service call /emergency_stop std_srvs/srv/Trigger
   ```
3. **Keyboard**: `Ctrl+C` w terminalu z node'm
4. **Kill process**: 
   ```bash
   ps aux | grep ros2
   kill -9 <PID>
   ```

### Q: Jak ustawić bezpieczne prędkości?

**W kodzie**:
```python
# Bezpieczne wartości dla testów
self.arm.set_max_velocity_scaling_factor(0.3)  # 30% max prędkości
self.arm.set_max_acceleration_scaling_factor(0.3)  # 30% max przyspieszenia
```

**W konfiguracji**:
```yaml
# config/moveit.yaml
move_group:
  ros__parameters:
    default_velocity_scaling_factor: 0.3
    default_acceleration_scaling_factor: 0.3
```

### Q: Jak dodać strefy bezpieczeństwa?

```python
from manipulation.planning_scene import add_safety_zone

def add_safety_zone(scene, position, radius):
    """Dodaje sferę bezpieczeństwa"""
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.w = 1.0
    
    scene.add_sphere("safety_zone", pose, radius=radius)

# Użycie
add_safety_zone(scene, position=[0.5, 0.0, 0.5], radius=0.2)
```

## 🎓 Dla Studentów

### Q: Od czego zacząć naukę tego projektu?

**Ścieżka nauki (krok po kroku)**:

1. **Tydzień 1**: Podstawy ROS 2
   - Zrozum koncepty: node, topic, message
   - Uruchom przykładowy talker/listener
   - Naucz się używać `ros2 topic`, `ros2 node`

2. **Tydzień 2**: Percepcja
   - Uruchom `object_detector`
   - Zrozum jak działa YOLOv5
   - Testuj detekcję różnych obiektów

3. **Tydzień 3**: MoveIt 2
   - Zainstaluj MoveIt 2
   - Zaplanuj prostą trajektorię w RViz
   - Zrozum kinematykę odwrotną (IK)

4. **Tydzień 4**: Integracja
   - Uruchom cały pipeline
   - Zaobserwuj przepływ danych
   - Przetestuj na prawdziwym robocie (jeśli dostępny)

### Q: Jakie projekty można zrobić na bazie tego repozytorium?

**Pomysły na projekty**:

1. **Łatwe** (1-2 tygodnie):
   - Dodaj detekcję nowych klas obiektów (custom YOLOv5)
   - Zaimplementuj lepszą estymację pozy (PnP, ICP)
   - Stwórz GUI do monitorowania systemu (rqt plugin)

2. **Średnie** (3-4 tygodnie):
   - Dodaj multi-object grasp planning
   - Zaimplementuj force control dla chwytaka
   - Dodaj human skeleton tracking (całe ciało, nie tylko dłoń)

3. **Trudne** (1-2 miesiące):
   - Wytrenuj własny model WMA
   - Zaimplementuj adaptive grasping (różne strategie dla różnych obiektów)
   - Dodaj collision avoidance w czasie rzeczywistym

### Q: Gdzie znaleźć więcej informacji?

**Zasoby edukacyjne**:

- **ROS 2**: https://docs.ros.org/en/humble/Tutorials.html
- **MoveIt 2**: https://moveit.picknik.ai/humble/index.html
- **Computer Vision**: https://opencv.org/university/
- **Deep Learning**: https://pytorch.org/tutorials/
- **Robotics**: https://www.coursera.org/specializations/modernrobotics

**Książki**:
- "Programming Robots with ROS" - Morgan Quigley
- "Modern Robotics" - Kevin Lynch
- "Computer Vision: Algorithms and Applications" - Richard Szeliski

## 💬 Pomoc i Wsparcie

### Q: Gdzie mogę zadać pytanie?

1. **GitHub Issues**: Otwórz issue z opisem problemu
2. **ROS Answers**: https://answers.ros.org/
3. **Stack Overflow**: Tag `ros2` lub `moveit`
4. **Discord/Slack**: Sprawdź czy jest społeczność projektu

### Q: Jak zgłosić błąd?

**Szablon zgłoszenia**:
```markdown
## Opis problemu
[Krótki opis co nie działa]

## Kroki do reprodukcji
1. Uruchom...
2. Wykonaj...
3. Zaobserwuj...

## Oczekiwane zachowanie
[Co powinno się stać]

## Aktualne zachowanie
[Co się dzieje]

## Logi
```bash
[Wklej logi z terminala]
```

## Środowisko
- Ubuntu: 22.04
- ROS 2: Humble
- Python: 3.10
- GPU: NVIDIA RTX 3060
```

---

**Nie znalazłeś odpowiedzi?** Otwórz Issue na GitHubie z dokładnym opisem problemu!
