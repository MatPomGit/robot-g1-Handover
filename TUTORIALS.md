# Tutoriale Krok po Kroku (Step-by-Step Tutorials)

## 📖 Wprowadzenie

Ten dokument zawiera szczegółowe tutoriale prowadzące przez najważniejsze funkcjonalności systemu Robot G1 Handover. Każdy tutorial jest zaprojektowany jako samodzielne ćwiczenie z jasno określonymi celami edukacyjnymi.

---

## 🎯 Tutorial 1: Pierwsze Uruchomienie Systemu

**Cel**: Uruchomić podstawowy pipeline i zrozumieć przepływ danych.

**Wymagania**: Ubuntu 22.04, ROS 2 Humble

**Czas**: 30 minut

### Krok 1: Przygotowanie środowiska

```bash
# Terminal 1: Przygotuj workspace
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Sprawdź czy pakiet jest widoczny
ros2 pkg list | grep g1_pick_and_handover
```

**Oczekiwany rezultat**: `g1_pick_and_handover` pojawia się na liście.

### Krok 2: Sprawdzenie struktury pakietu

```bash
# Zobacz pliki w pakiecie
ros2 pkg prefix g1_pick_and_handover

# Zobacz dostępne executable (node'y)
ros2 pkg executables g1_pick_and_handover
```

**Co widzisz?**
- `object_detector` - detekcja obiektów
- `pose_estimator_6d` - estymacja pozy 3D
- `human_hand_detector` - detekcja dłoni
- `execute_grasp` - chwytanie
- `execute_handover_wma` - przekazywanie

### Krok 3: Uruchomienie kamery (symulacja)

Jeśli nie masz fizycznej kamery, użyj danych testowych:

```bash
# Terminal 1: Symuluj topiki kamery
ros2 run image_publisher image_publisher_node \
    test_image.jpg \
    --ros-args \
    -r image_raw:=/camera/color/image_raw

# Terminal 2: Sprawdź czy topic istnieje
ros2 topic list | grep camera
ros2 topic echo /camera/color/image_raw --once
```

### Krok 4: Uruchomienie detektora obiektów

```bash
# Terminal 2: Uruchom object detector
ros2 run g1_pick_and_handover object_detector

# Terminal 3: Monitoruj detekcje
ros2 topic echo /object_detections
```

**Zadanie**: Połóż różne obiekty przed kamerą i obserwuj detekcje.

### Krok 5: Wizualizacja w RViz

```bash
# Terminal 4: Uruchom RViz
rviz2
```

**W RViz**:
1. Dodaj display: `Image` → Topic: `/camera/color/image_raw`
2. Dodaj display: `TF` → Zobacz transformacje
3. Dodaj display: `PoseStamped` → Topic: `/object_pose`

**Co się nauczyłeś?**
- ✅ Jak uruchomić node ROS 2
- ✅ Jak monitorować topiki
- ✅ Jak wizualizować dane w RViz
- ✅ Podstawowy przepływ danych w systemie

---

## 🎥 Tutorial 2: Zrozumienie Percepcji Wizyjnej

**Cel**: Zrozumieć jak system "widzi" świat.

**Czas**: 45 minut

### Krok 1: Detekcja obiektów z YOLOv5

```bash
# Uruchom detektor z różnymi modelami
ros2 run g1_pick_and_handover object_detector \
    --ros-args -p model_name:=yolov5s  # Szybki, mniej dokładny

ros2 run g1_pick_and_handover object_detector \
    --ros-args -p model_name:=yolov5m  # Średni

ros2 run g1_pick_and_handover object_detector \
    --ros-args -p model_name:=yolov5l  # Wolny, bardzo dokładny
```

**Eksperyment 1**: Testuj różne obiekty
- Kubek ☕
- Telefon 📱
- Książka 📖
- Laptop 💻

**Pytania do przemyślenia**:
1. Które obiekty są wykrywane najlepiej?
2. Jak oświetlenie wpływa na detekcję?
3. Czy model rozróżnia podobne obiekty (kubek vs butelka)?

### Krok 2: Estymacja pozy 3D

```bash
# Terminal 1: Detektor obiektów
ros2 run g1_pick_and_handover object_detector

# Terminal 2: Estymator pozy 6D
ros2 run g1_pick_and_handover pose_estimator_6d

# Terminal 3: Monitoruj pozycje 3D
ros2 topic echo /object_pose
```

**Eksperyment 2**: Zmierz dokładność
1. Połóż obiekt w znanej odległości (np. 50 cm od kamery)
2. Zobacz co zwraca `/object_pose`
3. Porównaj z rzeczywistą odległością

**Zapisz wyniki**:
```bash
# Nagraj dane
ros2 bag record /camera/color/image_raw /object_pose -o experiment_distance
```

### Krok 3: Zrozumienie Pinhole Camera Model

**Teoria**:
```
Konwersja 2D → 3D:
x_3d = (u - cx) * z / fx
y_3d = (v - cy) * z / fy
z_3d = depth[v, u]

gdzie:
- (u, v) = współrzędne piksela
- (cx, cy) = optyczny środek kamery
- (fx, fy) = długość ogniskowa kamery
- z = głębokość z sensora RGB-D
```

**Zadanie**: Napisz prosty skrypt testujący:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseAnalyzer(Node):
    def __init__(self):
        super().__init__('pose_analyzer')
        self.sub = self.create_subscription(
            PoseStamped, '/object_pose', self.cb, 10)
        
    def cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        distance = (x**2 + y**2 + z**2) ** 0.5
        
        self.get_logger().info(
            f'Object at: ({x:.2f}, {y:.2f}, {z:.2f}), '
            f'distance: {distance:.2f}m'
        )

def main():
    rclpy.init()
    rclpy.spin(PoseAnalyzer())

if __name__ == '__main__':
    main()
```

**Co się nauczyłeś?**
- ✅ Jak działa detekcja obiektów z YOLO
- ✅ Jak konwertować 2D → 3D
- ✅ Jak oceniać dokładność percepcji
- ✅ Pinhole camera model

---

## 🦾 Tutorial 3: Planowanie Ruchu z MoveIt 2

**Cel**: Nauczyć się planować trajektorie ramienia robota.

**Czas**: 60 minut

### Krok 1: Uruchomienie MoveIt 2 w symulacji

```bash
# Uruchom MoveIt 2 dla robota G1
ros2 launch g1_moveit_config demo.launch.py

# Otworzy się RViz z panelem MotionPlanning
```

**W RViz**:
1. W panelu `MotionPlanning`:
   - Planning Group: `arm`
   - Przeciągnij interactive marker (zielone strzałki)
2. Kliknij `Plan` aby zaplanować trajektorię
3. Kliknij `Execute` aby wykonać (w symulacji)

### Krok 2: Planowanie z kodu Python

Stwórz plik `test_moveit.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from manipulation.moveit_interface import MoveItInterface

class MoveItTester(Node):
    def __init__(self):
        super().__init__('moveit_tester')
        self.moveit = MoveItInterface()
        
        # Czekaj na inicjalizację
        self.get_logger().info('MoveIt initialized. Testing...')
        self.test_movements()
    
    def test_movements(self):
        # Test 1: Przesuń do pozycji home
        self.get_logger().info('Test 1: Moving to home')
        success = self.moveit.arm.go([0, 0, 0, 0, 0, 0], wait=True)
        self.get_logger().info(f'Result: {"Success" if success else "Failed"}')
        
        # Test 2: Przesuń do pozycji testowej
        self.get_logger().info('Test 2: Moving to test pose')
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.5
        pose.pose.orientation.w = 1.0
        
        success = self.moveit.move_to_pose(pose)
        self.get_logger().info(f'Result: {"Success" if success else "Failed"}')
        
        # Test 3: Sterowanie chwytakiem
        self.get_logger().info('Test 3: Gripper control')
        self.moveit.open_gripper()
        self.get_logger().info('Gripper opened')
        
        import time
        time.sleep(2)
        
        self.moveit.close_gripper()
        self.get_logger().info('Gripper closed')

def main():
    rclpy.init()
    node = MoveItTester()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Uruchom**:
```bash
chmod +x test_moveit.py
python3 test_moveit.py
```

### Krok 3: Unikanie kolizji

```python
# Dodaj przeszkody do sceny
from manipulation.planning_scene import add_table, add_human

# W metodzie __init__:
add_table(self.moveit.scene)
add_human(self.moveit.scene)

# Sprawdź w RViz - przeszkody powinny być widoczne
# MoveIt będzie teraz unikał kolizji z nimi
```

### Krok 4: Testowanie różnych plannerów

```python
# Zmień planner
self.moveit.arm.set_planner_id("RRTConnect")  # Domyślny, szybki
self.moveit.arm.set_planner_id("RRTstar")     # Wolniejszy, optymalizuje
self.moveit.arm.set_planner_id("PRM")         # Probabilistic Roadmap
self.moveit.arm.set_planner_id("BKPIECE")     # Bidirectional

# Zmierz czas planowania
import time
start = time.time()
success = self.moveit.move_to_pose(pose)
elapsed = time.time() - start
print(f"Planning time: {elapsed:.2f}s")
```

**Zadanie badawcze**:
Porównaj plannery:
- RRTConnect vs RRTstar: który szybszy? który generuje lepsze trajektorie?
- Jak zmiana `range` (w config/moveit.yaml) wpływa na czas planowania?

**Co się nauczyłeś?**
- ✅ Jak planować ruch z MoveIt 2
- ✅ Jak dodawać przeszkody
- ✅ Różnice między plannerami
- ✅ Interfejs Python do MoveIt 2

---

## 🤝 Tutorial 4: Kompletna Sekwencja Handover

**Cel**: Zrozumieć i uruchomić pełny system przekazywania obiektów.

**Czas**: 90 minut

### Krok 1: Przygotowanie systemu

```bash
# Terminal 1: MoveIt 2 (symulacja robota)
ros2 launch g1_moveit_config demo.launch.py

# Terminal 2: RViz (wizualizacja)
rviz2

# Terminal 3: Launch pełnego pipeline
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

### Krok 2: Symulacja człowieka wyciągającego rękę

Stwórz plik `simulate_human.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import time

class HumanSimulator(Node):
    def __init__(self):
        super().__init__('human_simulator')
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped, '/human_hand_pose', 10)
        self.intent_pub = self.create_publisher(
            Bool, '/human_reaching', 10)
        
        # Timer do publikacji
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.t = 0.0
        
    def publish_pose(self):
        # Symuluj ruch dłoni (sinusoida)
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Pozycja dłoni (symulacja wyciągnięcia)
        pose.pose.position.x = 0.6
        pose.pose.position.y = 0.1 * math.sin(self.t)
        pose.pose.position.z = 0.9
        pose.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose)
        
        # Intencja (wyciągnięcie ręki)
        intent = Bool()
        intent.data = True  # Człowiek wyciąga rękę
        self.intent_pub.publish(intent)
        
        self.t += 0.1
        
        self.get_logger().info(
            f'Publishing hand pose: x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}, z={pose.pose.position.z:.2f}'
        )

def main():
    rclpy.init()
    node = HumanSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Uruchom**:
```bash
python3 simulate_human.py
```

**Obserwuj**:
- Robot powinien reagować na pozycję dłoni
- Sprawdź logi w terminalu z `execute_handover_wma`

### Krok 3: Sekwencja TAKE_FROM_HUMAN

**Scenariusz**: Człowiek podaje obiekt robotowi.

```bash
# Terminal dodatkowy: Symuluj obiekt w dłoni
ros2 topic pub /object_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.6, y: 0.0, z: 0.9},
    orientation: {w: 1.0}
  }
}" --once

# Terminal dodatkowy: Człowiek wyciąga rękę
ros2 topic pub /human_reaching std_msgs/Bool "data: true" --once
ros2 topic pub /gripper_state std_msgs/Bool "data: false" --once
```

**Oczekiwany przepływ**:
1. WMA wykrywa: `human_reaching=True, gripper_occupied=False`
2. Decyzja: `TAKE_FROM_HUMAN`
3. Robot:
   - Oblicza pozycję handover
   - Planuje trajektorię
   - Wykonuje ruch
   - Zamyka chwytak

### Krok 4: Sekwencja GIVE_TO_HUMAN

**Scenariusz**: Robot przekazuje obiekt człowiekowi.

```bash
# Robot trzyma obiekt
ros2 topic pub /gripper_state std_msgs/Bool "data: true" --once

# Człowiek wyciąga rękę
ros2 topic pub /human_reaching std_msgs/Bool "data: true" --once
```

**Oczekiwany przepływ**:
1. WMA wykrywa: `human_reaching=True, gripper_occupied=True`
2. Decyzja: `GIVE_TO_HUMAN`
3. Robot:
   - Oblicza pozycję handover
   - Przesuwa obiekt do dłoni człowieka
   - Otwiera chwytak

### Krok 5: Analiza automatu stanów

**Narysuj diagram**:
```
Stan początkowy: IDLE
  |
  | (human_reaching && !gripper_occupied)
  v
TAKE_FROM_HUMAN
  |
  | (grasp successful)
  v
HOLD
  |
  | (human_reaching && gripper_occupied)
  v
GIVE_TO_HUMAN
  |
  | (object released)
  v
IDLE
```

**Zadanie**: Zmodyfikuj logikę decyzyjną
```python
# W execute_handover.py, metoda mock_decision():
def mock_decision(self) -> str:
    # Dodaj więcej warunków
    if self.human_reaching:
        # Sprawdź odległość
        distance = self.calculate_distance(self.human_pose)
        
        if distance < 0.3:  # Zbyt blisko
            return ACTION_IDLE
        
        if not self.gripper_occupied:
            return ACTION_TAKE_FROM_HUMAN
        else:
            return ACTION_GIVE_TO_HUMAN
    return ACTION_IDLE
```

**Co się nauczyłeś?**
- ✅ Pełny przepływ systemu handover
- ✅ Integracja percepcji, decyzji i manipulacji
- ✅ Automat stanów (FSM)
- ✅ Symulacja interakcji człowiek-robot

---

## 🧪 Tutorial 5: Eksperymentowanie z Parametrami

**Cel**: Zoptymalizować parametry systemu dla różnych scenariuszy.

**Czas**: 60 minut

### Eksperyment 1: Optymalizacja Chwytania

**Pytanie badawcze**: Jak `approach_distance` wpływa na success rate?

```bash
# Test 1: approach_distance = 0.05m
# Edytuj config/grasp_params.yaml
approach_distance: 0.05

# Test 2: approach_distance = 0.10m (domyślne)
approach_distance: 0.10

# Test 3: approach_distance = 0.15m
approach_distance: 0.15

# Dla każdego: wykonaj 10 prób chwytania, zapisz success rate
```

**Tabela wyników**:
```
| Offset | Sukces | Czas [s] | Uwagi          |
|--------|--------|----------|----------------|
| 0.05m  | 6/10   | 3.2      | Uderza w stół  |
| 0.10m  | 9/10   | 4.1      | Optymalne      |
| 0.15m  | 8/10   | 5.5      | Wolne          |
```

### Eksperyment 2: Planery MoveIt

**Pytanie badawcze**: Który planner jest najlepszy dla handover?

```python
planners = ["RRTConnect", "RRTstar", "PRM", "BKPIECE"]

for planner in planners:
    self.arm.set_planner_id(planner)
    
    # Zmierz czas i success rate
    start = time.time()
    success = self.move_to_pose(target_pose)
    elapsed = time.time() - start
    
    print(f"{planner}: {elapsed:.2f}s, {'OK' if success else 'FAIL'}")
```

### Eksperyment 3: Próg Pewności YOLOv5

**Pytanie badawcze**: Jaki `confidence_threshold` minimalizuje false positives?

```bash
# Test z różnymi progami
for threshold in 0.3 0.4 0.5 0.6 0.7 0.8 0.9
do
    echo "Testing threshold: $threshold"
    ros2 run g1_pick_and_handover object_detector \
        --ros-args -p confidence_threshold:=$threshold
    
    # Policz detekcje w 60 sekundach
    timeout 60 ros2 topic echo /object_detections | grep -c "detections"
done
```

**Wnioski**:
- Niski próg (0.3): Wiele detekcji, ale też false positives
- Wysoki próg (0.8): Mniej detekcji, ale pewniejsze
- Optimal: 0.6 (kompromis)

**Co się nauczyłeś?**
- ✅ Metodologia eksperymentów robotycznych
- ✅ Analiza wydajności systemu
- ✅ Optymalizacja parametrów
- ✅ Trade-offs (szybkość vs dokładność)

---

## 🎓 Zadania Projektowe

### Projekt 1: Custom Object Detector (Łatwy)

**Cel**: Dodać detekcję własnej klasy obiektów.

**Kroki**:
1. Zbierz dataset (50+ zdjęć)
2. Adnotuj dane (labelImg, Roboflow)
3. Wytrenuj YOLOv5:
   ```bash
   python train.py --data custom.yaml --weights yolov5s.pt --epochs 50
   ```
4. Załaduj w object_detector.py:
   ```python
   self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
   ```

### Projekt 2: Adaptive Grasping (Średni)

**Cel**: Robot dostosowuje strategię chwytania do wielkości obiektu.

**Pomysł**:
```python
def compute_adaptive_grasp(object_size):
    if object_size < 0.05:  # Mały obiekt
        return {"approach": 0.08, "speed": 0.2}
    elif object_size < 0.15:  # Średni obiekt
        return {"approach": 0.10, "speed": 0.3}
    else:  # Duży obiekt
        return {"approach": 0.15, "speed": 0.4}
```

### Projekt 3: Collision-Free Path Planning (Trudny)

**Cel**: Robot aktywnie unika ruchomych przeszkód.

**Pomysł**:
- Ciągłe aktualizowanie planning scene
- Monitorowanie czujników odległości
- Real-time replanning jeśli wykryto przeszkodę

---

**Powodzenia w nauce!** 🚀

Jeśli masz pytania lub napotkasz problemy, zajrzyj do [FAQ.md](FAQ.md) lub otwórz Issue na GitHubie.
