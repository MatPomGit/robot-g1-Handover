# Moduł Manipulacji (Manipulation Module)

## 📖 Wprowadzenie

Moduł manipulacji odpowiada za planowanie i wykonywanie ruchów ramienia robota oraz sterowanie chwytakiem. Jest to "ręka" robota, która pozwala mu fizycznie manipulować obiektami.

## 🎯 Funkcje Modułu

### 1. **Interfejs MoveIt 2** (`moveit_interface.py`)
- Uproszczony interfejs do MoveIt 2
- Planowanie trajektorii bez kolizji
- Sterowanie ramieniem robota
- Sterowanie chwytakiem (otwórz/zamknij)

### 2. **Planer Chwytania** (`grasp_planner.py`)
- Oblicza pozycję pre-grasp (nad obiektem)
- Strategia chwytania: approach -> grasp -> lift
- Ergonomiczne offsety

### 3. **Planer Przekazywania** (`handover_planner.py`)
- Oblicza pozycję przekazania obiektu
- Uwzględnia ergonomię człowieka
- Bezpieczne odległości

### 4. **Executor Chwytania** (`execute_grasp.py`)
- Wykonuje sekwencję chwytania obiektu
- Automat stanów (FSM)
- Integracja z WMA Task Manager

### 5. **Executor Przekazywania** (`execute_handover.py`)
- Wykonuje przekazywanie obiektów
- Integracja z WMA decyzjami
- Obsługa TAKE_FROM_HUMAN i GIVE_TO_HUMAN

### 6. **Scena Planowania** (`planning_scene.py`)
- Dodaje przeszkody do MoveIt 2
- Stół, człowiek, inne obiekty
- Unikanie kolizji

## 📊 Przepływ Danych

```
/object_pose -----> [Grasp Planner] -----> pre-grasp pose
                          |
                          v
                    [Execute Grasp]
                          |
                          v
                    MoveIt 2 (arm + gripper)

/human_hand_pose -> [Handover Planner] -> handover pose
                          |
                          v
                  [Execute Handover]
                          |
                          v
                    MoveIt 2 (arm + gripper)
```

## 🔧 Topiki ROS 2

### Subskrybowane (Input):
- `/object_pose` - Pozycja obiektu do chwycenia
- `/human_hand_pose` - Pozycja dłoni człowieka
- `/human_reaching` - Intencja człowieka
- `/gripper_state` - Stan chwytaka
- `/camera/color/image_raw` - Obraz z kamery (dla WMA)

### Publikowane (Output):
- MoveIt 2 internal topics (trajektorie, stany, itp.)
- Sterowanie ramieniem i chwytakiem

## 🚀 Użycie

### Uruchomienie chwytania obiektu:

```bash
# Terminal 1: MoveIt 2 (wymagane!)
ros2 launch g1_moveit_config demo.launch.py

# Terminal 2: Executor chwytania
ros2 run g1_pick_and_handover execute_grasp

# Terminal 3: Opublikuj pozycję obiektu (test)
ros2 topic pub /object_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.7}}}"
```

### Uruchomienie przekazywania obiektów:

```bash
# Terminal 1: MoveIt 2
ros2 launch g1_moveit_config demo.launch.py

# Terminal 2: Executor handover
ros2 run g1_pick_and_handover execute_handover_wma

# Terminal 3: Publikuj pozycję dłoni (test)
ros2 topic pub /human_hand_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.6, y: 0.0, z: 1.0}}}"
```

## 📚 Kluczowe Koncepty

### MoveIt 2
Framework do planowania ruchu manipulatorów:
- **Kinematyka odwrotna (IK)**: Oblicza kąty stawów dla zadanej pozycji końcówki
- **Planowanie trajektorii**: Znajduje ścieżkę bez kolizji (RRTConnect, OMPL)
- **Planning Scene**: Reprezentuje przeszkody w otoczeniu
- **Move Groups**: Grupy stawów ("arm", "gripper")

### Strategia Chwytania

```
1. APPROACH (pre-grasp):
   - Robot podnosi się nad obiekt (+10cm w górę)
   - Unika kolizji z stołem
   
2. GRASP (chwyt):
   - Robot opuszcza się do pozycji obiektu
   - Zamyka chwytak wokół obiektu
   
3. LIFT (podniesienie):
   - Robot podnosi obiekt (+15cm w górę)
   - Zabezpiecza obiekt przed upadkiem
```

### Ergonomia Przekazywania

- **Offset pionowy**: -5cm (robot nieco poniżej dłoni)
- **Bezpieczna odległość**: Robot nie podchodzi zbyt blisko
- **Naturalna pozycja**: Wygodna dla człowieka

## 🛠️ Konfiguracja

### Parametry chwytania (`config/grasp_params.yaml`):
```yaml
grasp:
  approach_distance: 0.10    # Pre-grasp offset (m)
  lift_distance: 0.15        # Wysokość podniesienia (m)
  gripper_open: 0.04         # Otwarcie chwytaka (m)
  gripper_closed: 0.0        # Zamknięcie chwytaka (m)
  max_force: 30.0            # Maksymalna siła (N)
```

### Named Targets dla chwytaka:
W pliku SRDF zdefiniuj:
```xml
<group_state name="open" group="gripper">
    <joint name="gripper_joint" value="0.04" />
</group_state>
<group_state name="closed" group="gripper">
    <joint name="gripper_joint" value="0.0" />
</group_state>
```

## 🔍 Debugowanie

### Problem: MoveIt nie planuje trajektorii

```bash
# Sprawdź czy MoveIt działa
ros2 node list | grep move_group

# Zobacz planning scene
rviz2
# Dodaj "PlanningScene" display

# Sprawdź czy cel jest osiągalny
# (czy pozycja jest w workspace robota?)
```

### Problem: Robot nie chwyta obiektu

Przyczyny:
1. **IK nie ma rozwiązania** - pozycja poza zasięgiem
2. **Kolizja** - trajektoria przechodzi przez przeszkodę
3. **Chwytak za mały** - obiekt za duży

Rozwiązania:
```python
# Zwiększ tolerancję IK
self.arm.set_goal_position_tolerance(0.01)

# Zwiększ timeout planowania
self.arm.set_planning_time(10.0)

# Spróbuj innego plannera
self.arm.set_planner_id("RRTstar")
```

### Problem: Robot uderza w stół

```python
# Sprawdź czy stół jest w planning scene
from manipulation.planning_scene import add_table
add_table(self.moveit.scene)

# Wizualizuj przeszkody w RViz
# Dodaj "Planning Scene" -> Scene Robot -> Show
```

## 📖 Tutorial dla Studentów

### Ćwiczenie 1: Zrozumienie MoveIt 2

1. Uruchom MoveIt demo:
   ```bash
   ros2 launch moveit2_tutorials demo.launch.py
   ```

2. W RViz, użyj "MotionPlanning" panel:
   - Ustaw "Planning Group" na "arm"
   - Przeciągnij interactive marker
   - Kliknij "Plan" aby zaplanować trajektorię
   - Kliknij "Execute" aby wykonać

3. **Zadanie**: Zaplanuj trajektorię do 5 różnych pozycji

### Ćwiczenie 2: Testowanie Chwytania

1. Uruchom symulator (Gazebo lub MuJoCo)

2. Uruchom execute_grasp

3. Umieść obiekt przed robotem

4. **Zadanie**: Zmierz jak często robot udanie chwyta obiekt (success rate)

### Ćwiczenie 3: Optymalizacja Offsetów

1. W `grasp_planner.py`, zmień:
   ```python
   pre.pose.position.z += 0.10  # Zmień na 0.05, 0.15, 0.20
   ```

2. Testuj różne wartości

3. **Zadanie**: Znajdź optymalny offset dla różnych wysokości obiektów

### Ćwiczenie 4: Dodawanie Przeszkód

1. Edytuj `planning_scene.py`:
   ```python
   def add_box_obstacle(scene):
       pose = PoseStamped()
       pose.header.frame_id = "base_link"
       pose.pose.position.x = 0.3
       pose.pose.position.z = 0.5
       pose.pose.orientation.w = 1.0
       scene.add_box("obstacle", pose, size=(0.2, 0.2, 0.2))
   ```

2. Dodaj do `execute_grasp.py`:
   ```python
   from manipulation.planning_scene import add_box_obstacle
   add_box_obstacle(self.moveit.scene)
   ```

3. **Zadanie**: Sprawdź czy robot omija przeszkodę

## 🔬 Zaawansowane

### Adaptacyjne Chwytanie
```python
def compute_adaptive_pregrasp(object_pose, object_height):
    """Dostosuj offset do wysokości obiektu"""
    offset = max(0.05, object_height * 0.5)
    return compute_pregrasp(object_pose, offset)
```

### Dual-Arm Manipulation
```python
# Dwa ramiona współpracujące
left_arm = MoveGroupCommander("left_arm")
right_arm = MoveGroupCommander("right_arm")

# Synchroniczne planowanie
# ...
```

### Force Control
```python
# Monitoruj siłę podczas chwytania
from sensor_msgs.msg import WrenchStamped

def force_cb(self, msg):
    force_z = msg.wrench.force.z
    if force_z > MAX_FORCE:
        self.gripper.stop()  # Zatrzymaj chwytanie
```

## 📚 Dodatkowe Zasoby

- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [OMPL Planners](https://ompl.kavrakilab.org/)
- [Grasp Planning Survey](https://arxiv.org/abs/1806.03536)
- [Robotic Grasping Book](http://www.robotics.stanford.edu/~ajuma/publications/MAS-review-of-grasping-2012.pdf)

---

**Pytania?** Otwórz Issue na GitHubie lub skonsultuj się z prowadzącym!
