# Pliki Konfiguracyjne (Config Files)

## 📖 Wprowadzenie

Katalog `config/` zawiera pliki konfiguracyjne dla różnych komponentów systemu. Pliki te definiują parametry, mapowania i diagramy używane przez robot.

## 📁 Dostępne Pliki

### 1. `grasp_params.yaml`
**Parametry chwytania obiektów**

```yaml
grasp:
  approach_distance: 0.10    # Odległość pre-grasp od obiektu (m)
  lift_distance: 0.15        # Wysokość podniesienia obiektu (m)
  gripper_open: 0.04         # Otwarcie chwytaka (m)
  gripper_closed: 0.0        # Zamknięcie chwytaka (m)
  max_force: 30.0            # Maksymalna siła chwytania (N)
```

**Opis parametrów:**

- **approach_distance (0.10m)**: 
  - Jak wysoko nad obiektem robot zatrzymuje się przed opuszczeniem
  - Większa wartość = bezpieczniejsze, ale wolniejsze
  - Mniejsza wartość = szybsze, ale ryzyko kolizji

- **lift_distance (0.15m)**:
  - Jak wysoko robot podnosi obiekt po chwyceniu
  - Musi być wystarczająco wysoko aby nie uderzyć w stół

- **gripper_open/closed**:
  - Konfiguracja otwarcia/zamknięcia chwytaka
  - Zależy od fizycznej budowy chwytaka robota G1

- **max_force (30.0N)**:
  - Maksymalna siła, z jaką chwytak może ścisnąć obiekt
  - Zapobiega zniszczeniu delikatnych obiektów

**Modyfikacja:**
```bash
# Edytuj plik
nano config/grasp_params.yaml

# Załaduj w node
parameters=[config_file]
```

### 2. `moveit.yaml`
**Konfiguracja MoveIt 2 (planowanie ruchu)**

```yaml
move_group:
  ros__parameters:
    planning_plugin: ompl_interface/OMPLPlanner
    request_adapters: >
      default_planner_request_adapters/AddTimeOptimalParameterization
      default_planner_request_adapters/FixWorkspaceBounds
      ...
    start_state_max_bounds_error: 0.1

ompl:
  planning_pipelines:
    - ompl
  
  planner_configs:
    RRTConnect:
      type: geometric::RRTConnect
      range: 0.3

  arm:
    planner_configs:
      - RRTConnect
```

**Opis sekcji:**

- **planning_plugin**: Plugin do planowania (OMPL - Open Motion Planning Library)
- **request_adapters**: Filtry przetwarzające plany (wygładzanie, time optimization)
- **RRTConnect**: Algorytm planowania trajektorii
  - **range (0.3)**: Krok próbkowania przestrzeni [metry]
  - Mniejszy range = dokładniejsze planowanie, wolniejsze
  - Większy range = szybsze, mniej precyzyjne

**Inne plannery OMPL:**
- **RRTstar**: Optymalizuje trajektorię (wolniejszy)
- **PRM**: Probabilistic Roadmap (dla wielu zapytań)
- **BKPIECE**: Bidirectional Kinematic Planning

### 3. `Mapowanie WMA na trajektorie MoveIt 2.yaml`
**Tabela mapowania akcji WMA na komendy MoveIt 2**

```yaml
| WMA Action        | MoveIt Trajectory               | Chwytak         |
| ----------------- | ------------------------------- | --------------- |
| `TAKE_FROM_HUMAN` | move_to_pose(pre-grasp → grasp) | close_gripper() |
| `GIVE_TO_HUMAN`   | move_to_pose(handover_pose)     | open_gripper()  |
| `IDLE`            | brak ruchu                      | brak ruchu      |
```

**Opis:**
- **TAKE_FROM_HUMAN**: 
  1. Robot oblicza pozycję handover (z offsetem)
  2. Planuje trajektorię od aktualnej pozycji do handover
  3. Wykonuje ruch
  4. Zamyka chwytak

- **GIVE_TO_HUMAN**:
  1. Robot (z obiektem) oblicza pozycję handover
  2. Planuje trajektorię
  3. Wykonuje ruch
  4. Otwiera chwytak (człowiek bierze obiekt)

- **IDLE**:
  - Robot pozostaje w miejscu
  - Monitoruje otoczenie
  - Czeka na zmianę sytuacji

### 4. `DiagramFSM.txt`
**Diagram automatu stanów (Finite State Machine)**

```
          ┌───────────────┐
          │     IDLE      │  ← Stan początkowy
          └──────┬────────┘
                 │
     human_reaching=True
     gripper_occupied=False
                 ▼
      ┌────────────────────┐
      │  TAKE_FROM_HUMAN   │  ← Robot bierze obiekt
      └──────┬─────────────┘
             │
 gripper_closed=True
             ▼
          ┌───────────────┐
          │     HOLD      │  ← Robot trzyma obiekt
          └──────┬────────┘
                 │
 human_reaching=True
 gripper_occupied=True
                 ▼
      ┌────────────────────┐
      │  GIVE_TO_HUMAN     │  ← Robot daje obiekt
      └──────┬─────────────┘
             │
 gripper_open=True
             ▼
          ┌───────────────┐
          │     IDLE      │  ← Powrót do początku
          └───────────────┘
```

**Przejścia:**
1. **IDLE → TAKE_FROM_HUMAN**: Człowiek wyciąga rękę z obiektem
2. **TAKE_FROM_HUMAN → HOLD**: Robot chwycił obiekt
3. **HOLD → GIVE_TO_HUMAN**: Człowiek wyciąga pustą rękę
4. **GIVE_TO_HUMAN → IDLE**: Robot przekazał obiekt

### 5. `Struktura pakietów ROS 2.txt`
**Diagram struktury pakietu**

```
g1_pick_and_handover/
├── perception/          # Moduł percepcji
│   ├── object_detector.py
│   ├── pose_estimator_6d.py
│   └── launch_perception.py
├── manipulation/        # Moduł manipulacji
│   ├── moveit_interface.py
│   ├── grasp_planner.py
│   └── execute_grasp.py
├── decision/            # Moduł decyzyjny
│   └── wma_task_manager.py
├── launch/              # Launch files
│   └── full_pipeline.launch.py
└── config/              # Pliki konfiguracyjne
    ├── grasp_params.yaml
    └── moveit.yaml
```

## 🛠️ Użycie Konfiguracji

### Ładowanie parametrów w node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Deklaruj parametr z wartością domyślną
        self.declare_parameter('approach_distance', 0.10)
        
        # Odczytaj wartość
        approach_dist = self.get_parameter('approach_distance').value
        
        self.get_logger().info(f'Approach distance: {approach_dist}')
```

### Przekazywanie z launch file:

```python
from ament_index_python.packages import get_package_share_directory
import os

config_file = os.path.join(
    get_package_share_directory('g1_pick_and_handover'),
    'config',
    'grasp_params.yaml'
)

Node(
    package='g1_pick_and_handover',
    executable='execute_grasp',
    parameters=[config_file]
)
```

### Nadpisywanie z linii komend:

```bash
ros2 run g1_pick_and_handover execute_grasp \
    --ros-args -p approach_distance:=0.15
```

## 📖 Tutorial dla Studentów

### Ćwiczenie 1: Modyfikacja Parametrów Chwytania

1. Otwórz `grasp_params.yaml`

2. Zmień `approach_distance` na różne wartości:
   - 0.05 (ryzykowne, szybkie)
   - 0.10 (domyślne)
   - 0.20 (bezpieczne, wolne)

3. **Zadanie**: Zmierz czas chwytania dla każdej wartości

### Ćwiczenie 2: Testowanie Plannerów MoveIt

1. W `moveit.yaml`, zmień planner:
   ```yaml
   arm:
     planner_configs:
       - RRTstar  # Zamiast RRTConnect
   ```

2. **Zadanie**: Porównaj:
   - Czas planowania
   - Płynność trajektorii
   - Success rate

### Ćwiczenie 3: Twój Plik Konfiguracyjny

1. Utwórz `my_params.yaml`:
   ```yaml
   my_node:
     ros__parameters:
       detection_rate: 30.0
       confidence_threshold: 0.7
       use_gpu: true
   ```

2. Załaduj w swoim node

3. **Zadanie**: Dodaj możliwość zmiany parametrów bez rebuildu

## 🔍 Debugowanie Konfiguracji

### Sprawdzenie parametrów node:

```bash
# Lista parametrów
ros2 param list /my_node

# Wartość parametru
ros2 param get /my_node approach_distance

# Zmiana w locie
ros2 param set /my_node approach_distance 0.15
```

### Problem: Parametr nie ładuje się

```bash
# Sprawdź czy plik istnieje
ls -la config/grasp_params.yaml

# Sprawdź syntax YAML
yamllint config/grasp_params.yaml

# Sprawdź logi node
ros2 run ... --ros-args --log-level debug
```

## 📚 Dobre Praktyki

1. **Komentarze**: Zawsze dodawaj komentarze w YAML
   ```yaml
   approach_distance: 0.10  # Meters - distance above object
   ```

2. **Jednostki**: Zawsze określaj jednostki
   ```yaml
   max_force: 30.0  # Newtons
   timeout: 5.0     # Seconds
   ```

3. **Walidacja**: Sprawdzaj zakresy w kodzie
   ```python
   dist = self.get_parameter('approach_distance').value
   assert 0.01 <= dist <= 0.50, "Invalid approach distance"
   ```

4. **Wersjonowanie**: Commituj zmiany w config
   ```bash
   git commit -m "Increase approach_distance for safety"
   ```

## 📚 Dodatkowe Zasoby

- [ROS 2 Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [YAML Syntax](https://yaml.org/)
- [MoveIt Configuration](https://moveit.picknik.ai/main/doc/examples/examples.html)

---

**Pytania?** Otwórz Issue na GitHubie!
