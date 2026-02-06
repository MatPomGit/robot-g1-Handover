# Pliki Uruchomieniowe (Launch Files)

## 📖 Wprowadzenie

Katalog `launch/` zawiera pliki uruchomieniowe ROS 2, które pozwalają na jednoczesne uruchomienie wielu node'ów z odpowiednimi parametrami i konfiguracją.

## 🎯 Dostępne Launch Files

### 1. `full_pipeline.launch.py`
**Podstawowy pipeline do testowania**

Uruchamia:
- `human_hand_detector` - Detekcja dłoni człowieka
- `execute_handover` - Executor przekazywania obiektów

**Użycie:**
```bash
ros2 launch g1_pick_and_handover full_pipeline.launch.py
```

**Idealny dla:**
- Prototypowania
- Debugowania handover
- Testowania bez pełnej percepcji

### 2. `full_handover_pipeline.launch.py`
**Kompletny system przekazywania obiektów**

Uruchamia:
- `static_tf_camera` - Transformacja kamery
- `object_detector` - Detekcja obiektów (YOLOv5)
- `pose_estimator_6d` - Estymacja pozy 3D
- `human_hand_detector` - Detekcja dłoni
- `execute_handover_wma` - Executor z WMA

**Użycie:**
```bash
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

**Idealny dla:**
- Demonstracji kompletnego systemu
- Produkcyjnego użycia
- Testów integracyjnych

## 📊 Porównanie Launch Files

| Feature | full_pipeline | full_handover_pipeline |
|---------|---------------|------------------------|
| Percepcja obiektów | ❌ | ✅ |
| Percepcja dłoni | ✅ | ✅ |
| Estymacja pozy 3D | ❌ | ✅ |
| TF transformacje | ❌ | ✅ |
| WMA decyzje | ❌ | ✅ |
| Idealny dla | Testowanie | Produkcja |

## 🚀 Użycie

### Podstawowe uruchomienie:

```bash
# Uruchom launch file
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

### Z parametrami (przykład):

```bash
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py \
    confidence_threshold:=0.7 \
    use_gpu:=true
```

### Debugowanie:

```bash
# Zobacz co się uruchamia
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py --show-args

# Zobacz logi wszystkich node'ów
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py --screen
```

## 🛠️ Tworzenie Własnego Launch File

### Szablon:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Twój node 1
        Node(
            package='g1_pick_and_handover',
            executable='my_node',
            name='my_node_name',
            output='screen',
            parameters=[{
                'param1': 'value1',
                'param2': 42
            }]
        ),
        
        # Twój node 2
        Node(
            package='g1_pick_and_handover',
            executable='another_node',
            remappings=[
                ('/input_topic', '/my_input')
            ]
        )
    ])
```

### Zaawansowane opcje:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Deklaruj argument
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Użyj symulacji'
    )
    
    # Użyj argumentu
    use_sim = LaunchConfiguration('use_sim')
    
    return LaunchDescription([
        use_sim_arg,
        
        Node(
            package='g1_pick_and_handover',
            executable='my_node',
            parameters=[{'use_simulation': use_sim}]
        )
    ])
```

## 📚 Dobre Praktyki

### 1. **Namespace**
Grupuj node'y w namespace:
```python
Node(
    package='g1_pick_and_handover',
    executable='my_node',
    namespace='robot1'  # Topic: /robot1/my_topic
)
```

### 2. **Output**
Włącz output dla debugowania:
```python
Node(
    package='g1_pick_and_handover',
    executable='my_node',
    output='screen'  # Zobacz print() i logi
)
```

### 3. **Respawn**
Automatyczny restart po crash:
```python
Node(
    package='g1_pick_and_handover',
    executable='my_node',
    respawn=True,
    respawn_delay=2.0
)
```

### 4. **Parametry z pliku**
Ładuj parametry z YAML:
```python
from ament_index_python.packages import get_package_share_directory
import os

config_file = os.path.join(
    get_package_share_directory('g1_pick_and_handover'),
    'config',
    'params.yaml'
)

Node(
    package='g1_pick_and_handover',
    executable='my_node',
    parameters=[config_file]
)
```

## 🔍 Debugowanie

### Sprawdzenie co się uruchamia:

```bash
# Lista aktywnych node'ów
ros2 node list

# Info o node
ros2 node info /my_node

# Lista topików
ros2 topic list

# Echo topicu
ros2 topic echo /my_topic
```

### Problem: Node nie startuje

```bash
# Sprawdź logi
ros2 launch ... --screen

# Sprawdź czy executable istnieje
ros2 pkg executables g1_pick_and_handover

# Sprawdź zależności
rosdep check g1_pick_and_handover
```

### Problem: Wrong namespace/remapping

```bash
# Zobacz graf node'ów
rqt_graph

# Zobacz wszystkie topiki z namespace
ros2 topic list | grep robot1
```

## 📖 Tutorial dla Studentów

### Ćwiczenie 1: Twój Pierwszy Launch File

1. Utwórz `my_launch.py`:
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   
   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='g1_pick_and_handover',
               executable='human_hand_detector'
           )
       ])
   ```

2. Uruchom:
   ```bash
   ros2 launch g1_pick_and_handover my_launch.py
   ```

3. **Zadanie**: Dodaj drugi node (object_detector)

### Ćwiczenie 2: Parametry

1. Dodaj parametry:
   ```python
   Node(
       package='g1_pick_and_handover',
       executable='object_detector',
       parameters=[{
           'confidence_threshold': 0.8,
           'model_size': 'yolov5m'
       }]
   )
   ```

2. **Zadanie**: Testuj z różnymi wartościami confidence_threshold

### Ćwiczenie 3: Launch Arguments

1. Dodaj argument:
   ```python
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   
   confidence_arg = DeclareLaunchArgument(
       'confidence',
       default_value='0.6'
   )
   
   confidence = LaunchConfiguration('confidence')
   
   # Użyj w Node
   parameters=[{'confidence_threshold': confidence}]
   ```

2. Uruchom z argumentem:
   ```bash
   ros2 launch ... confidence:=0.8
   ```

3. **Zadanie**: Dodaj więcej argumentów (use_gpu, model_size)

## 📚 Dodatkowe Zasoby

- [ROS 2 Launch Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Launch File Examples](https://github.com/ros2/launch/tree/humble/launch/examples)
- [Launch XML/YAML](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html)

---

**Pytania?** Otwórz Issue na GitHubie!
