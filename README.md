# Robot G1 - System Przekazywania Obiektów (Handover)

## 📖 Wprowadzenie

Ten projekt demonstruje system interakcji człowiek-robot dla robota humanoidalnego Unitree G1. System umożliwia robotowi:
- **Odbieranie obiektów** od człowieka (TAKE_FROM_HUMAN)
- **Przekazywanie obiektów** człowiekowi (GIVE_TO_HUMAN)
- Wykorzystanie **World Model AI (WMA)** do podejmowania inteligentnych decyzji

Projekt został stworzony w celach edukacyjnych, aby pomóc studentom zrozumieć:
- Programowanie robotów humanoidalnych w ROS 2
- Percepcję wizyjną (wykrywanie obiektów i dłoni człowieka)
- Planowanie ruchu ramienia robota z użyciem MoveIt 2
- Integrację sztucznej inteligencji (WMA) z systemami robotycznymi

## 🎯 Architektura Systemu

System składa się z czterech głównych modułów:

### 1. 📸 **Perception (Percepcja)**
Odpowiada za rozumienie otoczenia robota poprzez kamery i czujniki.

**Pliki:**
- `human_hand_detector.py` - Wykrywa pozycję dłoni człowieka w przestrzeni 3D
- `object_detector.py` - Wykrywa obiekty na obrazie z kamery (używa YOLOv5)
- `pose_estimator_6d.py` - Oblicza pozę 6D obiektów (pozycja x,y,z + orientacja)
- `static_tf_camera.py` - Definiuje transformację kamery względem bazy robota

**Jak to działa:**
1. Kamera RGB-D rejestruje obraz kolorowy i mapę głębokości
2. YOLOv5 wykrywa obiekty na obrazie RGB
3. Używając mapy głębokości, system oblicza pozycję 3D obiektu
4. MediaPipe/OpenPose wykrywa kluczowe punkty dłoni człowieka

### 2. 🦾 **Manipulation (Manipulacja)**
Odpowiada za planowanie i wykonywanie ruchów ramienia robota.

**Pliki:**
- `moveit_interface.py` - Interface do MoveIt 2 (planowanie trajektorii)
- `handover_planner.py` - Oblicza pozycję do przekazania obiektu
- `grasp_planner.py` - Oblicza pozycję pre-grasp przedchwyceniem
- `execute_grasp.py` - Wykonuje sekwencję chwytania obiektu
- `execute_handover.py` - Wykonuje sekwencję przekazywania obiektu
- `planning_scene.py` - Dodaje przeszkody (stół, człowiek) do sceny planowania

**Jak to działa:**
1. MoveIt 2 planuje trajektorię ruchu ramienia bez kolizji
2. System oblicza pozycje pośrednie (pre-grasp, grasp, handover)
3. Gripper (chwytak) otwiera się i zamyka w odpowiednich momentach
4. Robot wykonuje ruch płynnie i bezpiecznie

### 3. 🧠 **Decision (Decyzje)**
Wykorzystuje sztuczną inteligencję do podejmowania decyzji o akcjach robota.

**Pliki:**
- `wma_handover_manager.py` - Manager decyzji WMA dla przekazywania obiektów
- `wma_task_manager.py` - Manager stanów automatu skończonego (FSM)

**Jak to działa:**
1. World Model AI (WMA) analizuje obserwacje z kamer i czujników
2. WMA przewiduje intencje człowieka (czy chce dać/wziąć obiekt)
3. System podejmuje decyzję: TAKE_FROM_HUMAN, GIVE_TO_HUMAN lub IDLE
4. FSM (Finite State Machine) zarządza przejściami między stanami

### 4. 🚀 **Launch**
Pliki uruchomieniowe, które startują wszystkie komponenty systemu.

**Pliki:**
- `full_pipeline.launch.py` - Uruchamia podstawowy pipeline
- `full_handover_pipeline.launch.py` - Uruchamia kompletny system handover

## 🔧 Instalacja

### Wymagania wstępne

1. **Ubuntu 22.04 LTS** (zalecane)
2. **ROS 2 Humble** lub nowszy
3. **Python 3.10+**
4. **MoveIt 2** (dla planowania ruchu)
5. **Kamera RGB-D** (np. Intel RealSense D435)

### Krok 1: Instalacja ROS 2

```bash
# Dodaj repozytorium ROS 2
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Dodaj źródła ROS 2
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Zainstaluj ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### Krok 2: Instalacja MoveIt 2

```bash
sudo apt install ros-humble-moveit
```

### Krok 3: Instalacja zależności Python

```bash
# Zainstaluj pip jeśli nie masz
sudo apt install python3-pip

# Zainstaluj wymagane pakiety z pliku requirements.txt
pip3 install -r requirements.txt
```

### Krok 4: Klonowanie i konfiguracja workspace

```bash
# Utwórz workspace ROS 2
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Sklonuj to repozytorium
git clone https://github.com/MatPomGit/robot-g1-Handover.git

# Zainstaluj zależności ROS 2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Zbuduj workspace
colcon build
source install/setup.bash
```

### Krok 5: Konfiguracja modelu robota G1

```bash
# Pobierz opis robota Unitree G1
cd ~/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros.git
cd ~/ros2_ws
colcon build --packages-select g1_description
```

## 🎮 Użycie

### Uruchomienie pełnego systemu

```bash
# W pierwszym terminalu - uruchom źródło workspace
cd ~/ros2_ws
source install/setup.bash

# Uruchom kompletny pipeline handover
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

### Uruchomienie poszczególnych komponentów

#### 1. Tylko percepcja (wykrywanie obiektów i dłoni)

```bash
# Terminal 1: Detekcja obiektów
ros2 run g1_pick_and_handover object_detector

# Terminal 2: Estymacja pozy 6D
ros2 run g1_pick_and_handover pose_estimator_6d

# Terminal 3: Detekcja dłoni człowieka
ros2 run g1_pick_and_handover human_hand_detector
```

#### 2. Tylko manipulacja (test ruchu ramienia)

```bash
# Uruchom interfejs MoveIt
ros2 run g1_pick_and_handover moveit_interface
```

#### 3. System decyzyjny z WMA

```bash
ros2 run g1_pick_and_handover execute_handover_wma
```

## 📊 Przepływ Danych (Topics ROS 2)

System komunikuje się poprzez następujące topiki ROS 2:

| Topic | Typ | Opis |
|-------|-----|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Obraz RGB z kamery |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Mapa głębokości z kamery |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Kalibracja kamery |
| `/object_detections` | `vision_msgs/Detection2DArray` | Wykryte obiekty (2D bounding boxes) |
| `/object_pose` | `geometry_msgs/PoseStamped` | Pozycja 3D obiektu |
| `/human_hand_pose` | `geometry_msgs/PoseStamped` | Pozycja dłoni człowieka |
| `/human_reaching` | `std_msgs/Bool` | Czy człowiek wyciąga rękę |
| `/gripper_state` | `std_msgs/Bool` | Stan chwytaka (otwarty/zamknięty) |

## 🔄 Automat Stanów (FSM)

System działa według następującego automatu stanów:

```
         ┌───────────────┐
         │     IDLE      │  ← Stan początkowy
         └──────┬────────┘
                │
    human_reaching=True
    gripper_occupied=False
                ▼
     ┌────────────────────┐
     │  TAKE_FROM_HUMAN   │  ← Robot podjeżdża do dłoni człowieka
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
     │  GIVE_TO_HUMAN     │  ← Robot przekazuje obiekt człowiekowi
     └──────┬─────────────┘
            │
gripper_open=True
            ▼
         ┌───────────────┐
         │     IDLE      │  ← Powrót do stanu początkowego
         └───────────────┘
```

## 🎓 Tutoriale dla Studentów

### Tutorial 1: Zrozumienie percepcji wizyjnej

1. Uruchom detektor obiektów:
   ```bash
   ros2 run g1_pick_and_handover object_detector
   ```

2. Obserwuj wykryte obiekty:
   ```bash
   ros2 topic echo /object_detections
   ```

3. **Zadanie**: Połóż różne obiekty przed kamerą i obserwuj, jak system je wykrywa.

### Tutorial 2: Planowanie ruchu z MoveIt 2

1. Uruchom MoveIt 2 w trybie wizualizacji:
   ```bash
   ros2 launch moveit2_tutorials demo.launch.py
   ```

2. **Zadanie**: Używając RViz, zaplanuj trajektorię ruchu ramienia do różnych pozycji.

### Tutorial 3: Integracja WMA

1. Przestudiuj plik `wma_handover_manager.py`
2. Zrozum, jak obserwacje są przekształcane w tensory PyTorch
3. **Zadanie**: Dodaj nową obserwację (np. kolor obiektu) do systemu WMA

## 📁 Struktura Katalogów

```
robot-g1-Handover/
├── README.md                    # Ten plik
├── CONTRIBUTING.md              # Przewodnik dla kontrybutorów
├── requirements.txt             # Zależności Python
├── package.xml                  # Deskryptor pakietu ROS 2
├── setup.py                     # Konfiguracja instalacji Python
├── setup.cfg                    # Konfiguracja setuptools
├── perception/                  # Moduł percepcji
│   ├── __init__.py
│   ├── README.md
│   ├── human_hand_detector.py
│   ├── object_detector.py
│   ├── pose_estimator_6d.py
│   └── static_tf_camera.py
├── manipulation/                # Moduł manipulacji
│   ├── __init__.py
│   ├── README.md
│   ├── moveit_interface.py
│   ├── handover_planner.py
│   ├── grasp_planner.py
│   ├── execute_grasp.py
│   ├── execute_handover.py
│   └── planning_scene.py
├── decision/                    # Moduł decyzyjny
│   ├── __init__.py
│   ├── README.md
│   ├── wma_handover_manager.py
│   └── wma_task_manager.py
├── launch/                      # Pliki uruchomieniowe
│   ├── README.md
│   ├── full_pipeline.launch.py
│   ├── full_handover_pipeline.launch.py
│   └── launch_perception.launch.py
└── config/                      # Pliki konfiguracyjne
    ├── README.md
    ├── grasp_params.yaml
    └── moveit.yaml
```

## 🔍 Wizualizacja z MuJoCo

Aby zwizualizować robota G1 w symulatorze:

```bash
# Zainstaluj MuJoCo
pip install mujoco

# Uruchom viewer
python -m mujoco.viewer

# Przeciągnij plik MJCF/URDF modelu G1 do okna viewera
# Pliki modeli: https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description
```

## 🛠️ Konfiguracja

### Parametry chwytania (grasp_params.yaml)

- `approach_distance`: Odległość podejścia przed chwyceniem (0.10m)
- `lift_distance`: Wysokość podniesienia pochwyceniu (0.15m)
- `gripper_open`: Otwarcie chwytaka (0.04m)
- `gripper_closed`: Zamknięcie chwytaka (0.0m)
- `max_force`: Maksymalna siła chwytaka (30.0N)

### Parametry MoveIt 2 (moveit.yaml)

- Planner: RRTConnect (algorytm planowania trajektorii)
- Range: 0.3 (zakres próbkowania dla RRT)

## 🐛 Rozwiązywanie problemów

### Problem: Kamera nie jest wykrywana

```bash
# Sprawdź, czy kamera jest podłączona
rs-enumerate-devices

# Uruchom node kamery RealSense
ros2 run realsense2_camera realsense2_camera_node
```

### Problem: MoveIt 2 nie planuje trajektorii

1. Sprawdź, czy robot jest poprawnie zdefiniowany w URDF
2. Upewnij się, że scena planowania nie zawiera kolizji
3. Zwiększ timeout planowania

### Problem: WMA nie działa

1. Upewnij się, że masz zainstalowany PyTorch z CUDA (jeśli używasz GPU)
2. Sprawdź ścieżkę do checkpointu WMA
3. Sprawdź, czy obserwacje mają poprawny format

## 📚 Dokumentacja Projektu

### 📖 Podstawowa Dokumentacja
- **[FAQ.md](FAQ.md)** - Najczęściej zadawane pytania i rozwiązywanie problemów
- **[TUTORIALS.md](TUTORIALS.md)** - Szczegółowe tutoriale krok po kroku dla studentów
- **[GLOSSARY.md](GLOSSARY.md)** - Słownik terminów i konceptów
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Szybka ściąga z komendami i parametrami

### 🏗️ Dokumentacja Techniczna
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Architektura systemu i przepływ danych
- **[TESTING.md](TESTING.md)** - Strategia i implementacja testów
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - Przewodnik dla kontrybutorów

### 📁 Dokumentacja Modułów
- **[perception/README.md](perception/README.md)** - Moduł percepcji (kamery, detekcja)
- **[manipulation/README.md](manipulation/README.md)** - Moduł manipulacji (MoveIt, grasp)
- **[decision/README.md](decision/README.md)** - Moduł decyzyjny (WMA, FSM)
- **[launch/README.md](launch/README.md)** - Pliki uruchomieniowe
- **[config/README.md](config/README.md)** - Pliki konfiguracyjne

## 🌐 Zewnętrzne Zasoby

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/index.html)
- [Unitree G1 Robot](https://www.unitree.com/g1)
- [World Model AI](https://worldmodels.github.io/)
- [YOLOv5 Documentation](https://github.com/ultralytics/yolov5)

## 👥 Autorzy i Licencja

Ten projekt jest open-source i dostępny do celów edukacyjnych.

## 🤝 Współpraca

Jeśli masz pytania lub sugestie, otwórz Issue lub Pull Request na GitHubie!

---

**Uwaga**: Ten projekt jest w fazie rozwoju i służy celom edukacyjnym. Przed użyciem na prawdziwym robocie należy dokładnie przetestować wszystkie funkcje w symulacji.
