# 🤖 Robot G1 - System Przekazywania Obiektów (Handover)

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Open_Source-yellow)](LICENSE)

> **Edukacyjny system robotyki** demonstrujący inteligentną interakcję człowiek-robot z wykorzystaniem percepcji wizyjnej, planowania ruchu i AI.

---

## 🚀 Szybki Start (Quick Start)

**Chcesz od razu zobaczyć system w akcji? Skorzystaj z tego 5-minutowego przewodnika:**

```bash
# 1. Klonuj repozytorium
git clone https://github.com/MatPomGit/robot-g1-Handover.git
cd robot-g1-Handover

# 2. Zainstaluj zależności (wymaga Ubuntu 22.04 + ROS 2 Humble)
./scripts/quick_setup.sh

# 3. Uruchom demonstrację
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py

# 4. W osobnym terminalu - odtwórz własne dane z kamery (rosbag z odpowiednimi tematami)
ros2 bag play <twoj_plik_rosbag.bag> --loop
```

📖 **Pierwszy raz z ROS 2?** Zobacz [szczegółową instrukcję instalacji](#-instalacja) poniżej.

---

## 💡 Co robi ten system?

Ten projekt demonstruje system interakcji człowiek-robot dla robota humanoidalnego **Unitree G1**. Robot może:

| Funkcja | Opis | Status |
|---------|------|--------|
| 🤝 **Odbieranie obiektów** | Robot bierze przedmiot od człowieka | ✅ Działa |
| 📦 **Przekazywanie obiektów** | Robot podaje przedmiot człowiekowi | ✅ Działa |
| 🧠 **Inteligentne decyzje** | AI przewiduje intencje człowieka | ⚠️ Wymaga WMA |
| 👁️ **Percepcja wizualna** | Wykrywa obiekty i dłonie (YOLO + MediaPipe) | ✅ Działa |
| 🦾 **Planowanie ruchu** | Bezpieczne trajektorie z MoveIt 2 | ✅ Działa |

### 🎓 Dla kogo?

- **Studenci robotyki** - naucz się ROS 2, MoveIt 2, percepcji wizyjnej
- **Nauczyciele** - gotowy projekt do demonstracji HRI (Human-Robot Interaction)
- **Badacze** - platforma do eksperymentów z AI w robotyce
- **Entuzjaści** - poznaj jak działają zaawansowane systemy robotyczne

---

## 📊 Jak to działa? (Wizualny przegląd)

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                       │
│  CZŁOWIEK wyciąga rękę                    ROBOT wykrywa intencję    │
│      👤 🤚                                       🤖                   │
│       │                                          ↑                   │
│       └──────────────────────────────────────────┘                   │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘

          ┌──────────────┐         ┌──────────────┐         ┌──────────────┐
          │   PERCEPCJA  │────────▶│   DECYZJE    │────────▶│  MANIPULACJA │
          │              │         │              │         │              │
          │ 👁️ Kamery   │         │ 🧠 AI/WMA    │         │ 🦾 MoveIt 2  │
          │ 🎯 YOLO      │         │ 📋 FSM       │         │ ✋ Gripper    │
          │ ✋ MediaPipe  │         │              │         │              │
          └──────────────┘         └──────────────┘         └──────────────┘
               ↓                         ↓                        ↓
          Wykrywa obiekty       Podejmuje decyzję          Wykonuje ruch
          i pozycję dłoni       (Brać/Dać/Czekać)         bezkolizyjnie
```

**Przykładowy przepływ:**
1. 👁️ Kamera widzi człowieka wyciągającego rękę z kubkiem
2. 🎯 YOLO wykrywa kubek, MediaPipe wykrywa dłoń
3. 🧠 AI decyduje: "Człowiek chce mi dać kubek" → TAKE_FROM_HUMAN
4. 🦾 Robot planuje trajektorię i podjeżdża do dłoni
5. ✋ Gripper chwyta kubek, robot go podnosi

---

## 🎯 Architektura Systemu

<details>
<summary><b>📂 Kliknij aby zobaczyć szczegółową architekturę</b></summary>

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

</details>

---

## 🔧 Instalacja

### ⚡ Ekspresowa instalacja (Dla doświadczonych użytkowników ROS 2)

Jeśli masz już **Ubuntu 22.04 + ROS 2 Humble + MoveIt 2**, wystarczy:

```bash
# Sklonuj i zbuduj
git clone https://github.com/MatPomGit/robot-g1-Handover.git ~/ros2_ws/src/robot-g1-Handover
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r src/robot-g1-Handover/requirements.txt
colcon build --packages-select g1_pick_and_handover
source install/setup.bash

# Gotowe! 🎉
```

### 📚 Szczegółowa instalacja (Krok po kroku)

<details>
<summary><b>👆 Kliknij tutaj jeśli instalujesz po raz pierwszy</b></summary>

#### Wymagania wstępne

| Wymaganie | Wersja | Sprawdzenie |
|-----------|--------|-------------|
| Ubuntu | 22.04 LTS | `lsb_release -a` |
| ROS 2 | Humble+ | `ros2 --version` |
| Python | 3.10+ | `python3 --version` |
| MoveIt 2 | Humble | `ros2 pkg list \| grep moveit` |

#### Krok 1: Instalacja ROS 2 Humble

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

⚠️ **Uwaga:** Jeśli masz GPU NVIDIA, zainstaluj PyTorch z CUDA dla lepszej wydajności YOLO:
```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118
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

✅ **Gratulacje!** Instalacja zakończona. Przejdź do sekcji [Użycie](#-użycie).

</details>

---

## 🎮 Użycie

### 🚦 Opcja 1: Uruchomienie kompletnego systemu (Zalecane)

```bash
# TERMINAL 1: Uruchom główny system
cd ~/ros2_ws
source install/setup.bash
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

🎉 **System uruchomiony!** Zobaczysz logi poszczególnych modułów.

### 🧪 Opcja 2: Uruchomienie w trybie testowym (bez fizycznego robota)

```bash
# TERMINAL 1: Symuluj dane z kamery
ros2 bag play test_data.bag --loop

# TERMINAL 2: Uruchom system
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py

# TERMINAL 3: Wizualizacja w RViz
rviz2
```

### 🔍 Opcja 3: Uruchomienie poszczególnych komponentów (Debug)

<details>
<summary><b>📦 Kliknij aby zobaczyć komponenty do uruchomienia osobno</b></summary>

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

</details>

---

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

---

## ❓ Najczęstsze pytania (Quick FAQ)

<details>
<summary><b>❌ System nie uruchamia się - co robić?</b></summary>

**Kroki diagnozy:**
1. Sprawdź czy ROS 2 jest zainstalowany: `ros2 --version`
2. Source workspace: `source ~/ros2_ws/install/setup.bash`
3. Sprawdź pakiet: `ros2 pkg list | grep g1_pick_and_handover`
4. Zobacz szczegóły: [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

</details>

<details>
<summary><b>📷 Kamera nie działa - jak naprawić?</b></summary>

**Szybkie rozwiązania:**
1. Fizyczna kamera: `ros2 run realsense2_camera realsense2_camera_node`
2. Bez kamery: Użyj bag file `ros2 bag play test_data.bag --loop`
3. Zobacz: [TROUBLESHOOTING.md](TROUBLESHOOTING.md#-problem-brak-kamery)

</details>

<details>
<summary><b>🎯 YOLO nie wykrywa obiektów - dlaczego?</b></summary>

**Sprawdź:**
- Czy obiekt jest w zbiorze COCO (80 klas)?
- Obniż próg: `--ros-args -p confidence_threshold:=0.3`
- Większy model: `-p model_name:=yolov5m`
- Zobacz: [FAQ.md](FAQ.md#q-yolov5-nie-wykrywa-obiektów)

</details>

<details>
<summary><b>⚠️ "WMA not available" - czy to błąd?</b></summary>

**NIE, to normalne!** 🎉

System automatycznie używa prostego trybu decyzyjnego (if-else), który działa świetnie do nauki i testów. WMA jest opcjonalny i zaawansowany. Zobacz: [FAQ.md](FAQ.md#q-wma-nie-jest-dostępny)

</details>

<details>
<summary><b>🦾 MoveIt nie planuje - co sprawdzić?</b></summary>

**Checklist:**
- [ ] Cel w zasięgu? (0.3-0.8m dla G1)
- [ ] Zwiększ timeout: `self.arm.set_planning_time(15.0)`
- [ ] Zwiększ tolerancję: `self.arm.set_goal_position_tolerance(0.01)`
- [ ] Zobacz: [FAQ.md](FAQ.md#q-moveit-2-nie-planuje-trajektorii)

</details>

**Więcej pytań?** Zobacz pełne [FAQ.md](FAQ.md)

---

## 🐛 Rozwiązywanie problemów

### ⚡ Szybkie rozwiązania najczęstszych problemów

<details>
<summary><b>❌ Błąd: "package not found" podczas budowania</b></summary>

**Problem:** `colcon build` nie znajduje zależności.

**Rozwiązanie:**
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

**Sprawdź czy pomogło:**
```bash
ros2 pkg list | grep g1_pick_and_handover
# Powinno wyświetlić: g1_pick_and_handover
```
</details>

<details>
<summary><b>📷 Błąd: "Kamera nie jest wykrywana"</b></summary>

**Problem:** System nie widzi topików z kamery.

**Krok 1:** Sprawdź czy kamera jest podłączona (dla Intel RealSense)
```bash
rs-enumerate-devices
```

**Krok 2:** Uruchom driver kamery
```bash
ros2 run realsense2_camera realsense2_camera_node
```

**Krok 3:** Sprawdź topiki
```bash
ros2 topic list | grep camera
# Powinno pokazać: /camera/color/image_raw, /camera/depth/image_raw
```

**Alternatywa:** Użyj nagranych danych testowych
```bash
ros2 bag play test_data.bag --loop
```
</details>

<details>
<summary><b>🤖 Błąd: "MoveIt 2 nie planuje trajektorii"</b></summary>

**Problem:** `Planning failed` w logach.

**Przyczyna 1:** Cel poza zasięgiem robota
- ✅ **Rozwiązanie:** Sprawdź odległość - dla G1 workspace to 0.3-0.8m

**Przyczyna 2:** Kolizja z przeszkodami
- ✅ **Rozwiązanie:** Sprawdź scenę planowania w RViz

**Przyczyna 3:** Timeout planowania
```python
# Zwiększ timeout w kodzie
self.arm.set_planning_time(15.0)  # domyślnie: 5.0s
```

**Przyczyna 4:** IK nie ma rozwiązania
```python
# Zwiększ tolerancję
self.arm.set_goal_position_tolerance(0.01)  # domyślnie: 0.001
self.arm.set_goal_orientation_tolerance(0.05)
```
</details>

<details>
<summary><b>🎯 Błąd: "YOLOv5 nie wykrywa obiektów"</b></summary>

**Problem:** `/object_detections` jest puste mimo widocznych obiektów.

**Rozwiązanie 1:** Obniż próg pewności
```bash
ros2 run g1_pick_and_handover object_detector \
    --ros-args -p confidence_threshold:=0.3  # domyślnie: 0.6
```

**Rozwiązanie 2:** Użyj większego modelu
```bash
ros2 run g1_pick_and_handover object_detector \
    --ros-args -p model_name:=yolov5m  # domyślnie: yolov5s
```

**Rozwiązanie 3:** Sprawdź oświetlenie i czy obiekt jest w zbiorze COCO (80 klas)
</details>

<details>
<summary><b>🧠 Uwaga: "WMA not available, using mock decision making"</b></summary>

**To jest normalne!** 

System automatycznie przełącza się na prosty tryb decyzyjny oparty na regułach if-else, który działa wystarczająco dobrze do nauki i testowania.

**Nie potrzebujesz WMA** aby korzystać z systemu. Mock mode jest w pełni funkcjonalny.

📖 Więcej info: Zobacz [FAQ.md](FAQ.md#q-wma-nie-jest-dostępny)
</details>

### 📚 Więcej pomocy

- **FAQ:** [FAQ.md](FAQ.md) - Najczęściej zadawane pytania
- **Tutoriale:** [TUTORIALS.md](TUTORIALS.md) - Przewodniki krok po kroku
- **Architektura:** [ARCHITECTURE.md](ARCHITECTURE.md) - Szczegóły techniczne
- **Issues:** [GitHub Issues](https://github.com/MatPomGit/robot-g1-Handover/issues) - Zgłoś problem

---

## 📚 Dokumentacja Projektu

### 🚀 Start szybki
- **[QUICK_START.md](QUICK_START.md)** - ⚡ 5-minutowy przewodnik instalacji i uruchomienia
- **[STATUS.md](STATUS.md)** - 📊 Dashboard statusu systemu i wydajności
- **[EXAMPLES.md](EXAMPLES.md)** - 💻 Gotowe przykłady kodu do skopiowania
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - 🔍 Flowchart rozwiązywania problemów
- **[CHECKLIST.md](CHECKLIST.md)** - ✅ Lista kontrolna postępów

### 📖 Podstawowa Dokumentacja
- **[FAQ.md](FAQ.md)** - ❓ Najczęściej zadawane pytania i rozwiązywanie problemów
- **[TUTORIALS.md](TUTORIALS.md)** - 🎓 Szczegółowe tutoriale krok po kroku dla studentów
- **[GLOSSARY.md](GLOSSARY.md)** - 📚 Słownik terminów i konceptów
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - 📋 Szybka ściąga z komendami i parametrami

### 🏗️ Dokumentacja Techniczna
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - 🏛️ Architektura systemu i przepływ danych
- **[TESTING.md](TESTING.md)** - 🧪 Strategia i implementacja testów
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - 🤝 Przewodnik dla kontrybutorów

### ⚙️ Konfiguracja
- **[config/presets.yaml](config/presets.yaml)** - 🎚️ Gotowe presety: beginner, intermediate, advanced, simulation, debug

#---

## 💡 Wskazówki dla efektywnej nauki

### 📖 Rekomendowana ścieżka nauki

```
Dzień 1-2:  📄 QUICK_START.md → Uruchom system
            ↓
Dzień 3-5:  🎓 TUTORIALS.md → Zrozum percepcję i MoveIt 2
            ↓
Dzień 6-10: 🏗️ ARCHITECTURE.md → Zgłęb architekturę
            ↓
Dalej:      🔧 Modyfikuj i eksperymentuj!
```

### 🎯 Checklist początkującego

- [ ] System się uruchamia (✓ QUICK_START.md)
- [ ] Rozumiem topiki ROS 2 (✓ TUTORIALS.md - Tutorial 1)
- [ ] Wiem jak działa YOLO (✓ TUTORIALS.md - Tutorial 2)
- [ ] Potrafię planować trajektorie (✓ TUTORIALS.md - Tutorial 3)
- [ ] Rozumiem automat stanów (✓ ARCHITECTURE.md)
- [ ] Wiem jak debugować (✓ TROUBLESHOOTING.md)

### 💬 Społeczność i wsparcie

- **GitHub Issues**: [Zgłoś problem](https://github.com/MatPomGit/robot-g1-Handover/issues)
- **Discussions**: Zadaj pytanie społeczności
- **Pull Requests**: Współtwórz projekt!

---

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

## 📊 Status Projektu

| Moduł | Status | Notatki |
|-------|--------|---------|
| 👁️ **Percepcja (YOLO)** | ✅ Działa | YOLOv5 zaimplementowane |
| 📍 **Estymacja pozy 6D** | ✅ Działa | Pinhole camera model |
| ✋ **Detekcja dłoni** | ⚠️ Placeholder | Wymaga MediaPipe |
| 🦾 **MoveIt 2** | ✅ Działa | Interface gotowy |
| ✋ **Gripper control** | ✅ Działa | Open/close zaimplementowane |
| 🧠 **WMA (AI)** | ⚠️ Opcjonalny | Mock mode działa |
| 📋 **FSM** | ✅ Działa | Automat stanów gotowy |
| 🧪 **Testy** | 🚧 W toku | Unit testy do dodania |
| 📖 **Dokumentacja** | ✅ Kompletna | README, FAQ, Tutorials |

**Legenda:**
- ✅ Gotowe i działa
- ⚠️ Działa z ograniczeniami
- 🚧 W trakcie rozwoju
- ❌ Nie zaimplementowane

---

## 🏆 Osiągnięcia i Ulepszenia (Quality of Life)

### Niedawno dodane (2024)

- ✅ **Quick Start Guide** - 5-minutowa instalacja
- ✅ **Troubleshooting Flowchart** - Wizualna diagnostyka problemów
- ✅ **Configuration Presets** - Gotowe konfiguracje (beginner/advanced)
- ✅ **Enhanced Error Messages** - Komunikaty z sugestiami rozwiązań
- ✅ **Emoji Icons** - Kolorowe i intuicyjne logi
- ✅ **Collapsible Sections** - Lepsza organizacja README
- ✅ **Visual Diagrams** - ASCII art diagramy architektury

### Planowane

- [ ] Interactive Setup Wizard (bash script)
- [ ] Status Dashboard (CLI/TUI)
- [ ] Video Tutorials
- [ ] Docker Container
- [ ] Web-based UI Monitor

---

## 📄 Licencja

Ten projekt jest open-source i dostępny do celów edukacyjnych.

---

## ⭐ Podoba ci się projekt?

**Daj gwiazdkę na GitHubie!** ⭐ To motywuje nas do dalszego rozwoju.

**Podziel się z innymi!** 📢 Rozpowszechnij wiedzę o robotyce HRI.

---

**Uwaga**: Ten projekt jest w fazie rozwoju i służy celom edukacyjnym. Przed użyciem na prawdziwym robocie należy dokładnie przetestować wszystkie funkcje w symulacji.

---

<div align="center">

### 🤖 Zbudowane z ❤️ dla społeczności robotyki

**[⬆ Powrót do góry](#-robot-g1---system-przekazywania-obiektów-handover)**

</div>
