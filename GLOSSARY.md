# Słownik Terminów (Glossary)

## 📖 Wprowadzenie

Ten dokument wyjaśnia kluczowe terminy i koncepty używane w projekcie Robot G1 Handover. Idealny dla studentów rozpoczynających pracę z robotyką i ROS 2.

---

## 🤖 Podstawowe Koncepty Robotyki

### Robot Humanoidalny
Robot zbudowany tak, aby przypominał człowieka - ma tułów, ramiona, nogi, czasem głowę. Unitree G1 to przykład humanoidalnego robota.

**Dlaczego humanoidalny?** Łatwiej wchodzi w interakcje z ludźmi i może używać narzędzi zaprojektowanych dla ludzi.

### End-Effector (Efektor Końcowy)
Ostatni element ramienia robota - to co bezpośrednio wykonuje zadanie. W naszym przypadku: **gripper** (chwytak).

**Przykłady end-effectorów**:
- Gripper (chwytak) - chwyta obiekty
- Suction cup (przyssawka) - podnosi płaskie przedmioty
- Tool (narzędzie) - wkrętarka, lutownica, itp.

### Degrees of Freedom (DOF, Stopnie Swobody)
Liczba niezależnych ruchów, które robot może wykonać. Robot G1 ma:
- Ramię: 7 DOF (bardziej niż ludzkie ramię!)
- Gripper: 1-2 DOF (otwórz/zamknij)

**Im więcej DOF, tym robot bardziej zręczny, ale trudniejszy w sterowaniu.**

### Workspace (Przestrzeń Robocza)
Obszar, do którego robot może dosięgnąć swoim ramieniem. Dla G1: około 0.3-0.8m od bazy.

**Poza workspace**: Robot nie może wykonać ruchu (IK nie ma rozwiązania).

### Trajectory (Trajektoria)
Ścieżka, którą ramię robota przebywa z punktu A do punktu B. Zawiera:
- Pozycje pośrednie (waypoints)
- Prędkości
- Przyspieszenia

**Dobra trajektoria**: Płynna, unika kolizji, bezpieczna dla człowieka.

---

## 🔧 ROS 2 (Robot Operating System 2)

### Node (Węzeł)
Pojedynczy proces wykonujący określone zadanie. Przykłady z naszego projektu:
- `object_detector` - wykrywa obiekty
- `execute_grasp` - wykonuje chwytanie

**Filozofia ROS**: Wiele małych node'ów współpracujących ze sobą, zamiast jednego dużego programu.

### Topic (Temat)
Kanał komunikacji między node'ami. Node'y mogą:
- **Publikować** (publish) - wysyłać dane
- **Subskrybować** (subscribe) - odbierać dane

**Przykład**:
```
object_detector (publisher) ---> /object_detections ---> execute_grasp (subscriber)
```

### Message (Wiadomość)
Struktura danych przesyłana przez topic. Typy wiadomości:
- `std_msgs/Bool` - prawda/fałsz
- `geometry_msgs/PoseStamped` - pozycja 3D + orientacja
- `sensor_msgs/Image` - obraz z kamery

**Podobnie jak**: Klasa w Pythonie lub struct w C.

### Service (Usługa)
Komunikacja typu request-response (zapytanie-odpowiedź). W przeciwieństwie do topików:
- Synchroniczna (czeka na odpowiedź)
- Jednorazowa (nie ciągły strumień)

**Przykład**: `emergency_stop` - wywołaj aby zatrzymać robota.

### TF (Transform Framework)
System zarządzania układami współrzędnych. Pozwala konwertować pozycje między ramkami:
- `base_link` → `camera_link` - gdzie jest kamera względem bazy robota?
- `camera_link` → `object` - gdzie jest obiekt względem kamery?

**Kluczowe dla**: Percepcji 3D i planowania ruchu.

### Launch File
Plik Python uruchamiający wiele node'ów jednocześnie z odpowiednią konfiguracją.

**Zamiast**:
```bash
ros2 run pkg node1 &
ros2 run pkg node2 &
ros2 run pkg node3 &
```

**Używamy**:
```bash
ros2 launch pkg my_system.launch.py
```

---

## 📸 Computer Vision (Widzenie Komputerowe)

### RGB-D Camera (Kamera RGB-D)
Kamera, która rejestruje:
- **RGB**: Obraz kolorowy (Red, Green, Blue)
- **D**: Depth (głębokość) - odległość każdego piksela od kamery

**Przykłady**: Intel RealSense D435, Azure Kinect, Kinect v2

### Object Detection (Detekcja Obiektów)
Znalezienie obiektów na obrazie i narysowanie ramek (bounding boxes) wokół nich.

**Output**: Lista obiektów z:
- Klasą (np. "cup", "phone")
- Confidence (pewność, 0-1)
- Bounding box (x, y, width, height)

### YOLO (You Only Look Once)
Szybki algorytm detekcji obiektów. "Only Once" = jeden przebieg przez sieć neuronową.

**Wersje**:
- YOLOv3, v4, v5, v7, v8, v10
- YOLOv5s (small, szybki) → YOLOv5x (extra large, dokładny)

### Bounding Box
Prostokąt opisujący pozycję obiektu na obrazie.

**Format**:
- (x, y, width, height) - lewy górny róg + wymiary
- (x_min, y_min, x_max, y_max) - dwa przeciwległe rogi

### 6D Pose (Poza 6D)
Pełny opis pozycji i orientacji obiektu w przestrzeni 3D:
- **3D Position**: (x, y, z) - gdzie jest obiekt
- **3D Orientation**: (roll, pitch, yaw) lub quaternion - jak jest obrócony

**6 = 3 (position) + 3 (orientation)**

### Pinhole Camera Model
Matematyczny model kamery opisujący jak punkty 3D są projektowane na obraz 2D.

**Parametry**:
- `fx, fy` - długość ogniskowa (focal length)
- `cx, cy` - optyczny środek (principal point)

**Formuła**: 
```
u = fx * (X / Z) + cx
v = fy * (Y / Z) + cy
```

### Depth Map (Mapa Głębokości)
Obraz, gdzie każdy piksel reprezentuje odległość od kamery (zamiast koloru).

**Reprezentacja**:
- Jasny piksel = daleko
- Ciemny piksel = blisko
- Czarny = brak danych (invalid depth)

---

## 🦾 Motion Planning (Planowanie Ruchu)

### MoveIt 2
Framework do planowania ruchu manipulatorów w ROS 2. Zawiera:
- Plannery (algorytmy)
- Collision checking
- Inverse kinematics (IK)
- Trajectory execution

### Inverse Kinematics (IK, Kinematyka Odwrotna)
Obliczanie kątów stawów dla zadanej pozycji end-effectora.

**Problem**:
- Dana: Pozycja (x, y, z) gdzie chcemy aby był gripper
- Szukana: Kąty stawów (θ1, θ2, ..., θ7) aby to osiągnąć

**Trudność**: Może być wiele rozwiązań lub brak rozwiązania!

### Forward Kinematics (FK, Kinematyka Wprost)
Odwrotność IK: obliczanie pozycji end-effectora z kątów stawów.

**Problem**:
- Dane: Kąty stawów (θ1, θ2, ..., θ7)
- Szukana: Pozycja (x, y, z) grippera

**Łatwiejsze niż IK** - zawsze jedno rozwiązanie.

### Planning Scene
Reprezentacja otoczenia robota w MoveIt. Zawiera:
- Przeszkody (stół, ściany, człowiek)
- Robot i jego model
- Attached objects (obiekt w chwytaku)

**Używane do**: Collision detection podczas planowania.

### Collision Detection
Sprawdzanie czy trajektoria robota uderzy w przeszkodę.

**MoveIt robi to automatycznie!** Planner odrzuca trajektorie z kolizjami.

### RRT (Rapidly-exploring Random Tree)
Popularny algorytm planowania trajektorii. Działa poprzez:
1. Losowe próbkowanie przestrzeni konfiguracji
2. Budowanie drzewa od startu do celu
3. Znajdowanie ścieżki w drzewie

**Warianty**:
- RRTConnect - dwukierunkowy (szybki)
- RRTstar - optymalizuje trajektorię (wolny, lepszy)

### Trajectory Smoothing
Wygładzanie trajektorii aby ruch był płynniejszy.

**Przed**: Załamane, ostre zakręty
**Po**: Płynne przejścia, naturalne ruchy

### Velocity Scaling
Skalowanie prędkości ruchu.

**Zastosowanie**:
- `0.1` - bardzo wolno (testowanie, bezpieczeństwo)
- `0.5` - średnio (normalna praca)
- `1.0` - maksymalna prędkość (tylko dla eksperta!)

---

## 🧠 Artificial Intelligence (Sztuczna Inteligencja)

### World Model
Model AI przewidujący jak zmieni się środowisko po wykonaniu akcji.

**Koncept**:
```
Stan obecny + Akcja → World Model → Stan przyszły (przewidywany)
```

**Użycie**: Planowanie wieloetapowe (co się stanie jeśli...).

### WMA (World Model AI)
Konkretna implementacja World Model używana w tym projekcie.

**Komponenty**:
- Encoder - przetwarza obserwacje
- World Model - przewiduje przyszłość
- Policy - wybiera akcje

### Policy (Polityka)
Funkcja wybierająca akcję na podstawie obserwacji.

**Typy**:
- Deterministyczna - zawsze ta sama akcja dla danej obserwacji
- Stochastyczna - losuje akcję z rozkładu prawdopodobieństwa

### Observation (Obserwacja)
Dane wejściowe dla AI - wszystko co "widzi" system:
- Obraz z kamery
- Pozycja dłoni człowieka
- Stan chwytaka
- Czujniki siły

### Action (Akcja)
Decyzja AI - co robot ma zrobić:
- `TAKE_FROM_HUMAN` - odbierz obiekt
- `GIVE_TO_HUMAN` - przekaż obiekt
- `IDLE` - czekaj

### Inference (Wnioskowanie)
Uruchomienie wytrenowanego modelu AI na nowych danych.

**Proces**:
```
Obserwacje → Preprocessing → Model AI → Akcja
```

### Checkpoint
Zapisany stan wytrenowanego modelu AI (wagi sieci neuronowej).

**Plik**: `.pth` (PyTorch), `.h5` (Keras), `.onnx` (uniwersalny)

---

## 🤝 Human-Robot Interaction (HRI)

### Handover
Przekazywanie obiektów między człowiekiem a robotem.

**Typy**:
- Human-to-Robot (H2R): Człowiek daje obiekt robotowi
- Robot-to-Human (R2H): Robot daje obiekt człowiekowi

### Intention Recognition (Rozpoznawanie Intencji)
Określanie czego chce człowiek na podstawie gestów, pozy ciała, wzroku.

**W naszym projekcie**: `human_reaching` - czy człowiek wyciąga rękę?

### Ergonomics (Ergonomia)
Projektowanie interakcji tak, aby były wygodne i naturalne dla człowieka.

**Przykład**: Robot przekazuje obiekt nieco poniżej dłoni człowieka (offset -5cm), aby było łatwiej go chwycić.

### Safety Zone (Strefa Bezpieczeństwa)
Obszar wokół człowieka, do którego robot nie może wjechać zbyt szybko.

**W kodzie**: `EMERGENCY_STOP_DISTANCE = 0.05m`

---

## 🔄 State Machine (Automat Stanów)

### FSM (Finite State Machine)
System z ograniczoną liczbą stanów i przejść między nimi.

**W naszym projekcie**:
```
idle → approach → grasp → lift
```

### State (Stan)
Sytuacja, w której znajduje się system.

**Przykłady**:
- `idle` - czekanie
- `approach` - podchodzenie
- `grasp` - chwytanie

### Transition (Przejście)
Zmiana ze stanu A do stanu B.

**Warunek przejścia**: "Co musi się stać, aby zmienić stan?"

**Przykład**: `idle → approach` jeśli wykryto obiekt.

---

## 📦 Manipulation (Manipulacja)

### Grasp (Chwyt)
Uchwycenie obiektu chwytakiem.

**Typy chwytów**:
- Top grasp - z góry (najczęstszy w naszym projekcie)
- Side grasp - z boku
- Power grasp - mocny chwyt (całą dłonią)
- Precision grasp - precyzyjny (palcami)

### Pre-grasp
Pozycja tuż przed chwyceniem - zwykle nad obiektem.

**Dlaczego?** Bezpieczne podejście, uniknięcie kolizji ze stołem.

### Gripper (Chwytak)
Mechanizm na końcu ramienia służący do chwytania.

**Typy**:
- Parallel jaw - dwie równoległe szczęki
- Suction - przyssawka
- Soft gripper - miękki, dopasowuje się do kształtu

### Force Control (Sterowanie Siłowe)
Kontrola siły, z jaką chwytak ściska obiekt.

**Ważne**: Nie zmiażdżyć delikatnych obiektów (jajko, telefon).

---

## 🛡️ Safety (Bezpieczeństwo)

### E-STOP (Emergency Stop)
Natychmiastowe zatrzymanie wszystkich ruchów robota.

**Typy**:
- Hardware - fizyczny przycisk
- Software - komenda ROS 2

### Collision Avoidance
Unikanie kolizji z przeszkodami.

**Metody**:
- Planning-time - sprawdzanie podczas planowania
- Runtime - monitorowanie podczas ruchu

### Velocity Limiting
Ograniczenie maksymalnej prędkości ruchu.

**Bezpieczeństwo**: Wolniejszy robot = bezpieczniejszy dla człowieka.

---

## 📊 Metrics (Metryki)

### Success Rate
Odsetek udanych prób wykonania zadania.

**Przykład**: Robot udanie chwycił obiekt w 9 na 10 prób = 90% success rate.

### Latency (Opóźnienie)
Czas od wykrycia obiektu do rozpoczęcia ruchu.

**Cel**: Jak najniższe (real-time response).

### Accuracy (Dokładność)
Jak blisko robot trafi do celu.

**Przykład**: Robot powinien trafić w punkt (0.5, 0.0, 0.5), trafił w (0.51, 0.01, 0.49) → błąd ~1.4cm.

### FPS (Frames Per Second)
Ile klatek na sekundę przetwarza system percepcji.

**Typowe wartości**:
- 10 FPS - minimalne dla robotyki
- 30 FPS - dobre
- 60+ FPS - świetne

---

## 🎓 Dla Studentów

### Debugging
Proces znajdowania i naprawiania błędów w kodzie.

**Narzędzia**:
- `print()` / `self.get_logger().info()` - wypisywanie wartości
- RViz - wizualizacja
- `ros2 topic echo` - podglądanie danych
- Debugger (pdb, gdb) - krok po kroku

### Visualization (Wizualizacja)
Graficzne przedstawienie danych.

**Narzędzia**:
- RViz2 - główne narzędzie ROS 2
- Matplotlib - wykresy
- OpenCV - obrazy

### Simulation (Symulacja)
Testowanie robota w środowisku wirtualnym przed użyciem prawdziwego.

**Zalety**:
- Bezpieczne (żadne szkody)
- Szybsze iteracje
- Można testować niebezpieczne scenariusze

**Narzędzia**: Gazebo, MuJoCo, Isaac Sim

---

## 📚 Dodatkowe Terminy

### URDF (Unified Robot Description Format)
XML opisujący geometrię i kinematykę robota.

### SRDF (Semantic Robot Description Format)
Dodatkowe informacje semantyczne dla MoveIt (grupy, stany).

### Bag File
Plik z nagranym zapisem danych ROS 2. Można odtworzyć.

### Remapping
Zmiana nazwy topiku:
```bash
ros2 run pkg node --ros-args -r old_topic:=new_topic
```

### Parameter
Konfigurowalna wartość w node (można zmienić bez rebuildu).

---

**Masz pytanie o inny termin?** Otwórz Issue i dodamy go do słownika!
