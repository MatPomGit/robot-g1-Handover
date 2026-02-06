# Moduł Percepcji (Perception Module)

## 📖 Wprowadzenie

Moduł percepcji odpowiada za rozumienie otoczenia robota poprzez przetwarzanie danych z kamer i czujników. Jest to "oko" robota, które pozwala mu zobaczyć i zrozumieć co się dzieje w jego otoczeniu.

## 🎯 Funkcje Modułu

### 1. **Wykrywanie Obiektów** (`object_detector.py`)
- Używa sieci neuronowej YOLOv5 do detekcji obiektów
- Wykrywa 80 klas obiektów (COCO dataset)
- Zwraca bounding boxes (prostokąty) wokół obiektów
- Działa w czasie rzeczywistym (~30 FPS)

### 2. **Estymacja Pozy 6D** (`pose_estimator_6d.py`)
- Konwertuje detekcje 2D na pozycje 3D
- Używa mapy głębokości z kamery RGB-D
- Stosuje pinhole camera model
- Zwraca pozycję (x, y, z) obiektów w przestrzeni

### 3. **Detekcja Dłoni Człowieka** (`human_hand_detector.py`)
- Wykrywa pozycję dłoni człowieka w 3D
- Określa intencję człowieka (czy wyciąga rękę)
- Może używać MediaPipe lub OpenPose
- Kluczowe dla interakcji człowiek-robot

### 4. **Transformacje TF** (`static_tf_camera.py`)
- Definiuje pozycję kamery względem robota
- Publikuje statyczną transformację base_link -> camera_link
- Umożliwia konwersję współrzędnych między układami

## 📊 Przepływ Danych

```
Kamera RGB-D
    |
    +---> [Object Detector] ---> /object_detections
    |           |
    |           v
    +---> [Pose Estimator 6D] ---> /object_pose
    |
    +---> [Human Hand Detector] ---> /human_hand_pose
                                      /human_reaching
```

## 🔧 Topiki ROS 2

### Subskrybowane (Input):
- `/camera/color/image_raw` - Obraz RGB z kamery
- `/camera/depth/image_raw` - Mapa głębokości z kamery
- `/camera/color/camera_info` - Parametry kalibracji kamery

### Publikowane (Output):
- `/object_detections` - Wykryte obiekty (Detection2DArray)
- `/object_pose` - Pozycja 3D obiektu (PoseStamped)
- `/human_hand_pose` - Pozycja dłoni człowieka (PoseStamped)
- `/human_reaching` - Intencja człowieka (Bool)
- `/tf_static` - Transformacja kamery (TFMessage)

## 🚀 Użycie

### Uruchomienie wszystkich komponentów percepcji:

```bash
# Terminal 1: Statyczna transformacja kamery
ros2 run g1_pick_and_handover static_tf_camera

# Terminal 2: Detektor obiektów
ros2 run g1_pick_and_handover object_detector

# Terminal 3: Estymator pozy 6D
ros2 run g1_pick_and_handover pose_estimator_6d

# Terminal 4: Detektor dłoni
ros2 run g1_pick_and_handover human_hand_detector
```

### Sprawdzanie wyników:

```bash
# Zobacz wykryte obiekty
ros2 topic echo /object_detections

# Zobacz pozycję 3D obiektu
ros2 topic echo /object_pose

# Zobacz pozycję dłoni człowieka
ros2 topic echo /human_hand_pose
```

## 📚 Kluczowe Algorytmy

### YOLOv5 (You Only Look Once v5)
- Szybki detektor obiektów w czasie rzeczywistym
- Single-stage detector (jedna sieć neuronowa)
- Wersje: yolov5s (small, szybka), yolov5m, yolov5l, yolov5x (duża, dokładna)
- Pretrenowany na COCO dataset (80 klas)

### Pinhole Camera Model
Konwersja 2D (piksele) -> 3D (metry):
```
x = (u - cx) * z / fx
y = (v - cy) * z / fy
```
gdzie:
- (u, v) = pozycja piksela
- (cx, cy) = optyczny środek kamery
- (fx, fy) = ogniskowe kamery
- z = głębokość
- (x, y, z) = pozycja 3D

### MediaPipe Hand Tracking
- Wykrywa 21 kluczowych punktów dłoni
- Działa w czasie rzeczywistym
- Obsługuje jedną lub dwie ręce
- Zwraca pozycję 2D i 3D landmarków

## 🛠️ Konfiguracja

### Wymagania:
```bash
# PyTorch (dla YOLOv5)
pip install torch torchvision

# OpenCV (przetwarzanie obrazu)
pip install opencv-python

# MediaPipe (detekcja dłoni)
pip install mediapipe

# ROS 2 packages
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-vision-msgs
```

### Kamera RGB-D:
- Intel RealSense D435 (zalecane)
- Azure Kinect
- Lub inna kamera z depth sensing

## 🔍 Debugowanie

### Problem: Brak detekcji obiektów
```bash
# Sprawdź czy kamera działa
ros2 topic echo /camera/color/image_raw

# Sprawdź czy YOLOv5 się załadował
ros2 node info /object_detector

# Zobacz logi
ros2 run g1_pick_and_handover object_detector --ros-args --log-level debug
```

### Problem: Nieprawidłowa pozycja 3D
```bash
# Sprawdź transformację TF
ros2 run tf2_ros tf2_echo base_link camera_link

# Sprawdź parametry kalibracji
ros2 topic echo /camera/color/camera_info

# Wizualizuj w RViz
rviz2
```

### Problem: Nie wykrywa dłoni
- Upewnij się, że MediaPipe jest zainstalowany
- Sprawdź oświetlenie (dłoń musi być dobrze oświetlona)
- Zwiększ kontrast lub jasność obrazu

## 📖 Tutorial dla Studentów

### Ćwiczenie 1: Zrozumienie YOLOv5

1. Uruchom detektor:
   ```bash
   ros2 run g1_pick_and_handover object_detector
   ```

2. Połóż różne obiekty przed kamerą (kubek, telefon, klawiatura)

3. Obserwuj detekcje:
   ```bash
   ros2 topic echo /object_detections
   ```

4. **Zadanie**: Zapisz, które obiekty YOLOv5 wykrywa najlepiej

### Ćwiczenie 2: Estymacja Pozy 3D

1. Uruchom cały pipeline percepcji

2. Umieść obiekt w różnych pozycjach

3. Zapisuj pozycje 3D:
   ```bash
   ros2 topic echo /object_pose > positions.txt
   ```

4. **Zadanie**: Zmierz dokładność - porównaj ze zmierzoną taśmą odległością

### Ćwiczenie 3: Wizualizacja w RViz

1. Uruchom RViz:
   ```bash
   rviz2
   ```

2. Dodaj displays:
   - Image: `/camera/color/image_raw`
   - TF: Pokaż transformacje
   - PoseStamped: `/object_pose`

3. **Zadanie**: Zaobserwuj jak pozycja obiektu zmienia się w czasie rzeczywistym

## 🔬 Zaawansowane

### Modyfikacja progu pewności YOLOv5:
W `object_detector.py`, zmień:
```python
if conf < 0.6:  # Zmień 0.6 na inną wartość (0.0-1.0)
```

### Dodanie nowej klasy obiektów:
- Wytrenuj custom YOLOv5 na własnym datasecie
- Załaduj custom weights zamiast pretrained

### Integracja z innymi detektorami:
- Faster R-CNN (dokładniejszy, wolniejszy)
- EfficientDet (dobry kompromis)
- DETR (Transformer-based)

## 📚 Dodatkowe Zasoby

- [YOLOv5 Documentation](https://github.com/ultralytics/yolov5)
- [MediaPipe Hand Tracking](https://google.github.io/mediapipe/solutions/hands.html)
- [ROS 2 CV Bridge Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/CvBridge.html)
- [Camera Calibration](http://wiki.ros.org/camera_calibration)

---

**Pytania?** Otwórz Issue na GitHubie lub skonsultuj się z prowadzącym!
