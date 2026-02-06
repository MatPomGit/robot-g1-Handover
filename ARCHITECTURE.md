# Architektura Systemu Robot G1 Handover

## Przegląd

System przekazywania obiektów składa się z trzech głównych warstw:
1. **Warstwa Percepcji** - Rozumienie otoczenia
2. **Warstwa Decyzyjna** - Podejmowanie inteligentnych decyzji
3. **Warstwa Manipulacji** - Wykonywanie akcji fizycznych

## Diagram Architektury

```
┌─────────────────────────────────────────────────────────────┐
│                     WARSTWA PERCEPCJI                        │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Kamera     │  │   Detekcja   │  │   Estymacja  │      │
│  │   RGB-D      │→ │   Obiektów   │→ │   Pozy 6D    │      │
│  │              │  │   (YOLOv5)   │  │  (Pinhole)   │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                               │
│  ┌──────────────┐                                            │
│  │   Detekcja   │                                            │
│  │   Dłoni      │                                            │
│  │ (MediaPipe)  │                                            │
│  └──────────────┘                                            │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                    WARSTWA DECYZYJNA                         │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────┐       │
│  │          World Model AI (WMA)                     │       │
│  │  • Encoder (multimodal observations)              │       │
│  │  • World Model (predict future states)           │       │
│  │  • Policy (select best action)                   │       │
│  │  • Planner (plan action sequence)                │       │
│  └──────────────────────────────────────────────────┘       │
│                                                               │
│  Akcje: TAKE_FROM_HUMAN | GIVE_TO_HUMAN | IDLE              │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                   WARSTWA MANIPULACJI                        │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  Planowanie  │  │   MoveIt 2   │  │   Gripper    │      │
│  │   Chwytania  │→ │   Interface  │→ │   Control    │      │
│  │              │  │  (IK, Plan)  │  │              │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

## Przepływ Danych

### 1. Odbieranie Obiektu (TAKE_FROM_HUMAN)

```
Kamera → YOLOv5 → Pose6D → WMA → MoveIt2 → Gripper
                    ↓         ↓
              Hand Detector  Decision
                             (TAKE)
```

**Sekwencja:**
1. Człowiek wyciąga rękę z obiektem
2. Hand Detector wykrywa pozycję dłoni
3. Object Detector wykrywa obiekt
4. Pose Estimator oblicza pozycję 3D obiektu
5. WMA analizuje sytuację → TAKE_FROM_HUMAN
6. Grasp Planner oblicza pozycję pre-grasp
7. MoveIt2 planuje trajektorię
8. Robot wykonuje ruch i zamyka gripper

### 2. Przekazywanie Obiektu (GIVE_TO_HUMAN)

```
Hand Detector → WMA → Handover Planner → MoveIt2 → Gripper
                 ↓
           Decision (GIVE)
```

**Sekwencja:**
1. Robot trzyma obiekt (gripper_occupied=True)
2. Człowiek wyciąga rękę (human_reaching=True)
3. WMA analizuje sytuację → GIVE_TO_HUMAN
4. Handover Planner oblicza pozycję przekazania
5. MoveIt2 planuje trajektorię
6. Robot podjeżdża do dłoni człowieka
7. Robot otwiera gripper (człowiek odbiera obiekt)

## Moduły

### Perception (percepcja/)

#### object_detector.py
- **Funkcja**: Wykrywanie obiektów na obrazie
- **Technologia**: YOLOv5 (PyTorch)
- **Input**: `/camera/color/image_raw`
- **Output**: `/object_detections`
- **Konfiguracja**: `confidence_threshold`, `model_name`

#### pose_estimator_6d.py
- **Funkcja**: Obliczanie pozycji 3D obiektów
- **Technologia**: Pinhole Camera Model + Depth
- **Input**: `/object_detections`, `/camera/depth/image_raw`, `/camera/color/camera_info`
- **Output**: `/object_pose`
- **Parametry**: fx, fy, cx, cy (kalibracja kamery)

#### human_hand_detector.py
- **Funkcja**: Wykrywanie pozycji dłoni człowieka
- **Technologia**: MediaPipe Hands (TODO: zaimplementować)
- **Status**: PLACEHOLDER - wymaga implementacji
- **Output**: `/human_hand_pose`, `/human_reaching`

#### static_tf_camera.py
- **Funkcja**: Publikowanie transformacji camera→base_link
- **Technologia**: TF2 static transform
- **Output**: `/tf_static`

### Decision (decision/)

#### wma_handover_manager.py
- **Funkcja**: Podejmowanie decyzji za pomocą AI
- **Technologia**: World Model AI (UnifoLM-WMA)
- **Input**: Obserwacje multimodalne (RGB, state, pose)
- **Output**: Akcja (TAKE/GIVE/IDLE)
- **Model**: Checkpoint WMA (pretrained)

#### wma_task_manager.py
- **Funkcja**: Automat stanów (FSM) dla sekwencji chwytania
- **Stany**: idle → approach → grasp → lift
- **Uwaga**: Uproszczona wersja dydaktyczna

### Manipulation (manipulation/)

#### moveit_interface.py
- **Funkcja**: Interfejs do MoveIt2
- **Technologia**: MoveGroupCommander, PlanningSceneInterface
- **Grupy**: "arm", "gripper"
- **Metody**: `move_to_pose()`, `open_gripper()`, `close_gripper()`

#### grasp_planner.py
- **Funkcja**: Obliczanie pozycji pre-grasp
- **Strategia**: Podejście od góry (offset +Z)
- **Parametry**: `PREGRASP_OFFSET_Z = 0.10m`

#### handover_planner.py
- **Funkcja**: Obliczanie pozycji przekazania
- **Strategia**: Ergonomiczny offset względem dłoni
- **Parametry**: `HANDOVER_OFFSET_Z = -0.05m`

#### execute_grasp.py
- **Funkcja**: Wykonanie pełnej sekwencji chwytania
- **Sekwencja**: pre-grasp → grasp → lift

#### execute_handover.py
- **Funkcja**: Główny kontroler z WMA
- **Input**: Wszystkie topiki percepcji + stan robota
- **Output**: Sterowanie robotem

#### planning_scene.py
- **Funkcja**: Zarządzanie sceną planowania (przeszkody)
- **Obiekty**: Stół, człowiek, inne przeszkody

## Bezpieczeństwo

### Mechanizmy Bezpieczeństwa (DO ZAIMPLEMENTOWANIA)

1. **Emergency Stop**
   - Monitorowanie odległości człowiek-robot
   - Zatrzymanie jeśli dystans < `EMERGENCY_STOP_DISTANCE`

2. **Collision Avoidance**
   - MoveIt2 planuje trajektorie bez kolizji
   - Planning Scene zawiera modele przeszkód

3. **Force/Torque Monitoring**
   - Monitorowanie siły chwytaka
   - Limit: `GRIPPER_MAX_FORCE = 30N`

4. **Timeouts**
   - Każdy ruch ma timeout
   - Limit: `MOVEMENT_TIMEOUT_SEC = 10s`

5. **Velocity/Acceleration Limits**
   - `MAX_VELOCITY_SCALE = 0.5`
   - `MAX_ACCELERATION_SCALE = 0.5`

## Testowanie

### Unit Tests (DO ZAIMPLEMENTOWANIA)
- `test_object_detector.py`
- `test_pose_estimator.py`
- `test_grasp_planner.py`
- `test_handover_planner.py`

### Integration Tests (DO ZAIMPLEMENTOWANIA)
- `test_perception_pipeline.py`
- `test_manipulation_pipeline.py`
- `test_full_handover.py`

### Simulation Tests
- Gazebo + MuJoCo
- Syntetic camera data
- Mock WMA responses

## Znane Ograniczenia

1. **Human Hand Detector**: Tylko placeholder - nie wykrywa prawdziwych dłoni
2. **WMA Integration**: Import z `unifolm_wma.inference` może nie działać bez instalacji biblioteki
3. **Safety**: Brak implementacji mechanizmów bezpieczeństwa
4. **Error Handling**: Minimalna obsługa błędów w niektórych modułach
5. **Testing**: Brak testów automatycznych

## Roadmap Ulepszeń

### Faza 1 (Krytyczne)
- [x] Dodać obsługę błędów we wszystkich modułach
- [x] Zdefiniować stałe konfiguracyjne
- [ ] Zaimplementować prawdziwy Hand Detector (MediaPipe)
- [ ] Dodać mechanizmy bezpieczeństwa

### Faza 2 (Ważne)
- [ ] Utworzyć testy jednostkowe
- [ ] Utworzyć testy integracyjne
- [ ] Dodać CI/CD pipeline
- [ ] Dokumentacja API (Sphinx)

### Faza 3 (Optymalizacje)
- [ ] Refaktoryzacja do interfejsów/abstrakcji
- [ ] Optymalizacja wydajności
- [ ] Logging i monitoring
- [ ] Parametryzacja przez ROS params

### Faza 4 (Zaawansowane)
- [ ] Multi-object tracking
- [ ] Advanced grasp planning (różne orientacje)
- [ ] Learning from demonstrations
- [ ] Real-time adaptation

## Kontakt

Jeśli masz pytania dotyczące architektury, otwórz Issue na GitHubie lub skontaktuj się z zespołem.
