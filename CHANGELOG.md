# 📝 Historia Zmian (Changelog)

Wszystkie istotne zmiany w tym projekcie będą dokumentowane w tym pliku.

Format oparty na [Keep a Changelog](https://keepachangelog.com/pl/1.0.0/),
projekt przestrzega [Semantic Versioning](https://semver.org/lang/pl/).

---

## [1.0.0] - 2026-02-07

### 🎉 Pierwsze Oficjalne Wydanie

To jest pierwsze stabilne wydanie systemu Robot G1 Handover - edukacyjnej platformy do nauki interakcji człowiek-robot.

### ✨ Nowe Funkcje (Added)

#### Percepcja (Perception)
- **Detekcja obiektów 2D** z wykorzystaniem YOLOv5
  - Obsługa 80 klas obiektów z zestawu COCO
  - Konfigurowalne progi pewności (domyślnie 0.6)
  - Wydajność: 15-30 FPS na GPU
- **Estymacja pozy 6D** obiektów w przestrzeni 3D
  - Model pinhole camera z mapą głębokości
  - Dokładność: ±2cm
- **Statyczna transformacja kamery** (static_tf_camera.py)
  - Definiuje relację spatial kamery względem bazy robota
- **Szkielet detekcji dłoni** (human_hand_detector.py)
  - Przygotowany do integracji z MediaPipe

#### Manipulacja (Manipulation)
- **Interfejs MoveIt 2** (moveit_interface.py)
  - Planowanie trajektorii bez kolizji
  - Konfigurowalny planner (domyślnie: RRTConnect)
  - Timeout planowania: 5s (konfigurowalny do 15s)
- **Planer chwytania** (grasp_planner.py)
  - Oblicza pozycje pre-grasp i grasp
  - Parametry: approach_distance, lift_distance
- **Planer przekazywania** (handover_planner.py)
  - Oblicza bezpieczną pozycję do przekazania obiektu człowiekowi
- **Wykonanie chwytania** (execute_grasp.py)
  - Sekwencja: approach → grasp → lift
  - Kontrola gripper: otwarcie/zamknięcie
- **Wykonanie handover** (execute_handover.py)
  - Sekwencja: move_to_human → wait → release
- **Scena planowania** (planning_scene.py)
  - Dodaje przeszkody (stół, człowiek) do MoveIt

#### Decyzje (Decision Making)
- **World Model AI Manager** (wma_handover_manager.py)
  - Integracja z UnifoLM-WMA (opcjonalna)
  - Przewidywanie intencji człowieka
  - Automatyczne przełączanie na mock mode jeśli WMA niedostępny
- **Task Manager FSM** (wma_task_manager.py)
  - Automat stanów: IDLE → TAKE_FROM_HUMAN → HOLD → GIVE_TO_HUMAN → IDLE
  - Zarządzanie przejściami między stanami

#### Launch Files
- **full_pipeline.launch.py** - Podstawowy pipeline percepcja + manipulacja
- **full_handover_pipeline.launch.py** - Kompletny system z WMA
- **launch_perception.launch.py** - Tylko moduły percepcji

#### Konfiguracja
- **grasp_params.yaml** - Parametry chwytania
  - approach_distance: 0.10m
  - lift_distance: 0.15m
  - gripper_open/closed: 0.04m / 0.0m
  - max_force: 30.0N
- **moveit.yaml** - Parametry MoveIt 2
  - Planner: RRTConnect
  - Range: 0.3

### 📚 Dokumentacja (Added)

#### Dokumentacja Główna
- **README.md** - Kompletny przewodnik projektu (791 linii)
  - Szybki start w 5 minut
  - Szczegółowa instalacja krok po kroku
  - Architektura systemu z diagramami ASCII
  - Status komponentów
  - FAQ z najczęstszymi problemami
- **QUICK_START.md** - Ekspresowa ścieżka uruchomienia
- **STATUS.md** - Dashboard statusu systemu
- **CHECKLIST.md** - Lista kontrolna dla użytkowników

#### Tutoriale i Przewodniki
- **TUTORIALS.md** - Szczegółowe tutoriale dla studentów
  - Tutorial 1: Percepcja wizualna
  - Tutorial 2: Planowanie ruchu z MoveIt 2
  - Tutorial 3: Integracja World Model AI
- **EXAMPLES.md** - Gotowe przykłady kodu do skopiowania
- **QUICK_REFERENCE.md** - Szybka ściąga z komendami

#### Rozwiązywanie Problemów
- **TROUBLESHOOTING.md** - Flowchart diagnostyki problemów
- **FAQ.md** - 20+ najczęściej zadawanych pytań

#### Dokumentacja Techniczna
- **ARCHITECTURE.md** - Szczegółowa architektura systemu
- **TESTING.md** - Strategia testowania
- **CONTRIBUTING.md** - Przewodnik dla kontrybutorów
- **GLOSSARY.md** - Słownik 50+ terminów robotyki
- **IMPROVEMENT_SUMMARY.md** - Podsumowanie ulepszeń UX
- **UX_IMPROVEMENTS_SUMMARY.md** - Quality of Life features

### 🔧 Wymagania Systemowe

#### System Operacyjny
- Ubuntu 22.04 LTS (wymagane)
- Co najmniej 4GB RAM
- ~10GB wolnego miejsca na dysku

#### Oprogramowanie
- ROS 2 Humble lub nowszy
- Python 3.10+
- MoveIt 2 (ros-humble-moveit)

#### Biblioteki Python (requirements.txt)
- PyTorch ≥ 1.10.0
- torchvision ≥ 0.11.0
- opencv-python ≥ 4.5.0
- numpy ≥ 1.21.0
- ultralytics (YOLOv5)
- mediapipe (opcjonalny)

#### Sprzęt (Opcjonalny)
- Intel RealSense D435 (kamera RGB-D)
- Unitree G1 Robot (lub symulator)
- GPU NVIDIA (zalecane dla YOLOv5)

### 🎯 Funkcjonalności dla Studentów

#### Nauka ROS 2
- Przykłady tworzenia node'ów
- Komunikacja przez topiki
- Transformacje TF2
- Launch files i parametry

#### Nauka Computer Vision
- Wykrywanie obiektów z YOLO
- Estymacja pozy 3D/6D
- Kalibracja kamery
- Przetwarzanie obrazów RGB-D

#### Nauka Planowania Ruchu
- MoveIt 2 interface
- RRT path planning
- Collision avoidance
- Inverse kinematics

#### Nauka AI w Robotyce
- World Model Architecture
- State machines (FSM)
- Decision making
- Integracja modeli PyTorch

### 🐛 Znane Ograniczenia (Known Issues)

1. **Detekcja dłoni** - Wymaga integracji MediaPipe (obecnie placeholder)
2. **World Model AI** - Wymaga checkpointu modelu (działa mock mode)
3. **Testy jednostkowe** - W trakcie implementacji
4. **Wsparcie symulacji** - Wymaga Gazebo/MuJoCo setup
5. **Dokumentacja API** - Brak automatycznych docstrings

### 🔒 Bezpieczeństwo (Security)

- System przeznaczony **wyłącznie do celów edukacyjnych**
- **Ostrzeżenie**: Przed użyciem na prawdziwym robocie należy:
  - Dokładnie przetestować w symulacji
  - Zaimplementować safety limits
  - Dodać emergency stop
  - Przeprowadzić risk assessment

### 📦 Dystrybucja

- **Repozytorium GitHub**: https://github.com/MatPomGit/robot-g1-Handover
- **Licencja**: MIT (open source)
- **Język**: Python + ROS 2
- **Platforma**: Ubuntu 22.04 + ROS 2 Humble

### 👥 Autorzy

- Robot G1 Handover Team
- Kontakt: contact@robotg1handover.org

### 🙏 Podziękowania (Credits)

Ten projekt wykorzystuje następujące narzędzia open-source:
- **ROS 2 Humble** - Robot Operating System
- **MoveIt 2** - Motion planning framework
- **YOLOv5** (Ultralytics) - Object detection
- **PyTorch** - Deep learning framework
- **OpenCV** - Computer vision library
- **Intel RealSense SDK** - Camera drivers
- **Unitree Robotics** - G1 robot model

---

## [0.1.0] - 2024-XX-XX

### 🌱 Wersja Początkowa (Initial Development)

- Prototyp systemu
- Podstawowa struktura modułów
- Wczesne testy koncepcyjne

---

## Konwencje Changelogu

### Typy zmian:
- **Added** (✨ Nowe funkcje) - Nowe funkcjonalności
- **Changed** (🔄 Zmiany) - Zmiany w istniejących funkcjonalnościach
- **Deprecated** (⚠️ Przestarzałe) - Funkcje do usunięcia w przyszłości
- **Removed** (🗑️ Usunięte) - Usunięte funkcjonalności
- **Fixed** (🐛 Naprawione) - Poprawki błędów
- **Security** (🔒 Bezpieczeństwo) - Poprawki bezpieczeństwa

### Format wersji (Semantic Versioning):
- **MAJOR.MINOR.PATCH** (np. 1.0.0)
  - **MAJOR** - Niekompatybilne zmiany API
  - **MINOR** - Nowe funkcje (kompatybilne wstecz)
  - **PATCH** - Poprawki błędów (kompatybilne wstecz)

---

**[Unreleased]** - Nadchodzące zmiany (do następnej wersji)

### Planowane na 1.1.0
- [ ] Integracja MediaPipe dla detekcji dłoni
- [ ] Testy jednostkowe (pytest)
- [ ] Wsparcie dla Gazebo simulation
- [ ] Docker container
- [ ] CI/CD pipeline (GitHub Actions)
- [ ] Automatic API documentation (Sphinx)

### Planowane na 2.0.0
- [ ] Wsparcie dla wielu robotów
- [ ] Web-based monitoring dashboard
- [ ] Real-time 3D visualization
- [ ] Advanced safety features
- [ ] Multi-language support (English docs)

---

## Linki

- **Repository**: https://github.com/MatPomGit/robot-g1-Handover
- **Issues**: https://github.com/MatPomGit/robot-g1-Handover/issues
- **Dokumentacja**: Patrz README.md i pliki dokumentacji

---

<div align="center">

**Dziękujemy za korzystanie z Robot G1 Handover System!** 🤖❤️

</div>
