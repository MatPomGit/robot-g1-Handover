# 🔍 Troubleshooting Flowchart - Robot G1 Handover

Szybki przewodnik diagnozowania i rozwiązywania problemów.

---

## 🚦 Podstawowa diagnostyka - START TUTAJ

```
┌───────────────────────────────────────────────────────┐
│  Czy system się w ogóle uruchamia?                    │
└─────────────┬─────────────────────────────────────────┘
              │
         ┌────┴────┐
         │   TAK   │──────▶ Przejdź do sekcji "System działa"
         └─────────┘
         ┌─────────┐
         │   NIE   │──────▶ Przejdź do sekcji "System nie startuje"
         └─────────┘
```

---

## ❌ System nie startuje

### Krok 1: Sprawdź ROS 2

```
ros2 --version
```

**Wynik:**
- ✅ **Pokazuje wersję (np. "ros2 cli version: 0.18.5")**
  → ROS 2 OK, przejdź do Kroku 2

- ❌ **"command not found"**
  → Zainstaluj ROS 2:
  ```bash
  sudo apt install ros-humble-desktop
  source /opt/ros/humble/setup.bash
  ```

### Krok 2: Sprawdź pakiet

```
ros2 pkg list | grep g1_pick_and_handover
```

**Wynik:**
- ✅ **Pokazuje "g1_pick_and_handover"**
  → Pakiet OK, przejdź do Kroku 3

- ❌ **Nic nie pokazuje**
  → Zbuduj pakiet:
  ```bash
  cd ~/ros2_ws
  colcon build --packages-select g1_pick_and_handover
  source install/setup.bash
  ```

### Krok 3: Sprawdź zależności

```
pip3 list | grep -E "(torch|opencv|mediapipe)"
```

**Wynik:**
- ✅ **Pokazuje wszystkie pakiety**
  → Zależności OK, spróbuj uruchomić ponownie

- ❌ **Brakuje pakietów**
  → Zainstaluj:
  ```bash
  pip3 install -r requirements.txt
  ```

---

## ✅ System działa, ale...

### 🎯 Problem: "Brak detekcji obiektów"

```
┌─────────────────────────────────────┐
│  Czy topic /object_detections       │
│  publikuje wiadomości?              │
└──────────────┬──────────────────────┘
               │
         ┌─────┴─────┐
         │    TAK    │───▶ Obiekt nie jest wykrywany
         └───────────┘     │
                           ├─▶ Obniż próg: --ros-args -p confidence_threshold:=0.3
                           ├─▶ Większy model: -p model_name:=yolov5m
                           └─▶ Sprawdź oświetlenie
         ┌───────────┐
         │    NIE    │───▶ Node nie działa
         └───────────┘     │
                           ├─▶ Sprawdź: ros2 node list
                           ├─▶ Sprawdź logi: ros2 topic echo /rosout
                           └─▶ Uruchom ponownie object_detector
```

### 📷 Problem: "Brak kamery"

```
ros2 topic list | grep camera
```

**Wynik:**
- ✅ **Pokazuje /camera/color/image_raw**
  → Kamera OK

- ❌ **Brak topików kamery**
  → Rozwiązania:
  
  **Opcja A: Fizyczna kamera RealSense**
  ```bash
  rs-enumerate-devices  # Sprawdź czy widzi kamerę
  ros2 run realsense2_camera realsense2_camera_node
  ```
  
  **Opcja B: Użyj bag file (dane testowe)**
  > Uwaga: Repozytorium nie zawiera przykładowego pliku `.bag`. Użyj własnego nagrania ROS 2 (np. z `/camera/color/image_raw`) lub innego dostępnego pliku.
  ```bash
  ros2 bag play /sciezka/do/twojego_pliku.bag --loop
  ```
  
  **Opcja C: Symuluj topiki**
  ```bash
  ros2 run image_publisher image_publisher_node test.jpg \
      --ros-args -r image_raw:=/camera/color/image_raw
  ```

### 🦾 Problem: "MoveIt nie planuje"

```
┌─────────────────────────────────────────┐
│  Co mówi komunikat błędu?               │
└────────────────┬────────────────────────┘
                 │
    ┌────────────┼────────────┐
    │            │            │
    ▼            ▼            ▼
┌────────┐  ┌────────┐  ┌────────┐
│Planning│  │  IK    │  │Timeout │
│ failed │  │ failed │  │exceeded│
└───┬────┘  └───┬────┘  └───┬────┘
    │           │           │
    │           │           └─▶ Zwiększ czas:
    │           │               self.arm.set_planning_time(15.0)
    │           │
    │           └─▶ Zwiększ tolerancję:
    │               self.arm.set_goal_position_tolerance(0.01)
    │               self.arm.set_goal_orientation_tolerance(0.05)
    │
    └─▶ Sprawdź:
        1. Czy cel w zasięgu robota? (0.3-0.8m dla G1)
        2. Czy są kolizje w planning scene?
        3. Zmień planner: self.arm.set_planner_id("RRTstar")
```

---

## 🧠 Specjalne przypadki

### "WMA not available"

```
┌───────────────────────────────────────┐
│  To NORMALNE! ✓                       │
│                                       │
│  System używa prostego trybu mock    │
│  (reguły if-else)                    │
│                                       │
│  Nie potrzebujesz WMA!               │
└───────────────────────────────────────┘
```

**Nie musisz nic robić.** System działa poprawnie w trybie mock.

---

## 📊 Diagnostyka zaawansowana

### Sprawdź przepływ danych

```bash
# 1. Lista aktywnych node'ów
ros2 node list

# Oczekiwane node'y:
# - /object_detector
# - /pose_estimator_6d
# - /human_hand_detector
# - /execute_handover_wma

# 2. Wykres komunikacji
rqt_graph

# 3. Częstotliwość publikacji
ros2 topic hz /camera/color/image_raw  # Powinno być ~30 Hz
ros2 topic hz /object_detections       # Zależnie od FPS kamery
```

### Włącz szczegółowe logi

```bash
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py \
    --log-level debug
```

---

## 🆘 Nadal nie działa?

### Checklist końcowy

- [ ] Ubuntu 22.04?
- [ ] ROS 2 Humble zainstalowany?
- [ ] MoveIt 2 zainstalowany?
- [ ] Pakiet zbudowany? (`colcon build`)
- [ ] Workspace source'owany? (`source install/setup.bash`)
- [ ] Zależności Python? (`pip3 install -r requirements.txt`)
- [ ] Kamera/bag file uruchomione?

### Gdzie szukać pomocy?

1. **📖 FAQ.md** - Szczegółowe odpowiedzi na częste pytania
2. **🎓 TUTORIALS.md** - Tutoriale krok po kroku
3. **🏗️ ARCHITECTURE.md** - Zrozumienie architektury systemu
4. **💬 GitHub Issues** - Otwórz issue z opisem problemu
5. **🌐 ROS Answers** - https://answers.ros.org/

---

## 💡 Szablon zgłoszenia problemu

Jeśli otwierasz Issue, użyj tego szablonu:

```markdown
**Opis problemu:**
[Co nie działa?]

**Komendy użyte:**
```bash
[Wklej użyte komendy]
```

**Błędy w logach:**
```
[Wklej błędy z terminala]
```

**Środowisko:**
- Ubuntu: [wersja]
- ROS 2: [wersja]
- Python: [wersja]

**Co już próbowałem:**
- [ ] Przebudowanie pakietu
- [ ] Reinstalacja zależności
- [ ] Sprawdzenie FAQ.md
- [ ] ...
```

---

**Powodzenia w rozwiązywaniu problemów! 🚀**

Jeśli uda ci się rozwiązać problem, rozważ dodanie go do FAQ.md dla innych użytkowników!
