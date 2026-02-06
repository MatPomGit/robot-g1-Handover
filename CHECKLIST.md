# ✅ Checklist Użytkownika - Robot G1 Handover

Ten dokument pomaga śledzić postęp w nauce i konfiguracji systemu.

---

## 🚀 Checklist instalacji

### Krok 1: Środowisko systemu
- [ ] Ubuntu 22.04 LTS zainstalowane
- [ ] System zaktualizowany (`sudo apt update && sudo apt upgrade`)
- [ ] Co najmniej 4GB RAM dostępne
- [ ] ~10GB wolnego miejsca na dysku

### Krok 2: ROS 2 Humble
- [ ] ROS 2 Humble zainstalowany
- [ ] Działa komenda `ros2 --version`
- [ ] Auto-source w ~/.bashrc: `source /opt/ros/humble/setup.bash`
- [ ] MoveIt 2 zainstalowany: `ros2 pkg list | grep moveit`

### Krok 3: Zależności Python
- [ ] Python 3.10+ zainstalowany
- [ ] pip zaktualizowany: `pip3 install --upgrade pip`
- [ ] PyTorch zainstalowany: `pip3 list | grep torch`
- [ ] OpenCV zainstalowany: `pip3 list | grep opencv`
- [ ] Wszystkie pakiety z requirements.txt zainstalowane

### Krok 4: Workspace ROS 2
- [ ] Workspace utworzony w `~/ros2_ws/`
- [ ] Repozytorium sklonowane w `~/ros2_ws/src/`
- [ ] rosdep update wykonany
- [ ] Zależności ROS zainstalowane: `rosdep install --from-paths src`
- [ ] Pakiet zbudowany: `colcon build --packages-select g1_pick_and_handover`
- [ ] Workspace source'owany: `source ~/ros2_ws/install/setup.bash`

### Krok 5: Weryfikacja
- [ ] Pakiet widoczny: `ros2 pkg list | grep g1_pick_and_handover`
- [ ] Node'y dostępne: `ros2 pkg executables g1_pick_and_handover`
- [ ] Brak błędów podczas source'owania

---

## 🎓 Checklist nauki

### Poziom 1: Początkujący

#### Zrozumienie podstaw ROS 2
- [ ] Wiem czym jest node
- [ ] Wiem czym jest topic
- [ ] Potrafię uruchomić przykładowy node (`ros2 run`)
- [ ] Potrafię wyświetlić listę topików (`ros2 topic list`)
- [ ] Potrafię zobaczyć dane na topiku (`ros2 topic echo`)

#### Uruchomienie systemu
- [ ] Uruchomiłem kompletny system (full_handover_pipeline.launch.py)
- [ ] System działa bez błędów krytycznych
- [ ] Rozumiem co robią poszczególne node'y
- [ ] Potrafię zatrzymać system (Ctrl+C)

#### Podstawowa diagnostyka
- [ ] Potrafię sprawdzić status node'ów (`ros2 node list`)
- [ ] Potrafię zobaczyć topiki (`ros2 topic list`)
- [ ] Potrafię użyć rqt_graph do wizualizacji
- [ ] Przeczytałem QUICK_START.md
- [ ] Przeczytałem podstawowe sekcje README.md

### Poziom 2: Średniozaawansowany

#### Percepcja
- [ ] Rozumiem jak działa YOLOv5
- [ ] Potrafię zmienić próg pewności detekcji
- [ ] Rozumiem konwersję 2D → 3D (Pinhole Camera Model)
- [ ] Przetestowałem detekcję na różnych obiektach
- [ ] Wykonałem Tutorial 2 z TUTORIALS.md

#### Manipulacja
- [ ] Rozumiem jak działa MoveIt 2
- [ ] Potrafię zaplanować prostą trajektorię w RViz
- [ ] Rozumiem IK (Inverse Kinematics)
- [ ] Wiem jak dodać przeszkody do planning scene
- [ ] Wykonałem Tutorial 3 z TUTORIALS.md

#### Integracja
- [ ] Rozumiem przepływ danych (perception → decision → action)
- [ ] Uruchomiłem pełny pipeline handover
- [ ] Potrafię symulować człowieka (human_hand_pose)
- [ ] Wykonałem Tutorial 4 z TUTORIALS.md

### Poziom 3: Zaawansowany

#### Konfiguracja i optymalizacja
- [ ] Zmodyfikowałem parametry w grasp_params.yaml
- [ ] Przetestowałem różne presety (beginner/advanced)
- [ ] Zmierzyłem wydajność systemu (FPS, planning time)
- [ ] Zoptymalizowałem parametry dla mojego przypadku
- [ ] Wykonałem Tutorial 5 z TUTORIALS.md (Eksperymenty)

#### Rozwój
- [ ] Dodałem własny node ROS 2
- [ ] Zmodyfikowałem logikę decyzyjną
- [ ] Dodałem nową funkcję do systemu
- [ ] Przetestowałem zmiany w symulacji
- [ ] Stworzyłem Pull Request z ulepszeniem

#### Architektura i debugging
- [ ] Przeczytałem ARCHITECTURE.md
- [ ] Rozumiem automat stanów (FSM)
- [ ] Potrafię zdiagnozować błędy (TROUBLESHOOTING.md)
- [ ] Używam narzędzi debug (rqt_console, ros2 bag)
- [ ] Potrafię napisać test jednostkowy

---

## 🎯 Checklist projektowy

Jeśli robisz projekt oparty na tym systemie:

### Planowanie
- [ ] Zdefiniowałem cel projektu
- [ ] Określiłem wymagania (hardware, software)
- [ ] Stworzyłem timeline projektu
- [ ] Przygotowałem workspace i środowisko

### Implementacja
- [ ] Zidentyfikowałem moduły do modyfikacji
- [ ] Napisałem pseudokod/plan implementacji
- [ ] Stworzyłem branch w git
- [ ] Zaimplementowałem zmiany
- [ ] Przetestowałem każdy moduł osobno

### Testowanie
- [ ] Testy jednostkowe napisane
- [ ] Testy integracyjne napisane
- [ ] Symulacja działa poprawnie
- [ ] Zbadałem edge cases
- [ ] Zmierzyłem wydajność

### Dokumentacja
- [ ] Dodałem docstringi do kodu
- [ ] Zaktualizowałem README (jeśli potrzebne)
- [ ] Stworzyłem przykłady użycia
- [ ] Napisałem raport/sprawozdanie

### Finalizacja
- [ ] Code review przeprowadzony
- [ ] Merge do main branch
- [ ] Tag/Release utworzony
- [ ] Prezentacja/demo przygotowane

---

## 🔧 Checklist troubleshooting

Kiedy coś nie działa, przejdź przez tę listę:

### Podstawowe sprawdzenia
- [ ] System włączony i stabilny
- [ ] Żadne nagłe błędy w terminalu
- [ ] ROS 2 workspace source'owany
- [ ] Wystarczająca ilość RAM/CPU

### Diagnostyka ROS 2
- [ ] `ros2 node list` pokazuje wszystkie node'y
- [ ] `ros2 topic list` pokazuje wszystkie topiki
- [ ] `ros2 topic hz <topic>` pokazuje częstotliwość publikacji
- [ ] `rqt_graph` pokazuje poprawne połączenia

### Sprawdzenie modułów
- [ ] **Kamera**: `/camera/color/image_raw` publikuje
- [ ] **YOLO**: `/object_detections` publikuje
- [ ] **Pose**: `/object_pose` publikuje
- [ ] **MoveIt**: Node `move_group` działa

### Logi i debug
- [ ] Przeczytałem logi w terminalu
- [ ] Sprawdziłem `/rosout` (`ros2 topic echo /rosout`)
- [ ] Włączyłem debug level: `--log-level debug`
- [ ] Nagrałem problem: `ros2 bag record -a`

### Dokumentacja
- [ ] Przeszukałem FAQ.md
- [ ] Sprawdziłem TROUBLESHOOTING.md
- [ ] Znalazłem podobny problem w Issues
- [ ] Jeśli nic nie pomaga - otworzyłem Issue

---

## 📊 Checklist wydajności

Dla optymalizacji systemu:

### Percepcja
- [ ] FPS kamery: _____ (oczekiwane: >15 FPS)
- [ ] YOLO inference time: _____ ms (oczekiwane: <100ms)
- [ ] Detection rate: _____ Hz
- [ ] False positive rate: _____ %

### Manipulacja
- [ ] Czas planowania MoveIt: _____ s (oczekiwane: <5s)
- [ ] Success rate grasp: _____ % (oczekiwane: >80%)
- [ ] Dokładność pozycji: _____ cm (oczekiwane: ±2cm)
- [ ] Czas wykonania handover: _____ s

### Ogólne
- [ ] CPU usage: _____ % (oczekiwane: <70%)
- [ ] RAM usage: _____ GB (oczekiwane: <4GB)
- [ ] Latencja reakcji: _____ ms (oczekiwane: <1s)
- [ ] Stabilność: _____ crashes/hour (oczekiwane: 0)

---

## 🎉 Gratulacje!

Jeśli zaznaczyłeś większość checkboxów, jesteś gotowy do:
- ✅ Pracy z systemem robotycznym
- ✅ Modyfikowania i rozwijania projektu
- ✅ Tworzenia własnych aplikacji HRI
- ✅ Dzielenia się wiedzą z innymi!

**Powodzenia w robotyce! 🤖**

---

**Wskazówka:** Zapisz ten plik jako swoją osobistą listę postępów. Możesz ją wydrukować lub edytować lokalnie!
