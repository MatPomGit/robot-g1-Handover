# 🚀 Szybki Start - Robot G1 Handover

**Chcesz od razu zobaczyć system w akcji?** Ten przewodnik zajmie ci tylko **5 minut**!

---

## ⚡ Opcja 1: Szybka instalacja (masz już ROS 2 Humble)

```bash
# 1. Sklonuj repozytorium
cd ~/ros2_ws/src
git clone https://github.com/MatPomGit/robot-g1-Handover.git

# 2. Zainstaluj zależności
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r src/robot-g1-Handover/requirements.txt

# 3. Zbuduj pakiet
colcon build --packages-select g1_pick_and_handover

# 4. Source workspace
source install/setup.bash

# 5. Uruchom system! 🎉
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

**✅ Gotowe!** System jest uruchomiony.

---

## 📚 Opcja 2: Pierwsza instalacja (nie masz ROS 2)

### Krok 1: Zainstaluj ROS 2 Humble (Ubuntu 22.04)

```bash
# Dodaj repozytorium ROS 2
sudo apt update && sudo apt install -y curl software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Dodaj źródła
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Zainstaluj ROS 2 Humble + MoveIt 2
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-moveit

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

### Krok 2: Postępuj według Opcji 1 powyżej

---

## 🧪 Opcja 3: Test bez fizycznego robota

Jeśli nie masz kamery lub robota, możesz przetestować system w trybie symulacji:

```bash
# TERMINAL 1: Uruchom system
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py

# TERMINAL 2: Symuluj dane z kamery (jeśli masz plik bag)
ros2 bag play test_data.bag --loop

# TERMINAL 3: Wizualizacja w RViz
rviz2
```

---

## 🎯 Co dalej?

### Sprawdź czy system działa

```bash
# Zobacz aktywne node'y
ros2 node list

# Monitoruj wykrywanie obiektów
ros2 topic echo /object_detections

# Zobacz pozycję dłoni człowieka
ros2 topic echo /human_hand_pose
```

### Naucz się więcej

- 📖 **[README.md](README.md)** - Pełna dokumentacja
- 🎓 **[TUTORIALS.md](TUTORIALS.md)** - Tutoriale krok po kroku
- ❓ **[FAQ.md](FAQ.md)** - Najczęściej zadawane pytania
- 📋 **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Ściąga z komendami

---

## 🐛 Coś nie działa?

### Problem: "package not found"

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Problem: Brak kamery

Użyj danych testowych:
```bash
ros2 bag play test_data.bag --loop
```

### Problem: "WMA not available"

**To jest normalne!** System działa w trybie mock (prosty if-else). Nie potrzebujesz WMA.

### Więcej problemów?

Zobacz szczegółowe rozwiązania w [FAQ.md](FAQ.md) lub otwórz [Issue na GitHub](https://github.com/MatPomGit/robot-g1-Handover/issues).

---

## 🎉 Sukces!

Jeśli system się uruchomił, gratulacje! 🎊

**Następne kroki:**
1. Przejrzyj kod w katalogu `perception/`, `manipulation/`, `decision/`
2. Wypróbuj tutoriale z [TUTORIALS.md](TUTORIALS.md)
3. Zmodyfikuj parametry w `config/grasp_params.yaml`
4. Dodaj własne funkcje!

---

**Potrzebujesz pomocy?** Otwórz Issue lub skontaktuj się z społecznością! 💬
