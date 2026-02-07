# 📋 Instrukcja dla Maintainera - Aktualizacja Metadanych GitHub

Ten dokument zawiera instrukcje krok po kroku dla maintainera projektu Robot G1 Handover, jak zaktualizować metadane repozytorium GitHub.

---

## 📝 Spis Treści

1. [Aktualizacja pola "About"](#1-aktualizacja-pola-about)
2. [Tworzenie Release](#2-tworzenie-release)
3. [Tworzenie Milestones](#3-tworzenie-milestones)
4. [Tworzenie Projects](#4-tworzenie-projects)
5. [Aktualizacja Topics](#5-aktualizacja-topics)
6. [Konfiguracja Repository Settings](#6-konfiguracja-repository-settings)

---

## 1. Aktualizacja pola "About"

### Kroki:

1. **Przejdź do strony głównej repozytorium**
   - URL: https://github.com/MatPomGit/robot-g1-Handover

2. **Kliknij ikonę ⚙️ (ustawienia) obok sekcji "About"**
   - Znajduje się w prawym górnym rogu, nad listą plików

3. **Wypełnij formularz:**

   **Description** (max 350 znaków):
   ```
   Edukacyjny system interakcji człowiek-robot dla robota Unitree G1. 
   Demonstruje inteligentne przekazywanie obiektów z wykorzystaniem 
   percepcji wizyjnej (YOLO), planowania ruchu (MoveIt 2) i AI (World Model). 
   Kompletna dokumentacja dla studentów robotyki. ROS 2 Humble | Python 3.10+
   ```

   **Website** (opcjonalnie):
   ```
   https://matpomgit.github.io/robot-g1-Handover/
   ```
   *lub zostaw puste jeśli nie ma strony*

   **Topics** (wybierz lub dodaj):
   - Technologie: `ros2`, `moveit2`, `python`, `yolov5`, `computer-vision`, `pytorch`
   - Zastosowania: `robotics`, `human-robot-interaction`, `object-handover`, `motion-planning`
   - Edukacja: `educational`, `tutorial`, `learning`, `robotics-education`
   - Robot: `unitree-g1`, `humanoid-robot`
   - AI: `machine-learning`, `deep-learning`, `world-model`

   **Checkboxy:**
   - ☑️ Include in the home page (zalecane)
   - ☑️ Releases (będzie aktywne po utworzeniu release)

4. **Kliknij "Save changes"**

### Weryfikacja:

Odśwież stronę repozytorium - sekcja "About" powinna pokazywać:
- Opis projektu
- Ikonki topics (klikalne linki)
- Link do website (jeśli dodano)
- Liczba releases (po utworzeniu)

---

## 2. Tworzenie Release

### Kroki:

1. **Przejdź do zakładki "Releases"**
   - URL: https://github.com/MatPomGit/robot-g1-Handover/releases
   - Lub kliknij "Releases" w prawym panelu strony głównej

2. **Kliknij "Create a new release" (lub "Draft a new release")**

3. **Wypełnij formularz Release:**

   **Choose a tag:**
   - Wpisz: `v1.0.0`
   - Wybierz target: `main` branch
   - Kliknij "Create new tag: v1.0.0 on publish"

   **Release title:**
   ```
   v1.0.0 - Foundation Release
   ```

   **Describe this release:**
   
   Skopiuj zawartość z pliku `RELEASE_NOTES.md` (sekcja dla v1.0.0) lub użyj skróconej wersji:

   ```markdown
   # 🎉 Pierwsze Stabilne Wydanie - Robot G1 Handover v1.0.0
   
   **Data wydania**: 7 lutego 2026
   **Nazwa kodowa**: "Foundation"
   
   ## ✨ Główne Funkcjonalności
   
   ### 👁️ Percepcja Wizualna
   - ✅ YOLOv5 object detection (80 klas COCO)
   - ✅ 6D pose estimation (dokładność ±2cm)
   - ✅ RGB-D camera support (Intel RealSense)
   - ✅ Static TF transformations
   
   ### 🦾 Manipulacja i Planowanie
   - ✅ MoveIt 2 integration (RRTConnect planner)
   - ✅ Grasp planning (pre-grasp, grasp, lift)
   - ✅ Handover planning (safe trajectories)
   - ✅ Collision avoidance
   - ✅ Gripper control (open/close)
   
   ### 🧠 Podejmowanie Decyzji
   - ✅ Finite State Machine (5 stanów)
   - ✅ World Model AI framework
   - ✅ Mock decision mode (działa bez AI)
   - ✅ Task manager dla handover
   
   ### 📚 Dokumentacja
   - ✅ 15 plików dokumentacji (~15,000 słów)
   - ✅ 8 szczegółowych tutoriali
   - ✅ FAQ (20+ pytań)
   - ✅ Troubleshooting guide
   - ✅ Architecture document
   - ✅ Glossary (50+ terminów)
   
   ## 🚀 Szybki Start
   
   ```bash
   # 1. Sklonuj repozytorium
   git clone https://github.com/MatPomGit/robot-g1-Handover.git
   cd robot-g1-Handover
   
   # 2. Zainstaluj zależności
   rosdep install --from-paths . --ignore-src -r -y
   pip3 install -r requirements.txt
   
   # 3. Zbuduj i uruchom
   colcon build --packages-select g1_pick_and_handover
   source install/setup.bash
   ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
   ```
   
   ## 📊 Metryki
   
   - **4,847 linii** kodu Python
   - **15 modułów** (perception + manipulation + decision)
   - **6 executable nodes** ROS 2
   - **3 launch files**
   - **15 plików** dokumentacji
   - **8 tutoriali** dla studentów
   
   ## 📖 Dokumentacja
   
   - [README.md](README.md) - Główny przewodnik
   - [QUICK_START.md](QUICK_START.md) - 5-minutowa instalacja
   - [TUTORIALS.md](TUTORIALS.md) - Tutoriale krok po kroku
   - [FAQ.md](FAQ.md) - Najczęstsze pytania
   - [CHANGELOG.md](CHANGELOG.md) - Historia zmian
   
   ## ⚙️ Wymagania
   
   - Ubuntu 22.04 LTS
   - ROS 2 Humble
   - Python 3.10+
   - MoveIt 2
   - 4GB RAM minimum
   
   ## 🎓 Dla Kogo?
   
   - 👨‍🎓 **Studenci robotyki** - nauka ROS 2, MoveIt 2, computer vision
   - 👩‍🏫 **Nauczyciele** - gotowy materiał dydaktyczny
   - 🔬 **Badacze** - platforma do eksperymentów z HRI
   - 🤖 **Entuzjaści** - poznaj zaawansowane systemy robotyczne
   
   ## 🙏 Podziękowania
   
   Dziękujemy społeczności open source:
   - ROS 2 Humble
   - MoveIt 2
   - Ultralytics YOLOv5
   - PyTorch
   - Unitree Robotics
   
   ---
   
   **Pełne Release Notes**: Zobacz [RELEASE_NOTES.md](RELEASE_NOTES.md)
   
   **Zgłaszanie błędów**: https://github.com/MatPomGit/robot-g1-Handover/issues
   
   **Licencja**: MIT
   ```

   **Attach binaries** (opcjonalnie):
   - Możesz załączyć skompilowane pliki, checkpointy AI, test data bags, etc.

   **Checkboxy:**
   - ☐ Set as a pre-release (dla beta versions)
   - ☐ Set as the latest release (zaznacz dla v1.0.0)

4. **Kliknij "Publish release"**

### Weryfikacja:

- Release pojawi się na https://github.com/MatPomGit/robot-g1-Handover/releases
- Badge w README pokaże "release v1.0.0"
- Tag `v1.0.0` będzie widoczny w "Tags"

---

## 3. Tworzenie Milestones

### Kroki:

1. **Przejdź do zakładki "Issues"**
   - URL: https://github.com/MatPomGit/robot-g1-Handover/issues

2. **Kliknij "Milestones" (u góry, obok "Labels")**

3. **Kliknij "New milestone"**

4. **Utwórz każdy milestone z pliku MILESTONES.md:**

### Milestone 1: Foundation (v1.0.0) - Zakończony

- **Title**: `v1.0.0 - Foundation`
- **Due date**: `2026-02-07`
- **Description**:
  ```markdown
  ✅ **Zakończony**
  
  Solidna podstawa systemu handover z kompletną dokumentacją.
  
  **Główne cele:**
  - YOLOv5 object detection
  - MoveIt 2 integration
  - FSM dla handover
  - Kompletna dokumentacja (15 plików)
  
  **Metryki:**
  - 4,847 linii kodu
  - 15 modułów
  - 8 tutoriali
  
  Zobacz: [MILESTONES.md](MILESTONES.md#m1-foundation-v100)
  ```
- **Kliknij "Create milestone"**
- **Po utworzeniu**: Kliknij na milestone i "Close milestone" (ponieważ jest zakończony)

### Milestone 2: Hand Tracking (v1.1.0) - W toku

- **Title**: `v1.1.0 - Hand Tracking`
- **Due date**: `2026-04-30`
- **Description**:
  ```markdown
  🚧 **W toku** (30% postępu)
  
  Pełna detekcja dłoni człowieka z MediaPipe.
  
  **Cele:**
  - MediaPipe integration
  - Gesture recognition
  - Intention prediction
  - Multi-hand tracking
  - Real-time hand pose publishing
  
  **Metryki docelowe:**
  - Hand detection >20 FPS
  - Pose accuracy <5cm
  - Gesture recognition >85%
  
  Zobacz: [MILESTONES.md](MILESTONES.md#m2-hand-tracking-v110)
  ```
- **Pozostaw otwarty**

### Milestone 3: Testing Suite (v1.2.0) - Zaplanowany

- **Title**: `v1.2.0 - Testing Suite`
- **Due date**: `2026-06-30`
- **Description**:
  ```markdown
  📝 **Zaplanowany**
  
  Kompleksowe testy i CI/CD.
  
  **Cele:**
  - Unit tests (pytest)
  - Integration tests
  - End-to-end tests
  - CI/CD pipeline (GitHub Actions)
  - Code coverage >80%
  
  Zobacz: [MILESTONES.md](MILESTONES.md#m3-testing-suite-v120)
  ```

### Milestone 4: Simulation (v1.3.0) - Zaplanowany

- **Title**: `v1.3.0 - Simulation`
- **Due date**: `2026-09-30`
- **Description**:
  ```markdown
  📝 **Zaplanowany**
  
  Pełne wsparcie symulacji.
  
  **Cele:**
  - Gazebo integration
  - MuJoCo integration
  - Synthetic data generation
  - Sim-to-real transfer
  
  Zobacz: [MILESTONES.md](MILESTONES.md#m4-simulation-support-v130)
  ```

### Milestone 5: AI Enhancement (v2.0.0) - Zaplanowany

- **Title**: `v2.0.0 - AI Enhancement`
- **Due date**: `2026-12-31`
- **Description**:
  ```markdown
  📝 **Zaplanowany**
  
  Zaawansowane funkcje AI.
  
  **Cele:**
  - Full WMA integration
  - Model training pipeline
  - Improved intention recognition
  - Learning-based planning
  
  Zobacz: [MILESTONES.md](MILESTONES.md#m5-ai-enhancement-v200)
  ```

5. **Linkowanie Issues do Milestones:**
   - Przy tworzeniu issue, wybierz odpowiedni milestone z dropdown
   - Lub edytuj istniejące issues i przypisz do milestone

### Weryfikacja:

- Lista milestones widoczna na: https://github.com/MatPomGit/robot-g1-Handover/milestones
- Progress bars pokazują postęp (issues closed / total issues)

---

## 4. Tworzenie Projects

### Kroki:

1. **Przejdź do zakładki "Projects"**
   - URL: https://github.com/MatPomGit/robot-g1-Handover/projects

2. **Kliknij "New project"**

3. **Wybierz template:**
   - "Board" (Kanban board - zalecane dla projektów rozwojowych)
   - lub "Table" (spreadsheet view)

4. **Utwórz każdy projekt z pliku PROJECTS.md:**

### Project 1: Core System (Zakończony)

- **Project name**: `P1: Core System`
- **Description**:
  ```markdown
  ✅ **Zakończony** (100%)
  
  Solidna podstawa systemu handover.
  
  **Komponenty:**
  - Perception Module (YOLOv5, 6D pose)
  - Manipulation Module (MoveIt 2, grasp planning)
  - Decision Module (FSM, WMA framework)
  - Infrastructure (ROS 2 package, launch files)
  
  **Osiągnięcia:**
  - 4,847 linii kodu
  - 15 modułów
  - 85% success rate w planowaniu trajektorii
  
  Zobacz: [PROJECTS.md](PROJECTS.md#p1-core-system)
  ```
- **Template**: Board
- **Kliknij "Create project"**

### Project 2: Educational Platform (W toku)

- **Project name**: `P2: Educational Platform`
- **Description**:
  ```markdown
  🚧 **W toku** (75%)
  
  Platforma edukacyjna z tutorialami i materiałami.
  
  **Zakres:**
  - Dokumentacja (90% ✅)
  - Interactive Elements (50% 🚧)
  - Community (60% 🚧)
  
  **Osiągnięcia:**
  - 15 plików dokumentacji
  - 8 tutoriali
  - 50+ terminów w słowniczku
  
  Zobacz: [PROJECTS.md](PROJECTS.md#p2-educational-platform)
  ```

### Project 3: Advanced Perception (W toku)

- **Project name**: `P3: Advanced Perception`
- **Description**:
  ```markdown
  🚧 **W toku** (40%)
  
  Zaawansowane funkcje percepcji.
  
  **Zakres:**
  - Hand Tracking (50% 🚧)
  - Object Tracking (20% 🚧)
  - Scene Understanding (5% ⏳)
  
  Zobacz: [PROJECTS.md](PROJECTS.md#p3-advanced-perception)
  ```

### Project 4: AI & Learning (Zaplanowany)

- **Project name**: `P4: AI & Learning`
- **Description**:
  ```markdown
  📝 **Zaplanowany** (10%)
  
  Integracja zaawansowanych technik AI.
  
  **Zakres:**
  - World Model AI
  - Learning-based Planning
  - Intention Prediction
  
  Zobacz: [PROJECTS.md](PROJECTS.md#p4-ai--learning)
  ```

5. **Dodaj kolumny do Board:**
   - `📋 Backlog` - Zadania do zrobienia
   - `🚧 In Progress` - W trakcie
   - `👀 Review` - Do przeglądu
   - `✅ Done` - Zakończone

6. **Dodaj issues/karty do projektu:**
   - Z poziomu Project Board: kliknij "+ Add item"
   - Możesz dodać istniejące issues lub utworzyć nowe karty

### Weryfikacja:

- Projekty widoczne na: https://github.com/MatPomGit/robot-g1-Handover/projects
- Kanban board pokazuje postęp zadań

---

## 5. Aktualizacja Topics

### Kroki:

1. **Z poziomu sekcji "About" (patrz punkt 1)**

2. **Lista rekomendowanych topics:**

   **Kategorie:**
   
   **Technologie (9):**
   - `ros2`
   - `ros2-humble`
   - `moveit2`
   - `python`
   - `pytorch`
   - `yolov5`
   - `computer-vision`
   - `deep-learning`
   - `object-detection`

   **Robotyka (6):**
   - `robotics`
   - `humanoid-robot`
   - `unitree-g1`
   - `robot-manipulation`
   - `motion-planning`
   - `grasp-planning`

   **HRI & Aplikacje (4):**
   - `human-robot-interaction`
   - `hri`
   - `object-handover`
   - `pose-estimation`

   **Edukacja (3):**
   - `educational`
   - `tutorial`
   - `robotics-education`

   **AI (2):**
   - `machine-learning`
   - `world-model`

   **TOTAL: 24 topics** (GitHub limit: max 20)

3. **Wybierz 20 najważniejszych:**

   ```
   ros2, moveit2, python, yolov5, computer-vision, 
   robotics, human-robot-interaction, object-handover, 
   motion-planning, educational, tutorial, unitree-g1, 
   humanoid-robot, pytorch, deep-learning, object-detection, 
   pose-estimation, robot-manipulation, grasp-planning, world-model
   ```

4. **Dodaj topics:**
   - W polu "Topics" wpisz każdy topic i naciśnij Enter
   - GitHub podpowie istniejące topics (użyj ich jeśli pasują)

### Weryfikacja:

- Topics widoczne jako klikalne badges w sekcji "About"
- Repozytorium będzie pojawiać się w wynikach wyszukiwania dla tych topics

---

## 6. Konfiguracja Repository Settings

### Recommended Settings:

1. **Przejdź do Settings → General**

   **Features:**
   - ☑️ Issues
   - ☑️ Projects
   - ☐ Wiki (opcjonalnie, jeśli chcesz użyć)
   - ☑️ Discussions (zalecane dla Q&A społeczności)

   **Pull Requests:**
   - ☑️ Allow merge commits
   - ☑️ Allow squash merging (zalecane)
   - ☐ Allow rebase merging
   - ☑️ Always suggest updating pull request branches
   - ☑️ Automatically delete head branches

2. **Settings → Branches**

   **Branch protection rules dla `main`:**
   - Kliknij "Add rule"
   - Branch name pattern: `main`
   - ☑️ Require a pull request before merging
   - ☑️ Require approvals: 1
   - ☑️ Dismiss stale pull request approvals when new commits are pushed
   - ☑️ Require status checks to pass before merging (gdy będzie CI/CD)
   - ☐ Require conversation resolution before merging
   - ☑️ Include administrators (opcjonalnie)

3. **Settings → Actions**

   **General:**
   - ☑️ Allow all actions and reusable workflows (dla CI/CD w przyszłości)

4. **Settings → Security**

   **Dependabot:**
   - ☑️ Enable Dependabot alerts
   - ☑️ Enable Dependabot security updates

### Weryfikacja:

- Settings zapisane
- Branch protection aktywna (próbuj push do main - powinno wymagać PR)

---

## 📋 Quick Checklist dla v1.0.0 Release

### Przed Release:

- [ ] Wersja zaktualizowana (1.0.0) w package.xml i setup.py
- [ ] CHANGELOG.md utworzony i zaktualizowany
- [ ] RELEASE_NOTES.md utworzony dla v1.0.0
- [ ] MILESTONES.md utworzony
- [ ] PROJECTS.md utworzony
- [ ] ROADMAP.md utworzony
- [ ] ABOUT.md utworzony
- [ ] Wszystkie README.md w modułach zaktualizowane
- [ ] Dokumentacja kompletna i aktualna
- [ ] Testy przechodzą (jeśli są)
- [ ] Brak znanych critical bugs

### W GitHub:

- [ ] Pole "About" zaktualizowane
- [ ] Topics dodane (20 topics)
- [ ] Release v1.0.0 utworzony
- [ ] Tag v1.0.0 utworzony
- [ ] Milestones utworzone (M1-M5)
- [ ] M1: Foundation zamknięty jako "completed"
- [ ] Projects utworzone (P1-P4)
- [ ] Repository settings skonfigurowane
- [ ] Branch protection rules ustawione

### Po Release:

- [ ] Announcement w Discussions
- [ ] Tweet/social media (jeśli dotyczy)
- [ ] Email do contributors (jeśli jest lista)
- [ ] Update README badges
- [ ] Celebrate! 🎉

---

## 🆘 Troubleshooting

### Problem: Nie mogę utworzyć Release

**Rozwiązanie:**
- Upewnij się że masz uprawnienia "Write" do repozytorium
- Sprawdź czy branch `main` istnieje
- Tag musi być unikalny (nie może istnieć `v1.0.0`)

### Problem: Milestones nie pokazują postępu

**Rozwiązanie:**
- Upewnij się że issues są przypisane do milestone
- Zamykaj issues po zakończeniu (progress = closed/total)

### Problem: Topics się nie zapisują

**Rozwiązanie:**
- Max 20 topics
- Topics muszą być lowercase, bez spacji
- Używaj myślników `-` zamiast spacji

---

## 📞 Pytania?

Jeśli masz pytania dotyczące tej instrukcji:

- Otwórz Issue: https://github.com/MatPomGit/robot-g1-Handover/issues
- Email: contact@robotg1handover.org

---

<div align="center">

**Powodzenia z aktualizacją repozytorium!** 🚀

</div>
