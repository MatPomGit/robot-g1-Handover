# Moduł Decyzyjny (Decision Module)

## 📖 Wprowadzenie

Moduł decyzyjny wykorzystuje sztuczną inteligencję (World Model AI - WMA) do podejmowania inteligentnych decyzji o akcjach robota. Jest to "mózg" robota, który analizuje sytuację i wybiera najlepsze działanie.

## 🎯 Funkcje Modułu

### 1. **WMA Handover Manager** (`wma_handover_manager.py`)
- Podejmuje decyzje o akcjach handover
- Analizuje multimodalne obserwacje
- Przewiduje intencje człowieka
- Planuje sekwencje akcji

### 2. **WMA Task Manager** (`wma_task_manager.py`)
- Zarządza automatem stanów (FSM)
- Sekwencja chwytania: idle -> approach -> grasp -> lift
- Uproszczony model dydaktyczny

## 🧠 Co to jest World Model AI (WMA)?

World Model AI to zaawansowany model sztucznej inteligencji, który:

### Architektura:
```
Obserwacje (RGB, głębokość, stan)
        ↓
    Encoder (CNN, Transformer)
        ↓
Reprezentacja Latentna (compressed state)
        ↓
  World Model (przewiduje przyszłość)
        ↓
    Policy (wybiera akcje)
        ↓
   Akcje (TAKE/GIVE/IDLE)
```

### Kluczowe Cechy:
1. **Multimodalne**: Przetwarza obraz, czujniki, stan
2. **Predykcyjne**: Przewiduje co się stanie po akcji
3. **Hierarchiczne**: Planuje na różnych poziomach abstrakcji
4. **Adaptacyjne**: Uczy się z doświadczenia

## 📊 Przepływ Decyzji

```
Obserwacje:
├── camera_rgb (obraz z kamery)
├── gripper_state (stan chwytaka)
├── human_pose (pozycja dłoni)
└── human_reaching (intencja)
        ↓
    [WMA Preprocessing]
        ↓
  Tensory PyTorch:
  ├── camera_tensor [1, 3, H, W]
  ├── binary_features [1, 2]
  └── human_pos [1, 3]
        ↓
    [WMA Inference]
        ↓
Akcje: ['IDLE', 'TAKE_FROM_HUMAN', 'GIVE_TO_HUMAN']
        ↓
    [Wybór pierwszej akcji]
        ↓
    Wykonanie (MoveIt 2)
```

## 🔧 Decyzje WMA

### TAKE_FROM_HUMAN
**Kiedy:** Człowiek trzyma obiekt i wyciąga rękę w kierunku robota

**Sekwencja:**
1. Oblicz pozycję handover (z offsetem)
2. Przesuń ramię do pozycji
3. Zamknij chwytak (chwyć obiekt)

**Obserwacje wskazujące na TAKE:**
- human_reaching = True
- gripper_state = False (pusty)
- Człowiek trzyma obiekt (z obrazu)
- Dłoń blisko robota

### GIVE_TO_HUMAN
**Kiedy:** Robot trzyma obiekt, a człowiek wyciąga rękę

**Sekwencja:**
1. Oblicz pozycję handover
2. Przesuń ramię (z obiektem) do pozycji
3. Otwórz chwytak (puść obiekt)

**Obserwacje wskazujące na GIVE:**
- human_reaching = True
- gripper_state = True (trzyma obiekt)
- Dłoń blisko robota
- Otwarta dłoń (z obrazu)

### IDLE
**Kiedy:** Brak jasnej intencji lub sytuacja niejednoznaczna

**Zachowanie:**
- Robot czeka
- Nie wykonuje ruchu
- Monitoruje sytuację

**Obserwacje wskazujące na IDLE:**
- human_reaching = False
- Dłoń daleko od robota
- Niejednoznaczna gestykulacja

## 🚀 Użycie

### Uruchomienie WMA Handover:

```bash
# Terminal 1: MoveIt 2
ros2 launch g1_moveit_config demo.launch.py

# Terminal 2: WMA Handover Manager
ros2 run g1_pick_and_handover execute_handover_wma

# Terminal 3: Publikuj obserwacje (test)
ros2 topic pub /human_hand_pose geometry_msgs/PoseStamped "{...}"
ros2 topic pub /human_reaching std_msgs/Bool "data: true"
```

### Testowanie decyzji:

```python
# Python REPL
import rclpy
from decision.wma_handover_manager import WMAHandoverManager

wma = WMAHandoverManager()

observation = {
    "camera_rgb": image,         # numpy array
    "gripper_state": False,
    "human_pose": pose,
    "human_reaching": True
}

action = wma.infer_action(observation)
print(f"WMA Decision: {action}")  # 'TAKE_FROM_HUMAN'
```

## 📚 Kluczowe Algorytmy

### Model Predictive Control (MPC)
WMA używa MPC do planowania:

```
Horizon = 8 kroków
t=0: IDLE
t=1: IDLE
t=2: TAKE_FROM_HUMAN ← wykonaj teraz
t=3: IDLE
t=4: IDLE
t=5: GIVE_TO_HUMAN
t=6: IDLE
t=7: IDLE
```

### Preprocessing Obserwacji

```python
# Obraz RGB
camera_tensor = torch.from_numpy(camera)
camera_tensor = camera_tensor.permute(2, 0, 1)  # HWC -> CHW
camera_tensor = camera_tensor.float() / 255.0   # [0-255] -> [0-1]
camera_tensor = camera_tensor.unsqueeze(0)      # [C,H,W] -> [1,C,H,W]

# Binary features
binary = torch.tensor([gripper_state, human_reaching])
binary = binary.float().unsqueeze(0)            # [2] -> [1, 2]

# Pozycja człowieka
human_pos = torch.tensor([x, y, z])
human_pos = human_pos.float().unsqueeze(0)      # [3] -> [1, 3]
```

### Automat Stanów (FSM) - Uproszczony

```
    idle
     ↓
  approach (pre-grasp)
     ↓
   grasp (chwyt)
     ↓
    lift (podniesienie)
```

**UWAGA**: To uproszczenie dydaktyczne! Prawdziwy WMA używa probabilistycznych przejść.

## 🛠️ Konfiguracja

### Checkpoint WMA:

```bash
# Pobierz pretrenowany checkpoint
wget https://example.com/unifolm_wma_checkpoint.pth

# Lub wytrenuj własny
python train_wma.py --dataset handover_data --epochs 100
```

### Parametry WMA:

```python
wma = WMAHandoverManager(
    checkpoint_path="/path/to/checkpoint",
    device="cuda",           # lub "cpu"
    horizon=8,               # Horyzont planowania
    temperature=1.0          # Temperatura sampling
)
```

## 🔍 Debugowanie

### Problem: WMA nie podejmuje decyzji

```python
# Sprawdź czy checkpoint istnieje
import os
assert os.path.exists(checkpoint_path)

# Sprawdź czy obserwacje są poprawne
print(f"Camera shape: {observation['camera_rgb'].shape}")
print(f"Gripper state: {observation['gripper_state']}")

# Włącz verbose logging
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Problem: Złe decyzje

Przyczyny:
1. **Checkpoint nie pasuje** - wytrenowany na innych danych
2. **Nieprawidłowy preprocessing** - format tensorów
3. **Brak danych treningowych** - mało przykładów tej sytuacji

Rozwiązania:
```python
# Re-trenuj na własnych danych
# Zbierz więcej przykładów problematycznych sytuacji
# Użyj data augmentation
```

### Problem: Wolny inference

```bash
# Użyj GPU
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Zmniejsz rozmiar obrazu
camera_resized = cv2.resize(camera, (320, 240))

# Użyj TensorRT (dla NVIDIA GPU)
# Konwertuj model na TensorRT engine
```

## 📖 Tutorial dla Studentów

### Ćwiczenie 1: Analiza Decyzji WMA

1. Uruchom WMA w trybie debug

2. Zbierz log decyzji dla różnych scenariuszy:
   - Człowiek wyciąga rękę z obiektem
   - Człowiek wyciąga pustą rękę
   - Człowiek daleko od robota

3. **Zadanie**: Stwórz tabelę: Obserwacje -> Decyzja

### Ćwiczenie 2: Modyfikacja Preprocessing

1. W `wma_handover_manager.py`, dodaj nową feature:
   ```python
   object_in_hand = detect_object_in_hand(observation['camera_rgb'])
   features = torch.cat([binary_features, object_in_hand], dim=1)
   ```

2. **Zadanie**: Sprawdź czy to poprawia decyzje

### Ćwiczenie 3: Porównanie z Regułami

1. Zaimplementuj prosty system reguł:
   ```python
   if human_reaching and not gripper_occupied:
       return "TAKE_FROM_HUMAN"
   elif human_reaching and gripper_occupied:
       return "GIVE_TO_HUMAN"
   else:
       return "IDLE"
   ```

2. **Zadanie**: Porównaj success rate: WMA vs Reguły

### Ćwiczenie 4: Trenowanie WMA

1. Zbierz dataset handover (100+ przykładów)

2. Format danych:
   ```
   dataset/
   ├── episode_0001/
   │   ├── obs_0.png (obraz)
   │   ├── state_0.json (stan)
   │   └── action_0.txt (akcja)
   ├── episode_0002/
   ...
   ```

3. **Zadanie**: Wytrenuj własny WMA checkpoint

## 🔬 Zaawansowane

### Hierarchical WMA

```python
# High-level policy
high_action = wma_high.infer("handover_task")  # "approach"

# Low-level policy
if high_action == "approach":
    low_actions = wma_low.plan_trajectory(
        start=current_pose,
        goal=target_pose
    )
```

### Multi-Task WMA

```python
# Jeden WMA dla wielu zadań
wma = WorldModelPolicy.from_pretrained("multi_task_checkpoint")

action = wma.infer_action(
    observation=obs,
    task="handover"  # lub "pick", "place", "clean"
)
```

### Online Learning

```python
# Robot uczy się podczas pracy
experience = (observation, action, reward, next_observation)
wma.update_online(experience)

# Periodic re-training
if steps % 1000 == 0:
    wma.finetune(replay_buffer)
```

## 📚 Dodatkowe Zasoby

- [World Models Paper](https://worldmodels.github.io/)
- [Model Predictive Control](https://en.wikipedia.org/wiki/Model_predictive_control)
- [Reinforcement Learning](https://spinningup.openai.com/)
- [PyTorch Tutorial](https://pytorch.org/tutorials/)

---

**Pytania?** Otwórz Issue na GitHubie lub skonsultuj się z prowadzącym!
