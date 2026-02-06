"""
Manager decyzji WMA dla przekazywania obiektów (WMA Handover Manager)

Ten moduł wykorzystuje World Model AI (WMA) do podejmowania inteligentnych
decyzji o akcjach robota podczas interakcji człowiek-robot.

CO TO JEST WORLD MODEL AI (WMA)?
World Model AI to zaawansowany model sztucznej inteligencji, który:
- Rozumie świat poprzez multimodalne obserwacje (obraz, czujniki, stan)
- Przewiduje przyszłe stany środowiska
- Planuje sekwencje akcji na horyzoncie czasowym
- Generuje wysokopoziomowe decyzje semantyczne

ARCHITEKTURA WMA:
1. Encoder: Przetwarza obserwacje (RGB, głębokość, stan) na reprezentację latentną
2. World Model: Przewiduje dynamikę świata (jak się zmieni po akcji?)
3. Policy: Wybiera najlepszą akcję na podstawie przewidywań
4. Planner: Generuje sekwencję akcji na horyzoncie (np. 8 kroków do przodu)

ZASTOSOWANIE W HANDOVER:
WMA analizuje:
- Obraz z kamery (pozycja dłoni, obiekt, gestyka)
- Stan chwytaka (otwarty/zamknięty, siła)
- Pozycję dłoni człowieka (3D)
- Intencję człowieka (czy wyciąga rękę)

I podejmuje decyzję:
- TAKE_FROM_HUMAN: Człowiek chce dać obiekt
- GIVE_TO_HUMAN: Człowiek chce wziąć obiekt
- IDLE: Brak intencji, czekaj

KROK PO KROKU:
1. Preprocessing obserwacji (konwersja do tensorów PyTorch)
2. Wywołanie WMA.plan_actions() z zadaniem "handover_decision"
3. Otrzymanie sekwencji zaplanowanych akcji
4. Zwrócenie pierwszej akcji do wykonania
"""

import torch
from unifolm_wma.inference import WorldModelPolicy
from geometry_msgs.msg import PoseStamped

class WMAHandoverManager:
    """
    Manager decyzji dla przekazywania obiektów używający WMA
    
    Zastępuje tradycyjny HandoverManager prostymi regułami if-else.
    WMA uczy się z danych i generalizuje na nowe sytuacje.
    
    PRZEWAGI WMA NAD REGUŁAMI:
    - Obsługuje złożone scenariusze (częściowa occluzja, wieloznaczne gesty)
    - Adaptuje się do różnych użytkowników (dzieci, osoby starsze)
    - Przewiduje intencje z wyprzedzeniem (proaktywność)
    - Uczy się z experience replay (continuous improvement)
    """

    def __init__(self, checkpoint_path="/path/to/unifolm_wma_checkpoint"):
        """
        Inicjalizacja WMA Handover Manager
        
        KROK PO KROKU:
        1. Wykrywa dostępność GPU (CUDA)
        2. Ładuje pretrenowany model WMA z checkpointu
        3. Przygotowuje model do inference
        
        Args:
            checkpoint_path: Ścieżka do zapisanego modelu WMA (weights + config)
        
        UWAGA: Checkpoint WMA powinien być:
        - Wytrenowany na danych handover (human-robot interaction dataset)
        - Zawierać enkodery dla wszystkich modalności (RGB, state)
        - Być kompatybilny z interfejsem WorldModelPolicy
        
        GPU vs CPU:
        - GPU (CUDA): Szybki inference (~30 FPS), wymaga NVIDIA GPU
        - CPU: Wolniejszy inference (~5 FPS), działa wszędzie
        """
        # Wykrywanie dostępności GPU
        # torch.cuda.is_available() zwraca True jeśli CUDA jest dostępna
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        
        # Ładowanie pretrenowanego modelu WMA
        # WorldModelPolicy.from_pretrained():
        # 1. Ładuje konfigurację modelu z checkpointu
        # 2. Inicjalizuje architekturę sieci neuronowej
        # 3. Ładuje wagi (parameters) modelu
        # 4. Ustawia model w tryb evaluation (model.eval())
        self.policy = WorldModelPolicy.from_pretrained(checkpoint_path, device=self.device)

    def infer_action(self, observation):
        """
        Wywnioskuj akcję na podstawie obserwacji
        
        KROK PO KROKU:
        1. Preprocessing: Konwertuje obserwacje ROS na tensory PyTorch
        2. Planning: WMA planuje sekwencję akcji na horyzoncie
        3. Selection: Wybiera pierwszą akcję z sekwencji
        4. Zwraca akcję jako string
        
        Args:
            observation: dict z polami:
                - 'camera_rgb': np.array HxWx3 (obraz RGB z kamery)
                - 'gripper_state': bool (True = trzyma obiekt)
                - 'human_pose': PoseStamped (pozycja dłoni człowieka)
                - 'human_reaching': bool (True = wyciąga rękę)
        
        Returns:
            string: Akcja do wykonania
                - 'TAKE_FROM_HUMAN': Robot powinien chwycić obiekt od człowieka
                - 'GIVE_TO_HUMAN': Robot powinien dać obiekt człowiekowi
                - 'IDLE': Robot powinien czekać (brak jasnej intencji)
        
        PLANOWANIE NA HORYZONCIE:
        WMA nie wybiera jednej akcji, ale sekwencję akcji do przodu:
        - Horizon=8 oznacza planowanie 8 kroków do przodu (~2.5 sekundy przy 30Hz)
        - Model przewiduje konsekwencje każdej akcji
        - Wybiera sekwencję maksymalizującą reward/success
        - Pierwsza akcja jest wykonywana teraz, reszta to plan
        
        PRZYKŁAD SEKWENCJI:
        ['IDLE', 'IDLE', 'TAKE_FROM_HUMAN', 'IDLE', 'IDLE', 'GIVE_TO_HUMAN', 'IDLE', 'IDLE']
        -> Pierwsza akcja: 'IDLE' (czekaj, człowiek jeszcze nie gotowy)
        """
        # === PREPROCESSING OBSERWACJI ===
        # Konwertuje surowe dane ROS na format tensorów PyTorch
        # _preprocess_observation() szczegóły poniżej
        obs_tensor = self._preprocess_observation(observation)

        # === PLANOWANIE SEKWENCJI AKCJI ===
        # WMA.plan_actions() wykonuje:
        # 1. Enkodowanie obserwacji (RGB -> features, state -> embeddings)
        # 2. Latent world model rollout (symulacja przyszłości)
        # 3. Policy evaluation (które akcje są najlepsze?)
        # 4. Beam search / sampling (wybór sekwencji akcji)
        action_seq = self.policy.plan_actions(
            observation=obs_tensor,
            task="handover_decision",   # Semantyczny task ID (zdefiniowany podczas treningu)
            horizon=8                    # Planuj 8 kroków do przodu
        )

        # === WYBÓR AKCJI ===
        # Najprostszy wybór: pierwsza zaplanowana akcja
        # W bardziej zaawansowanych systemach:
        # - Model Predictive Control (MPC): re-plan co krok
        # - Hierarchical planning: high-level + low-level actions
        # - Multi-action execution: wykonaj kilka akcji równolegle
        return action_seq[0] if action_seq else "IDLE"

    def _preprocess_observation(self, observation):
        """
        Konwertuje obserwacje ROS na tensory dla WMA
        
        KROK PO KROKU:
        1. Konwertuje obraz RGB (numpy) na tensor PyTorch
        2. Normalizuje piksele do zakresu [0, 1]
        3. Zmienia format z HWC na CHW (Height-Width-Channels -> Channels-Height-Width)
        4. Enkoduje binary features (gripper_state, human_reaching)
        5. Ekstraktuje pozycję dłoni jako wektor 3D
        6. Łączy wszystko w słownik tensorów
        
        Args:
            observation: dict z surowymi obserwacjami (numpy arrays, ROS messages)
        
        Returns:
            dict: Słownik tensorów PyTorch gotowych do WMA
                - "camera_rgb": torch.Tensor [1, 3, H, W]
                - "binary_features": torch.Tensor [1, 2]
                - "human_pos": torch.Tensor [1, 3]
        
        FORMAT TENSORÓW:
        - Batch dimension: Pierwszy wymiar to batch (tutaj zawsze 1)
        - PyTorch convention: Channels-first [B, C, H, W] nie [B, H, W, C]
        - Normalizacja: Piksele w [0, 1] lub [-1, 1] (zależnie od treningu)
        """
        import numpy as np
        
        # === PRZETWARZANIE OBRAZU RGB ===
        camera = observation['camera_rgb']    # numpy array [H, W, 3], dtype uint8, range [0, 255]
        
        # Konwersja: numpy -> torch
        camera_tensor = torch.from_numpy(camera)
        
        # Zmiana układu z HWC -> CHW (Height-Width-Channels -> Channels-Height-Width)
        # PyTorch CNN oczekuje formatu [Batch, Channels, Height, Width]
        # permute(2, 0, 1) zmienia [H, W, C] -> [C, H, W]
        camera_tensor = camera_tensor.permute(2, 0, 1)
        
        # Konwersja do float i normalizacja
        # uint8 [0-255] -> float32 [0.0-1.0]
        camera_tensor = camera_tensor.float() / 255.0
        
        # Dodanie batch dimension: [C, H, W] -> [1, C, H, W]
        # unsqueeze(0) dodaje nowy wymiar na początku
        camera_tensor = camera_tensor.unsqueeze(0)

        # === PRZETWARZANIE BINARY FEATURES ===
        # Zakoduj gripper_state i human_reaching jako tensor binarny
        # [0, 0] = gripper pusty, człowiek nie wyciąga ręki
        # [1, 0] = gripper trzyma obiekt, człowiek nie wyciąga ręki
        # [0, 1] = gripper pusty, człowiek wyciąga rękę
        # [1, 1] = gripper trzyma obiekt, człowiek wyciąga rękę
        binary_features = torch.tensor([
            int(observation['gripper_state']),    # 0 lub 1
            int(observation['human_reaching'])    # 0 lub 1
        ]).float().unsqueeze(0)  # [2] -> [1, 2]

        # === PRZETWARZANIE POZYCJI DŁONI CZŁOWIEKA ===
        # Pozycję ręki człowieka dodajemy jako wektor 3D (x, y, z)
        human_pos = observation['human_pose'].pose.position
        human_tensor = torch.tensor([
            human_pos.x,  # Pozycja w metrach (np. 0.6)
            human_pos.y,  # Pozycja w metrach (np. 0.0)
            human_pos.z   # Pozycja w metrach (np. 1.0)
        ]).float().unsqueeze(0)  # [3] -> [1, 3]

        # === SKŁADANIE SŁOWNIKA OBSERWACJI ===
        # Połącz wszystko w słownik zgodny z interfejsem WMA
        # Klucze muszą odpowiadać tym użytym podczas treningu modelu!
        return {
            "camera_rgb": camera_tensor,        # [1, 3, H, W] - obraz RGB
            "binary_features": binary_features, # [1, 2] - gripper + reaching
            "human_pos": human_tensor           # [1, 3] - pozycja dłoni xyz
        }
        
        # UWAGI DLA STUDENTÓW:
        # 1. Ten format obserwacji musi pasować do tego, na czym trenowano WMA
        # 2. Można dodać więcej modalności:
        #    - "depth_image": mapa głębokości
        #    - "joint_angles": aktualne kąty stawów robota
        #    - "force_sensor": czujnik siły w chwytaku
        # 3. Preprocessing może zawierać augmentację:
        #    - Normalizacja pozy do zakresu [-1, 1]
        #    - Crop/resize obrazu do stałego rozmiaru
        #    - Color jittering (tylko podczas treningu)
