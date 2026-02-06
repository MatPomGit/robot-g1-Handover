"""
Manager zadań WMA (WMA Task Manager)

Ten moduł implementuje uproszczony automat stanów (FSM - Finite State Machine)
do zarządzania sekwencją chwytania obiektu.

UWAGA DYDAKTYCZNA:
Jest to świadome uproszczenie dla celów edukacyjnych!
Rzeczywisty WMA (World Model AI) generowałby decyzje probabilistyczne
na podstawie obserwacji i przewidywań, a nie deterministycznych przejść.

AUTOMAT STANÓW (FSM):
    idle -> approach -> grasp -> lift

STANY:
- idle: Stan początkowy, czekanie na obiekt
- approach: Podejście do pozycji pre-grasp (nad obiektem)
- grasp: Chwyt obiektu (opuszczenie + zamknięcie chwytaka)
- lift: Podniesienie obiektu (zabezpieczenie przed upadkiem)

PRZEJŚCIA:
- Każde wywołanie update() przesuwa stan o jeden krok do przodu
- Deterministyczne (zawsze w tej samej kolejności)
- Brak warunków czy sprawdzeń (uproszczenie!)

PRAWDZIWY WMA RÓŻNIŁBY SIĘ:
- Decyzje oparte na obserwacjach (czy obiekt został chwycony?)
- Probabilistyczne przejścia (może trzeba powtórzyć grasp?)
- Adaptacja do niepowodzeń (re-grasp jeśli obiekt spadł)
- Planowanie na horyzoncie (przewiduj co się stanie)
"""

class WMATaskManager:
    """
    Uproszczony manager stanów do sekwencji chwytania
    
    W rzeczywistym systemie WMA:
    - Analizowałby obraz kamery (czy obiekt jest w chwytaku?)
    - Używał czujników siły (czy kontakt został nawiązany?)
    - Generował probabilistyczne decyzje (confidence scores)
    - Adaptował się do zmian (obiekt się przesunął)
    
    To jest deterministyczna wersja dla zrozumienia konceptu!
    """

    def __init__(self):
        """
        Inicjalizacja WMA Task Manager
        
        Ustawia stan początkowy na "idle" (czekanie).
        
        STANY MOŻLIWE:
        - "idle": Czekanie na obiekt do chwycenia
        - "approach": W trakcie podchodzenia do pre-grasp
        - "grasp": W trakcie chwytania obiektu
        - "lift": W trakcie podnoszenia obiektu
        """
        # Stan początkowy - robot czeka na obiekt
        self.state = "idle"

    def update(self, object_pose):
        """
        Aktualizuje stan automatu (FSM state transition)
        
        KROK PO KROKU:
        1. Sprawdza aktualny stan
        2. Przechodzi do następnego stanu
        3. Zwraca nowy stan
        
        Args:
            object_pose: PoseStamped - pozycja obiektu (nie używana w uproszczeniu!)
        
        Returns:
            string: Aktualny stan po przejściu
        
        UWAGA: W tym uproszczeniu object_pose jest ignorowana.
        Rzeczywisty WMA analizowałby pozycję obiektu aby:
        - Sprawdzić czy obiekt jest w zasięgu
        - Ocenić trudność chwytania (orientacja, kształt)
        - Dostosować strategię (approach z góry vs z boku)
        
        PRZEJŚCIA STANÓW:
        idle -> approach: Rozpocznij sekwencję chwytania
        approach -> grasp: Podejście zakończone, czas chwycić
        grasp -> lift: Obiekt chwycony, podnieś go
        lift -> lift: Stan końcowy (stay in lift)
        """
        if self.state == "idle":
            # === PRZEJŚCIE: idle -> approach ===
            # Obiekt został wykryty, rozpoczynamy sekwencję chwytania
            # Robot przejdzie do pozycji nad obiektem (pre-grasp)
            self.state = "approach"
            
        elif self.state == "approach":
            # === PRZEJŚCIE: approach -> grasp ===
            # Robot dotarł do pozycji pre-grasp
            # Czas opuścić się i chwycić obiekt
            self.state = "grasp"
            
        elif self.state == "grasp":
            # === PRZEJŚCIE: grasp -> lift ===
            # Robot chwycił obiekt (domniemanie!)
            # Czas podnieść obiekt do góry
            self.state = "lift"
            
        # else: self.state == "lift"
        #    Stan końcowy - pozostajemy w "lift"
        #    W prawdziwym systemie można by przejść do:
        #    - "retreat": cofnij ramię do pozycji bezpiecznej
        #    - "place": umieść obiekt w docelowej lokacji
        #    - "idle": powrót do stanu początkowego (czekaj na następny obiekt)
        
        return self.state

# === KOMENTARZ DLA STUDENTÓW ===
# To świadome uproszczenie dydaktyczne – realny WMA generuje decyzje probabilistyczne!
# 
# PRZYKŁAD PRAWDZIWEGO WMA:
#
# class RealWMATaskManager:
#     def __init__(self):
#         self.wma = WorldModelPolicy.from_pretrained("grasp_checkpoint")
#         self.state = "idle"
#     
#     def update(self, observation):
#         # WMA analizuje obserwacje (obraz, czujniki)
#         obs_tensor = preprocess(observation)
#         
#         # WMA przewiduje najlepszą akcję
#         action_probs = self.wma.infer(obs_tensor, current_state=self.state)
#         # action_probs = {"approach": 0.1, "grasp": 0.7, "lift": 0.2}
#         
#         # Wybieramy akcję z największym prawdopodobieństwem
#         next_state = max(action_probs, key=action_probs.get)
#         
#         # Dodatkowe sprawdzenia (czy grasp się powiódł?)
#         if next_state == "lift" and not is_object_grasped():
#             next_state = "grasp"  # Spróbuj ponownie
#         
#         self.state = next_state
#         return self.state
#
# DLACZEGO WMA JEST LEPSZY NIŻ DETERMINISTYCZNY FSM?
# 1. Obsługuje niepowodzenia (obiekt spadł -> retry grasp)
# 2. Adaptuje się do zmieniającego się środowiska (obiekt się przesunął)
# 3. Uczy się z doświadczenia (continuous improvement)
# 4. Przewiduje przyszłość (czy grasp się powiedzie?)
# 5. Generalizuje na nowe sytuacje (różne kształty obiektów)