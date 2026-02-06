"""
Moduł wykonywania przekazywania obiektów z WMA (Execute Handover with WMA)

Ten node ROS 2 jest głównym kontrolerem systemu przekazywania obiektów
między człowiekiem a robotem. Wykorzystuje World Model AI (WMA) do
podejmowania inteligentnych decyzji o akcjach robota.

ARCHITEKTURA:
1. Percepcja: Odbiera dane z kamer i czujników
2. Decyzja: WMA analizuje sytuację i wybiera akcję
3. Wykonanie: Robot wykonuje wybraną akcję (TAKE/GIVE/IDLE)

WORLD MODEL AI (WMA):
WMA to model sztucznej inteligencji, który:
- Przetwarza multimodalne obserwacje (obraz, stan chwytaka, pozycja człowieka)
- Przewiduje intencje człowieka (czy chce dać/wziąć obiekt)
- Planuje sekwencje akcji na horyzoncie czasowym
- Generuje wysokopoziomowe decyzje

KROK PO KROKU:
1. Subskrybuje topiki: obraz kamery, pozycja dłoni, stan chwytaka
2. Aggreguje obserwacje w słownik
3. Przekazuje obserwacje do WMA
4. WMA zwraca akcję: TAKE_FROM_HUMAN, GIVE_TO_HUMAN lub IDLE
5. Robot wykonuje odpowiednie ruchy i operacje chwytaka

TOPIKI ROS 2:
- Subskrybuje: /human_hand_pose (geometry_msgs/PoseStamped) - pozycja dłoni
- Subskrybuje: /human_reaching (std_msgs/Bool) - intencja człowieka
- Subskrybuje: /gripper_state (std_msgs/Bool) - stan chwytaka
- Subskrybuje: /camera/color/image_raw (sensor_msgs/Image) - obraz RGB
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from manipulation.moveit_interface import MoveItInterface
from manipulation.handover_planner import compute_handover_pose
from decision.wma_handover_manager import WMAHandoverManager
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ExecuteHandoverWMA(Node):
    """
    Node ROS 2 wykonujący przekazywanie obiektów z użyciem WMA
    
    Główny kontroler systemu handover, który integruje:
    - Percepcję (kamera, detekcja dłoni)
    - Decyzje AI (WMA)
    - Manipulację (MoveIt 2, chwytak)
    """

    def __init__(self):
        """
        Inicjalizacja node'a ExecuteHandoverWMA
        
        KROK PO KROKU:
        1. Inicjalizuje interfejs MoveIt 2 (sterowanie ramieniem)
        2. Inicjalizuje World Model AI (podejmowanie decyzji)
        3. Tworzy zmienne stanu systemu
        4. Subskrybuje wszystkie potrzebne topiki
        
        ZMIENNE STANU:
        - human_pose: Pozycja dłoni człowieka w 3D
        - human_reaching: Czy człowiek wyciąga rękę
        - gripper_occupied: Czy robot trzyma obiekt
        - camera_rgb: Aktualny obraz z kamery RGB
        """
        super().__init__("execute_handover_wma")
        
        # Inicjalizacja interfejsu do MoveIt 2
        # Pozwala na planowanie i wykonywanie ruchów ramienia + sterowanie chwytakiem
        self.moveit = MoveItInterface()
        
        # Inicjalizacja World Model AI
        # WMA będzie podejmował decyzje na podstawie obserwacji
        self.wma = WMAHandoverManager()

        # === ZMIENNE STANU SYSTEMU ===
        # Pozycja dłoni człowieka (None dopóki nie otrzymamy pierwszej wiadomości)
        self.human_pose = None
        
        # Czy człowiek wyciąga rękę w kierunku robota (intencja)
        self.human_reaching = False
        
        # Czy chwytak robota jest zajęty (trzyma obiekt)
        # True = robot trzyma obiekt, False = chwytak pusty
        self.gripper_occupied = False
        
        # Aktualny obraz RGB z kamery (numpy array)
        self.camera_rgb = None
        
        # CvBridge do konwersji ROS Image <-> OpenCV numpy array
        self.bridge = CvBridge()

        # === SUBSKRYPCJE TOPIKÓW ===
        # Subskrypcja pozycji dłoni człowieka
        # Callback human_cb wywoływany przy każdej nowej pozycji
        self.create_subscription(PoseStamped, "/human_hand_pose", self.human_cb, 10)
        
        # Subskrypcja intencji człowieka (czy wyciąga rękę)
        self.create_subscription(Bool, "/human_reaching", self.intent_cb, 10)
        
        # Subskrypcja stanu chwytaka (otwarty/zamknięty)
        self.create_subscription(Bool, "/gripper_state", self.gripper_cb, 10)
        
        # Subskrypcja obrazu RGB z kamery
        self.create_subscription(Image, "/camera/color/image_raw", self.image_cb, 10)

    def human_cb(self, msg):
        """
        Callback odbierający pozycję dłoni człowieka
        
        KROK PO KROKU:
        1. Zapisuje pozycję dłoni do self.human_pose
        2. Wywołuje update() aby przetworzyć nową obserwację
        
        Args:
            msg: PoseStamped - pozycja dłoni w 3D
        
        UWAGA: update() jest wywoływany tylko przy nowej pozycji dłoni,
        bo to najważniejsza informacja dla decyzji o handover.
        Jeśli dłoń się nie rusza, nie ma potrzeby ciągłego wywołania WMA.
        """
        # Zapisujemy pozycję dłoni
        self.human_pose = msg
        
        # Wywołujemy przetwarzanie z nową obserwacją
        # To uruchomi WMA i potencjalnie akcję robota
        self.update()

    def intent_cb(self, msg):
        """
        Callback odbierający informację o intencji człowieka
        
        Args:
            msg: Bool - True jeśli człowiek wyciąga rękę, False w przeciwnym razie
        
        INTENCJA CZŁOWIEKA:
        - True = człowiek wyciąga rękę (chce dać lub wziąć obiekt)
        - False = człowiek nie wyciąga ręki (brak intencji interakcji)
        """
        self.human_reaching = msg.data

    def gripper_cb(self, msg):
        """
        Callback odbierający stan chwytaka
        
        Args:
            msg: Bool - True jeśli chwytak trzyma obiekt, False jeśli pusty
        
        STAN CHWYTAKA:
        - True = chwytak zamknięty, robot trzyma obiekt
        - False = chwytak otwarty lub pusty
        
        UWAGA: W rzeczywistym systemie należy używać sensory force/torque
        lub czujników dotyku aby niezawodnie wykryć obecność obiektu.
        """
        self.gripper_occupied = msg.data

    def image_cb(self, msg):
        """
        Callback odbierający obraz RGB z kamery
        
        Args:
            msg: Image - obraz z kamery w formacie ROS
        
        KROK PO KROKU:
        1. Konwertuje ROS Image na OpenCV numpy array (format BGR)
        2. Zapisuje do self.camera_rgb
        
        UWAGA: Format BGR jest używany przez OpenCV (Blue-Green-Red)
        WMA może wymagać konwersji do RGB w preprocessing.
        """
        # Konwersja ROS Image -> numpy array (format BGR)
        self.camera_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def update(self):
        """
        Główna funkcja przetwarzania - podejmuje i wykonuje decyzje WMA
        
        KROK PO KROKU:
        1. Sprawdza czy mamy wszystkie wymagane dane
        2. Tworzy słownik obserwacji dla WMA
        3. Wywołuje WMA aby uzyskać akcję
        4. Wykonuje akcję (TAKE_FROM_HUMAN, GIVE_TO_HUMAN, lub IDLE)
        
        AKCJE:
        - TAKE_FROM_HUMAN: Robot podjeżdża do dłoni i zamyka chwytak
        - GIVE_TO_HUMAN: Robot podjeżdża do dłoni i otwiera chwytak
        - IDLE: Robot nie wykonuje akcji (czeka)
        
        BEZPIECZEŃSTWO:
        W produkcyjnym systemie należy dodać:
        - Sprawdzanie kolizji przed ruchem
        - Monitorowanie siły podczas kontaktu
        - Emergency stop jeśli człowiek jest zbyt blisko
        - Timeout dla akcji (jeśli ruch trwa zbyt długo)
        """
        # Sprawdzamy czy mamy wszystkie potrzebne dane
        if self.human_pose is None or self.camera_rgb is None:
            # Brakuje pozycji dłoni lub obrazu z kamery
            # Nie możemy podjąć decyzji, więc wychodzimy
            return

        # === PRZYGOTOWANIE OBSERWACJI DLA WMA ===
        # Tworzymy słownik zawierający wszystkie dane potrzebne WMA
        observation = {
            "camera_rgb": self.camera_rgb,           # Obraz RGB (numpy array)
            "gripper_state": self.gripper_occupied,  # Czy robot trzyma obiekt
            "human_pose": self.human_pose,           # Pozycja dłoni człowieka
            "human_reaching": self.human_reaching    # Czy człowiek wyciąga rękę
        }

        # === WYWOŁANIE WMA - PODEJMOWANIE DECYZJI ===
        # WMA analizuje obserwacje i zwraca akcję
        # Wewnątrz infer_action():
        # 1. Preprocessing obserwacji (konwersja do tensorów)
        # 2. Przepuszczenie przez sieć neuronową
        # 3. Planowanie sekwencji akcji na horyzoncie
        # 4. Wybór najlepszej akcji
        action = self.wma.infer_action(observation)

        # === WYKONANIE AKCJI ===
        if action == "TAKE_FROM_HUMAN":
            # AKCJA: Robot bierze obiekt od człowieka
            # 
            # KROK PO KROKU:
            # 1. Oblicz pozycję handover (z ergonomicznym offsetem)
            pose = compute_handover_pose(self.human_pose)
            
            # 2. Przesuń ramię do pozycji handover
            #    MoveIt 2 zaplanuje trajektorię bez kolizji
            self.moveit.move_to_pose(pose)
            
            # 3. Zamknij chwytak (chwyć obiekt)
            #    Robot delikatnie zamyka palce wokół obiektu
            self.moveit.close_gripper()
            
            # EFEKT: Robot chwycił obiekt z dłoni człowieka

        elif action == "GIVE_TO_HUMAN":
            # AKCJA: Robot daje obiekt człowiekowi
            # 
            # KROK PO KROKU:
            # 1. Oblicz pozycję handover
            pose = compute_handover_pose(self.human_pose)
            
            # 2. Przesuń ramię z obiektem do pozycji handover
            self.moveit.move_to_pose(pose)
            
            # 3. Otwórz chwytak (puść obiekt)
            #    Człowiek powinien złapać obiekt w tym momencie
            self.moveit.open_gripper()
            
            # EFEKT: Robot przekazał obiekt człowiekowi

        # else: action == "IDLE"
        #    Robot nie wykonuje żadnej akcji
        #    Czeka na zmianę sytuacji (człowiek wyciąga rękę, etc.)

def main():
    """
    Funkcja główna uruchamiająca node ExecuteHandoverWMA
    
    KROK PO KROKU:
    1. Inicjalizuje system ROS 2
    2. Tworzy instancję node'a ExecuteHandoverWMA
    3. Uruchamia pętlę przetwarzania (czeka na callbacki)
    4. Po zakończeniu (Ctrl+C) czyści zasoby
    
    Użycie:
        ros2 run g1_pick_and_handover execute_handover_wma
    
    Wymagania:
        - Uruchomiony human_hand_detector (pozycja dłoni)
        - Uruchomiona kamera (obraz RGB)
        - Uruchomiony MoveIt 2 (planowanie ruchu)
        - Zainstalowany checkpoint WMA (model AI)
        - Uruchomione sensory chwytaka
    
    TESTOWANIE:
    1. Uruchom ten node
    2. Wyciągnij rękę w kierunku robota
    3. Obserwuj jak WMA podejmuje decyzję
    4. Robot powinien się przemieścić i otworzyć/zamknąć chwytak
    """
    # Inicjalizacja ROS 2
    rclpy.init()
    
    # Uruchomienie node'a - spin() przetwarza callbacki
    # Każda nowa pozycja dłoni wywołuje human_cb() -> update()
    rclpy.spin(ExecuteHandoverWMA())
    
    # Czyszczenie po zakończeniu
    rclpy.shutdown()
